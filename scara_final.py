"""
SCARA YOLO11 Locked-Snapshot Pick-and-Place Controller
------------------------------------------------------
Optimized with verified inverse kinematics (IK) engine and high-speed dynamic 7-step sequence. 
Features dual-mode dynamic sorting based on shape and color selection.

Robot architecture:
    Motor 1: Base Stepper
    Motor 2: Elbow Stepper
    Motor 3: Z-Axis Stepper
    Motor 4: Gripper Rotation Stepper, handled by Arduino as J4 = -(J1 + J2)
    Motor 5: Jaw Servo, Python sends 0=open or 50=closed
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise RuntimeError("Install ultralytics first: pip install ultralytics") from exc

try:
    import serial
except ImportError:
    serial = None


# =============================================================================
# Configuration
# =============================================================================

YOLO_MODEL_PATH = Path(r"D:\PycharmProjects\opencv\runs\detect\train\weights\best.pt") # Add your best.pt file's path here
CALIBRATION_FILE = Path(__file__).with_name("calibration_matrix.npy") # Add your calibration matrix here

SERIAL_PORT = "COM5" # You can update it according to your port
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT_SECONDS = 0.01

CAMERA_INDEX = 2 # You can update it according to your port
CONFIDENCE_THRESHOLD = 0.60
WINDOW_NAME = "SCARA YOLO11 Locked Snapshot Controller"

# Elbow length
L1_MM = 290.0
L2_MM = 180.0

Z_HIGH_DEG = 550.0
Z_LOW_DEG = 0.0

JAW_OPEN = 0.0
JAW_CLOSED = 50.0

# Fallback default drop zone if parsing fails
DROP_X_MM = -300.0
DROP_Y_MM = 300.0

HOME_J1_DEG = 0.0
HOME_J2_DEG = 0.0

STATE_IDLE = "IDLE"
STATE_WAITING_DONE = "WAITING_DONE"


@dataclass
class Detection:
    class_name: str
    confidence: float
    box_xyxy: Tuple[int, int, int, int]
    center_px: Tuple[float, float]
    x_mm: float
    y_mm: float
    j1_deg: float
    j2_deg: float


@dataclass
class MoveCommand:
    label: str
    j1_deg: float
    j2_deg: float
    z_deg: float
    jaw_deg: float

    def to_serial(self) -> str:
        return (
            f"MOVE,{self.j1_deg:.2f},"
            f"{self.j2_deg:.2f},"
            f"{self.z_deg:.2f},"
            f"{self.jaw_deg:.2f}\n"
        )


class NonBlockingScaraController:
    def __init__(self, serial_conn: Optional["serial.Serial"]):
        self.serial_conn = serial_conn
        self.state = STATE_IDLE

        self.job_queue: List[Detection] = []
        self.command_queue: List[MoveCommand] = []

        self.active_target: Optional[Detection] = None
        self.active_command: Optional[MoveCommand] = None

        self.sim_done_time = 0.0
        self.last_serial_line = ""

        # Track if the current object is the first item in the batch for acceleration tuning
        self.is_first_of_batch = True

        # Sorting criterion configuration: None, "SHAPE", or "COLOUR"
        self.sorting_mode: Optional[str] = None

    def start_jobs(self, detections_snapshot: List[Detection]) -> None:
        if self.state != STATE_IDLE:
            print("Robot is busy. Ignoring new trigger.")
            return

        if not detections_snapshot:
            print("Snapshot queue is empty.")
            return

        self.job_queue = list(detections_snapshot)

        # Reset first-in-batch flag upon initializing a new detection queueে
        self.is_first_of_batch = True

        print(f"Locked snapshot queue with {len(self.job_queue)} target(s) for {self.sorting_mode} sorting.")
        self.start_next_job()

    def start_next_job(self) -> None:
        # Return to home position only when both the job queue and active target are emptyে
        if not self.job_queue and self.active_target is None:
            self.active_target = None
            self.command_queue = build_home_sequence()
            print("Snapshot queue complete. Sending arm home.")
            self.send_next_command()
            return

        # Prevent homing if the queue is exhausted but active command execution is pendingে
        if not self.job_queue and self.active_command is not None:
            return

        self.active_target = self.job_queue.pop(0)

        # Generate motion sequence passing the currently selected sorting modeছে
        commands = build_safe_pick_place_sequence(
            self.active_target,
            is_first=self.is_first_of_batch,
            sorting_mode=self.sorting_mode
        )

        # Clear first-in-batch flag once the initial item sequence is generated
        self.is_first_of_batch = False

        if commands is None:
            print("Skipping unreachable target.")
            self.start_next_job()
            return

        self.command_queue = commands

        print(
            f"Processing locked target: {self.active_target.class_name} "
            f"mm=({self.active_target.x_mm:.1f}, {self.active_target.y_mm:.1f}) "
            f"remaining={len(self.job_queue)}"
        )

        self.send_next_command()

    def update(self) -> None:
        if self.state == STATE_IDLE:
            return

        if self.serial_conn is None:
            if time.time() >= self.sim_done_time:
                print("SIM <- DONE")
                self.handle_done()
            return

        try:
            while self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                self.last_serial_line = line
                print(f"ARDUINO <- {line}")

                if line == "DONE":
                    self.handle_done()
                    return

                if line.startswith("ERROR"):
                    print("Arduino reported ERROR. Aborting command queue.")
                    self.command_queue.clear()
                    self.job_queue.clear()
                    self.active_command = None
                    self.active_target = None
                    self.state = STATE_IDLE
                    return

        except Exception as exc:
            print(f"Serial read error: {exc}")

    def send_next_command(self) -> None:
        if not self.command_queue:
            self.active_command = None

            if self.active_target is not None:
                print("Finished locked target.")
                self.active_target = None
                self.start_next_job()
                return

            self.state = STATE_IDLE
            print("All snapshot jobs complete. Returning to IDLE and resuming YOLO.")
            return

        self.active_command = self.command_queue.pop(0)
        serial_text = self.active_command.to_serial()

        if self.serial_conn is None:
            print(f"SIM -> {self.active_command.label}: {serial_text.strip()}")
            self.sim_done_time = time.time() + 0.35
        else:
            try:
                self.serial_conn.write(serial_text.encode("utf-8"))
                print(f"PYTHON -> {self.active_command.label}: {serial_text.strip()}")
            except Exception as exc:
                print(f"Serial write error: {exc}")
                self.command_queue.clear()
                self.job_queue.clear()
                self.active_command = None
                self.active_target = None
                self.state = STATE_IDLE
                return

        self.state = STATE_WAITING_DONE

    def handle_done(self) -> None:
        if self.active_command is not None:
            print(f"Finished step: {self.active_command.label}")

        self.active_command = None
        self.send_next_command()

    def status_text(self) -> str:
        if self.sorting_mode is None:
            return "WORKFLOW SELECTOR: Waiting for Sorting Criteria..."

        if self.state == STATE_IDLE:
            return f"State: IDLE | Mode: SORT BY {self.sorting_mode} | Press 's' to lock snapshot"

        if self.active_command is None:
            return f"State: BUSY | Mode: SORT BY {self.sorting_mode} | Queue Active"

        target_text = ""
        if self.active_target is not None:
            target_text = (
                f" | target {self.active_target.class_name} "
                f"({self.active_target.x_mm:.0f}, {self.active_target.y_mm:.0f})mm"
            )

        return f"State: BUSY - {self.active_command.label}{target_text}"


def load_homography() -> np.ndarray:
    if not CALIBRATION_FILE.exists():
        raise FileNotFoundError(f"Calibration matrix not found: {CALIBRATION_FILE}")

    matrix = np.load(CALIBRATION_FILE)
    matrix = np.asarray(matrix, dtype=np.float64)

    if matrix.shape != (3, 3):
        raise ValueError(f"Calibration matrix must be 3x3, got {matrix.shape}")

    return matrix


def load_model() -> YOLO:
    if not YOLO_MODEL_PATH.exists():
        raise FileNotFoundError(f"YOLO model not found: {YOLO_MODEL_PATH}")

    return YOLO(str(YOLO_MODEL_PATH))


def open_serial() -> Optional["serial.Serial"]:
    if serial is None:
        print("pyserial not installed. Running in simulation mode.")
        return None

    try:
        conn = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUDRATE,
            timeout=SERIAL_TIMEOUT_SECONDS,
        )
        time.sleep(2.0)
        print(f"Serial connected on {SERIAL_PORT} @ {SERIAL_BAUDRATE}")
        return conn

    except Exception as exc:
        print(f"Serial connection failed: {exc}")
        print("Running in simulation mode.")
        return None


def pixel_to_robot_mm(pixel_x: float, pixel_y: float, homography: np.ndarray) -> Tuple[float, float]:
    point = np.asarray([[[pixel_x, pixel_y]]], dtype=np.float32)
    world = cv2.perspectiveTransform(point, homography)[0, 0]
    return float(world[0]), float(world[1])


def calculate_inverse_kinematics_mm(x_mm: float, y_mm: float) -> Optional[Tuple[float, float]]:
    """
Calculates 2-DOF SCARA inverse kinematics with inverted J2 motor mapping adjustment.
    """
    l1 = L1_MM
    l2 = L2_MM

    r = math.sqrt(x_mm**2 + y_mm**2)

    # Reach limit checkpoint
    if r > l1 + l2 or r < abs(l1 - l2):
        print(f"Target Out of Reach: X={x_mm:.1f} mm, Y={y_mm:.1f} mm")
        return None

    cos_angle2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_angle2 = max(-1.0, min(1.0, cos_angle2))

    angle2 = math.acos(cos_angle2)

    k1 = l1 + l2 * math.cos(angle2)
    k2 = l2 * math.sin(angle2)
    angle1 = math.atan2(y_mm, x_mm) - math.atan2(k2, k1)

    q1_deg = math.degrees(angle1)
    q2_deg = -math.degrees(angle2)  # Correct J2 rotation direction to match hardware kinematics

    return q1_deg, q2_deg


def run_yolo_detections(frame: np.ndarray, model: YOLO, homography: np.ndarray) -> List[Detection]:
    detections: List[Detection] = []

    results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)

    for result in results:
        if result.boxes is None:
            continue

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy()

            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0

            x_mm, y_mm = pixel_to_robot_mm(center_x, center_y, homography)

            ik = calculate_inverse_kinematics_mm(x_mm, y_mm)

            if ik is None:
                continue

            class_id = int(box.cls[0].detach().cpu().item())
            confidence = float(box.conf[0].detach().cpu().item())
            class_name = str(model.names.get(class_id, class_id))

            detections.append(
                Detection(
                    class_name=class_name,
                    confidence=confidence,
                    box_xyxy=(int(x1), int(y1), int(x2), int(y2)),
                    center_px=(center_x, center_y),
                    x_mm=x_mm,
                    y_mm=y_mm,
                    j1_deg=ik[0],
                    j2_deg=ik[1],
                )
            )

    return detections


def build_safe_pick_place_sequence(target: Detection, is_first: bool, sorting_mode: Optional[str]) -> Optional[List[MoveCommand]]:
    """
Builds a 7-step pick-and-place instruction sequence based on dynamic drop zone mapping.
    """
    # Parse object class name into shape and color attributes (e.g., 'circle_orange')
    parts = target.class_name.lower().split('_')
    obj_shape = parts[0] if len(parts) > 0 else ""
    obj_color = parts[1] if len(parts) > 1 else ""

    # Initialize fallback drop coordinates
    drop_x = DROP_X_MM
    drop_y = DROP_Y_MM

    # Case 1: Color-based sorting matrix
    if sorting_mode == "COLOUR":
        if "orange" in obj_color:
            drop_x, drop_y = -200.0, 400.0
        elif "blue" in obj_color:
            drop_x, drop_y = -200.0, 170.0

    # Case 2: Shape-based sorting matrix
    elif sorting_mode == "SHAPE":
        if "circle" in obj_shape:
            drop_x, drop_y = -200.0, 280.0
        elif "square" in obj_shape:
            drop_x, drop_y = -200.0, 400.0
        elif "rect" in obj_shape:  # Handle spelling variations for rectangular geometries
            drop_x, drop_y = -200.0, 170.0

    # Compute target kinematics for the evaluated drop location
    drop_ik = calculate_inverse_kinematics_mm(drop_x, drop_y)

    if drop_ik is None:
        print(f"Target Dynamic Drop Point ({drop_x}, {drop_y}) is unreachable.")
        return None

    drop_j1, drop_j2 = drop_ik
    commands = []

    # Perform safety lift from home coordinates only for the first object in sequence
    if is_first:
        commands.append(MoveCommand("1 Safety Lift From Home", HOME_J1_DEG, HOME_J2_DEG, Z_HIGH_DEG, JAW_OPEN))

    # Remaining 6-step pick, lift, and place sequence
    commands.extend([
        MoveCommand("2 Move Above Object", target.j1_deg, target.j2_deg, Z_HIGH_DEG, JAW_OPEN),
        MoveCommand("3 Descend to Object", target.j1_deg, target.j2_deg, Z_LOW_DEG, JAW_OPEN),
        MoveCommand("4 Grip Object", target.j1_deg, target.j2_deg, Z_LOW_DEG, JAW_CLOSED),
        MoveCommand("5 Vertical Lift Safe", target.j1_deg, target.j2_deg, Z_HIGH_DEG, JAW_CLOSED),
        MoveCommand("6 Move to Drop Zone", drop_j1, drop_j2, Z_HIGH_DEG, JAW_CLOSED),
        MoveCommand("7 Release Object", drop_j1, drop_j2, Z_HIGH_DEG, JAW_OPEN),
    ])

    return commands


def build_home_sequence() -> List[MoveCommand]:
    return [
        MoveCommand("HOME Lift Safe", HOME_J1_DEG, HOME_J2_DEG, Z_HIGH_DEG, JAW_OPEN),
        MoveCommand("HOME Stable", HOME_J1_DEG, HOME_J2_DEG, Z_LOW_DEG, JAW_OPEN),
    ]


def draw_overlay(
    frame: np.ndarray,
    detections: List[Detection],
    controller: NonBlockingScaraController,
) -> None:
    # Render semi-transparent dark status bar at the top of the viewport
    overlay = frame.copy()
    cv2.rectangle(overlay, (8, 8), (950, 76), (0, 0, 0), -1)  # ওপরে কালো বক্স
    alpha = 0.4  # Define opacity ratio for heads-up-display overlay
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    # Prompt user to select configuration criteria if sorting mode is undefined
    if controller.sorting_mode is None:
        cv2.putText(
            frame,
            "CRITERIA SELECT MODULE",
            (18, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (0, 165, 255),
            2,
        )
        cv2.putText(
            frame,
            "Press 's' to Sort by SHAPE  |  Press 'c' to Sort by COLOUR  |  'q' to Quit",
            (18, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )
        return

    # Render bounding boxes and targets when sorting mode is active
    for index, detection in enumerate(detections, start=1):
        x1, y1, x2, y2 = detection.box_xyxy
        center_x, center_y = detection.center_px

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), cv2.FILLED)

        label = f"{index}. {detection.class_name} {detection.confidence:.2f}"
        coords = f"mm=({detection.x_mm:.1f}, {detection.y_mm:.1f})"
        angles = f"J1={detection.j1_deg:.1f}, J2={detection.j2_deg:.1f}"

        text_y = max(24, y1 - 54)

        cv2.putText(frame, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2)
        cv2.putText(frame, coords, (x1, text_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 0), 2)
        cv2.putText(frame, angles, (x1, text_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 0), 2)

    cv2.putText(
        frame,
        controller.status_text(),
        (18, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        (
            f"Detections: {len(detections)} | "
            f"Locked Queue: {len(controller.job_queue)} | "
            f"Press 's' to Snapshot/Lock | 'r' to Reset Criteria"
        ),
        (18, 64),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2,
    )


def main() -> int:
    try:
        homography = load_homography()
        model = load_model()

    except Exception as exc:
        print(f"Startup error: {exc}")
        return 1

    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"Could not open camera index {CAMERA_INDEX}")
        return 1

    serial_conn = open_serial()
    controller = NonBlockingScaraController(serial_conn)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    latest_detections: List[Detection] = []

    print("SCARA dual-mode controller running.")
    print("Initial State: Standby workflow selector loop active.")

    try:
        while True:
            ok, frame = cap.read()

            if not ok:
                print("Camera frame read failed.")
                break

            # Execute YOLO inference only if a sorting criteria is active and the arm is idleে
            if controller.sorting_mode is not None and controller.state == STATE_IDLE:
                latest_detections = run_yolo_detections(frame, model, homography)
            else:
                latest_detections = []

            controller.update()

            display = frame.copy()
            draw_overlay(display, latest_detections, controller)
            cv2.imshow(WINDOW_NAME, display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            # Keybind 'r': Reset runtime parameters and return to workflow selector
            if key == ord("r"):
                if controller.state == STATE_IDLE:
                    controller.sorting_mode = None
                    print("Sorting criteria reset. Back to workflow selector.")
                else:
                    print("Robot is busy. Cannot reset mode right now.")

            # Handle conditional operational flags for UI control
            if key == ord("s"):
                # Context 1: Set operation profile to shape-based sorting
                if controller.sorting_mode is None:
                    controller.sorting_mode = "SHAPE"
                    print("Workflow Activated: Sorting by SHAPE. YOLO initiated.")
                    continue

                # দ্# Context 2: Lock current detection frame as static job queue by pressing 's'
                if controller.state != STATE_IDLE:
                    print("Robot is busy. Wait for current locked queue to finish.")
                    continue

                if not latest_detections:
                    print("No live detections available to lock.")
                    continue

                locked_snapshot = list(latest_detections)
                print(f"Locked {len(locked_snapshot)} target(s) from live frame.")
                controller.start_jobs(locked_snapshot)

            elif key == ord("c"):
                # ম# Context 3: Set operation profile to color-based sorting by pressing 'c'
                if controller.sorting_mode is None:
                    controller.sorting_mode = "COLOUR"
                    print("Workflow Activated: Sorting by COLOUR. YOLO initiated.")
                    continue

    finally:
        cap.release()

        if serial_conn is not None:
            serial_conn.close()

        cv2.destroyAllWindows()
        print("SCARA controller stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
