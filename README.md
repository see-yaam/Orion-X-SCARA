# 4-DOF SCARA Robotic Arm with Dual Mode Object Sorting
 
An advanced, open-architecture **4-DOF SCARA (Selective Compliance Assembly Robot Arm)** designed for automated industrial pick-and-place sorting operations. This system integrates real-time Object Detection using **YOLOv11** with custom analytical **Inverse Kinematics (IK)** to map coordinates dynamically from a 2D camera frame to physical robotic joint angles.
 
---
 
## 🧩 Problem Statement
 
Imagine a beverage factory that produces multiple flavours across multiple bottle sizes. At some point for some production glitch in the process, batches get mixed into a single pile. Sorting this by hand is slow, repetitive, and error-prone, especially at any real production volume.
 
This project tackles that problem with a camera-guided SCARA arm capable of **dual-mode sorting**:
 
* **Colour-based sorting** — bottle colour maps to flavour (e.g. red → strawberry, orange → orange, purple → grape).
* **Shape/size-based sorting** — bottle size class maps to packaging category (e.g. 250ml vs 500ml).
The operator picks a sorting criterion, the vision system identifies and localizes each bottle, and the arm picks and places them into the correct bin automatically.
 
---

## 🚀 Key Features

* **4-DOF Motion Architecture:** Features 3 Revolute (Yaw/Roll) joints for horizontal rotation and gripper orientation, and 1 Prismatic (Linear) joint for vertical Z-axis depth tracking.
* **AI-Powered Computer Vision:** Real-time localization and class classification using a custom-trained **YOLOv11** model pipeline processed via OpenCV.
* **Analytical Inverse Kinematics:** Implements a closed-form geometric solution using the **Law of Cosines** and **2-argument Arctangent (`atan2`)** for instantaneous, iteration-free joint angle computation.
* **Camera-to-Robot Spatial Mapping:** Uses a custom perspective transformation (Homography Matrix) calibration system to convert raw pixel coordinates $(X_{pixel}, Y_{pixel})$ into absolute physical world dimensions ($mm$).
* **Open Source Control Pipeline:** Asynchronous serial communication protocol (`PySerial`) syncing the Python core AI with a low-level hardware controller (Arduino).

---

## 🛠️ System Architecture & Workflow

The entire automation loop executes in milliseconds following this systematic pipeline:

1. **Vision Acquisition:** A top-down USB camera streams frames to the Python workspace.
2. **YOLOv11 Inference:** The custom model infers bounding boxes and extracts the center coordinates of target objects.
3. **Spatial Calibration:** Pixels are mapped directly to physical millimeters ($mm$) based on a pre-calibrated reference grid.
4. **Inverse Kinematics Solver:** Python translates the targeted $(X, Y, Z)$ spatial coordinates and desired orientation angle ($\phi$) into specific joint angles $(\theta_1, \theta_2, \theta_4)$.
5. **Actuator Execution:** Target angles are converted into discrete step pulses and transmitted via Serial to the Arduino. Stepper drivers sequentially fire the NEMA 17 motors to execute precise motion.

---

## 🧭 Path Planning & Motion Safety Logic
 
Every pick-and-place operation follows a fixed 7-step motion sequence, designed to avoid collisions with neighboring bottles and keep the arm's trajectory predictable:
 
1. **Safety lift from home** — only runs once, at the start of a new batch, to clear the arm off the base before any motion begins.
2. **Move above object** — the arm travels to the target's (X, Y) at a high Z, staying clear of any bottles in its path instead of cutting diagonally through the workspace.
3. **Descend to object** — Z drops straight down onto the target, only once the arm is already correctly positioned above it.
4. **Grip object** — the jaw closes on the bottle.
5. **Vertical lift (safe height)** — the arm lifts straight up again before moving horizontally, so it doesn't drag the bottle sideways through other bottles.
6. **Move to drop zone** — travels at the safe height to the bin corresponding to the detected flavour/size.
7. **Release object** — jaw opens, bottle drops into the correct bin.
Once every object in a locked batch has been sorted, the arm automatically returns home. This "always lift before moving sideways" rule is the core safety logic — it's a simple but effective way to avoid the arm clipping other bottles on a crowded tray without needing full obstacle-avoidance planning.
 
Sorting is handled in two selectable modes:
 
* **SHAPE mode** — detected class name is split into shape/colour tags, and shape drives the drop-zone selection.
* **COLOUR mode** — same split, but colour drives the drop-zone selection instead.
Each mode maps to a fixed set of drop coordinates, which are run through the same IK solver as the pick target to get the joint angles for the drop-off.
 
---

## 📐 Mathematical Foundations (Inverse Kinematics)

Instead of relying on heavy numerical solvers, the kinematics profile is computed analytically:

### 1. Horizontal Plane Geometry ($X-Y$ Position)
The base joint ($\theta_1$) and elbow joint ($\theta_2$) form a 2-link planar system. By calculating the radial distance $r = \sqrt{X^2 + Y^2}$ from the base to the target, we extract $\theta_2$ using the **Law of Cosines**:

$$\cos(\theta_2) = \frac{X^2 + Y^2 - L_1^2 - L_2^2}{2L_1L_2}$$

$$\theta_2 = \arccos\left(\frac{X^2 + Y^2 - L_1^2 - L_2^2}{2L_1L_2}\right)$$

Once $\theta_2$ is found, $\theta_1$ is computed using vector trigonometry to eliminate quadrant ambiguity:

$$\theta_1 = \arctan2(Y, X) - \arctan2(L_2\sin\theta_2, L_1 + L_2\cos\theta_2)$$

### 2. Gripper Orientation ($\theta_4$)
To match the alignment of the object ($\phi$) relative to the current physical pose of the arm, the end-effector roll is derived via angular subtraction:

$$\theta_4 = \phi - (\theta_1 + \theta_2)$$

---

## 🔌 Communication Protocol
 
Python and the Arduino stay in sync over a simple non-blocking serial protocol:
 
* **Command packet:** `MOVE,J1,J2,Z,JAW\n` — joint angles in degrees, comma-delimited.
* **State machine:** the Python controller sits in `IDLE` until a batch is locked in, then moves to `WAITING_DONE` after sending each command.
* **Handshake:** the Arduino executes the move and replies with `DONE` once it finishes, which is what tells Python to pop the next command off the queue.
* **Error handling:** if the Arduino reports `ERROR`, the current job and queue are cleared immediately and the controller resets to `IDLE` rather than continuing blindly.
This keeps the Python side from ever "getting ahead" of the physical arm — every motion command waits for hardware confirmation before the next one is sent.
 
---

## ⚙️ Tech Stack & Components

### Hardware Architecture:
* **Actuators:** NEMA 17 Stepper Motors (High Torque).
* **Motor Drivers:** A4988 / TMC2209 Electronic Drivers configured with microstepping for smooth acceleration profiles.
* **Microcontroller:** Arduino Uno (Acts as the hard real-time step generator).
* **Vision Sensor:** USB Webcam.
* **Mechanical Structure:** Custom-fabricated 4-DOF SCARA Framework.

### Software Stack:
* **Language:** Python 3.13+ & C++ (Arduino Sketch).
* **AI Framework:** Ultralytics YOLOv11 (Custom dataset labeled via Roboflow).
* **Computer Vision:** OpenCV (Image pre-processing & spatial warping).
* **Firmware Controls:** AccelStepper Library (For jerk-free stepper acceleration).

---

## 📁 Repository Structure

```text
├── Arduino Control/
│   └── ScaraArmControllerPySerial.ino  # The arduino Code
├── Shape and Colour.v2i.yolov11/       # Custom image dataset pipeline
│   ├── test/                           
│   ├── train/                          
│   ├── valid/                          
│   ├── README.dataset.txt              
│   ├── README.roboflow.txt             
│   └── data.yaml                       # YOLO model dataset configuration mapping
├── calibration.py                      # Spatial homography & grid calibration module
├── scara_final.py                      # Main real-time automation loop and SCARA's main GUI Interface(ML + IK + Serial)
├── yolo11n.pt                          # Custom trained YOLOv11 object detection weights
└── README.md                           # System documentation and setup guide
```

---

## ⚙️ Installation & Setup Guide

Follow these steps to get the arm running from a fresh clone.

### 1. Clone the Repository
```bash
git clone https://github.com/see-yaam/Orion-X-SCARA.git
cd Orion-X-SCARA
```

### 2. Set Up a Virtual Environment (Recommended)
```bash
# Create a virtual environment
python -m venv .venv

# Activate the environment (Windows)
.venv\Scripts\activate

# Activate the environment (Mac/Linux)
source .venv/bin/activate
```

### 3. Install Required Dependencies
```bash
pip install ultralytics opencv-python pyserial numpy
```

### 4. Hardware Setup
1. Connect your Arduino Uno to your computer via USB.
2. Open the Arduino IDE and load `Arduino Control/ScaraArmControllerPySerial.ino`.
3. Verify and upload the sketch to your board.
4. Note which serial port the Arduino connects to (e.g. `COM5` on Windows, or `/dev/ttyUSB0` / `/dev/cu.usbmodemXXXX` on Mac/Linux) — you'll need it in the next step.

### 5. Configure File Paths
Before running, open `scara_final.py` and update these variables near the top of the file to match your setup:
* `YOLO_MODEL_PATH` → path to your trained `best.pt` weights file
* `CALIBRATION_FILE` → path to your `calibration_matrix.npy` (generated in the next step)
* `SERIAL_PORT` → the port you noted in the hardware setup step
* `CAMERA_INDEX` → your webcam's device index (usually `0`, but try `1` or `2` if it opens the wrong camera)

### 6. Run Camera Calibration (One-time setup)
Generate the homography matrix that maps camera pixels to real-world millimeters:
```bash
python calibration.py
```
This produces `calibration_matrix.npy`, which `scara_final.py` needs to run — skipping this step will cause the main script to fail on startup.

> ⚠️ **Important:** Calibration is tied to the camera's exact position. Once calibrated, don't move or bump the camera — if you do, the homography mapping breaks and you'll need to re-run `calibration.py`. As long as the camera stays fixed, you only need to calibrate once.

### 7. Run the System
With the Arduino connected and calibration done, launch the main control interface:
```bash
python scara_final.py
```

---
 
## 🐛 Troubleshooting / Challenges Faced
 
A few real issues that came up during development and testing:
 
* **Calibration breaking after camera movement:** the homography matrix is only valid for the exact camera position it was calibrated at. Any accidental nudge to the camera mount throws off the pixel-to-mm mapping, and the arm starts missing targets. Fix is always the same, re-run calibration after any camera movement, and avoid touching the mount once it's calibrated.
* **Detection center drifting under inconsistent lighting:** YOLO locates each bottle and uses the bounding box's center point as the pick target. Under uneven lighting or unexpected shadows, that center point can shift slightly from frame to frame, which occasionally causes the gripper to miss-grip. This isn't a mechanical or axis-calculation issue — it traces back to the detected bounding box itself shifting due to lighting, not a bug in the IK math.
---
 
## ⚠️ Limitations (Current)
 
* **Camera position is fixed post-calibration** — the setup can't tolerate camera movement without a full recalibration pass.
* **Lighting-sensitive detection** — poor lighting or harsh shadows can shift the detected center point enough to affect pick accuracy.
* **No object-orientation-aware gripper alignment** — the $\phi$-based $\theta_4$ formula is defined but not yet implemented; the gripper currently holds a fixed world-frame orientation only.
* **Static objects only** — the current pipeline assumes bottles are stationary when picked; no motion tracking yet.
---
 
## 🛣️ Future Roadmap
 
* **Moving object pick-and-place:** extend the system to work with objects moving on a conveyor belt, with the arm tracking and picking objects in motion rather than only static ones.
* **Object-orientation-aware gripper alignment:** implement the $\phi$-based $\theta_4$ formula so the gripper can match a bottle's actual rotation instead of holding a fixed orientation.
* **Lighting-robust detection:** improve the vision pipeline's tolerance to lighting variation to reduce center-point drift.
