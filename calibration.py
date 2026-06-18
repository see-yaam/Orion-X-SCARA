"""
Camera-to-robot homography calibration tool for a SCARA pick-and-place system.

Workflow:
1. Click calibration points on the live overhead camera image.
2. Type the matching real robot-surface coordinates in milimeters.
3. Save calibration. The script writes calibration_matrix.npy beside this file.

The saved 3x3 homography maps:
    pixel coordinate (x_pixel, y_pixel) -> real coordinate (X_mm, Y_mm)
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox, ttk

try:
    from PIL import Image, ImageTk
except ImportError as exc:
    raise RuntimeError("Pillow is required. Install it with: pip install Pillow") from exc


CAMERA_INDEX = 0
CALIBRATION_FILE = Path(__file__).with_name("calibration_matrix.npy")


class CalibrationPoint:
    """Stores one clicked pixel point and its matching real-world entry fields."""

    def __init__(self, pixel_x: float, pixel_y: float, x_entry: tk.Entry, y_entry: tk.Entry):
        self.pixel_x = pixel_x
        self.pixel_y = pixel_y
        self.x_entry = x_entry
        self.y_entry = y_entry

    def real_coordinate(self) -> Tuple[float, float]:
        return float(self.x_entry.get()), float(self.y_entry.get())


class CalibrationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SCARA Camera Calibration")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}.")

        self.points: List[CalibrationPoint] = []
        self.current_frame: np.ndarray | None = None
        self.tk_image: ImageTk.PhotoImage | None = None

        # Display geometry used to convert canvas clicks back to real frame pixels.
        self.display_x = 0
        self.display_y = 0
        self.display_w = 1
        self.display_h = 1
        self.display_scale = 1.0

        self.build_layout()
        self.update_frame()

    def build_layout(self) -> None:
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)

        self.video_canvas = tk.Canvas(self.root, bg="black", highlightthickness=0)
        self.video_canvas.grid(row=0, column=0, sticky="nsew")
        self.video_canvas.bind("<Button-1>", self.on_canvas_click)

        sidebar = ttk.Frame(self.root, width=360)
        sidebar.grid(row=0, column=1, sticky="ns")
        sidebar.rowconfigure(1, weight=1)

        title = ttk.Label(sidebar, text="Calibration Points", font=("Arial", 14, "bold"))
        title.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 4))

        help_text = (
            "Click a point on the camera feed, then enter its real "
            "robot coordinate in centimeters relative to base origin (0,0)."
        )
        ttk.Label(sidebar, text=help_text, wraplength=320).grid(row=1, column=0, sticky="new", padx=10)

        self.scroll_canvas = tk.Canvas(sidebar, width=350, highlightthickness=0)
        self.scroll_canvas.grid(row=2, column=0, sticky="nsew", padx=(10, 0), pady=8)

        scrollbar = ttk.Scrollbar(sidebar, orient="vertical", command=self.scroll_canvas.yview)
        scrollbar.grid(row=2, column=1, sticky="ns", pady=8)
        self.scroll_canvas.configure(yscrollcommand=scrollbar.set)

        self.points_frame = ttk.Frame(self.scroll_canvas)
        self.scroll_canvas_window = self.scroll_canvas.create_window((0, 0), window=self.points_frame, anchor="nw")
        self.points_frame.bind("<Configure>", self.on_points_frame_configure)
        self.scroll_canvas.bind("<Configure>", self.on_scroll_canvas_configure)

        self.save_button = ttk.Button(sidebar, text="Save Calibration", command=self.save_calibration)
        self.save_button.grid(row=3, column=0, columnspan=2, sticky="ew", padx=10, pady=(4, 10))

    def on_points_frame_configure(self, _event: tk.Event) -> None:
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def on_scroll_canvas_configure(self, event: tk.Event) -> None:
        self.scroll_canvas.itemconfigure(self.scroll_canvas_window, width=event.width)

    def update_frame(self) -> None:
        ok, frame = self.cap.read()
        if ok:
            self.current_frame = frame
            self.render_frame(frame)

        self.root.after(15, self.update_frame)

    def render_frame(self, frame_bgr: np.ndarray) -> None:
        canvas_w = max(1, self.video_canvas.winfo_width())
        canvas_h = max(1, self.video_canvas.winfo_height())
        frame_h, frame_w = frame_bgr.shape[:2]

        self.display_scale = min(canvas_w / frame_w, canvas_h / frame_h)
        self.display_w = max(1, int(frame_w * self.display_scale))
        self.display_h = max(1, int(frame_h * self.display_scale))
        self.display_x = (canvas_w - self.display_w) // 2
        self.display_y = (canvas_h - self.display_h) // 2

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame_rgb).resize((self.display_w, self.display_h), Image.Resampling.LANCZOS)
        self.tk_image = ImageTk.PhotoImage(image=image)

        self.video_canvas.delete("all")
        self.video_canvas.create_image(self.display_x, self.display_y, anchor="nw", image=self.tk_image)
        self.draw_clicked_points()

    def draw_clicked_points(self) -> None:
        for index, point in enumerate(self.points, start=1):
            x = self.display_x + point.pixel_x * self.display_scale
            y = self.display_y + point.pixel_y * self.display_scale
            self.video_canvas.create_oval(x - 6, y - 6, x + 6, y + 6, fill="red", outline="white", width=2)
            self.video_canvas.create_text(
                x + 10,
                y - 10,
                text=f"Point {index}",
                fill="red",
                font=("Arial", 11, "bold"),
                anchor="w",
            )

    def on_canvas_click(self, event: tk.Event) -> None:
        if self.current_frame is None:
            return

        if not (self.display_x <= event.x <= self.display_x + self.display_w):
            return
        if not (self.display_y <= event.y <= self.display_y + self.display_h):
            return

        pixel_x = (event.x - self.display_x) / self.display_scale
        pixel_y = (event.y - self.display_y) / self.display_scale
        self.add_point_row(pixel_x, pixel_y)

    def add_point_row(self, pixel_x: float, pixel_y: float) -> None:
        index = len(self.points) + 1
        row = index - 1

        point_frame = ttk.LabelFrame(self.points_frame, text=f"Point {index}")
        point_frame.grid(row=row, column=0, sticky="ew", padx=4, pady=5)
        point_frame.columnconfigure(1, weight=1)

        pixel_label = ttk.Label(point_frame, text=f"Pixel: {pixel_x:.1f}, {pixel_y:.1f}")
        pixel_label.grid(row=0, column=0, columnspan=4, sticky="w", padx=6, pady=(4, 2))

        ttk.Label(point_frame, text="Real X (cm)").grid(row=1, column=0, padx=6, pady=4)
        x_entry = ttk.Entry(point_frame, width=10)
        x_entry.grid(row=1, column=1, padx=4, pady=4)

        ttk.Label(point_frame, text="Real Y (cm)").grid(row=1, column=2, padx=6, pady=4)
        y_entry = ttk.Entry(point_frame, width=10)
        y_entry.grid(row=1, column=3, padx=4, pady=4)

        self.points.append(CalibrationPoint(pixel_x, pixel_y, x_entry, y_entry))
        self.scroll_canvas.yview_moveto(1.0)

    def save_calibration(self) -> None:
        if len(self.points) < 4:
            messagebox.showerror("Not Enough Points", "Please add at least 4 calibration points.")
            return

        pixel_points = []
        real_points = []

        try:
            for point in self.points:
                pixel_points.append([point.pixel_x, point.pixel_y])
                real_x, real_y = point.real_coordinate()
                real_points.append([real_x, real_y])
        except ValueError:
            messagebox.showerror("Invalid Input", "Every real-world X/Y field must contain a valid number.")
            return

        pts_pixel = np.asarray(pixel_points, dtype=np.float32)
        pts_real_cm = np.asarray(real_points, dtype=np.float32)

        homography, mask = cv2.findHomography(pts_pixel, pts_real_cm, method=0)
        if homography is None:
            messagebox.showerror("Calibration Failed", "OpenCV could not compute a homography from these points.")
            return

        np.save(CALIBRATION_FILE, homography)

        projected = cv2.perspectiveTransform(pts_pixel.reshape(-1, 1, 2), homography).reshape(-1, 2)
        errors = np.linalg.norm(projected - pts_real_cm, axis=1)
        mean_error = float(np.mean(errors))

        print("Calibration saved successfully.")
        print(f"File: {CALIBRATION_FILE}")
        print("Homography matrix:")
        print(homography)
        print(f"Mean reprojection error: {mean_error:.4f} cm")
        if mask is not None:
            print(f"Points used by solver: {int(mask.sum())}/{len(mask)}")

        messagebox.showinfo("Calibration Saved", f"Saved calibration_matrix.npy\nMean error: {mean_error:.3f} cm")
        self.close()

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    root.geometry("1200x720")
    try:
        CalibrationApp(root)
    except Exception as exc:
        messagebox.showerror("Calibration Error", str(exc))
        print(f"Calibration error: {exc}", file=sys.stderr)
        return 1

    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
