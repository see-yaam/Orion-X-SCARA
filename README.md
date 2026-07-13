# 4-DOF SCARA Robotic Arm with Dual mode Object Sorting

An advanced, open-architecture **4-DOF SCARA (Selective Compliance Assembly Robot Arm)** designed for automated industrial pick-and-place sorting operations. This system integrates real-time Object Detection using **YOLOv11** with custom analytical **Inverse Kinematics (IK)** to map coordinates dynamically from a 2D camera frame to physical robotic joint angles.

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

### 7. Run the System
With the Arduino connected and calibration done, launch the main control interface:
```bash
python scara_final.py
```