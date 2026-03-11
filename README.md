# Camera-Based Line Following Robot

A ROS2-based autonomous robot that performs **line following, obstacle detection, and visual image recognition** using a camera and a machine learning classifier.

The system processes live camera input, detects a guiding line on the ground, identifies images, and switches between operational modes dynamically.

---

# Overview

This project implements a **line-based navigation system** for a mobile robot using **ROS2 (python scripts) and OpenCV**.

The robot:

* Follow a black line on a white surface
* Detect obstacles in its path
* Recognize images using a trained neural network
* Switch between different behavioral modes based on the environment

The project is structured as a **ROS2 package** where different robot capabilities are implemented as independent nodes.

---

# Features

* IR-based **line following**
* **Obstacle detection**
* **Machine learning classifier** for image recognition
* **Automatic camera mode switching**
* Modular **ROS2 node architecture**
* Configurable parameters via YAML
* ONNX model support for efficient inference

---

# System Architecture

The robot operates using multiple ROS2 nodes:

### Camera Stream

Continuously captures frames from the robot camera and publishes them to other nodes.

### Line Follower

Processes IR data to detect the line and calculates steering commands.

### Image Scanner

Detects images placed on A4 sheets and performs classification using a YOLO neural network.

### Obstacle Detector

Monitors ultrasonic sensor for obstacles and prevents collisions.

### Camera Switcher

Servo based control of camera.

### Streamer Mode

Handles visual debugging or streaming of camera feed.

---

# Project Structure

```
robot_control/
│
├── config/
│   └── params.yaml
│       Robot parameters and tuning values
│
├── launch/
│   └── robot_control.launch.py
│       Launch file to start the entire system
│
├── models/
│   └── classifier.onnx
│       Neural network used for marker classification
│
├── resource/
│   └── robot_control
│       ROS2 package resource file
│
├── robot_control/
│   ├── __init__.py
│   ├── camera_switcher.py
│   ├── image_scanner_two.py
│   ├── line_follower.py
│   ├── obstacle_detector.py
│   ├── streamer_mode.py
│   └── unbiased_best.pt
│
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
│
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

# Node Descriptions

## line_follower.py

Implements the main **line tracking algorithm**.

Functions include:

* IR array input
* Line detection
* Error calculation
* Steering command generation

---

## image_scanner_two.py

Detects special **images printed on an A4 sheet** and performs classification using a trained model.

Steps include:

1. Detect A4 sheet in camera frame
2. Extract region of interest
3. Run ML inference
4. Publish classification result

---

## obstacle_detector.py

Detects obstacles appearing in the robot’s path and triggers avoidance behavior.

---

## camera_switcher.py

Manages which direction the camera is looking through servo control.

* Normal operation → Line follower
* Image detected → Image scanner
* Obstacle detected → Obstacle avoidance

---

## streamer_mode.py

Provides a debug or monitoring mode for viewing the camera stream and detection outputs.

---

# Machine Learning Model

The project uses a trained YOLO neural network for image classification.

Supported formats:

* `.onnx` (preferred for deployment)
* `.pt` (PyTorch training format)

Model location:

```
models/classifier.onnx
```

---

# Configuration

Robot parameters are stored in:

```
config/params.yaml
```

Example parameters:

* Camera resolution
* Threshold values
* Detection sensitivity
* Control tuning constants

---

# Requirements

### Software

* ROS2 (Humble / Iron recommended)
* Python 3.10+
* OpenCV
* NumPy
* PyTorch
* ONNX Runtime

Install dependencies:

```bash
pip install opencv-python numpy torch onnxruntime
```

---

# Building the Package

Navigate to your ROS2 workspace:

```bash
cd ~/ros2_ws
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

# Running the Robot

Launch the system using:

```bash
ros2 launch robot_control robot_control.launch.py
```

This will start:

* Camera streaming
* Line follower
* Obstacle detector
* Image scanner
* Mode switching

---



