# 7DOF Kinova Arm Pick and Place using Phone Camera + YOLOv8 + Homography

> This implementation is currently **Windows-only**.  
> A ROS Noetic version will be uploaded separately.  
> Make sure the **Kinova Gen2.0 Runtime SDK (Windows)** is installed.

## Overview
This project implements a **7-DOF Kinova Gen2 robotic arm pick-and-place system** using:

- Phone camera for vision input  
- YOLOv8 for bottle detection  
- Homography for pixel → real-world coordinate mapping  
- C++ control using Kinova Runtime SDK  
- Python GUI for orchestration
  


### System Workflow

1. Phone camera captures video feed  
2. YOLOv8 detects the bottle  
3. Homography converts pixel coordinates → real-world coordinates  
4. Coordinates are saved to file  
5. C++ reads target position  
6. Kinova arm executes pick-and-place  



## Project Setup Workflow

1. Install Kinova Runtime SDK
2. Update SDK path in CMakeLists.txt
3. Build C++ (cmake steps)
4. Create & activate Python environment
5. Install Python requirements
6. Run GUI



### Project Structure
```
kinova7dof-pick-place-yolov8/
│
├── CMakeLists.txt
├── README.md
├── GUI_kinova7d_pick_place_yolov8.py
│
├── include/
│   ├── kinova_ik.h
│   └── move_to_point_angular_input.h
│
├── src/
│   ├── main.cpp
│   ├── kinova_ik.cpp
│   └── move_to_point_angular_input.cpp
│
├── bottle_tracking_and_saving/
│   ├── homography_main.py
│   ├── homography_tracking_module.py
│   ├── calibration_homography_pixel_to_real_world_mapping.py
│   └── Requirements.txt
│
├── models/
│   └── yolov8n.pt
│
└── data/
    ├── homography_calibration_pixel_to_real.txt
    ├── homography_calibration_points.txt
    └── homography_detected_bottle_centers.txt
```


## Requirements

### 1. Robot Runtime Requirement

To operate the Kinova arm, you must first install:

**Kinova Gen2.0 Runtime SDK (Windows)**

After installation, locate the SDK directory (example):
```
C:\Program Files (x86)\JACO-SDK\API
```

Then update the SDK path inside `CMakeLists.txt`:

```cmake
set(JACO_SDK_DIR "C:/Program Files (x86)/JACO-SDK/API")
```

### 2. Python Environment Setup

Create a virtual environment:
```bash
python -m venv ml_env
ml_env\Scripts\activate
```
Install required packages:
```bash
pip install -r bottle_tracking_and_saving/Requirements.txt
```
> The environment must be activated before running GUI_kinova7d_pick_place_yolov8.py



### 3. Build the C++ Application

Update the SDK path in `CMakeLists.txt` (if not done already):

```cmake
set(JACO_SDK_DIR "C:/Program Files (x86)/JACO-SDK/API")
```
From the project root directory, run:
```cmake
mkdir build
cd build
cmake ..
cmake --build . --config Debug
```



## Upcoming Version (ROS Noetic)

A ROS Noetic–based version of this project is under development.

The upcoming version will:

- Use **ROS Noetic (Ubuntu)** instead of the Windows Runtime SDK  
- Replace the phone camera with an **RGB-D camera** for object detection  
- Use depth information for improved spatial accuracy  
- Replace file-based communication with ROS topics and services  
- Integrate motion control using ROS-compatible Kinova drivers  
- Provide a modular ROS-based architecture for research and scalability  

This version will remove Windows-specific dependencies and enable a fully Linux-based robotics workflow.

Details and repository link will be added once available.
