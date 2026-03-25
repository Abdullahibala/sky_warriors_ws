PX4 ROS2 ArUco Precision Landing Simulation

This project implements a vision-based precision landing system for a simulated drone using PX4 Autopilot, ROS 2, and Gazebo.

The system enables a UAV to detect an ArUco marker using onboard vision and land autonomously on it.

Objective

To design and simulate a pipeline where a drone:

Captures images from a downward-facing camera
Detects a ArUco Marker using computer vision
Estimates the marker’s relative position
Adjusts its trajectory in real time
Performs autonomous precision landing

Gazebo Simulation (Drone + Camera)
        ↓
Camera Image (/image_raw)
        ↓
ArUco Detection (OpenCV)
        ↓
Marker Pose (/aruco_poses)
        ↓
ROS2 Control Node
        ↓
PX4 Offboard Control
        ↓
Autonomous Landing

Technologies Used
PX4 Autopilot — flight control and SITL simulation
ROS 2 (Jazzy) — middleware and communication
Gazebo (Harmonic) — physics-based simulation
OpenCV — ArUco marker detection
px4_ros_com — DDS interface
Micro XRCE-DDS Agent — communication layer

Current Status
✅ PX4 SITL + Gazebo simulation running
✅ ROS2 ↔ PX4 communication established
✅ Camera topic integration (/image_raw)
✅ ArUco detection pipeline configured
🔄 Final tuning of camera + landing controller in progress
