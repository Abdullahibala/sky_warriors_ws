PX4 ROS2 PRECISION LANDING SIMULATION

A vision-based precision landing system for a simulated drone using PX4 Autopilot, ROS 2, and Gazebo.

The system enables a UAV to detect an ArUco marker using onboard vision and land autonomously on it.


Objective
To design and simulate a pipeline where a drone:

1) Captures images from a downward-facing camera
2) Detects a ArUco Marker using computer vision
3) Estimates the marker’s relative position
4) Adjusts its trajectory in real time
5) Performs autonomous precision landing



Technologies Used

1) PX4 Autopilot — flight control and SITL simulation
2) ROS 2 (Jazzy) — middleware and communication
3) Gazebo (Harmonic) — physics-based simulation
4) OpenCV — ArUco marker detection
5) px4_ros_com — DDS interface
6) Micro XRCE-DDS Agent — communication layer

Current Status
1)  PX4 SITL + Gazebo simulation running
2)  ROS2 ↔ PX4 communication established
3)  Camera topic integration (/image_raw)
4)  ArUco detection pipeline configured
5)  Final tuning of camera + landing controller in progress
