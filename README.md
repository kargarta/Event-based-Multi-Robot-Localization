# Multi-Robot Cooperative Localization

This repository contains a Python implementation for time-triggered multi-robot localization using advanced sensor fusion techniques. The goal of this project is to localize a group of robots by fusing sensor data through time-triggered control mechanisms, enhancing the precision and robustness of robot positioning in complex environments.

## Features
- Time-triggered control for multi-robot localization
- Sensor fusion for improved accuracy in localization
- Supports a variety of sensors such as GPS, LIDAR, and IMU
- Modular design for easy customization and extension
- Scalable to different multi-robot systems

## Requirements

Before running the code, ensure you have the following dependencies installed:

- Python 3.x
- NumPy
- SciPy
- Matplotlib
- ROS (if using robot operating systems)
- Any other sensor-specific libraries (e.g., `pyserial` for GPS, `rospy` for ROS, etc.)

You can install the required Python libraries using the following command:

```bash
pip install -r requirements.txt
