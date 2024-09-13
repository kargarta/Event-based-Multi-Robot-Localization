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

Setup and Usage
Clone the repository to your local machine:

bash
Copy code
git clone https://github.com/yourusername/multi-robot-localization.git
Navigate to the project directory:

bash
Copy code
cd multi-robot-localization
To run the localization algorithm, execute the Python script:

bash
Copy code
python Time-Triggered_Multi-Robot_Localization.py
Customization
Sensors: The script can be easily adapted to work with a variety of sensors. Simply modify the sensor input handling section of the code to integrate your desired sensors.
Robots: To increase the number of robots or change the robots' configuration, update the robot parameters in the initialization section of the script.
Control Algorithm: You can modify the time-triggered control algorithm to incorporate event-based triggers or other control mechanisms depending on your application.
Files
Time-Triggered_Multi-Robot_Localization.py: Main script implementing the time-triggered localization algorithm.
Example
Below is an example of running the localization system:

bash
Copy code
python Time-Triggered_Multi-Robot_Localization.py
The system will simulate a set of robots and output the estimated positions based on the sensor data and control algorithm.

