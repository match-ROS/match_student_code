# IMU-Based Nozzle Stabilization for Additive Manufacturing on Uneven Terrain

## Overview

This folder contains all the ROS packages and scripts used in the student thesis project. The project investigates real-time stabilization of end effector of UR10 robotic arms mounted on MiR600 platform using IMU inclination data (roll and pitch). The algorithm is developed in Python and implemented within the ROS framework. The entire system is tested in the Gazebo simulation environment using a suspended MiR600 platform. Simulated disturbances are introduced through applied torques to evaluate the performance of the stabilization algorithm.


The system was developed as part of the Bachelor's thesis:

**“Real-Time Compensation of Ground Irregularities for Mobile Robot in Construction Additive Manufacturing”**  
at **Leibniz Universität Hannover**

The following ROS packages are included:

- `imu_compensation`: IMU-based Python algorithm that stabilizes the end effector in real-time.
- `pose_change`: Node to set the robot’s pose before operation.
- `twist_controller`: Launch file to switch the robot’s control mode to Twist (velocity) control.
- `test`: Script that applies simulated torque to induce disturbances for testing.

**Author**: Pushkar Singh  
**E-Mail**: pushkar.singh@stud.uni-hannover.de

---
This project requires ROS Noetic and Python 3.8 or newer. The match_mobile_robotics repository must be installed for the simulation environment to work properly.

## Installation

1. Clone or download this folder.

2. Copy the ROS packages into your catkin workspace:
```bash
cp -r imu_compensation pose_change twist_controller test ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
---
```markdown


## Simulation Launch Sequence
```bash
1. roslaunch match_gazebo big_square.launch
2. roslaunch mur_launch_sim mir600.launch
3. roslaunch pose_change move_arm.launch
4. roslaunch twist_controller twist_change.launch
5. rosrun imu_compensation real_time_compensation.py
6. rosrun test test.py
---
Now the left end effector is controlled by the controller.



 

