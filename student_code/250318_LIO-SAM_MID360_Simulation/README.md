## Overview
This folder contains all the codes and function packages used in the student thesis project. This project studies the application of LIO-SAM algorithm on Livox-Mid-360 radar and Mir600 mobile robot platform. The experiment is carried out in the simulation software Gazebo.
Using the evo tool library, the positioning accuracy of LIO-SAM and AMCL algorithms is compared.

**Author:** Xinwei Fu

**E-Mail:** xinwei.fu@stud.uni-hannover.de

## Packages
### LIO-SAM-MID360
Implements the LIO-SAM algorithm specifically adapted for the Livox MID360 LiDAR.
### livox_laser_simulation
Official Livox package for simulating Livox LiDAR sensors in ROS.
### livox_ros_driver
Official Livox ROS driver for connecting and interfacing Livox LiDAR sensors.
### match_mobile_robotics
Containing simulation files of robot platform and map.

## Installation
Create a catkin_ws/src folder in your home directory and move all files from the current page into the src folder.

Run the **setup.sh** script to install and build the packages.
The script will do the following steps automatically:

### 1. Build match_mobile_robotics

Temporarily move LIO-SAM-MID360, livox_laser_simulation, and livox_ros_driver out of the current workspace.
Use the setup_full.sh script in the match_mobile_robotics folder to install all dependencies and build the package.

### 2. Build the entire workspace

Move LIO-SAM-MID360, livox_laser_simulation, and livox_ros_driver back into the src directory of your workspace.
Install the necessary dependencies and build everything.

## Usage
### After compilation, launch the following files:

<!-- **Start the robot simulation**
```bash
roslaunch mir_examples single_mir_600.launch
```
**Start AMCL localization**
```bash
roslaunch mir_examples amcl.launch
``` -->
**Start LIO-SAM localization with mir600 robot**
```bash
roslaunch lio_sam run6axis.launch
```
**Start the RQT tool to manually control the robot**
```bash
rosrun rqt_robot_steering rqt_robot_steering
```

