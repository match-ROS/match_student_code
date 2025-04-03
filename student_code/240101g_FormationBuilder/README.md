# Formation Builder
## Overview

This package contains all the scripts and files required to use the PMADMU-planner. The PMADMU-planner is a multi-agent path planning algorithm with which robots can be transferred into a predefined formation. It's functionality is described in the masters thesis "Entwicklung einer skalierbaren Multiagenten-Pfadplanung zur Formationsbildung für nicht-holonome mobile Roboter" which was written at the "Institut für Montagetechnik" (match) at the Leibniz Universität Hannover.

**Author:** Max Westermann
**E-Mail:** m.westermann@stud.uni-hannover.de


## Installation

This project requires ROS noetic and python 3.8 or newer. Add this project as a package to the catkin workspace. Also add/install the following:

Required Python Libraries:
- numpy https://numpy.org/install/
- open cv2 https://pypi.org/project/opencv-python/

Required Packages:
- cv_bridge https://index.ros.org/p/cv_bridge/
- tf2_sensor_msgs https://wiki.ros.org/tf2_sensor_msgs



## Usage SCALE

To use this project with the MIR600 Platforms at Scale use the following commands in that specific order. To use the project in a simulated environment use the commands listed under "Usage SIM".

*** LAUNCH ON ROSCORE: login via "ssh roscore" ***
roscore

*** LAUNCH ON ROBOTS: login via "ssh mur620x" (replace x with a, b, c, d) and paste the 2 commands for each robot x. ***
ROS_NAMESPACE=mir1 roslaunch mir_launch_hardware mir_600.launch tf_prefix:=mir1
roslaunch formation_builder launch_follower_single.launch robot_id:=1

ROS_NAMESPACE=mir2 roslaunch mir_launch_hardware mir_600.launch tf_prefix:=mir2
roslaunch formation_builder launch_follower_single.launch robot_id:=2

ROS_NAMESPACE=mir3 roslaunch mir_launch_hardware mir_600.launch tf_prefix:=mir3
roslaunch formation_builder launch_follower_single.launch robot_id:=3

ROS_NAMESPACE=mir4 roslaunch mir_launch_hardware mir_600.launch tf_prefix:=mir4
roslaunch formation_builder launch_follower_single.launch robot_id:=4

*** LAUNCH ON MAIN PC ***
roslaunch formation_builder launch_rviz_only.launch
roslaunch formation_builder launch_map_reader.launch
roslaunch formation_builder central_controller.launch



## Usage SIM

To use this project in an simulated GAZEBO-environment use the following commands in that specific order. To use the project at SCALE use the commands listed under "Usage SCALE".

roslaunch formation_builder launch_four_robots.launch
roslaunch formation_builder launch_map_reader.launch
roslaunch formation_builder launch_followers.launch
roslaunch formation_builder launch_central_controller.launch



## Additional Informations:
In the current version, target positions and priorities cannot be specified within the launch file nor by a ROS-Interface. To adjust the values, the position values must be adjusted at the end of the central_controller.py file.