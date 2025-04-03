# Creating Bézier-Splines on the basis of path planners
## Overview
This folder contains all the code used in this work. The ROS Package is implemented in ROS Noetic on Ubuntu 20.04. Python3 and C++ is used. 

**Author:** Henrik Wonnemann

**E-Mail:** henrik.wonnemann@stud.uni-hannover.de

## Packages
### bezier_path 
Contains the whole project

## Launch files
### test.launch
Starts everything including the algorithm. Launching this file will generate a spline.

### other launch files in launch directory of bezier_path
slightly modified launch files from match_mobile_robotics according to currently used path planner and start/end pose

## class
### bezier_splines
base-, cubic- and quintic-bezier-spline provide functions to calculate bezier-splines. 
Code was copied from "formation-path-planning/fp_utils/src" and modified according to the algorithm. Folders geometry_info and visualization_helper need to be found but are not used.

## Source and header files
### main
Started by test.launch. Callback methods and calls functions from other source files to calculate waypoints and Bézier-splines.
To change the operating mode of the algorithm, change the variable "size_t detail" in the function findBestPath(). Explanation of the modes is commented. Other parameters that influence the spline can also be changed in this function.

### waypoints
Functions involved in calculation of waypoints. Functions called by main.cpp and itself.

### bezierpath
Functions involved in calculation of the parameters of the spline segments. Functions called by main.cpp and itself.

### rviz
Visualization functions for RVIZ

### utils
'other' functions
There has been the problem that all used path planners planned part of the paths through the very outer area of the costmap.
A function has been implemented to reduce the costmap after the path has already been planned as far as necessary for the path to be free of the costmap. That function is called in main.cpp in the mapCallback.

### matlab
Functions that were used to collect and export data to .csv

## Scripts
### activate_vscode.py 
VSCode always minimized when the program starts so this maximizes it again. Obviously not essential. Called by test.launch

### set_pose.py
Sets start and end pose, can be changed here, but also needs to be changed in mir_100_changed and general_mir_changed launch. Called by test.launch.

## topdown.rviz
Saved rviz configuration.

## RaStar
Free to use Relaxed A-Star path planner downloaded from github. Used to test another planner besides navfnRos.

## Dependencies
### match_gazebo
From match_mobile_robotics
Due to the issue described in 'utils' the costmap has to be received multiple times. In order for the callback method to work, 'always_use_full_costmap = true' has to be added in mir/mir_navigation/config/Costmap/costmap_common_params.yaml

### mir_launch_sim
From match_mobile_robotics/mir

### mir_navigation
From match_mobile_robotics/mir