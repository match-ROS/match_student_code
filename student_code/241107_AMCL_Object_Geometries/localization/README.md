# AMCL Object Geometries
## Overview

This folder contains all the code used for the bachelor thesis. The implementation was with ROS Noetic and Python 3.

**Author:** Rafael Soto 

**E-Mail:** rafael.soto.salas@stud.uni-hannover.de

## Packages
### localization 
For this package to work, the files from the match_mobile_robotics repository are required.

## Scripts
The scripts used for the simulations in Gazebo are in the folder mur620_gazebo and the scripts used for experiments with the real robot are in the folder mur620d. Scripts that are not found outside these folders can be used for both robots.
### send_mur620_to_pose.py 
Send the robot in a Gazebo world to a specific pose.

### insert_box.py 
Inserts a box in the Gazebo world. The dimensions width, depth and height are taken from the launch file load_object.launch

### object_OccupancyGrid.py 
Inserts the shape of a box into the Occupancy Grid in RVIZ. The dimensions width and depth are taken from the launch file load_object.launch

### collect_data.py 
Send the robot to 5 poses 30 times, when the robot determines that it has reached the target position it saves the amcl pose and ground truth values into a file.ods. The script starts recording a rosbag at the beginning of each cycle and saves in the database in which second from the beginning of the recording the robot reached the desired poses.

### calculate_accuracy.py 
This script extracts the data from the .ods file and saves it in a data frame to later calculate precision, positioning error, orientation error, standard deviation etc. It also calculates the statistics of the individual poses where the robot was sent. The variable d determines whether to calculate the accuracy of the experiments or the simulations. The variable file_path is the path to the .ods file, in that same file the 3 generated tables 'Accuracy', 'Positions Stadistics' and 'Orientation Stadistics' will be saved.

## Launch Files
Only for Gazebo simulations.

### todo_amcl.launch
Start the simulation of the gazebo world with the robot inside and initialize the amcl location method.

### load_object.launch
Allows inserting objects into the gazebo world by specifying width, depth and height.
If the map_and_gazebo parameter is equal to 1, the object geometry is inserted into the ROS Occupancy Grid.



