# LIO-SAM-MID360
## Overview

This package implements indoor robot localization using the LIO-SAM algorithm with Livox MID360 LiDAR and the AMCL algorithm using two 2D LiDARs.  
It includes the necessary launch files for running the LIO-SAM algorithm.


**Author:** Xinwei Fu

**E-Mail:** xinwei.fu@stud.uni-hannover.de



## Usage

To launch the robot platform and map simulation, run the following command:

`roslaunch mir_examples single_mir_600.launch`

To start the LIO-SAM algorithm, run:

`roslaunch lio_sam run6axis.launch`

To run the AMCL algorithm, execute:

`roslaunch mir_examples amcl.launch`

To manually control the robot's motion using the rqt tool, use:

`rosrun rqt_robot_steering rqt_robot_steering`

## Config files
- `params.yaml`: The original configuration file of LIOSAM algorithm. Used for traditional mechanical radar.
- `paramsLivoxIMU.yaml`: Parameter configuration specifically used for Livox radar.

## Launch files
- `run6axis.launch`: The launch file to be run when using a 6-axis IMU (the simulation uses a 6-axis IMU)

- `run9axis.launch`: The launch file to be run when using a 9-axis IMU


## Nodes
### example_node
Description of example_ndoe

#### Subscribed Topics
- `example_sub` (std_msgs/Bool)

	Describe the data on this topic.

#### Published Topics
See `Subscribed Topics` for the structure of how to describe it.
