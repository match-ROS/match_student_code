# Messungen

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)

## About <a name = "about"></a>

Package with scripts for measurements done for the master's thesis.

## Getting Started <a name = "getting_started"></a>

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

ROS Noetic has to be installed on the system.\\
The following python packages are needed:
- _numpy_
- _scipy_
- _pandas_
- _matplotlib_

### Installing

Add the package into a catkin workspace.

## Usage <a name = "usage"></a>

The following scripts are included:
### measure_manipulibility.py
Publishes the manipulability of the robot arm to topic "/manipulibility"
### measurements_eef.py
- publishes important transformations
- to record the most important topics use:
```
rosbag record /ur_to_eef_in_map /ur_base_to_map /mur/velocity_command /map_to_eef /ur_base_to_map_in_base /ur_to_eef /mur/ur/joint_states
```
### move_for_distance.py
- input: /velocity_measurement
- output: /mur/velocity_command

publishes velocity until defined distance is reached. Can be included in own code or run as main.
If excecuted by main: distance measured from frame "map" to frame "mur/ur/wrist_3_link"

### move_and_measure.py
uses a combination of [move_for_distance](#move_for_distancepy) and [measurements_eef](#measurements_eefpy) to move the endeffector 1,1 m.

### fago_scanner.py
library to import csv data from fago scanner.
Transformation between two coordinates can be calculated by measuring a minimum of two Axis of the target frame (map) in the coorinates of the source frame (fago-frame).