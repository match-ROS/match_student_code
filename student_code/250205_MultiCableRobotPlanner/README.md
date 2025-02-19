# Multi Cable Robot Planner
## Overview

This folder contains all code for the work on a global path planner for a cabled multi robot system.
It contains three packages and should be contained in an ROS noetic catkin workspace for building and using the packages.

**Author:** Henrik Depke

**E-Mail:** henrik.depke@stud.uni-hannover.de

## Installation

For usage with move base flex on ROS noetic with catkin workspace:

```bash
cd /path/to/workspace/src/match_student_code
git submodule update --init --recursive
cd /path/to/workspace/
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Contained Packages

## cable_layer

Costmap layer plugin that marks the cable positions behind the robots on an additional costmap layer.

## mrp_gazebo

This package contains the world, map and launch files for the simulation of the multi robot planner in gazebo.

## multi_robot_planner

Global planner for a cabled multi robot system.
