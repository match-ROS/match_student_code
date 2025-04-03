# Redundacy Resolution for mobile Manipulator
## Overview
This folder contains all the code used in the master's thesis. The ROS Package is implemented in ROS Noetic. Python3 is used. 

**Author:** Hauke Heeren

**E-Mail:** hauke.heeren@stud.uni-hannover.de

## Packages
### optimization_algo 
Uses an Optimization to calculate the velocity of the MiR. UR velocity is cartesian velocity command minus the MiR velocity.
For more details see [Readme](optimization_algo/README.md).

#### Scripts
##### optimize.py 
- input: cartesian velocity command
- output: UR velocity
- output: MiR velocity

Can either be used as a class to be implemented in own script, or otherwise by just running the main.
Needs a running controller for the Robot arm listening to topic at `cooperative_manipulation/cartesian_velocity_command`.

##### control/ur_admittance_control_hw.py
- input: cartesian velocity command via topic `cooperative_manipulation/cartesian_velocity_command`
- output: joint_velocities: `/mur/ur/joint_group_vel_controller/command`
parallel force_pos controller for the robot arm using _mur.lauch_ of _match_mobile_robotics_

### match_mobile_robotics
For more details see [Readme](match_mobile_robotics/README.md).

see also: [github](https://github.com/pumablattlaus/match_mobile_robotics/tree/working_with_optimization)

### messungen
Package to take the measurements used in the master's thesis.

For more details see [Readme](messungen/README.md).
