# Formation Builder
## Overview

This package contains all the scripts and files required to use the stabilizing algortihm. The algorithm controlls the endeffektor of an UR20 on the MIR600. It's functionality is described in the project thesis "Auslegung eines Echtzeit-Erfassungssystems von Bodenunebenheiten mit synchronisierter Kompensation des Endeffektors für mobile Roboter" which was written at the "Institut für Montagetechnik und Industrierobotik" (match) at the Leibniz Universität Hannover.

**Author:** Maximilian Lippold
**E-Mail:** Maximilian.Lippold@stud.uni-hannover.de


## Installation

This project requires ROS noetic and python 3.8 or newer. 

The match_mobile_robotics repository needs to be installed


*** LAUNCH ***
```sh
roslaunch imu launch_mur.launch
roslaunch imu twist_sim.launch
rosrun imu switch_URs_to_twist_control.py
rosrun imu kalman_getrackt.py
rosrun steuerung regelung_neu.py
```

now the right endeffektor is controlled by this controller.


## Additional Informations:
In the current version, the endeffektor has to be specified before. You can switch to the left or right endeffektor of the MIR600. 