# formation_launch
## Overview
This package contains the launch files of different formations in different worlds.

**Author:** Rayen Hajji

**E-Mail:** rayen.hajji@stud.uni-hannover.de

## Usage
To run the launch files the following command:

* Two robots formation: `roslaunch formation_launch formation_two_parallel_robots.launch`
* Three robots formation: `roslaunch formation_launch formation_three_asym_triangular_robots.launch`
* Three robots asymmetric formation: `roslaunch formation_launch formation_three_triangular_robots.launch`
* Four robots formation: `roslaunch formation_launch formation_four_rectangular_robots.launch`

## Config files 
Folder : **config**

There are three configuration files for each formation: 
- `formation_costmap_params.yaml`: this file contains the robot configurations in the formation.
- `global_planner_params.yaml`: this file contains the global planner parameter of the robots. 
- `local_planner_params.yaml`: this file contains the local planner of the robots.

## Launch files
Folder : **launch**

There are four different formation launch files. Each launch file gives the possibility to launch the formation in three different maps  (warehouse/wide_maze/hallway). For each map, changes in the global planner parameter are required (uncomment the corresponding 'offset' parameter). An example of the launch file:  
- `formation_two_parallel_robots.launch`: launch file of two parallel robots fomration.

	Argument list:
	- `robots_number` (type: `[int]`, value: `2`): this parameter indicates the number of robots in the formation. It is necessary for the formation layer to work.
    - `robot_name` (type: `[str]`, value: `$(arg robot0_name)`): contains the name of robot0.
    - `robot_name` (type: `[str]`, value: `$(arg robot1_name)`): contains the name of robot1.

## Maps
Folder: **maps**

The folder contains the maps of the three environments(hallway(building)/warehouse/wide_maze). Each map consists of two files:
- `.pgm` : grayscale 2D image of the Map.
- `.yaml`: configuration file of the Map. 

## Models
Folder: **models**

This folder contains models that are used to build the simulation worlds.

## worlds
Folder: **worlds**

This folder contains the worlds used for the simulations. To be able to launch the simulation worlds, add the following line to .bashrc file:
`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/"Path to the package"/formation_launch/models`

## Rviz Configuration
Folder: **rviz**

This folder contains the rviz configurations of the four formations.

