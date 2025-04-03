# Splined Voronoi Planner
## Overview

This folder contains all code for the work on a global path planner on voronoi diagrams with smoothing.
It contains submodules to relevant packages and should be contained in an ROS noetic catkin workspace for building and using the packages.
It also contains packages for evaluating and generating examples.

**Author:** Sönke Prophet

**E-Mail:** soenke.prophet@stud.uni-hannover.de

The data of the evaluation is stored on an USB-drive because of large filesize.
To reproduce the results please copy the files to the data folder.
Also copy relevant maps to splined_voronoi_analysis/maps.

## Installation

For usage with move base flex on ROS noetic with catkin workspace:

```bash
cd /path/to/workspace/src/match_student_code
git submodule update --init --recursive
cd student_code/002_SplinedVoronoiPlanner/match_path_planning/
./setup.sh
cd /path/to/workspace/
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Contained Packages

## match_path_planning/splined_voronoi

Global Path planning and smoothing based on voronoi diagrams. Code is already contained in match_path_planning and therefore only contained as git submodule.

For installation and overview of functions and parameters see [README](match_path_planning/splined_voronoi/splined_voronoi/README.md).

## pure_voronoi_planner
Global path planning on voronoi diagrams without smoothing. Used for comparison between dijkstra and astar and planning with and without additional freespaces.

For more Information see [README](pure_voronoi_planner/README.md).

## splined_voronoi_analysis
A package which contains scripts for generating evaluation data and creating plots.
Also contains launchfiles for starting simulation and global planner.

For more Information see [README](splined_voronoi_analysis/README.md).

## splined_relaxed_astar
Reference Planner for comparison; Implementation taken from https://github.com/match-ROS/formation-path-planning. Code was reduced to contain only relevant parts and interface was added for automatic plan generation.

## data
Contains bagfiles used to generate results. 
The data is stored separately due to large filesize.
For validating results copy files to this folder and run scripts.

- compare_planner: contains bagfiles to compare splined_voronoi planner with splined_relaxed_astar planner
- test_planner: contains bagfiles to analyze planning success of smoothed planner
- voronoi: contains bagfiles to compare voronoi generation by different algorithms and difference between dijkstra and astar planning
- plots: where generated plots from analyzation will be saved
