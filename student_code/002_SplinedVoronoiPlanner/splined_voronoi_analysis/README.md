# splined voronoi scripts

This is a package containing scripts useful for analyzing the splined voronoi planner.

It is used for:
- launching the simulation and the global path planner
- generating random plans
- feeding plans in planner and save results in bagfiles
- generating example plots
- read out stats from bagfiles and perform analyzation
- plotting of data

First create a venv inside of package for installing requirements:
```bash
cd splined_voronoi_analysis
python3 -m venv venv
source venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

For generating results run scripts from 002_SplinedVoronoiPlanner folder.


# launch
Has launchfiles to start all nodes neccessary for path planning.

launching gazebo environment with turtlebot3 in it:
- needs argument for used map as path to a folder which contains a world.world and map.yaml without ending slash
```bash
roslaunch splined_voronoi_analysis turtlebot_gazebo.launch path_to_map_folder:=/path/to/maps/folder
```

launching move base flex with splined_voronoi as global planner:
```bash
roslaunch splined_voronoi_analysis splined_voronoi.launch
```

launching move base flex with pure_voronoi_planner as global planner:
```bash
roslaunch splined_voronoi_analysis pure_voronoi_planner.launch
```

launching move base flex with splined_relaxed_a_star as global planner:
```bash
roslaunch splined_voronoi_analysis splined_relaxed_a_star.launch
```

# scripts

## bagfile_examples.py
creates plot for example splining and optimizing process
- sparsing_example_from_bag: creates plot for exaple sparse path
- splining_exmple_from_bag: creates plot for example splined and optimized path

## bagfile_helpers.py
contains helper function and class for loading data from bagfiles
- get_contents_from_bagfile: assumes single data per topic and saved them in dictionary
- PlanningBagContents, MakeNavPlanBagContents, MakeNavPlanWithStatsBagContents: classes for loading and accessing information saved in bagfiles from different service interfaces.

## compare_algorithms.py
prints quota and time for each optimization algorithm

## compare_sra_and_voronoi.py

- calculate and plot distances to obstacles for every successful path
- compare achieved curvature and path length
- print time and success quota

## compare_voronoi.py

- overlay_voronoi_results: combines output from Sweepline and dynamicvoronoi in single image
- plot_voronoi_times: loads resulting times from txt and plots them
- compare_dijkstra_and_astar: analyse planning time for dijkstar vs astar with and without freespaces from bagfiles
- compare_freespace_inclusion: loads data from bagfiles with and without freespace planning and saves them to numpy format
- create_freespace_boxplot: loads data from numpy format and plots path lengths and distances to obstacles with and without freespaces

## move_base_simple_relay.py
feeds goal point from rviz in move base flex; taken from https://github.com/uos/mbf_tutorials

## path_helpers.py
contains functions which are useful for path handling
- points_from_path: type conversion from ROS-Message type to numpy array
- calc_angles: calculates orientation of each point in path from direction to next point
- calc_derivative: calculates numerical derivative of discrete samples
- offset_path: for a path with angles apply a constant offset to get single robot location
- world_to_map: transform 2d points from metric world frame to pixel based map frame
- PathAnalyzer: class for handling a single path with functions for calculating path length and distance to obstacles

## plan_caller:
scripts to automatically call path planning and saving results in bagfiles.
- single_path_planner: waits on topics /start_pose and /goal_pose and feeds them to planner
- generate_random_plans: waits on map and generates start goal pairs and saves them in single bagfile
- random_plan_caller: reads starts and goals from bagfile and feeds them to path planner and saves results in bagfiles
- specific_plan_caller: reads starts and goals from List.md and feeds them to splined_voronoi and splined_relaxed_astar for comparison and saves results in bagfiles

## single_formation_path.py
analyzes single path as example for planning success
- plots path of two robots of formation with 1m radius
- plot distance of formation center to obstacles to show collision avoidance
- plot orientation and anglular velocities of robots to show path is feasible with nonholonomic robots

## spline_helpers.py
contains functions which are useful for spline generation and analyzing which are used in various different scripts such as:
- calc_connection_points: calculates connection points from a given set of waypoints
- calc_control_points_for_path: calculates control points for quintic bézier spline from waypoints and length of the tangents
- calc_control_points_for_path_sra: calculates control points for quintic bézier spline by method used in splined relaxed a star. This uses waypoint positions as connection points.
- get_coefficient_matrices: creates bernstein polynomials for each bézier curve in bézier spline
- get_sample: get value of an bézier curve based on the bernstein polynomials and a fraction (position on curve between 0 and 1)
- sample_spline_path: get samples from control points of bézier spline
- SplineAnalyzer: class which gets waypoints, tangent lengths and map from bagfile and calculates info an spline and distance to obstacles 

## splining_example.py
creates plots of an example quintic bézier-spline
- path_smooth_old: splined relaxed astar version of quintic spline generation from waypoints and consequences of tangent length variation
- path_smooth: my generation of connection points between waypoints and corresponding tangents
- path_smooth_vars: effect of tangent length on bézier spline
- path_smooth_opt: optimal choice of tangent length for good aproximation of initial path

also creates plot for single quintic bézier curve
- single_quintic_bezier_curve: bézier curve with control points and tangents for visualization
