# Multi Robot Planner

This multi robot planner plans the global paths of multiple cabled robots.

## Launching the multi robot planner

Standard launch file is in [multi_robot_planner/launch](../multi_robot_planner/launch/global_planner_robots.launch)

The standard launchfile launches six mir 600 in the world "square house", the start goal publisher node and the global planner node.

It can be customized to launch different quantities of robots in different worlds with different start and goal positions.
The launch files for the robots and worlds are contained in [mrp_gazebo/launch].
The start goal publisher nodes are contained in [multi_robot_planner/src].
