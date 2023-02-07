# Pure voronoi Planner

for evaluation of voronoi generation and path planning on voronoi a pure planner is introduced.


Contains:
- pure planner with dynamic reconfiguration for freespace and dijkstra vs astar comparison
- dynamicvoronoi and sweepline voronoi generation for comparison
- dynamicvoronoi algorithm taken from http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
- comparison between dynamicvoronoi and sweepline for images with different resolutions

## Starting pure voronoi planner

Can be used as plugin of move_base_flex with name pure_voronoi_planner/PureVoronoiPlanner

Example Launchfile is in [splined_voronoi_analysis/launch](../splined_voronoi_analysis/launch/pure_voronoi_planner.launch)

Available Parameters (can be adapted in yaml or via dynamic reconfigure):
- free_cell_threshold: threshold for defining free space from costmap. Should be chosen so that a single robot is guaranteed without collision.
- min_radius: minimal radius that robot formation takes; costmap is enlargened accordingly
- use_dijkstra: if dijkstra should be used for path planning otherwise a* is used
- free_space_factor: factor for min_radius. At this distance to obstacles freespace is added to planning space regardless of voronoi diagram
- large_free_spaces: if additional free spaces should be added; otherwise pure voronoi diagram is used for planning


## Starting comparison of voronoi generation

```bash
rosrun pure_voronoi_planner compare_voronoi_results path/to/map.pgm
```

Feeds map in dynamicvoronoi and sweepline and saves results and intermediate images in data/voronoi folder.
Scales map to 4 resolutions and
saves times for generating in txt file for later plotting of data.

