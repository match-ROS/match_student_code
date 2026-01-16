# Wall Localization Demo

## Overview
This folder contains the code developed for wall-based scanner localization using geometric and appearance-based matching. The approach localizes a vertical profiler or scanner relative to a known wall geometry by combining 2D image-based matching (LBP or SIFT) with 3D ICP refinement.

**Author:** Linh Thach Dang  
**E-Mail:** linh.thach.dang@stud.uni-hannover.de


## Installation
Copy this folder into the `src` directory of a catkin workspace and build it using `catkin_tools`.

```bash
cd ~/catkin_ws/src
cp -r wall_localization_demo .
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Add the folder data from the additional thesis files and move the wall_mesh.obj from data to models/printed wall.
To correctly spawn the wall in gazebo, add the following line to your ~/.bashrc:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/wall_localization_demo/models
```

Install missing python packages:
```bash
python3 -m pip install scikit-image
```

## Package
### wall_localization_demo

This package contains all nodes, scripts, and launch files required for:
- offline wall map generation,
- live localization,
- evaluation against ground truth.


## Scripts

All core scripts are located in `src/wall_localization_demo/`.

### main.py
Runs the live localization pipeline.  
Subscribes to a LaserScan topic, transforms scan points into the wall frame, builds a local patch image, performs 2D localization (SIFT or LBP), refines the pose using ICP, and publishes the estimated scanner pose as the TF frame `scanner_icp_refined`.

### localization_core.py
Implements the core localization algorithms used by `main.py`, including patch generation, descriptor extraction, database matching, ICP refinement, and pose estimation.

### compare_poses.py
Logs ground truth and estimated poses during localization.  
Samples TF and writes two CSV files:
- `groundtruth_poses.csv` (wall → vertical_profiler_link)
- `estimated_poses.csv` (wall → scanner_icp_refined)

### move_along_wall.py
Moves the robot along the wall at a fixed offset to generate continuous scan data during experiments.

### pcl2texture.py
Utility script to convert point cloud data into image or texture representations of the wall.
