# wall_localization_demo

## Overview
This package implements a wall-based localization approach for a vertical profiler using LaserScan data in ROS Noetic.  
The scanner pose is estimated relative to a known wall geometry by combining 2D image-based feature matching (LBP or SIFT) with 3D ICP refinement.  
The package supports offline wall map generation, live localization in Gazebo simulation, and quantitative evaluation against ground truth data.

**Author:** Linh Thach Dang 
**E-Mail:** linh.thach.dang@stud.uni-hannover.de

## Usage
To run the live localization experiment in Gazebo, launch:
```bash
roslaunch wall_localization_demo run_localization.launch
```
This starts the simulation, initializes the localization pipeline, publishes required static transforms, and records ground truth and estimated poses.


## Config files

- ```offline_map.yaml```  
Contains file paths to the offline wall map, point cloud, and descriptor databases used during localization.

- ```estimator_params.yaml```  
Defines parameters for the live localization pipeline, including frame definitions, filtering limits, and ICP settings.

- ```localization_params.yaml```  
Contains parameters for the 2D feature matching stage, such as descriptor configuration and matching thresholds.


## Launch files

- ```run_localization.launch```  
Runs the complete live localization pipeline in simulation.

Argument list:  
method (type: string, default: sift): Feature matching method used for localization (sift or lbp).  
pointcloud_topic (type: string, default: /scan_cloud): LaserScan input topic.

- ```build_offline_map.launch```  
Builds the offline wall map and descriptor databases from recorded scan data.

- ```run_mapping_recording.launch```  
Records wall scan data while the robot moves along the wall.

- ```spawn_world_and_robot.launch```  
Spawns the Gazebo world, wall model, and robot equipped with the vertical profiler sensor.


## Nodes

```scanner_pose_estimator```  

This node performs the live wall-based localization. It processes incoming LaserScan data, builds local wall patches, performs 2D image-based matching, refines the pose using ICP, and publishes the estimated scanner pose as a TF transform.


## Subscribed Topics

```scan_cloud``` (sensor_msgs/LaserScan)  
Laser scan data acquired by the vertical profiler sensor.

```odom``` (nav_msgs/Odometry)  
Odometry data of the mobile robot.

## Parameters
```wall_frame``` (type: string, default: wall): Reference frame attached to the wall geometry.

```scanner_frame``` (type: string, default: vertical_profiler_link): Frame of the vertical profiler sensor.

```base_frame``` (type: string, default: base_link): Robot base frame used for odometry and TF lookups.

```odom_topic``` (type: string, default: /odom): Odometry topic used for motion tracking.

```s_min_live``` (type: float, default: -0.6): Minimum along-wall coordinate for live scan filtering.

```s_max_live``` (type: float, default: 0.6): Maximum along-wall coordinate for live scan filtering.

```z_min_live``` (type: float, default: 0.0): Minimum height for live scan filtering.

```z_max_live``` (type: float, default: 2.0): Maximum height for live scan filtering.

```live_img_w``` (type: int, default: 6): Width of the live patch image.

```live_img_h``` (type: int, default: 11): Height of the live patch image.

```icp_max_iter``` (type: int, default: 50): Maximum number of ICP iterations.

```icp_max_dist``` (type: float, default: 0.0035): Maximum correspondence distance used during ICP refinement.

```method``` (type: string, default: sift): Feature matching method used for 2D localization.

```stride``` (type: int, default: 16): Step size used when sliding the query window over the offline wall map during 2D localization.

```max_query_kps``` (type: int, default: 60): Maximum number of keypoints extracted from the live patch and used for feature matching.

```patch_h``` (type: int, default: 100): Height of the image patch used for 2D feature extraction and matching.

```patch_w``` (type: int, default: 100): Width of the image patch used for 2D feature extraction and matching.

```lbp_radius``` (type: int, default: 2): Radius of the circular neighborhood used for LBP feature computation.

```lbp_points``` (type: int, default: 16): Number of sampling points on the LBP circular neighborhood.

```lbphf_keep``` (type: int, default: 24): Number of most significant LBP-HF histogram bins retained for matching.

```coarse_topk``` (type: int, default: 10): Number of top candidate matches retained during coarse 2D localization.

```sift_ratio``` (type: float, default: 0.7): Lowe’s ratio threshold used to filter ambiguous SIFT feature matches.

```sift_pca_dim``` (type: int, default: 4): Dimensionality of SIFT descriptors after PCA compression.

```scale_bins``` (type: int, default: 3): Number of scale bins used for multi-scale SIFT feature extraction.

```flann_trees``` (type: int, default: 4): Number of KD-trees used by the FLANN-based SIFT matcher.

```flann_checks``` (type: int, default: 60): Number of search checks performed by the FLANN matcher during feature matching.

# General Usage

## 1. Convert a point cloud (CSV) into an offline texture map

This step converts a recorded wall point cloud in CSV format into a 2D texture
representation that is later used for localization.

Run the following script:
```
python3 src/wall_localization_demo/pcl2texture.py \
--cloud data/mapping_run/wall_points_bag.csv \
--midpath data/mapping_run/wall_midpath_bag.csv \
--delta_s 0.005 \
--delta_z 0.005 \
--out_map data/mapping_run/offline_map.npy \
--out_points data/mapping_run/offline_points.npy
```
Relevant parameters:
- `--delta_s`: resolution along the wall direction in m
- `--delta_z`: vertical resolution in m
- `--cloud`, `--midpath`: input CSV files
- `--out_map`, `--out_points`: output files

Outputs:
- offline_map.npy, used for 2D feature matching
- offline_points.npy, used for ICP refinement


## 2. Build SIFT and LBP descriptor databases

Offline descriptor databases are generated from the texture map using
localization_2d.py.

To build the SIFT database:
```
python3 src/wall_localization_demo/localization_2d.py \
--method sift \
--pixel_map data/mapping_run/offline_map.npy \
--new
```
To build the LBP database:
```
python3 src/wall_localization_demo/localization_2d.py \
--method lbp \
--pixel_map data/mapping_run/offline_map.npy \
--new
```
Relevant parameters:
- Feature extraction and matching parameters are defined in
  config/localization_params.yaml (e.g. stride, patch_h, patch_w,
  lbp_radius, sift_ratio, flann_trees)
- `--new` forces rebuilding the database if it already exists

Outputs:
- data/mapping_run/offline_sift_db.pkl
- data/mapping_run/offline_lbp_db.pkl


## 3. Spawn the Gazebo simulation

This step starts the simulation environment including the wall model and the
robot equipped with the vertical profiler.
```
roslaunch wall_localization_demo spawn_world_and_robot.launch
```
The robot can move along the wall by running:
```
python3 src/wall_localization_demo/move_along_wall.py \
--speed 0.05 \
--duration 60
```
or to move the robot and perform the recording at the same time:
```
roslaunch wall_localization_demo run_mapping_recording.launch
```


## 4. Run localization and evaluation

Run the complete localization and evaluation pipeline:
```
roslaunch wall_localization_demo run_localization.launch
```
What happens:
- main.py estimates the scanner pose and publishes the TF frame
  scanner_icp_refined
- compare_poses.py records ground truth and estimated poses into CSV files

Relevant parameters:
- In config/estimator_params.yaml:
  - `method`: sift or lbp
  - `pointcloud_topic`
  - `s_min_live`,` s_max_live`
  - `z_min_live`, `z_max_live`
  - `live_patch_s_length`, `live_img_w`, `live_img_h`
  - `icp_max_iter`, `icp_max_dist`
- In the launch file:
  - output paths for ground truth and estimated CSV files

Outputs:
- data/localization_run/groundtruth_poses.csv
- data/localization_run/estimated_poses.csv


