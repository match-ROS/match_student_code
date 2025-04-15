# LIO-SAM-MID360
## Overview

This package implements indoor robot localization using the LIO-SAM algorithm with Livox MID360 LiDAR and the AMCL algorithm using two 2D LiDARs.  
It includes the necessary launch files for running the LIO-SAM algorithm.


**Author:** Xinwei Fu

**E-Mail:** xinwei.fu@stud.uni-hannover.de

## Dependency

This package requires the same dependencies as LIO-SAM, refer to:  
[https://github.com/TixiaoShan/LIO-SAM?tab=readme-ov-file](https://github.com/TixiaoShan/LIO-SAM)

Install Livox SDK:  
[https://github.com/Livox-SDK/Livox-SDK  ](https://github.com/Livox-SDK/Livox-SDK)

Install libignition:  
```
sudo apt install libignition-math4-dev
```

## Usage

To launch the robot platform and map simulation, run the following command:

`roslaunch mir_examples single_mir_600.launch`

To start the LIO-SAM algorithm, run:

`roslaunch lio_sam run6axis.launch`

To run the AMCL algorithm, execute:

`roslaunch mir_examples amcl.launch`

To manually control the robot's motion using the rqt tool, use:

`rosrun rqt_robot_steering rqt_robot_steering`

## Config files
- `params.yaml`: The original configuration file of LIOSAM algorithm. Used for traditional mechanical lidar.
- `paramsLivoxIMU.yaml`: Parameter configuration specifically used for Livox lidar.

## Launch files
- `run6axis.launch`: The launch file to be run when using a 6-axis IMU (the simulation uses a 6-axis IMU)

- `run9axis.launch`: The launch file to be run when using a 9-axis IMU


## Nodes

### lio_sam_imuPreintegration
Performs IMU pre-integration for LIO-SAM.
This node integrates high-frequency IMU data to provide motion prediction and support pose estimation in the absence of LiDAR input.

**Subscribed Topics**
- `/livox/imu` (`sensor_msgs/Imu`)  
  Raw IMU data from the simulated IMU of Livox MID360 LiDAR.
- `/lio_sam/mapping/odometry` (`nav_msgs/Odometry`)  
  Optimized odometry from the mapping module.
- `/lio_sam/mapping/odometry_incremental` (`nav_msgs/Odometry`)  
  Incremental odometry for IMU integration.
   
**Published Topics**
- `/odometry/imu` (`nav_msgs/Odometry`)  
  Predicted robot motion based on IMU pre-integration.
- `/odometry/imu_incremental` (`nav_msgs/Odometry`)  
  Incremental motion estimate for deskewing LiDAR scans.

### lio_sam_imageProjection
Projects Livox LiDAR data into deskewed point cloud using IMU data.

**Subscribed Topics**
- `/livox/lidar_custom` (`livox_ros_driver/CustomMsg`)  
  Raw point cloud data from Livox MID360.
- `/livox/imu` (`sensor_msgs/Imu`)  
  IMU data for motion compensation.
- `/odometry/imu_incremental` (`nav_msgs/Odometry`)  
  Incremental odometry from IMU pre-integration.

**Published Topics**
- `/lio_sam/deskew/cloud_info` (`lio_sam/cloud_info`)  
  Structured point cloud and motion data for feature extraction.


### lio_sam_featureExtraction
Extracts geometric features (edges and planar) from the deskewed LiDAR scan.

**Subscribed Topics**
- `/lio_sam/deskew/cloud_info` (`lio_sam/cloud_info`)  
  Structured point cloud and motion data for feature extraction.

**Published Topics**
- `/lio_sam/feature/cloud_info` (`lio_sam/cloud_info`)  
  Updated cloud info including feature labels for further processing.

### lio_sam_mapOptmization
Performs pose graph optimization and map building using extracted features, optional GPS input, and loop closure detection.

**Subscribed Topics**
- `/lio_sam/feature/cloud_info` (`lio_sam/cloud_info`)  
  Structured cloud and features for pose graph optimization.
- `/lio_loop/loop_closure_detection` (custom message)  
  Loop closure detection result used to correct accumulated pose drift.
- `/odometry/gpsz` (optional)  
  GPS input for global pose correction (if enabled).
  
**Published Topics**
- `/lio_sam/mapping/odometry` (`nav_msgs/Odometry`)  
  Optimized robot pose after feature-based mapping.
- `/lio_sam/mapping/odometry_incremental` (`nav_msgs/Odometry`)  
  Incremental odometry for IMU integration.


### laser_scan_merger
Merging two 2D LiDAR scans (front and back),this merged scan is then used by the AMCL algorithm for localization.

**Subscribed Topics**
- `/f_scan` (`sensor_msgs/LaserScan`)  
  Front 2D LiDAR scan.
- `/b_scan` (`sensor_msgs/LaserScan`)  
  Back 2D LiDAR scan.

**Published Topics**
- `/merged_scan` (`sensor_msgs/LaserScan`)  
  Combined 2D laser scan used for AMCL localization.


