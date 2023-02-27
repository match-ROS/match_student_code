# ips sensor fusion - student package

## Overview
this submodule contains the students code for implementing a sensor fusion approach for indoor localization using an ultrasonic indoor positioning system. It implements two Kalman filters, one fusing relative data from an imu and the odometry as input to an amcl node, the other fusioning absolute data from the marvelmind ips and the amcl localization. A Seperate Node is used to set the amcl's initial pose when kidnapping is detected.

This package contains launch files for live sensor fusion as well as an offline replay version for tuning ekf and amcl parameters.

**Author** :  Pitt Müller

**E-Mail**: p.mueller@stud.uni-hannover.de

## Usage
### Live ips sensor fusion
* For scout 1:
    1. ssh: on scout, login as rosmaster:  `$ssh rosmatch@agilex`
    2. on scout: run `$roslaunch ips_sensor_fusion scout1_launch_all.launch`
    3. on mobilemaster: run `$roslaunch ips_sensor_fusion rviz_launch_withconfig.launch`
    4. on mobilemaster: run `$roslaunch ips_sensor_fusion mobilemaster_launch_all.launch`
* For scout 2:
    1. ssh: on scout, login as rosmaster:  `$ssh rosmatch@agilex`
    2. grand port permissions: `$sudo chmod 666 /dev/ttyACM0` and `$sudo chmod 666 /dev/ttyACM1`
    3. on scout: run `$roslaunch ips_sensor_fusion scout2_launch_all.launch`
    4. on mobilemaster: run `$roslaunch ips_sensor_fusion rviz_launch_withconfig.launch`
    5. on mobilemaster: run `$roslaunch ips_sensor_fusion mobilemaster_launch_all.launch`

### noEKF
Start launchfiles as in live ips sensor fusion, exchanging `scou1_launch_all.launch` with `scout1_launch_all_noEKF.launch` and `mobilemaster_launch_all.launch` with `mobilemaster_launch_all_noEKF.launch`. This may be used for independent amcl parameter tuning.

### Data recording
1. launch system as in live usage
2. on mobilemaster navigate to desired bagfile directory
3. use `$rosbag record -a -O bag_name` to record all messages, stop it using `str+c`

### Offline replay
1. For new, unfiltered bags, use the following commands to keep relevant topics and remove the rest, for alredy filtered bags proceed with step 2:
    * prepare ros bags by filtering relevant topics (provide topics to be kept):
    `$rosbag filter input_bag_name.bag output_bag_name.bag 'topic == "/scan" or topic == "/odom" or topic == "/cmd_vel" or topic == "/hedge1/beacons_pos_a" or topic == "/hedge1/hedge_imu_fusion" or topic == "/hedge1/hedge_imu_raw" or topic == "/hedge1/hedge_pos" or topic == "/hedge1/hedge_pos_a" or topic == "/hedge1/hedge_pos_ang" or topic == "/hedge2/beacons_pos_a" or topic == "/hedge2/hedge_imu_fusion" or topic == "/hedge2/hedge_imu_raw" or topic == "/hedge2/hedge_pos" or topic == "/hedge2/hedge_pos_a" or topic == "/hedge2/hedge_pos_ang"'` 
    * crop data to desired timestamp using `$osbag filter input_bag_name.bag output_bag_name.bag "t.secs <= 1673961654.509"` (example time)
2. launch rviz seperately via `$roslaunch ips_sensor_fusion rviz_launch_withconfig.launch`
3. start ips sensor fusion and replay of previously captured bag files by launching `$roslaunch  ips_sensor_fusion offline_replay.launch`
4. alternatively use `$roslaunch  ips_sensor_fusion offline_replay.launch bagfile:=other_bagfile.bag` to replay a specific bag file

## Config files
* `absolute_ekf_config.yaml`: contains the configuration for the kalman filter fusing absolute marvelmind and amcl poses
* `relative_ekf_config.yaml`: contains the configuration for the kalman filter fusing relative imu and odometry data
* `amcl_config.yaml`: contains the configuration for the amcl node
* `hedgehog_covariances.yaml`: contains the covariance matrices for the marvelmind hedgehog

## Launch files
most relevant launch files, further structure can be found in launch folder Readme.
* `scout1_launch_all.launch`: launch file to be run on scout computer in live usage
* `mobilemaster_launch_all.launch`: launch file for mobilemaster in live usage
* `offline_replay.launch`: launch file for offline replay of previously recorded data
* `rviz_launch_withconfig.launch`: launch file for rviz for offline and live usage

## Nodes
### amcl_recovery.py
this node detects kidnapping by measuring the distance betwenn the hedgehog and amcl pose and sets the amcl pose to the hedgehog pose if the distance is greater than a threshold. It also publishes the distance between the hedgehog and amcl pose as a diagnostic message.
#### Subscribed Topics
* `/amcl_pose` ([geometry_msgs/PoseWithCovarianceStamped]
* `/transformed_pose` ([geometry_msgs/PoseWithCovarianceStamped] marvelmind pose transformed to map

#### Published Topics
* `/initialpose` ([geometry_msgs/PoseWithCovarianceStamped]
* `/set_pose` ([geometry_msgs/PoseWithCovarianceStamped]

### beaconpos_publisher.py
This node publishes the beacon positions. It is used to visualize the beacons in rviz.
#### Subscribed Topics
* `/hedge1/beacons_pos_a` ([marvelmind_nav/beacon_pos_a])

#### Published Topics
* `/beacon_positions` ([geometry_msgs/PointStamped])

### convert_faro_ros.py
This node converts all faro tracker csv files in the faro_raw directory to a rosbag file. The faro transform launcher needs to be launched first.

### faro_calibration_point_publisher.py
This Node publishes hardcoded calibration points recorded using the faro tracker to visualize transform fit in rviz.
#### Published Topics
* `/calibration_points` ([geometry_msgs/PointStamped])
### hedgepos_to_posestamped.py
Node to convert Marvelmind hedgehog messages to ROS usable message type.
#### Subscribed Topics
* `/hedge1/hedge_pos_ang` ([marvelmind_nav/hedge_pos_ang])
#### Published Topics
* `/marvelmind_pos` ([geometry_msgs/PoseWithCovarianceStamped])
### imu_fusion_to_imu_message.py
Convert Hedgehogs IMU Fusion message to IMU message type. Only Contains Orientation data.
#### Subscribed Topics
* `/hedge1/hedge_imu_fusion` ([marvelmind_nav/hedge_imu_fusion])
#### Published Topics
* `/imu_fusion_data` ([sensor_msgs/Imu])
### imu_raw_to_imu_message.py
Convert Hedgehogs IMU Raw message to IMU message type.
#### Subscribed Topics
* `/hedge1/hedge_imu_raw` ([marvelmind_nav/hedge_imu_raw])
#### Published Topics
* `/imu_data` ([sensor_msgs/Imu])
### publish_transformed_positions.py
This node publishes the marvelmind hedgehog position transformed to the map frame.
#### Subscribed Topics
* `/marvelmind_pos` ([geometry_msgs/PoseWithCovarianceStamped])
#### Published Topics
* `/transformed_pos` ([geometry_msgs/PoseWithCovarianceStamped])

## Test setup
### Record floorplan using gmapping
1. disable sudo password prompt
2. synchronize ssh timing between agilex and rosmaster (on rosmaster: `$ssh agilex sudo date -s @$(date -u +"%s")`) check with shared terminal `$date`
3. on rosmaster: start roscore
4. on scout: start scout driver `$roslaunch ips_sensor_fusion mini_lidar_scout1.alunch`
5. on rosmaster: `$roslaunch scout_navigation mapping_rviz.launch`
6. openslam_gmapping and slam_gmapping need to be installed
7. for saving map: `$ rosrun map_server map_saver -f my_map`

### Setup faro tracker
1. seperate Windows PC with Faro Tracker utilities software needed
2. disable firewall
3. set TCP/IPv4 Adress manually to `128.128.128.10` with the subnet mask `255.255.255.0`
4. connect faro tracker via ethernet and start faro tracker
5. run tracker utilities program
    * startup checks, skip warmup
6. tracker pad
    * home Position
7. measure pad, setup: 
    * instant
    * Time 0.1s
    * to file
    * samples per point: 1 
    * provide file name with .csv extension
    * when ready, start measurement
### Standalone Marvelmind interface
* two hedgehogs are mounted + and - 10cm in y direction from the center of the robot
* connect hedgehogs via USB to the mobile robots computer
* check for their ports, usually /dev/ttyACM0 and /dev/ttyACM1
* use `$rosrun marvelmind_nav hedge_rcv_bin /dev/ttyACM0` to start the marvelmind node

## static points
scripts and files for calculating marvelmind standard deviation at several static points. contains recorded bags and faro data. 