# launchfile structure
Either launch `scout1_launch_all.launch` (on scout) with `mobilemaster_launch_all.launch` and `rviz_launch_with_config.launch` or `offline_replay.launch` with `rviz_launch_with_config.launch`.

The Launchfile `mapping_rviz.lauch` can be used to create new maps and `faroconverter.launch` is used to read faro data from a .csv, transform it and write it to a bag.

## scout1_launch_all (launch on scout)
	├── mini_lidar_scout1
	|	├── static_transform: baselink -> laser
	|	├── remap: tf -> new_tf
	|	├── scout_mini_omni_minimal
	|	└── rplidar_a3
	├── multi_hedge
	|	├── hedge_rcv_bin hedge1
	|	└── hedge_rcv_bin hedge2
	└── 1_port_setup

## mobilemaster_launch_all
	├── sync_time
	├── amcl
	|	├── amcl_config
	|	└── amcl
	├── launch_map_match
	|	├── map_file
	|	├── map_server
	|	├── hedgepos_to_posestamped
	|	├── imu_raw_to_imu_message
	|	├── imu_fusion_to_imu_message
	|	├── hedgehog_covariances
	|	├── publish_transformed_positions
	|	├── beaconpos_publisher
	|	├── amcl_recovery
	|	└── faro_transform_launcher
	|			├── static_transform: map -> faro
	|			└── static_transform: faro -> marv
	├── single_mir_ps4_drive
	└── ekf_localization
		├── ekf_localization_node: ekf_absolute
		└── ekf_localization_node: ekf_relative
			└── remap: odometry/filtered -> ekf_pose

## mobilemaster_launch_all_without_ekf
	├── sync_time
	├── amcl
	|	├── amcl_config
	|	└── amcl
	├── launch_map_match
	|	├── map_file
	|	├── map_server
	|	├── hedgepos_to_posestamped
	|	├── imu_raw_to_imu_message
	|	├── imu_fusion_to_imu_message
	|	├── hedgehog_covariances
	|	├── publish_transformed positions
	|	├── beaconpos_publisher
	|	├── amcl_recovery
	|	├── faro_transform_launcher
	|		├── static_transform: map -> faro
	|		└── static_transform: faro -> marv
	└── single_mir_ps4_drive

## offline_replay
	├── map_file
	├── map_server
	├── hedgepos_to_posestamped
	├── imu_raw_to_imu_message
	├── imu_fusion_to_imu_message
	├── hedgehog_covariances
	├── publish_transformed positions
	├── beaconpos_publisher
	├── amcl_recovery
	├── faro_calibration_point_publisher
	├── scout_mini_model
	├── scout_description
	├── static_transform: baselink -> laser
	├── rosbag_play: filtered bag file
	├── amcl
	|	├── amcl_config
	|	└── amcl
	├── ekf_localization
	|	├── ekf_localization_node: ekf_absolute
	|	└── ekf_localization_node: ekf_relative
	|		└── remap: odometry/filtered -> ekf_pose
	└── faro_transform_launcher
		├── static_transform: map -> faro
		└── static_transform: faro -> marv

## rviz_launch_with_config
	└── rviz
		└── rviz_config

## mapping_rviz
	├── rviz
	└── mapping
		└── gmapping

## faroconverter
	├── faro_transform_launcher
	└── convert_faro_ros
