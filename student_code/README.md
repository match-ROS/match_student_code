项目整体：
 该文件夹包含了学生论文项目中所使用到的所有代码和功能包。该项目研究了LIO-SAM算法在Livox-Mid-360雷达和Mir600移动机器人平台上的应用。实验在仿真软件Gazebo中进行。
使用evo工具库，对比LIO-SAM和AMCL算法的定位精度。

作者：Xinwei Fu
电子邮件：xinwei.fu@stud.uni-hannover.de

包：
LIO-SAM-MID360
livox_laser_simulation
livox_ros_driver
match_mobile_robotics

- **LIO-SAM-MID360**: Implements the LIO-SAM algorithm specifically adapted for the Livox MID360 LiDAR.
- **livox_laser_simulation**: Official Livox package for simulating Livox LiDAR sensors in ROS.
- **livox_ros_driver**: Official Livox ROS driver for connecting and interfacing Livox LiDAR sensors.
- **match_mobile_robotics**: 包含机器人平台和地图的仿真文件
