cmake_minimum_required(VERSION 3.0.2)
project(wall_localization_demo)

## Python3 required
set(CMAKE_CXX_STANDARD 14)
set(PYTHON_EXECUTABLE /usr/bin/python3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf_conversions
  pcl_ros
  pcl_msgs
  gazebo_ros
)

## OpenCV (used inside localization_2d.py and main.py)
find_package(OpenCV REQUIRED)

catkin_python_setup()

## Open3D is a Python dependency, not C++.
## sklearn/skimage/numpy also Python dependencies.

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf_conversions
    pcl_ros
    pcl_msgs
    gazebo_ros
)

#############
## INSTALL ##
#############

## Install Python scripts so rosrun/roslaunch can find them
catkin_install_python(
  PROGRAMS
    src/wall_localization_demo/main.py
    src/wall_localization_demo/move_along_wall.py
    src/wall_localization_demo/compare_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Install models (for Gazebo)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

## Install worlds
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

## Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

## Install any data (optional)
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)
