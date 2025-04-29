## Overview
This folder contains all the codes and function packages used in the master thesis project. This project investigated the application of the fast-lio algorithm to build 3D maps on the Livox-Mid-360 radar and Mir600 mobile robot platforms, and developed a 3D localization algorithm for scan-to-map matching based on build-in maps. The experiment is carried out in the simulation software Gazebo.
Using scripts from the repository, the positioning accuracy of localization and AMCL algorithms is compared.

**Author:** Haotian Zhou

**E-Mail:** haotian.zhou@stud.uni-hannover.de
## Packages
### fast-lio
Implements the fast-lio algorithm specifically adapted for the Livox MID360 LiDAR on Mir-600.
### livox_laser_simulation
Official Livox package for simulating Livox LiDAR sensors in ROS.
### livox_ros_driver
Official Livox ROS driver for connecting and interfacing Livox LiDAR sensors.
### match_mobile_robotics
Containing simulation files of robot platform and map. Changes to the Mir600's .xacro and .urdf files integrate the Mid-360 radar.
### FAST_LIO_LOCALIZATION
A simple localization framework that can re-localize in built maps.
## Usage
Create a catkin_ws/src folder in your home directory and move all files from the current page into the src folder.

# FastLIO + Livox LiDAR Instructions 

## 1.Livox installation configuration

```bash
cd ~
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build
cmake .. && make -j8
sudo make install

cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src/livox_ros_driver 
cd ~/livox_ws/src/ws_livox
catkin_make
```
Add it at the end of the `~/.bashrc` file:
```bash
source ~/livox_ws/src/ws_livox/devel/setup.bash
```
## 2.FastLIO and Relocate Configuration

Place the FastLIO and Relocate packages in the `~/catkin_ws/src/` directory:

```bash
cd ~/catkin_ws
rm -rf build devel
catkin_make
```
Add it at the end of the `~/.bashrc` file:
```bash
source ~/catkin_ws/devel/setup.bash
```
### Change map file path

```bash
cd ~/catkin_ws/FAST_LIO_LOCALIZATION/launch
gedit localization_mid360.launch
```

To change the map path on line 20, simply change the `/home/byy/catkin_ws/src/` prefix.

## 3.Run FastLIO to build the map and generate `.pcd`.

### Build 3D maps

```bash
roslaunch mir_examples single_mir_600.launch
roslaunch fast_lio mapping_mid360.launch
```

Control the robot movement to complete the build. When the mapping is complete, press `Ctrl+C` to stop FastLIO and the `scans.pcd` file will be generated.

## 4.Localization

### real-time positioning

```bash
roslaunch mir_examples single_mir_600.launch
roslaunch fast_lio_localization localization_mid360.launch
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
```

- Wait for the map to appear in RViz
- Setting the initial position
- Control robot movement for repositioning

**Output topics:**

```bash
rostopic echo /Odometry
```

## Scripts
### positioningerro_time.py 
Generates velocity-over-time plots.
###boxplots.py 
boxplots.py: Creates box plots for error distribution.
###rotationrpe.py
rotationrpe.py: Computes and plots rotational RPE.
### translationalrpe.py
translationalrpe.py: Computes and plots translational RPE.
### vel_angularvel_time.py
vel_angularvel_time.py: Plots linear and angular velocity over time.

These Python scripts help analyze and visualize the performance metrics (ATE, RPE, box plots) of the localization experiments.
