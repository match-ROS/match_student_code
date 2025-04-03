# Scout IPS Sensor Fusion
This package contains The code used in a student research project examining the use of the ultrasonic indoor positioning system (IPS) by marvelmind robotics in mobile robotics. It implements a sensor fusion concept for fusing the ultrasonic IPS data with odometry and imu data, as well as AMCL localization data using two kalman filters. Ground truth data is recorded by a faro laser tracker.

The Students' code is contained in the ips_sensor_fusion package. In Addition to these packages, the marvelmin Dashboard is used.

**Author** :  Pitt Müller

**E-Mail**: p.mueller@stud.uni-hannover.de

## installation
* create catkin workspace
* `$sudo apt-get update`
* `$sudo apt-get install build-essential git cmake libasio-dev`
* navigate to catkin workspace `src` folder
* clone this repository using `$git clone [repo name] --recursive`
* use `$git submodule update --init` if alredy cloned submodules are empty. For submodules of Submodules `$git submodule update --init --recursive` may be necessary
* copy recording files from flash drive or match server to `/ips_sensor_fusion/data/`

## Marvelmind Dashboard installation
* Download latest Dashboard version and copy linux folder to desired path
* Copy `libdashapi.so` to `/usr/local/lib` using `$sudo cp libdashapi.so /usr/local/lib`
* grant user serial access permission using the dialout group
    * `$sudo adduser $dialout`
    * add file "99-tty.rules" to `/etc/udev/rules.d`
    * add file content: \
        #Marvelmind serial port rules \
        KERNEL == "ttyACM0", GROUP="dialout", MODE="666"
* execute `$sudo ldconfig`
* make dashboard executable `$sudo chmod 0777 ./dashboard_x86`
* run dashboard

## packages
    .
    ├── ips_sensor fusion       #student code
    │   ├── final_configuration_recordings  #results and recorded bags
    |   ├── floorplan                       #amcl floorplan 
    |   ├── launch                          #all launch files
    |   ├── data                            #directory for recorded bags, put recorded files here
    |   ├── parameter_configs               #covariance and filter config
    |   ├── static_points                   #marvelmind accuracy test
    |   └── utility                         #data conversion scripts
    |
    |   #third party submodules
    ├── openslam_gmapping       #gmapping submodule
    ├── slam_gmapping           #another gmapping submodule
    ├── ps4_controller          #control scout robot via ps4 remote
    ├── robot_localization      #kalman filter package
    ├── ros_marvelmind_package  #interface ros and marvelmind
    ├── rplidar_ros             #lidar driver
    ├── scout_mini              #main scout robot driver
    ├── scout_ros               #scout ros interface
    └── ugv_sdk                 #scout can-Bus interface

## used submodules
* rplidar_ros: https://github.com/Slamtec/rplidar_ros
* ros_marvelmind_package: https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package.git
* ugv_sdk: https://github.com/westonrobot/ugv_sdk.git
* scout_ros:https://github.com/westonrobot/scout_ros.git
* slam_gmapping: https://github.com/ros-perception/slam_gmapping.git
* openslam_gmapping: https://github.com/ros-perception/openslam_gmapping.git
* robot_localization: https://github.com/cra-ros-pkg/robot_localization.git branch = noetic-devel