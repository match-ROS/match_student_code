# Master Thesis Yassine Mechri

  ## Marvelmind Hardware 
  * Marvelmind Indoor Navigation System is an off-the-shelf indoor navigation system, designed to provide precise (±2cm) location data to autonomous robots
  * Mobile beacon’s location is calculated based on a propagation delay of an ultrasonic pulses (Time-Of-Flight or TOF) between stationary and mobile beacons using trilateration algorithm. 

  ## Steps to set up the System

  1. Place the Stationary Beacons on the wall in a way that will provide an optimal ultrasonic coverage. 
  2. Connect the modem to a PC and run the Dashboard Software to wake up all the Beacons (stationary and Mobile)
  3. The map will form and zoom in automatically  
  4. Check that the radio settings on the modem and the radio settings on the beacon are the same
  5. The distaces between the stationary beacons (x, y) will be measured systematically and displayed in a table .
  6. The hight of the Sattionary beacons should be set manually:

  - Stationary beacon Werkstatt (157) x: -1.96m y: -1.13m z: 2.33m  f:31000Hz
  - Stationary beacon Hiwi_Raum (81) x: 4.11m y: -0.9m z: 2.33m  f:37000Hz
  - Stationary beacon Matchtower (159) x: 3.635m y: 2.503m z: 2.33m  f:25000Hz
  - Moboile beacon Robot right (153) : f:31000Hz
  - Moboile beacon Robot left (205) : f:45926Hz
  7. add a map of the room and adjust its position according to known beacon position in the room: 

  - right click on floor-> Add floormap
  - upload map.png
  - right click on the floor map and scale settig  

  * More detailed description of the system is to find on the manual privided by [Marvelmind] (https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).

  ## Necessary Packages for the Work
  
  * Package for marvelmind data hadeling 
    - package name = ros_marvelmind_package
    - url = https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package.git

  * Package for laser scanner data handeling
    - package name = rplidar_ros
    - url = https://github.com/robopeak/rplidar_ros.git

  * Packages and dependencies for Scout mini Agilex 
    - package name = scout_ros
    - url = https://github.com/westonrobot/scout_ros.git
    - package name = ugv_sdk
    - url = https://github.com/westonrobot/ugv_sdk.git

    - Build up dependecies: `$ sudo apt-get update` and
      `$ sudo apt-get install build-essential git cmake libasio-dev`

  * Package for Extanded Kalman Filter 
    - package name = robot_localization
    - url = https://github.com/cra-ros-pkg/robot_localization.git

  ## Marvelmind using ROS

  * Hedgehog Setting: In Interfaces -> Steaming Output must be Set to USB+Uart. Protocol on UART/USB output must be set to Marvelmind

  * Get Location data from Hedgehog:
  1. Connect hedgehog via USB and check for the port (usually /dev/ttyACM0 or /dev/ttyACM1 in case to mobile beacons are used)
  2. Start roscore with `$roscore`
  3. Start hedgehog data receiving script `$rosrun marvelmind_nav hedge_rcv_bin /dev/ttyACM0`
  4. Show location data `$rostopic echo /hedge_pos_a`
  5. Show IMU data `$rostopic echo /hedge_imu_fusion`

  ## Useful functions with ros 

  * live visulisation of topics:
  - use plotjuggler on host pc to visualize topic data `$rosrun plotjuggler plotjuggler`

  * Recording of topics: `$ rosbag record --duration= /topic_name`
  `$ rosbag record /cmd_vel /joint_states /map /map_metadata /odom /rosout /rosout_agg /scan /scout_light_control /scout_status /tf /tf_static`  

  * display frequency of publishing: `$rostopic hz /topicname`

  * synchronize ssh timing between agilex and rosmaster (on rosmaster: `$ssh agilex sudo date -s @$(date -u +"%s")`) check with shared terminal `$date`

  ## Start the Robot's Initialisation and the Extended Kalman Filter

  * Change to sgilx `$ ssh agilx`
  * Connect to the robot `$ sudo chmod 666 /dev/ttyUSB0`
  * Connect to Hedge 1 `$ sudo chmod 666 /dev/ttyACM0`
  * Connect to Hedge 2 `$ sudo chmod 666 /dev/ttyACM1`
  * synchronize ssh timing between agilex and rosmaster (on rosmaster: `$ssh agilex sudo date -s @$(date -u +"%s")`) check with shared terminal `$date`
  * launch the robot, the hedges and the map in Agilex `$ roslaunch my_scripts start_omni_agilex.launch`
  * launch rviz, initialisation and EKF in rosmaster `$ roslaunch my_scripts initialisation.launch  `
  * in case CAN connection failed `$ candump can0` in Agilex to check if CAN connection established
  * `$ rosrun scout_bringup bringup_can2usb.bash ` to connect to CAN in case no connection has been established
