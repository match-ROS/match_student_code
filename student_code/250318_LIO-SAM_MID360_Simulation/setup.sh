#!/bin/bash
mkdir -p ~/temp
mv LIO-SAM-MID360/ ~/temp/LIO-SAM-MID360/
mv livox_laser_simulation/ ~/temp/livox_laser_simulation/
mv livox_ros_driver/ ~/temp/livox_ros_driver/
cd match_mobile_robotics
./setup_full.sh
cd ..
mv ~/temp/LIO-SAM-MID360/ LIO-SAM-MID360/
mv ~/temp/livox_laser_simulation/ livox_laser_simulation/
mv ~/temp/livox_ros_driver/ livox_ros_driver/
rm -r ~/temp
source ../../devel/setup.bash

# install libignition-math4
sudo apt update
sudo apt install -y libignition-math4-dev

# install gtsam
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# install livox-sdk
cd ~
if [ ! -d "Livox-SDK" ]; then
  git clone https://github.com/Livox-SDK/Livox-SDK.git
fi
cd Livox-SDK
mkdir -p build/src && cd build
cmake ..
make -j$(nproc)
sudo make install

# source ../../devel/setup.bash
roscd && cd ..

# install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Compile the package livox_ros_driver first
catkin_make --pkg livox_ros_driver

# Compile
catkin_make
source devel/setup.bash

