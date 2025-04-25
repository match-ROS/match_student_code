#!/bin/bash

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
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install

cd ~/catkin_ws

# install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Compile the package livox_ros_driver first
catkin_make --pkg livox_ros_driver

# Compile
catkin_make
source devel/setup.bash

