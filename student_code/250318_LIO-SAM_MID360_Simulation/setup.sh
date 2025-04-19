#!/bin/bash

# install libignition-math4
sudo apt update
sudo apt install -y libignition-math4-dev

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

catkin_make
source devel/setup.bash

