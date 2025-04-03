<p align="center">
  <a href="" rel="noopener">
 <img width=200px height=200px src="include/abb/ma_img.png" alt="Project logo"></a>
</p>

<h3 align="center">Optimization of mur movement</h3>

<div align="center">

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

---

<p align="center"> Optimization for murs with impedance control
    <br> 
</p>

## 📝 Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Problem solving](#problems)
- [Usage](#usage)
- [TODO](./TODO.md)

## 🧐 About <a name = "about"></a>

Optimization for murs with impedance control in regard to cartesian ur-path

## 🏁 Getting Started <a name = "getting_started"></a>

put project in catkin folder, install [Prerequisites](#prerequisites) and build catkin workspace.

### Prerequisites

#### hrl_kdl:
```
git clone -b noetic_dev https://github.com/pumablattlaus/hrl-kdl.git
cd hrl-kdl/pykdl_utils
pip3 install .
cd ..
cd hrl_geom
pip3 install .
```

#### modules to install:
```
pip3 install PyKDL
pip3 install numdifftools
pip3 install scipy
```
#### rosdep
```
# from catkin_ws root folder:
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## 🔧 Problem solving <a name = "problems"></a>

Admittance Controller movement: Make sure vel_command is at 100 Hz (not that importatnt anymore because time is measured)

```
rostopic hz /mur/ur/joint_group_vel_controller/command
```
Same for optimization Node:
```
rostopic hz /mur/cooperative_manipulation/cartesian_velocity_command
```

## 🎈 Usage <a name="usage"></a>

One robot in hardware (mur):
```
# On mur:
roslaunch mur_launch_hardware mur.launch
roslaunch optimization_algo control.launch
```

Or if splitted computation:
```
# On mur:
roslaunch optimization_algo control.launch launch_optimization:=false

#on rosmaster:
roslaunch optimization_algo control.launch launch_admittance:=false
```

Geschwindigkeitsbefehle des EEF:
```
# cmds:
rostopic pub /mur/velocity_command geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0 
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```