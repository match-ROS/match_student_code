# two_ur16e_student_project

## Packages

### config
This folder containts the ur16e controller yaml file

### launch
This folder containts a folder called real_robot which contanints the file to start the ur.16e_bringup.launch file from the ur_robot_driver with the namespace of robot1
In this package there are also the files to launch the moveit packages with different namespaces, the jacobian matrix, the Trajectory of both robots and the movement of the homeposition.

### scripts
This folder containts all the relevant python scripts for the esitmaton task.

### urdf
This folder containts the custimized xacro files for the ur16e robot

## Usage for Robot1
To run all necessary scripts to launch robot1 use the following command:

`roslaunch simulation launch_R1.launch`

It's also possible to start the scripts seperatly. For this follow the next setps and commands:

1. Start the hardware of Robot1: `roslaunch simulation real_ur16e.launch
2. Start the moveit Package of Robot1: `roslaunch simulation robot1_demo.launch
3. Publish the Jacobian Matrix of Robot1: `rosrun simulation pub_jacobianR1.py
4. Start the script for the forward kinematics: `rosrun simulation FK_rob1.py

## Usage for Robot2
To run the hardware of Robot2 you have to use the drive ssh mur: In this drive you have to use the following command:

'roslaunch simulation real_ur16eR2.launch`

To run the necessary scripts like it's done for Robot1 you can use the follwing command:

`roslaunch simulation launch _R2.launch`

It's also possible too to start the scripts seperatly. The steps are the same as for Robot1.

## Example how to launch the Trajectory launch file to move the Robots

If you have launched the files like in the description before, you can use the following command to start the Trajectory for both robots:

'roslaunch simulation Trajektorie.launch'
