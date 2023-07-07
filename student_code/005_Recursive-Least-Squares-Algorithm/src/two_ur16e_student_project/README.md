# two_ur16e_student_project

## Packages

### config
This folder contains the customized ur16e controller .yaml file.

### launch
This folder contains a folder called real_robot, which contains the file to start the ur.16e_bringup.launch file from the ur_robot_driver with the namespace of robot1.
In this package there are also the files to launch the moveit packages with different namespaces, the jacobian matrix, the trajectory of both robots and the movement to the homeposition.

### scripts
This folder contains all the relevant python scripts for the esitmaton task.

### urdf
This folder contains the custimized xacro files for the ur16e robot.

## Usage for Robot1
To run all necessary scripts to launch Robot1, use the following command:

`roslaunch simulation launch_R1.launch`

It's also possible to start the scripts seperately. For this follow the next steps and commands:

1. Start the hardware of Robot1: `roslaunch simulation real_ur16e.launch
2. Start the moveit Package of Robot1: `roslaunch simulation robot1_demo.launch
3. Publish the Jacobian Matrix of Robot1: `rosrun simulation pub_jacobianR1.py
4. Start the script for the forward kinematics: `rosrun simulation FK_rob1.py

## Usage for Robot2
To run the hardware of Robot2 you have to use the drive ssh mur: In this drive you have to use the following command:

`roslaunch simulation real_ur16eR2.launch`

To run the necessary scripts (equivalent like Robot1), you can use the follwing command:

`roslaunch simulation launch _R2.launch`

It's also possible to to start the scripts seperately. The steps are the same as for Robot1.

## Example how to launch the Trajectory launch file to move the Robots

If you have succesfully launched the filee, you can use the following command to start the trajectory for both robots:

`roslaunch simulation Trajektorie.launch`
