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

## Usage
To run all necessary scripts to launch robot1 use the following command:
  `roslaunch simulation launch_R1.launch`

