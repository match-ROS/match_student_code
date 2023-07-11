## Scripts
### FK_rob1.py
This script publishes the forward kinematic of robot1

### FK_rob2.py
This script publishes the forward kinematic of robot2

### algorithm.py
This script contains the Recursive-Least-Squares-Algorithm to estimate the unknown dynamic parameters. 
It subscribes the wrenches and the velocity and acceleration of robot1 to get the data matrix.

### go_to-home_rob1.py
This script contains the information about the defined home position of robot1 in joint_space. The movement is determind by
a proportional gain to move the robot1 with the joint_vel_controller (q_vel = K_p*(q_target-q_act)).

### go_to_home_rob2.py
This script contains the information about the defined home position of robot2 in joint_space. The movement is determind by
a proportional gain to move the robot1 with the joint_vel_controller (q_vel = K_p*(q_target-q_act)). The values of the 
joint_angles are different from robot1 because the robot2 is on a disk.

### moveit_go_to_home.py
With this script you can moove the robots in rviz in the homeposition
The comment part are the angles for robot2.

### pub_jacobianR1.py
This script publishes the jacobian matrix of robot1 based on the moveit configuration. The jacobian matrix is used for
the inverse differential kinematics to move the robot1 in a defined trajectory.

### pub_jacobianR2.py
This script publishes the jacobian matrix of robot2 based on the moveit configuration. The jacobian matrix is used for
the inverse differential kinematics to move the robot2 in a defined trajectory.

### relative_postion.py
With this script it is possible to get the relative position of the two robots based on the motion capture system of QauliSys.

### robot1_Trajektorie.py
This script contains the calculation of the optimized identification Trajectory which is used for robot1.
The calculated velocities and acceleration gets published for the data matrix in algorithm.py.

### robot2_Trajektorie.py
This script contains the calculation of the optimized identification Trajectory which is used for robot2.
