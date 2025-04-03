## Scripts
### FK_rob1.py
This script publishes the forward kinematic of Robot1

### FK_rob2.py
This script publishes the forward kinematic of Robot2

### algorithm.py
This script contains the Recursive-Least-Squares-Algorithm to estimate the unknown dynamic parameters of an object. 
It subscribes the wrenches and the velocity. acceleration and the actual gravity vector (depending on the current orientation) of Robot1 to get the data matrix.

### go_to-home_rob1.py
This script contains the information about the defined home position of Robot1 in joint_space. The movement is determind by
a proportional gain to move the Robot1 with the joint_vel_controller (q_vel = K_p*(q_target-q_act)).

### go_to_home_rob2.py
This script contains the information about the defined home position of Robot2 in joint_space. The movement is determind by
a proportional gain to move the Robot2 with the joint_vel_controller (q_vel = K_p*(q_target-q_act)). The values of the 
joint_angles are different from Robot1 because the Robot2 is on a disk and 2 cm higher.

### moveit_go_to_home.py
With this script you can move the robots in rviz in the homeposition
The comment part are the angles for Robot2.

### pub_jacobianR1.py
This script publishes the jacobian matrix of Robot1 based on the moveit configuration. The jacobian matrix is used for
the inverse differential kinematics to move the Robot1 in a defined trajectory.

### pub_jacobianR2.py
This script publishes the jacobian matrix of Robot2 based on the moveit configuration. The jacobian matrix is used for
the inverse differential kinematics to move the Robot2 in a defined trajectory.

### relative_postion.py
With this script it is possible to get the relative position of the two robots TCPs based on the motion capture system of QualiSys.

### robot1_Trajektorie.py
This script contains the calculation of the optimized identification trajectory which is used for Robot1.
The calculated velocities and accelerations will be published for the data matrix in algorithm.py.

### robot2_Trajektorie.py
This script contains the calculation of the optimized identification trajectory which is used for Robot2.
