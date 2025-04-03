#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException


robot_names = rospy.get_param('~robot_names', ['mur620'])
UR_prefix = rospy.get_param('~UR_prefix', ['UR10_l', 'UR10_r'])

# Position 
new_target_pose = [1.57, -0.52, 0.52, -1.57, -1.57, 0.0]  


moveit_commander.roscpp_initialize([])


rospy.init_node('multi_robot_moveit', anonymous=False)

try:
    # Welche Roboter 
    for robot_name in robot_names:
        
        robot = moveit_commander.RobotCommander(robot_description=robot_name + '/robot_description', ns=robot_name)

        # Beide Arme
        left_arm_group = moveit_commander.MoveGroupCommander("UR_arm_l", robot_description=robot_name + '/robot_description', ns=robot_name)
        right_arm_group = moveit_commander.MoveGroupCommander("UR_arm_r", robot_description=robot_name + '/robot_description', ns=robot_name)

       
        right_arm_group.set_joint_value_target(new_target_pose)
        if right_arm_group.go(wait=True):
            rospy.loginfo(f"{robot_name} right arm successfully moved to new target pose")
        else:
            rospy.logwarn(f"{robot_name} right arm failed to move to new target pose")

except MoveItCommanderException as e:
    rospy.logerr(f"MoveIt error: {e}")


moveit_commander.roscpp_shutdown()