#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException

# Get robot name and arm prefixes from ROS parameters
robot_name = rospy.get_param('~robot_name', 'mur620')  # Default to 'mur620'
UR_prefix = rospy.get_param('~UR_prefix', ['UR10_l', 'UR10_r'])

# Define the target positions for the arms (90-degree outward position)
right_arm_target_pose = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]  # Right arm outward
left_arm_target_pose = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]  # Left arm outward

# Initialize MoveIt
moveit_commander.roscpp_initialize([])
rospy.init_node('single_robot_moveit', anonymous=False)

try:
    rospy.loginfo(f"Configuring MoveIt for robot: {robot_name}")

    # Initialize RobotCommander for the robot
    robot = moveit_commander.RobotCommander(robot_description=robot_name + '/robot_description', ns=robot_name)

    # Create MoveGroupCommander for both arms
    left_arm_group = moveit_commander.MoveGroupCommander("UR_arm_l", robot_description=robot_name + '/robot_description', ns=robot_name)
    right_arm_group = moveit_commander.MoveGroupCommander("UR_arm_r", robot_description=robot_name + '/robot_description', ns=robot_name)

    rospy.sleep(2)  # Give time for MoveIt to initialize

    # Move the right arm
    rospy.loginfo(f"Moving {robot_name} right arm to target pose...")
    right_arm_group.set_joint_value_target(right_arm_target_pose)
    if right_arm_group.go(wait=True):
        rospy.loginfo(f"{robot_name} right arm successfully moved to target pose")
    else:
        rospy.logwarn(f"{robot_name} right arm failed to move to target pose")

    # Move the left arm
    rospy.loginfo(f"Moving {robot_name} left arm to target pose...")
    left_arm_group.set_joint_value_target(left_arm_target_pose)
    if left_arm_group.go(wait=True):
        rospy.loginfo(f"{robot_name} left arm successfully moved to target pose")
    else:
        rospy.logwarn(f"{robot_name} left arm failed to move to target pose")

except MoveItCommanderException as e:
    rospy.logerr(f"MoveIt error: {e}")

# Shutdown MoveIt cleanly
moveit_commander.roscpp_shutdown()
