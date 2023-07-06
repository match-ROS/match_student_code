#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
import moveit_commander
from math import pi, tau

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_to_home_node", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "UR16e_arm2"
move_group = moveit_commander.MoveGroupCommander(group_name)

tau = 2.0 * pi
a = 2*pi/360
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
"""joint_goal[0] = -90.06*a
joint_goal[1] = -90.08*a
joint_goal[2] = -93.57*a
joint_goal[3] = -86.34*a
joint_goal[4] = 90.37*a +tau/8
joint_goal[5] = -0.04*a"""

joint_goal[0] = 90.0*a 
joint_goal[1] = -90.0*a
joint_goal[2] = -90.0*a
joint_goal[3] = -90.0*a
joint_goal[4] = 90.0*a 
joint_goal[5] = -0.0*a

move_group.go(joint_goal, wait=True)

move_group.stop()
