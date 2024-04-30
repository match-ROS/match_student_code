#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def set_initial_pose_and_goal():
    rospy.init_node('set_initial_pose_and_goal')

    initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(2)  # Warte kurz, damit die Publisher startklar sind

    # Setze die Initialpose
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = -8.0 # also change in mir_100_changed and general_mir_changed launch
    initial_pose.pose.pose.position.y = -8.0
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0
    initial_pose_publisher.publish(initial_pose)
    rospy.loginfo("Initial pose set")

    rospy.sleep(1)  # Kurze Pause, um sicherzustellen, dass die Initialpose gesetzt wurde und damit Rviz den Path anzeigt(wird nur 1 mal gepublished)

    # Setze das Navigationsziel
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = -4.0
    goal.pose.position.y = 4.0
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    goal_publisher.publish(goal)
    rospy.loginfo("Goal published")

if __name__ == '__main__':
    try:
        set_initial_pose_and_goal()
    except rospy.ROSInterruptException:
        pass
