#! /usr/bin/env python3
# Dieses Script dient dazu, zu der Initial Pose (tf map to odom anhand vom AMCL) upzudaten anhang der Pose-Daten von den mobielen Beacons Daten kommen aus dem node define variance

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped



initpose_msg = PoseWithCovarianceStamped()

def callback_position(msg_in):
    #transfer Data to PoseWithCovariancestamped

    initpose_msg.header.seq = msg_in.header.seq
    initpose_msg.header.stamp = msg_in.header.stamp
    initpose_msg.header.frame_id = msg_in.header.frame_id
    initpose_msg.pose = msg_in.pose

    


if __name__=='__main__':
    rospy.init_node("setup_initial_pose")

    rate = rospy.Rate(1)
    sub = rospy.Subscriber("/pose_mean", PoseWithCovarianceStamped, callback_position)
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

    while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            if connections > 1 and initpose_msg.pose.pose.position.x != 0:
                pub.publish(initpose_msg)
                rospy.loginfo('Initial Pose has been set')
                break
rate.sleep ()
