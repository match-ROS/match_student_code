#! /usr/bin/env python3
"""apply tf transform to marvelmind position

    from marvelmind frame to map"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

# function to strip covariance because do_transform_pose can only do PoseStamped
def remove_covariance(with_cov):
    """strip covariance and return the message and covariance seperately"""
    
    no_cov = PoseStamped()
    no_cov.header = with_cov.header
    no_cov.pose = with_cov.pose.pose
    cov = with_cov.pose.covariance
    return(no_cov,cov)

# ..and re-add the covariance
def add_covariance(no_cov,cov):
    """take covariance and message and re-combine them"""
    
    with_cov = PoseWithCovarianceStamped()
    with_cov.header = no_cov.header
    with_cov.pose.pose = no_cov.pose
    with_cov.pose.covariance = cov
    return(with_cov)

def callback_pose(msg_in):
    """on receiving marvelmind pose strip covariance, do transform, re-add it and publish
    
    do_transform_pose cannot handle PoseWithCovariance"""
    
    pose_removed_covariance,cov = remove_covariance(msg_in)
    
    transform = tfBuffer.lookup_transform("map",
                                    "marv_frame",
                                    rospy.Time(0), #get the tf at first available time
                                    rospy.Duration(1.0))

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_removed_covariance, transform)
    pose_added_covariance = add_covariance(pose_transformed, cov)
    
    pub.publish(pose_added_covariance)

if __name__=='__main__':
    rospy.init_node("pose_transform")
    
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    
    sub_pose = rospy.Subscriber("/marvelmind_pos", PoseWithCovarianceStamped, callback_pose)
    pub = rospy.Publisher("/transformed_pos", PoseWithCovarianceStamped, queue_size=10)
    rospy.spin()
    
    