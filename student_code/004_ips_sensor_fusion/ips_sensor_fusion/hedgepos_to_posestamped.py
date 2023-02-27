#! /usr/bin/env python3
"""read hedgehog positions and re-publish

    convert from proprietary marvelmind message to PoseWithCovarianceStamped"""

import rospy
from marvelmind_nav.msg import hedge_pos_ang
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from tf.transformations import quaternion_from_euler

seq_counter = 0

def callback_convert_pose(msg_in):
    """ on receiving marvelmind message extract data and republish
    
        also read covariance from parameter server and add to message
        and convert from euler angles to quaternion"""
    
    #type definition
    msg_out = PoseWithCovarianceStamped()

    #header
    global seq_counter #sequential counter
    msg_out.header.seq = seq_counter
    seq_counter += 1
    
    msg_out.header.stamp = rospy.get_rostime() #in ros time format (#seconds & #nanoseconds)
    msg_out.header.frame_id = "marv_frame" 
        
    #position
    msg_out.pose.pose.position.x = msg_in.x_m
    msg_out.pose.pose.position.y = msg_in.y_m
    msg_out.pose.pose.position.z = msg_in.z_m
        
    #orientation
    angle_rad = (msg_in.angle/180)*np.pi #convert degrees to radians
    [x_quat,y_quat,z_quat,w_quat] = quaternion_from_euler(0,0,angle_rad)    #calculate quaternions from euler notation

    #angle as quaternions
    msg_out.pose.pose.orientation.x = x_quat
    msg_out.pose.pose.orientation.y = y_quat
    msg_out.pose.pose.orientation.z = z_quat
    msg_out.pose.pose.orientation.w = w_quat
    
    msg_out.pose.covariance = rospy.get_param("/pose_covariance")
    
    #publish converted message
    pub.publish(msg_out)

if __name__=='__main__':
    rospy.init_node("convert_hedgepos_posestamped")
    sub = rospy.Subscriber("hedge1/hedge_pos_ang", hedge_pos_ang, callback_convert_pose)
    pub = rospy.Publisher("/marvelmind_pos", PoseWithCovarianceStamped, queue_size=10)
    rospy.spin()