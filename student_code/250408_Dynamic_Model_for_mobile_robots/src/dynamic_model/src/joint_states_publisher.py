#!/usr/bin/env python3
import rospy

#Ros Messages
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

import pandas as pd
import numpy as np

v_z = 0.0
z_total = 0.0
imu_last = 0.0
Last_Stamp = 0.0
encLeft_last = 0.0

def encoderCallback(msg):
    global Last_Stamp
    global encLeft_last

    if Last_Stamp == 0.0:
        Last_Stamp = rospy.Time.now()
    encLeft = msg.position[0]
    encRight = msg.position[1]
    time = rospy.Time.now()

    pub_msg = PoseStamped()
    pub_msg.header.stamp = time

    time_diff = (time - Last_Stamp).to_sec()
    if time_diff == 0:
        angle_speed = 0
    else:
        angle_speed = (encLeft - encLeft_last)/time_diff
    encLeft_last = encLeft
    Last_Stamp = time 

    pub_msg.pose.position.x = encLeft
    pub_msg.pose.position.y = angle_speed 
    pose_publisher.publish(pub_msg)

def imu_integrator(msg):
    global imu_last
    global imu_last_time 
    global v_z
    global z_total
    
    a_z = msg.linear_acceleration.z-9.81
    dt_imu = (rospy.get_rostime()-imu_last_time).to_sec()
    if dt_imu >= 0.5:
        v_z = 0.0
        z_total = 0.0
        imu_last_time = rospy.get_rostime()
    else:
        v_z = v_z + a_z * dt_imu
        z_total = z_total + v_z * dt_imu
        imu_last_time = rospy.get_rostime()

    pub_imu = PoseStamped()
    pub_imu.header.stamp = rospy.Time.now()

    pub_imu.pose.position.x = a_z
    pub_imu.pose.position.y = v_z
    pub_imu.pose.position.z = z_total 
    imu_publisher.publish(pub_imu)
    

if __name__ == "__main__":
    rospy.init_node('convert_encoder')
    global imu_last_time
    imu_last_time = rospy.get_rostime()

    rospy.Subscriber("/joint_states", JointState, encoderCallback)
    rospy.Subscriber("/imu_data", Imu, imu_integrator)


    pose_publisher = rospy.Publisher('/encoder_states', PoseStamped, queue_size=10)
    imu_publisher = rospy.Publisher('/z_integrate', PoseStamped, queue_size=10)

    # Timer for publishing
    rospy.spin()