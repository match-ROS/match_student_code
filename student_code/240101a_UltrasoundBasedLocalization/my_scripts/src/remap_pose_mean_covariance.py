#!/usr/bin/env python3

# mit diesem Skript werden Daten aus dem Marvelmind hedge_pos_ang und hedge_imu_fusion Topics in einem PoseWithCovarianceStamped msg gespeichert werden 

import rospy
from marvelmind_nav.msg import hedge_pos_ang, hedge_imu_fusion
from geometry_msgs.msg import PoseWithCovarianceStamped
from marvelmind_nav.msg import  hedge_imu_raw
from sensor_msgs.msg import Imu
from math import pi
seq_init = 0 
pose_out = PoseWithCovarianceStamped()
pose_out_transformed=PoseWithCovarianceStamped()
imu_out = Imu()

x_1 = 0
y_1 = 0
z_1 = 0
x_2 = 0
y_0 = 0
z_2 = 0

#subscriber imu
def callback_marvelmind_imu(msg_pos_imu):   
    global  seq_init, yaw_accel_var
    
# define header imu
    imu_out.header.seq = seq_init 
    seq_init += 1
    imu_out.header.stamp = rospy.Time.now()
    imu_out.header.frame_id = 'base_link'
    
#imu_angular velocity # Resolution of LSB for gyro data is 0.0175 dps, so value 1000 means 17.5 degrees per second.
    imu_out.angular_velocity.z = msg_pos_imu.gyro_z / (1000*17.5)/180*pi 
    yaw_accel_var = rospy.get_param("/yaw_accel_var")
    imu_out.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, yaw_accel_var]

#subscriber pose
def callback_marvelmind_pos_1(msg_pos_1):   
    global  x_1, y_1, z_1, angle

    x_1 = msg_pos_1.x_m
    y_1 = msg_pos_1.y_m
    z_1 = msg_pos_1.z_m
    
def callback_marvelmind_pos_2(msg_pos_2):   
    global seq_init, x_2, y_2, z_2
    
    x_2 = msg_pos_2.x_m
    y_2 = msg_pos_2.y_m
    z_2 = msg_pos_2.z_m
    
# define header pose
    pose_out.header.seq = seq_init
    seq_init += 1
    pose_out.header.stamp = rospy.Time.now()
    pose_out.header.frame_id = 'beacon_map'
# define header transformed marvelmind zum plotten

    pose_out.header.seq = seq_init
    seq_init += 1
    pose_out_transformed.header.stamp = rospy.Time.now()
    pose_out_transformed.header.frame_id = 'map'
# mean position from Beacons pose data
    pose_out.pose.pose.position.x = ((x_1 + x_2)/2)
    pose_out.pose.pose.position.y = ((y_1 + y_2)/2)
    pose_out.pose.pose.position.z = 0      # mit Absicht auf null
#zum plotten
    pose_out_transformed.pose.pose.position.x = ((x_1 + x_2)/2)+1.394
    pose_out_transformed.pose.pose.position.y = ((y_1 + y_2)/2)+0.417
# Quaterion from imu_fused data
def callback_marvelmind_pose_3(msg_pos_imu):   

    pose_out.pose.pose.orientation.x = 0 # mit Absicht auf null
    pose_out.pose.pose.orientation.y = 0 # mit Absicht auf null
    pose_out.pose.pose.orientation.z = msg_pos_imu.qz
    pose_out.pose.pose.orientation.w = msg_pos_imu.qw

# covariance matrix with variance data from stationary state of the Beacons 
    pose_out.pose.covariance = rospy.get_param("/pose_cov")

#publish topics

    pub_1.publish(pose_out)
    pub_2.publish(imu_out)
    pub_3.publish(pose_out_transformed)
if __name__ =='__main__':
    rospy.init_node('remap_pose_mean_covariance')

    rate = rospy.Rate(10)

    sub_1 = rospy.Subscriber("/hedge1/hedge_pos_ang", hedge_pos_ang, callback_marvelmind_pos_1)
    sub_2 = rospy.Subscriber("/hedge2/hedge_pos_ang", hedge_pos_ang, callback_marvelmind_pos_2)
    sub_3 = rospy.Subscriber("/hedge1/hedge_imu_fusion", hedge_imu_fusion , callback_marvelmind_pose_3)
    sub_4 = rospy.Subscriber("/hedge2/hedge_imu_raw", hedge_imu_raw , callback_marvelmind_imu)
    pub_1 = rospy.Publisher("/position_marvelmind_with_covariance", PoseWithCovarianceStamped, queue_size=10)
    pub_2 = rospy.Publisher("/imu_marvelmind_with_covariance", Imu, queue_size=10)
    pub_3 = rospy.Publisher("/marvelmind_transformed", PoseWithCovarianceStamped, queue_size=10)
    rospy.spin()