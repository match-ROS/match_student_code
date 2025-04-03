#!/usr/bin/env python3

# mit diesem Skript werden Daten aus dem Marvelmind hedge_pos_ang und hedge_imu_fusion Topics in einem PoseWithCovarianceStamped msg gespeichert werden 

import rospy
from marvelmind_nav.msg import hedge_pos_ang, hedge_imu_fusion
from geometry_msgs.msg import Pose

seq_init = 0 
pose_out = Pose()

x_1 = 0
y_1 = 0
z_1 = 0
x_2 = 0
y_0 = 0
z_2 = 0


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
    

# mean position from Beacons pose data
    pose_out.position.x = (x_1 + x_2)/2
    pose_out.position.y = (y_1 + y_2)/2
    pose_out.position.z = 0      

# Quaterion from imu_fused data
def callback_marvelmind_pos_3(msg_pos_imu):   

    pose_out.orientation.x = msg_pos_imu.qx
    pose_out.orientation.y = msg_pos_imu.qy
    pose_out.orientation.z = msg_pos_imu.qz
    pose_out.orientation.w = msg_pos_imu.qw

    pub.publish(pose_out)

if __name__ =='__main__':
    rospy.init_node('remap_pose_mean_covariance')

    rate = rospy.Rate(10)
    sub_1 = rospy.Subscriber("/hedge1/hedge_pos_ang", hedge_pos_ang, callback_marvelmind_pos_1)
    sub_2 = rospy.Subscriber("/hedge2/hedge_pos_ang", hedge_pos_ang, callback_marvelmind_pos_2)
    sub_3 = rospy.Subscriber("/hedge1/hedge_imu_fusion", hedge_imu_fusion , callback_marvelmind_pos_3)
    pub=rospy.Publisher("/middle_point", Pose, queue_size=10)

    rospy.spin()