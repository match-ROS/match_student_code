#! /usr/bin/env python3

# Diesen Script berechnet die Variance der einzelnen variabeln für die Covarianzmatrix 
# daten werden aus dem Stationary Zustand ausgewertet und der Noise des System damit ermittelt
# daten commen aus dem Node middle point

import rospy
import math
import time
import statistics
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from marvelmind_nav.msg import  hedge_imu_raw
from tf import transformations
import numpy as np

seq_init = 0 
x = []
y = []
z = []
roll = []
pitch =[]
yaw = []
w = []
quat= []
euler= []
yaw_deg =[]
variance_yaw=[]
yaw_accel =[]
pose_out_var= PoseWithCovarianceStamped()

def callback_marvelmind_pos(msg):   
    global x, y, z, roll, pitch, yaw, w, quat, euler, variance_x, variance_y, variance_z, variance_yaw, mean_x, mean_y, mean_z, mean_roll, mean_yaw, mean_pitch, mean_w, seq_init, initial_yaw

    # mean poition
    x.append(msg.position.x)
    y.append(msg.position.y)
    z.append(msg.position.z)
    
    mean_x = np.mean(x)+ 1.394 # for translation into mpa frame
    mean_y = np.mean(y)+ 0.417 # for translation into mpa frame
    mean_z = np.mean(z) #wont be used in 2D localizastion
    
    # mean orientation from quaternion to rad
    roll.append(msg.orientation.x)
    pitch.append(msg.orientation.y)
    yaw.append(msg.orientation.z)
    w.append(msg.orientation.w)
    
    mean_roll = np.mean(roll)
    mean_pitch = np.mean(pitch)
    mean_yaw = np.mean(yaw)
    mean_w = np.mean(w)
    
    quat = [mean_roll, mean_pitch, mean_yaw,mean_w]
    euler = transformations.euler_from_quaternion (quat)
    initial_yaw = euler[2]
    
    # Header and publisher parameters
    pose_out_var.header.seq = seq_init
    seq_init += 1
    
    pose_out_var.header.stamp = rospy.Time.now()
    pose_out_var.header.frame_id = 'map'
    
    pose_out_var.pose.pose.position.x = mean_x
    pose_out_var.pose.pose.position.y = mean_y
    pose_out_var.pose.pose.position.z = mean_z
    
    pose_out_var.pose.pose.orientation.x = mean_roll
    pose_out_var.pose.pose.orientation.y = mean_pitch
    pose_out_var.pose.pose.orientation.z = mean_yaw 
    pose_out_var.pose.pose.orientation.w = mean_w
    
def callback_marvelmind_imu(msg):   
    global yaw_accel, mean_yaw_accel
    # angular accel
    yaw_accel.append(msg.gyro_z / 1000)
    # covariance matrix with variance data from stationary state of the Beacons 
    mean_yaw_accel = np.mean (yaw_accel)
    
if __name__ =='__main__':
    rospy.init_node('define_variance')
    sub=rospy.Subscriber("/middle_point", Pose, callback_marvelmind_pos)
    sub_1=rospy.Subscriber("/hedge2/hedge_imu_raw", hedge_imu_raw, callback_marvelmind_imu)
    pub=rospy.Publisher("/pose_mean", PoseWithCovarianceStamped, queue_size=10)
    
    time.sleep(5)
    
    # covariance matrix with variance data from stationary state of the Beacons 
    
    variance_x = statistics.variance(x)
    variance_y = statistics.variance(y)
    variance_z = statistics.variance(z)
    variance_yaw = statistics.variance(yaw)
    variance_yaw= statistics.variance(yaw_accel)

    pose_out_var.pose.covariance = [variance_x, 0, 0, 0, 0, 0,
                                    0,variance_y, 0, 0, 0, 0,
                                    0, 0, variance_z, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, variance_yaw ]
    
    rospy.set_param("/pose_cov",pose_out_var.pose.covariance)
    rospy.set_param("/amcl/initial_pose_x",float(pose_out_var.pose.pose.position.x))
    rospy.set_param("/amcl/initial_pose_y",float(pose_out_var.pose.pose.position.y))
    rospy.set_param("/amcl/initial_pose_a",initial_yaw)
    rospy.set_param("/yaw_accel_var",variance_yaw)
    
    pub.publish(pose_out_var)
