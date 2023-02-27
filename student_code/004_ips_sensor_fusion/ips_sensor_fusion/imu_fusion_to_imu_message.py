#! /usr/bin/env python3
"""convert imu fusion data from marvelmind hedge_imu_fusion to IMU

  IMU Fusion only provides orientation data"""
import rospy
from marvelmind_nav.msg import hedge_imu_fusion
from sensor_msgs.msg import Imu

seq_counter = 0

def callback_convert_imu_message(msg_in):
    """on receiving message extract orientation and publish as IMU"""
    
    #type definition
    msg_out = Imu()

    #header
    global seq_counter #sequential counter
    msg_out.header.seq = seq_counter
    seq_counter += 1
    
    msg_out.header.stamp = rospy.get_rostime() #in ros time format (#seconds & #nanoseconds)
    msg_out.header.frame_id = "base_link" 
        
    #linear acceleration
    msg_out.linear_acceleration.x = 0
    msg_out.linear_acceleration.y = 0
    msg_out.linear_acceleration.z = 0
    
    msg_out.linear_acceleration_covariance = rospy.get_param("/imu_linear_acceleration_covariance")

    #angular velocity
    msg_out.angular_velocity.x = 0
    msg_out.angular_velocity.y = 0
    msg_out.angular_velocity.z = 0
    
    msg_out.angular_velocity_covariance = rospy.get_param("/imu_angular_velocity_covariance")

    #orientation
    msg_out.orientation.x = msg_in.qx
    msg_out.orientation.y = msg_in.qy
    msg_out.orientation.z = msg_in.qz
    msg_out.orientation.w = msg_in.qw

    msg_out.orientation_covariance = rospy.get_param("/imu_fusion_orientation_covariance")
    
    #publish converted message
    pub.publish(msg_out)

    
if __name__=='__main__':
    rospy.init_node("convert_imu_fusion")
    sub = rospy.Subscriber("hedge1/hedge_imu_fusion", hedge_imu_fusion, callback_convert_imu_message)
    pub = rospy.Publisher("/imu_fusion_data", Imu, queue_size=10)
    rospy.spin()
