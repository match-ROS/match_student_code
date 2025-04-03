#! /usr/bin/env python3
"""convert message types from raw and fused IMUs

  conversion from propietary marvelmind hedge_imu_raw and hedge_pos_ang to Imu
  combining imu raw data with orientation data from imu fused
"""

import rospy
from marvelmind_nav.msg import hedge_imu_raw
from sensor_msgs.msg import Imu

seq_counter = 0
angle_rad = 0

def callback_convert_imu_message(msg_in):
    """on receiving imu raw message add orientation and covariance
    
      orientation from global variable,
      covariance from parameter server
    """
    #type definition
    msg_out = Imu()

    #header
    global seq_counter #sequential counter
    msg_out.header.seq = seq_counter
    seq_counter += 1
    
    msg_out.header.stamp = rospy.get_rostime() #in ros time format (#seconds & #nanoseconds)
    msg_out.header.frame_id = "base_link" 
        
    #linear acceleration
    msg_out.linear_acceleration.x = msg_in.acc_x/1000
    msg_out.linear_acceleration.y = msg_in.acc_y/1000
    msg_out.linear_acceleration.z = (msg_in.acc_z/1000)-1
    
    msg_out.linear_acceleration_covariance = rospy.get_param("/imu_linear_acceleration_covariance")
    
    #angular velocity
    msg_out.angular_velocity.x = msg_in.gyro_x/1000
    msg_out.angular_velocity.y = msg_in.gyro_y/1000
    msg_out.angular_velocity.z = msg_in.gyro_z/1000
    
    msg_out.angular_velocity_covariance = rospy.get_param("/imu_angular_velocity_covariance")

    #orientation not availabe and handled by imu_fusion
    msg_out.orientation.x = 0
    msg_out.orientation.y = 0
    msg_out.orientation.z = 0
    msg_out.orientation.w = 0
    
    msg_out.orientation_covariance = [[0,0,0],
                                      [0,0,0],
                                      [0,0,0]]
    
    #publish converted message
    pub.publish(msg_out)

if __name__=='__main__':
    rospy.init_node("convert_imu")
    sub = rospy.Subscriber("hedge1/hedge_imu_raw", hedge_imu_raw, callback_convert_imu_message)
    pub = rospy.Publisher("/imu_data", Imu, queue_size=10)
    rospy.spin()