#! /usr/bin/env python3
""" publish beacon positions for visual validation of transformations
"""

import rospy

from geometry_msgs.msg import  PointStamped
from marvelmind_nav.msg import beacon_pos_a

seq_counter = 0

def callback_pose(msg_in):
    """extract stationary beacon positions from topic and re-publish as Point"""
    
    #stationary beacon adresses: 157, 81, 159
    if(msg_in.address == 157 or msg_in.address == 81 or msg_in.address == 159): #only for beacons, not hedgehogs
        msg_out = PointStamped()
        
        #header
        global seq_counter #sequential counter
        msg_out.header.seq = seq_counter
        seq_counter += 1
        
        msg_out.header.stamp = rospy.get_rostime() #in ros time format (#seconds & #nanoseconds)
        msg_out.header.frame_id = "marv_frame" 

        #point
        msg_out.point.x = msg_in.x_m
        msg_out.point.y = msg_in.y_m
        msg_out.point.z = 0 #ignore height
        
        pub.publish(msg_out)

if __name__=='__main__':
    rospy.init_node("beaconpos_publisher")
    sub_pose = rospy.Subscriber("/hedge1/beacons_pos_a", beacon_pos_a, callback_pose)
    pub = rospy.Publisher("/beacon_positions", PointStamped, queue_size=10)
    rospy.spin()
    
    