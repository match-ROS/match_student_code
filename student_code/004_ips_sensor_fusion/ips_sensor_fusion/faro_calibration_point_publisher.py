#! /usr/bin/env python3
"""publish measured points for visual calibration validation"""
import rospy
from geometry_msgs.msg import  PointStamped

seq_counter = 0

#hardcoded points measured manually by faro tracker
target_radius = 0.01905 #meters
#             x,                     y
points =   [[5.483                , 2.196], #Beacon_157
            [-0.409               , 2.186], #Beacon_81
            [7.172 + target_radius, 0.803], #Tor_1
            [7.172 + target_radius, 1.316], #Tor_2
            [2.484,2.197 + target_radius]]  #Wand


def callback_sendposition(points):
    """read point list and publish them"""
    msg_out = PointStamped()
    
    for point in points:
        #header
        global seq_counter #sequential counter
        msg_out.header.seq = seq_counter
        seq_counter += 1
        
        msg_out.header.stamp = rospy.get_rostime() #in ros time format (#seconds & #nanoseconds)
        msg_out.header.frame_id = "faro_frame" 

        #point
        msg_out.point.x = point[0]
        msg_out.point.y = point[1]
        msg_out.point.z = 0
        
        pub.publish(msg_out)

if __name__=='__main__':

    rospy.init_node("faro_calibration_points")

    r = rospy.Rate(10) # 10hz
    pub = rospy.Publisher("/calibration_points", PointStamped, queue_size=10)

    while not rospy.is_shutdown():
        callback_sendposition(points)
        r.sleep()
    