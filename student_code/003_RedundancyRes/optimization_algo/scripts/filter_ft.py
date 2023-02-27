#!/usr/bin/env python3

from geometry_msgs.msg import WrenchStamped, Wrench
from match_lib.filter import LowPassFilter

import rospy
import numpy as np

class Ft_Filter(LowPassFilter):
    def __init__(self, k=0.2, topic='/mur216/ft_sensor/raw'):
        super(Ft_Filter, self).__init__((6,), k)
        self.ft_pub = rospy.Publisher("/mur216/ft_sensor/filtered", WrenchStamped, queue_size=10)
        rospy.Subscriber(topic, WrenchStamped, self.ft_cb)

    def pub_filtered(self, data, stamp, frame_id=''):
        msg = WrenchStamped()
        msg.wrench.force.x = data[0]
        msg.wrench.force.y = data[1]
        msg.wrench.force.z = data[2]
        msg.wrench.torque.x = data[3]
        msg.wrench.torque.y = data[4]
        msg.wrench.torque.z = data[5]
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        self.ft_pub.publish(msg)

    def ft_cb(self, msg=WrenchStamped()):
        data = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        data = self.filter(data)
        self.pub_filtered(data, msg.header.stamp, msg.header.frame_id)

if __name__ == "__main__":
    rospy.init_node('filter_ft')
    ft_filter = Ft_Filter(k=0.1)
    
    rospy.spin()