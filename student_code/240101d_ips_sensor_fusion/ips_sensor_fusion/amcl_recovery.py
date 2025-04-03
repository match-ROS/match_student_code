#! /usr/bin/env python3
"""Recovers from amcl kidnapping by setting initial pose

Calculating distance between amcl and marvelmind pose
if this distance is larger than 0.5m the marvelmind pose
is sent to the /initialpose topic
"""

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import tf_conversions
import tf2_ros

from geometry_msgs.msg import PoseWithCovarianceStamped


seq_counter = 0

amcl_pose = PoseWithCovarianceStamped()
distance_threshold = 0.5 #meters

def publish_initialpose(msg_in):
  """on marvelmind message calculate difference to amcl and reset if needed"""
  
  global amcl_pose
  
  #euclidian distance
  xdiff = msg_in.pose.pose.position.x - amcl_pose.pose.pose.position.x
  ydiff = msg_in.pose.pose.position.y - amcl_pose.pose.pose.position.y

  dist = np.sqrt(xdiff**2 + ydiff**2)
  print("distance: " + str(dist))

  if dist > distance_threshold:

    #type definition
    msg_out = msg_in

    #publish converted message
    pub.publish(msg_out)
    pub2.publish(msg_out)

    print("initialpose set!")


def save_amcl_pose(msg_in):
  """save amcl pose as global variable"""
  global amcl_pose
  amcl_pose = msg_in


if __name__=='__main__':
    rospy.init_node("amcl_recovery")
    sub_marv = rospy.Subscriber("/transformed_pos", PoseWithCovarianceStamped, publish_initialpose)
    sub_amcl = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, save_amcl_pose)
    
    #/initialpose and /set_pose for amcl and ekf
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
    pub2 = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=10)
    rospy.spin()
