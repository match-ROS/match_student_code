#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist

class MoveForDistance:
    def __init__(self, distance_t: float, frame_target = "/map", frame_source = "/mur/ur/wrist_3_link"):
        """publishes velocity to '/mur/velocity_command' until Distance is reached.
        Subscribes to '/velocity_measurement' to get velocity command.

        Args:
            distance_t (float): distance to move in meter
            frame_target (str, optional): for distance measurement. Defaults to "/map".
            frame_source (str, optional): for distance measurement. Defaults to "/mur/ur/wrist_3_link".
        """
        self.pub_vel = rospy.Publisher('/mur/velocity_command', Twist, queue_size=1)
        rospy.on_shutdown(lambda: self.pub_vel.publish(Twist())) # 0 movement on shutdown
        # self.pub_transform = rospy.Publisher('/measurement/transform', TransformStamped, queue_size=1)
        rospy.Subscriber("velocity_measurement", Twist, self.cb_vel_command)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(50)
        self.distance_goal = distance_t
        
        self.frame_target = frame_target
        self.frame_source = frame_source
        self.listener.waitForTransform(frame_target, frame_source, rospy.Time(0), rospy.Duration(8.0))

    def cb_vel_command(self, msg):
        rospy.logdebug('Got velocity command')
        t,r = self.listener.lookupTransform(self.frame_target, self.frame_source, rospy.Time(0))
        self.transl_init = np.array(t)
        self.rot_init = np.array(r)
        # self.fin_pos = self.init_pos + self.distance_remaining*self.vel # as array, but for safety compare total distance
        # self.vel = msg
        self.run(msg)
    
    def check_distance(self, p1: np.ndarray, p2: np.ndarray):
        rospy.logdebug(f"p1: {p1}, \n p2: {p2}")
        dist = np.linalg.norm(p1-p2)
        rospy.loginfo('Distance: {}'.format(dist))
        
        return dist > self.distance_goal
                                         
    def run(self, vel):
        rospy.loginfo('Starting to move')
        self.pub_vel.publish(vel)
        while not rospy.is_shutdown():
            try:
                t,r = self.listener.lookupTransform(self.frame_target, self.frame_source, rospy.Time())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr('Could not get transform')
                self.rate.sleep()
                continue
            if self.check_distance(np.array(t), self.transl_init):
                self.pub_vel.publish(Twist())
                rospy.loginfo('Distance reached')
                break
            self.rate.sleep()

if __name__ == '__main__':
    frame_target = "/map"
    # frame_source = "/mur/mir/base_footprint"
    frame_source = "/mur/ur/wrist_3_link"
    rospy.init_node('move_for_distance', log_level=rospy.DEBUG)
    
    move = MoveForDistance(0.1, frame_target, frame_source)
    
    rospy.spin()
    
    
