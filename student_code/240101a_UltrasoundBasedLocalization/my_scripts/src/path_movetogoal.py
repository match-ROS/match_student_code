#!/usr/bin/env python3

#this skript was used for trajectory with rotation

import rospy
import math
from geometry_msgs.msg import Twist
class path:

    def __init__(self):
        
        # register this function to be called on shutdown
        rospy.on_shutdown(self.cleanup)

        # publish to cmd_vel
        self.pub = rospy.Publisher('cmd_vel', Twist)

        # give the node/publisher time to connect
        rospy.sleep(1)
        r = rospy.Rate(100)
        vel_lin = 0.4 #0.4 #0.6 #0.7 #0.8
        vel_ang = 1.57/2
        t_rot = 250
        t_rot2 = 125
        t_rot3 = 135
        while not rospy.is_shutdown():

            # create a Twist message, robot moves forward direction Hiwi Raum
            twist = Twist()
            twist.linear.x = vel_lin
            for i in range(720):   # 1390 #927  #796 #697   #time adjustment: current vel/new vel*current range
                self.pub.publish(twist)
                r.sleep()
            # create a twist message, robot turns 
            twist = Twist()
            twist.angular.z = vel_ang     # 45 deg/s * 2sec = 90 degrees (had to test it cause it didnt work with 200)
            for i in range(t_rot):         
                self.pub.publish(twist)
                r.sleep()
            # create a Twist message, robot moves forward direction Match Tower
            twist = Twist()
            twist.linear.x = vel_lin
            for i in range(450):  #764  #509    #436 #382
                self.pub.publish(twist)
                r.sleep()
            # create a twist message, robot turns 
            twist = Twist()
            twist.angular.z = vel_ang     # 45 deg/s * 2sec = 90 degrees (had to test it cause it didnt work with 200)
            for i in range(t_rot):         
                self.pub.publish(twist)
                r.sleep()
            # create a Twist message, robot moves forward direction desck
            twist = Twist()
            twist.linear.x = vel_lin
            for i in range(450):  #414  #276    #236 #207
                self.pub.publish(twist)
                r.sleep()
            # create a twist message, robot turns 
            twist = Twist()
            twist.angular.z = vel_ang     # 45 deg/s * 2sec = 90 degrees (had to test it cause it didnt work with 200)
            for i in range(t_rot2):         
                self.pub.publish(twist)
                r.sleep()
            # create a Twist message, robot moves forward direction desck
            twist = Twist()
            twist.linear.x = vel_lin
            for i in range(380):  #414  #276    #236 #207
                self.pub.publish(twist)
                r.sleep()
            twist = Twist()
            # create a twist message, robot turns            
            twist.angular.z = -vel_ang     # 45 deg/s * 2sec = 90 degrees (had to test it cause it didnt work with 200)
            for i in range(t_rot3):         
                self.pub.publish(twist)
                r.sleep()
            twist = Twist()
            r.sleep(5)
            
        print('robot has achieved one round with a linear velocity v_x_lin = ', vel_lin , 'and an angular velocity v_x_ang=', vel_ang)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub.publish(twist)

if __name__=="__main__":
    rospy.init_node('movetogoal')
    path()