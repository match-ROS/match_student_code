#!/usr/bin/env python3

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
        vel_lin = 0.4 # m/s
        vel_ang = 1.57/2  #Rad/s
        t_rot = 250      #Roation time
        t_rot2 = 260  #Roation time
        t_rot_neg = 265   #Roation time
        while not rospy.is_shutdown():

            # create a Twist message, robot moves forward direction Hiwi Raum
            twist = Twist()
            twist.linear.x = vel_lin
            for i in range(720):   
                self.pub.publish(twist)
                r.sleep()
            # create a Twist message, robot moves forward direction Match Tower
            twist = Twist()
            twist.linear.y = vel_lin
            for i in range(450):  
                self.pub.publish(twist)
                r.sleep()
            # create a Twist message, robot moves forward direction desck
            twist = Twist()
            twist.linear.x = -vel_lin
            for i in range(450):  
                self.pub.publish(twist)
                r.sleep()
            twist = Twist()
                        # create a Twist message, robot moves backwords direction hall gate
            twist = Twist()
            twist.linear.x = -vel_lin
            twist.linear.y = -vel_lin
            for i in range(200):  
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