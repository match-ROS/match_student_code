#!/usr/bin/env python3 
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from math import pi
import numpy as np

Pi = pi

class relative_position():

    def __init__(self):
        
        rospy.init_node("Relative_Position")
        
        # Initialization of the target base positions in the world, the tcp-position in the base frame 
        self.tcp_robot1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.tcp_robot2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.base_robot1_position= [0.0, 0.0, 0.0]
        self.base_robot2_position= [0.0, 0.0, 0.0]
        self.data_to_send = Float64MultiArray()
        
        # Subscriber of TCP-position and base position, Publisher of relative position
        
        rospy.Subscriber("/robot1/TCP_robot1", Float64MultiArray, self.callback_TCP_robot1)
        rospy.Subscriber('/robot2/TCP_robot2', Float64MultiArray, self.callback_TCP_robot2)
        rospy.Subscriber("/qualisys/UR10_black/pose", PoseStamped, self.callback_base_robot2)
        rospy.Subscriber('/qualisys/UR10_white/pose', PoseStamped, self.callback_base_robot1)
        self.pub_relative_position = rospy.Publisher("/robot1/Relative_Position", Float64MultiArray, queue_size=10)

        self.run()

    def run(self):
        rate = rospy.Rate(500)
        while not rospy.is_shutdown():
            self.controller()
            self.data_to_send.data = self.relative_distance_1_2
            self.pub_relative_position.publish(self.data_to_send)
            rate.sleep()
        
    def controller(self):
        
        # Calculating of TCP-position in the world frame of Robot1
        tcp_robot1_pos = [self.tcp_robot1[1], -self.tcp_robot1[0], self.tcp_robot1[2]]
        base_to_tcp_1 = np.add(self.base_robot1_position, tcp_robot1_pos)
        
        # Calculating of TCP-position in the world frame of Robot1
        tcp_robot2_pos = [-self.tcp_robot2[1], self.tcp_robot2[0], self.tcp_robot2[2]]
        base_to_tcp_2 = np.add(self.base_robot2_position, tcp_robot2_pos)
        
        # Calculating of relative distance between Robot1 and Robot2
        self.relative_distance_1_2 = np.negative(base_to_tcp_1)
        self.relative_distance_1_2 = np.add(self.relative_distance_1_2, base_to_tcp_2) 
        #print(self.relative_distance_1_2)
        
    def callback_TCP_robot1(self,data):
        self.tcp_robot1 = data.data

    def callback_TCP_robot2(self,data):
        self.tcp_robot2 = data.data

    def callback_base_robot1(self,msg):
        self.base_robot1_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
    def callback_base_robot2(self,msg):
        self.base_robot2_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

if __name__=="__main__":
    relative_position()