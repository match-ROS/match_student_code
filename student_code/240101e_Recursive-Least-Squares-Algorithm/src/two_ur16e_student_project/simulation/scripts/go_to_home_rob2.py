#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from math import pi
import time

tau = 2.0 * pi
a = 2*pi/360

# Class to move Robot2 into start pose in joint space
class ur16e_start_pose2():

    def __init__(self):
        rospy.init_node("ur16e_start_pose_rob2")
        
        # Initialization of the target joint position q, the joint velocity dq and the actual joint position q_act
        self.dq =Float64MultiArray()
        # get the target position of Robot2 in degree because the Base of Robot2 is approx. 2 cm higher than Robot1
        self.q = [-90.0*a, -90.08*a, -93.57*a, -86.34*a, 90.37*a , -0.04*a]
        self.dq.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.q_act = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Definition of the proportional gain for the P-controller
        self.Kp = 0.5 
        
        # Initialization of the subscriber and publisher
        rospy.Subscriber("/robot2/joint_states", JointState, self.callback)
        self.pub_joint_velocity = rospy.Publisher("/robot2/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        self.run()
        
    def run(self):  
        timeout =time.time() +10
        rate = rospy.Rate(500)
            
        while not rospy.is_shutdown() and time.time() < timeout:
            # Set the joint velocity dq to zero if the absolute value is smaller than 0.00001
            for i in range(0,5):
                if abs(self.dq.data[i]) < 0.00001:
                    self.dq.data[i] = 0 
                    
            self.joint_pose_controller() 
            rospy.loginfo(self.dq)
            # Publishing the joint velocity dq
            self.pub_joint_velocity.publish(self.dq)
            
        # Set the joint velocity dq to zero for safety reasons
        self.dq.data = [0.0,0.0,0.0,0.0,0.0,0.0]    
        self.pub_joint_velocity.publish(self.dq)
        
    def joint_pose_controller(self):
        
        e = [0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Get error e of the difference between the target joint position q and the actual joint position q_act
        e[0] = self.q[0] - self.q_act[2] 
        e[1] = self.q[1] - self.q_act[1] 
        e[2] = self.q[2] - self.q_act[0] 
        e[3] = self.q[3] - self.q_act[3] 
        e[4] = self.q[4] - self.q_act[4]  
        e[5] = self.q[5] - self.q_act[5] 
        
        # Calculate the joint velocity dq with the P-controller

        self.dq.data[0] = self.Kp * e[0]
        self.dq.data[1] = self.Kp * e[1]
        self.dq.data[2] = self.Kp * e[2]
        self.dq.data[3] = self.Kp * e[3]
        self.dq.data[4] = self.Kp * e[4]
        self.dq.data[5] = self.Kp * e[5]
     
    def callback(self,data):
        self.q_act = data.position
        
if __name__=="__main__":
    ur16e_start_pose2()

