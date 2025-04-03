#!/usr/bin/env python3 
import rospy
import time
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray


class RLS_algorithm():


    def __init__(self):

        rospy.init_node("ur16e_algorithm")
        self.data_to_send = Float64MultiArray()

        # Distance vector between Robot1 and Robot2, Vector of unknown object parameters Qm2 for better visualisation, Wrench of Robot1 and Robot2
        self.vectorR1_R2 = [0, 0.81, 0.0]
        self.duration = 20
        
        self.Qm2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.wrench1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        self.wrench2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Vector of Velocity, Acceleration and Rotation of the TCP (Robot1)
        self.vel_acc_rot = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Grasp Matrix between Robot1 and Robot2, Gripper is a 6x6 Unit Matrix, Gripper1 includes the relation between Robot1 and Robot2
        Gripper1 = np.array([[0, -self.vectorR1_R2[2], self.vectorR1_R2[1]], [self.vectorR1_R2[2], 0, -self.vectorR1_R2[0]], [-self.vectorR1_R2[1], self.vectorR1_R2[0], 0]])
        Gripper = np.vstack((np.hstack((np.identity(3), np.zeros((3,3)))), np.hstack((Gripper1, np.identity(3)))))
        self.Grasp = np.hstack((np.identity(6), Gripper))

        # Subscription of Robot1 and Robot2 Wrench, Vel_Acc_Rot of Robot1
        rospy.Subscriber("/robot1/wrench", WrenchStamped, self.callback_wrench)
        rospy.Subscriber("/robot2/wrench", WrenchStamped, self.callback_wrench2)
        rospy.Subscriber("/robot1/vel_acc_rot", Float64MultiArray, self.callback_vel_acc_rot)

        # Publishing the Estimation of the RLS-Algorithm
        self.algorithm = rospy.Publisher("/robot1/algorithm", Float64MultiArray, queue_size=10)

        
        # Startvector of P, Forget Factor Sigma, Mass of object and Vector Qm of unknown object parameters
        self.P_K = np.identity(10) * 100; self.sigma_1 = 1; self.m = 1
        self.Q_m = np.array([[self.m], [0], [0], [0], [0.0], [0.0], [0.0], [0.0],[0.0],[0.0]])

        self.run()

    def run(self):

        self.rate = rospy.Rate(500)
        # Duration of the algorithm
        timeout =time.time() + self.duration

        while not rospy.is_shutdown() and time.time() < timeout:
            
            # Get datas for calculation
            self.get_datas()

            # RLS-Estimation
            self.calculator_Recursive_Least_Squares()

            # Calculating the Center of Mass 
            for l in range(10):
                self.Qm2[l] = self.Q_m[l]
            self.Qm2[1] = self.Q_m[1] / self.m
            self.Qm2[2] = self.Q_m[2] / self.m
            self.Qm2[3] = self.Q_m[3] / self.m
            self.Qm2 = np.array(self.Qm2)
            rospy.loginfo(self.Qm2)
            self.data_to_send.data = self.Qm2
            self.algorithm.publish(self.data_to_send) 
           
    def get_datas(self):
        
        # Calculation of Datamatrix phi 6x10
        self.phi_m  = np.array([[self.vel_acc_rot[6] - self.vel_acc_rot[12], -(self.vel_acc_rot[4]**2) - (self.vel_acc_rot[5]**2), self.vel_acc_rot[3]*self.vel_acc_rot[4] - self.vel_acc_rot[11], self.vel_acc_rot[3]*self.vel_acc_rot[5] + self.vel_acc_rot[10], 0, 0, 0, 0, 0, 0], 
                  [self.vel_acc_rot[7]-self.vel_acc_rot[13], self.vel_acc_rot[3]*self.vel_acc_rot[4] + self.vel_acc_rot[11], -(self.vel_acc_rot[3]**2)-(self.vel_acc_rot[5]**2), self.vel_acc_rot[4]*self.vel_acc_rot[5] - self.vel_acc_rot[9], 0, 0, 0, 0, 0, 0],
                  [self.vel_acc_rot[8]-self.vel_acc_rot[14], self.vel_acc_rot[3]*self.vel_acc_rot[5] - self.vel_acc_rot[10], self.vel_acc_rot[4]*self.vel_acc_rot[5] + self.vel_acc_rot[9], -(self.vel_acc_rot[4]**2)-(self.vel_acc_rot[3]**2), 0, 0, 0, 0, 0, 0],
                  [0, 0, self.vel_acc_rot[8] - self.vel_acc_rot[14], self.vel_acc_rot[13] - self.vel_acc_rot[7], self.vel_acc_rot[9], self.vel_acc_rot[10] - self.vel_acc_rot[3]*self.vel_acc_rot[5], self.vel_acc_rot[11] + self.vel_acc_rot[3]*self.vel_acc_rot[4], -self.vel_acc_rot[4]*self.vel_acc_rot[5], self.vel_acc_rot[4]**2-(self.vel_acc_rot[5]**2), self.vel_acc_rot[4]*self.vel_acc_rot[5]],
                  [0, self.vel_acc_rot[14]-self.vel_acc_rot[8], 0, self.vel_acc_rot[6]-self.vel_acc_rot[12], self.vel_acc_rot[5]*self.vel_acc_rot[3], self.vel_acc_rot[9] + self.vel_acc_rot[4]*self.vel_acc_rot[5], self.vel_acc_rot[5]**2-(self.vel_acc_rot[3]**2), self.vel_acc_rot[10], self.vel_acc_rot[11] - self.vel_acc_rot[3]*self.vel_acc_rot[4], -self.vel_acc_rot[3]*self.vel_acc_rot[5]],
                  [0, self.vel_acc_rot[7]-self.vel_acc_rot[13], self.vel_acc_rot[12]-self.vel_acc_rot[6], 0, -self.vel_acc_rot[3]*self.vel_acc_rot[4], self.vel_acc_rot[3]**2-(self.vel_acc_rot[4]**2), self.vel_acc_rot[9] - self.vel_acc_rot[4]*self.vel_acc_rot[5], self.vel_acc_rot[3]*self.vel_acc_rot[4], self.vel_acc_rot[10] + self.vel_acc_rot[3]*self.vel_acc_rot[5], self.vel_acc_rot[11]]])
        
        # Calculation of estimated wrench vector R1_u_Object with the Graspmatrix and the measured wrenches of Robot1 and Robot2
        self.R1_u0 = self.Grasp @ np.array([[self.wrench1[0]], [self.wrench1[1]], [self.wrench1[2]], [self.wrench1[3]], [self.wrench1[4]], [self.wrench1[5]], [-self.wrench2[0]], [-self.wrench2[1]], [self.wrench2[2]], [-self.wrench2[3]], [-self.wrench2[4]], [self.wrench2[5]]])
        
        # Calculation of the estimated wrench vector R1_u_Object with the datamatrix phi and the vector of unknown object parameters Qm
        self.R1_u0_dach = self.phi_m @ self.Q_m
                
    def calculator_Recursive_Least_Squares(self):

        # Calculating of the Gain Matrix K_K
        self.K_K = self.P_K @ self.phi_m.T @ (np.linalg.inv(np.identity(6)*(self.sigma_1) + self.phi_m @ self.P_K @ self.phi_m.T))
        
        # Calculating of the current Qm
        self.Q_m = self.Q_m + self.K_K @ (self.R1_u0 - self.R1_u0_dach)
        
        # Calculating of P_K
        self.P_K = (1/self.sigma_1) * (self.P_K - self.K_K @ self.phi_m @ self.P_K)

    def callback_wrench(self,msg):

        self.wrench1 = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        
    def callback_wrench2(self,msg):

        self.wrench2 = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

    def callback_vel_acc_rot(self,msg):

        self.vel_acc_rot=msg.data
        
        
if __name__=="__main__":
    RLS_algorithm()