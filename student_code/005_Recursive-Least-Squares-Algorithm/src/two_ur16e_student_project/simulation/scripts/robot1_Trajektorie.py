#!/usr/bin/env python3 
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
from tf.transformations import  euler_from_quaternion, euler_matrix 
import time
import numpy as np


Pi = pi

class inv_diff_kinematics():

    def __init__(self):
        rospy.init_node("robot1_trajectory")

        # Initialization of Joint_velocity
        self.dq2 = Float64MultiArray(); self.dq2.data = np.zeros(6); self.dq = np.zeros(6)
        
        # Initialization of current TCP-Position of Robot1 
        self.tcp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Initialization of the coefficients for trajectory a0 to a5 (translation) and b0 to b5 (rotation)
        self.a0 = [0.0, 0.0, 0.0]; self.a1 = [0.0, 0.0, 0.0]; self.a2 = [0.0, 0.0, 0.0]; self.a3 = [0.0, 0.0, 0.0]; self.a4 = [0.0, 0.0, 0.0]; self.a5 = [0.0, 0.0, 0.0]
        self.b0 = [0.0, 0.0, 0.0, 0.0]; self.b1 = [0.0, 0.0, 0.0]; self.b2 = [0.0, 0.0, 0.0]; self.b3 = [0.0, 0.0, 0.0]; self.b4 = [0.0, 0.0, 0.0]; self.b5 = [0.0, 0.0, 0.0]
        
        # Initialization of end position and end orientation
        self.end_pos = [0.0, 0.0, 0.0]; self.eulerZiel = [0.0, 0.0, 0.0]
        
        # Initialization of position, velocity and acceleration of the trajectory
        self.pos= [0.0, 0.0, 0.0]; self.vel = [0.0, 0.0, 0.0]; self.acc= [0.0, 0.0, 0.0]
        
        # Initialization of angle, angular velocity and angular acceleration of the trajectory
        self.posQ= [0.0, 0.0, 0.0]; self.velQ = [0.0, 0.0, 0.0]; self.accQ= [0.0, 0.0, 0.0]
        
        # Initialization of calculation parameters for trajectory planning
        self.vel1 = [0.0, 0.0, 0.0]; self.acc1 = [0.0, 0.0, 0.0]; self.velQ1 = [0.0, 0.0, 0.0]
        
        # Initialization of trajectory duration and current time, def_starttime is the current time at the beginning, amount of cycles 
        self.duration = 5; self.def_starttime = 0; self.current_time2 = 0; self.cycles = 4
        
        # Initialization of jacobian matrix
        self.jm = np.zeros((6,6))
        
        # Initialization of gravity vector depending on TCP orientation
        self.gravity = np.array([[0], [0], [-9.81], [1]]); self.rotation_gravity = np.array([[0],[0],[0], [0]])
        
        # Initialization of motion cycles
        self.z = 0
        
        # Initialization of the rotation angle of the object coordinate system, viewed from Robot1
        self.xx = pi/64
        self.yy = pi/8
        self.zz = pi/32
        
        # Initialization of geometrical distance between TCP of Robot1 and Robot2
        self.distance = 0.81
        
        # Subscription von Jointstates, jacobian matrix, position, TCP-position
        rospy.Subscriber("/robot1/joint_states", JointState, self.cb_joint_states)
        rospy.Subscriber('/robot1/jacobian', Float64MultiArray, self.cb_jacobi)
        rospy.Subscriber('/robot1/TCP_robot1', Float64MultiArray, self.cb_tcp)
        
        # Publisher for joint velocities, end effector velocity and acceleration as well as gravitational force
        self.pub_joint_velocity = rospy.Publisher("/robot1/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        self.pub_vel_acc_rot = rospy.Publisher("/robot1/vel_acc_rot", Float64MultiArray, queue_size=10)
        self.run()
    
    def run(self):
        rate = rospy.Rate(500)
        
        # Total time of the trajectory
        timeout =time.time() + self.cycles*self.duration

        while not rospy.is_shutdown() and time.time() < timeout:
            
            # If loop for determining the correct trajectory at the necessary point in time
            if self.current_time <= self.duration:
                
                # Determination of the trajectory parameters: actual position and orientation, target position and orientation
                if self.z==0:
                    rospy.wait_for_message("/robot1/TCP_robot1", Float64MultiArray)
                    self.a0 = [self.tcp[0], self.tcp[1], self.tcp[2]]
                    self.b0 = [self.tcp[3], self.tcp[4], self.tcp[5], self.tcp[6]]
                    self.start_pos_rot = [self.a0[0], self.a0[1], self.a0[2], self.b0[0], self.b0[1], self.b0[2], self.b0[3]]
                    self.b0 = euler_from_quaternion(self.b0)
                    self.position1()
                    self.z += 1
                # Loop for trajectory calculation and calculation of TCP velocities for the calculation of inverse differential kinematics
                self.controller(); self.get_inv_jacobian(); self.get_inv_diff_kin()
                
            elif self.current_time > self.duration and self.current_time <= self.duration *2:
                if self.z==1:
                    rospy.wait_for_message("/robot1/TCP_robot1", Float64MultiArray)
                    self.a0 = [self.tcp[0], self.tcp[1], self.tcp[2]]
                    self.b0 = [self.tcp[3], self.tcp[4], self.tcp[5], self.tcp[6]]
                    self.b0 = euler_from_quaternion(self.b0)
                    self.position0()
                    self.z += 1
                self.current_time2 = self.current_time - self.duration
                self.controller(); self.get_inv_jacobian(); self.get_inv_diff_kin()

            elif self.current_time >self.duration *2 and self.current_time <=self.duration *3:
                if self.z==2:
                    rospy.wait_for_message("/robot1/TCP_robot1", Float64MultiArray)
                    self.a0 = [self.tcp[0], self.tcp[1], self.tcp[2]]
                    self.b0 = [self.tcp[3], self.tcp[4], self.tcp[5], self.tcp[6]]
                    self.b0 = euler_from_quaternion(self.b0)
                    self.position1()
                    self.z += 1
                self.current_time2 = self.current_time - (self.duration *2)
                self.controller(); self.get_inv_jacobian(); self.get_inv_diff_kin()
                
            elif self.current_time > self.duration *3 and self.current_time <=self.duration *4:
                if self.z==3:
                    rospy.wait_for_message("/robot1/TCP_robot1", Float64MultiArray)
                    self.a0 = [self.tcp[0], self.tcp[1], self.tcp[2]]
                    self.b0 = [self.tcp[3], self.tcp[4], self.tcp[5], self.tcp[6]]
                    self.b0 = euler_from_quaternion(self.b0)                    
                    self.position0()
                    self.z +=1
                self.current_time2 = self.current_time - self.duration *3
                self.controller(); self.get_inv_jacobian(); self.get_inv_diff_kin()
                
                
                
            # Publish current joint velocities
            self.pub_joint_velocity.publish(self.dq2)
            self.current_time2 = self.current_time
            
            # Publish TCP-velocity, TCP-acceleration and current gravitational force relative to the rotation of the TCP for the algorithm.
            self.combined = [self.vel[0], self.vel[1], self.vel[2], self.velQ[0], self.velQ[1], self.velQ[2], self.acc[0], self.acc[1], self.acc[2], self.accQ[0], self.accQ[1], self.accQ[2], -self.rotation_gravity[0], -self.rotation_gravity[1], -self.rotation_gravity[2]]
            data_to_send = Float64MultiArray()
            data_to_send.data = self.combined
            self.pub_vel_acc_rot.publish(data_to_send) 
            rate.sleep()
        
        # Final publishing of joint velocities to safely stop robots after time expires
        self.dq2.data = [0.0,0.0,0.0,0.0,0.0,0.0]   
        self.pub_joint_velocity.publish(self.dq2)

        
    def controller(self):
        # Calculation of the Trajectory: pos = position posQ = orientation
        for i in range (3):
            self.posQ[i] = self.b0[i] + self.b1[i] * self.current_time2 + self.b2[i] * self.current_time2 ** 2 + self.b3[i] * self.current_time2 ** 3 + self.b4[i] * self.current_time2 ** 4 + self.b5[i] * self.current_time2 ** 5
            self.pos[i] = self.a0[i] + self.a1[i] * self.current_time2 + self.a2[i] * self.current_time2 ** 2 + self.a3[i] * self.current_time2 ** 3 + self.a4[i] * self.current_time2 ** 4 + self.a5[i] * self.current_time2 ** 5
            self.vel[i] = self.a1[i] + 2*self.a2[i]*self.current_time2 + 3*self.a3[i] *self.current_time2 **2 + 4*self.a4[i] *self.current_time2 **3 + 5*self.a5[i] *self.current_time2 **4
            self.velQ[i] = self.b1[i] + 2*self.b2[i]*self.current_time2 + 3*self.b3[i] *self.current_time2 **2 + 4*self.b4[i] *self.current_time2 **3 + 5*self.b5[i] *self.current_time2 **4
            self.acc[i] = 2*self.a2[i] + 6*self.a3[i]*self.current_time2 + 12*self.a4[i]*self.current_time2**2 + 20*self.a5[i]*self.current_time2**3 
            self.accQ[i] = 2*self.b2[i] + 6*self.b3[i]*self.current_time2 + 12*self.b4[i]*self.current_time2**2 + 20*self.b5[i]*self.current_time2**3                                                                                       
        
        # Calculation of the current gravity vector depending on the actual TCP-rotation
        k0 = [self.tcp[3], self.tcp[4], self.tcp[5], self.tcp[6]]
        k0 = euler_from_quaternion(k0)
        self.euler_matrix = euler_matrix(k0[0], k0[1], k0[2])
        self.rotation_gravity = self.euler_matrix @ self.gravity
        
        # Vector of specified velocity of the TCP (translation and rotation)
        self.dxE2 =np.array([self.vel[0], self.vel[1], self.vel[2], self.velQ[0], self.velQ[1], self.velQ[2]])

    # Calculation of the inverse jacobian matrix
    def get_inv_jacobian(self):
        self.jm_inverse = np.linalg.pinv(self.jm)

    # Calculation of the inverse differential kinematic
    def get_inv_diff_kin(self):
        self.dq = np.matmul(self.jm_inverse, np.transpose(self.dxE2))
        self.dq2.data = (np.asarray(self.dq)).flatten()
        
    # Callback of current Jointstate TimeStamp 
    def cb_joint_states(self,data):
        self.q_act = data.position
        # definition of start time to get the timedifference
        if self.def_starttime == 0:
            self.start_time = data.header.stamp.to_sec()
            self.def_starttime += 1
        # get the current time
        self.current_time = data.header.stamp.to_sec() - self.start_time
    
    # Callback of current jacobian matrix   
    def cb_jacobi(self, msg):  
        a = np.array(msg.data, dtype=np.float64)
        self.jm = a.reshape(6,6)

    # Callback of current TCP-Position and TCP-Orientation
    def cb_tcp(self, data):
        self.tcp = data.data

    # Calculation of the start position and orientation of the TCP
    def position0(self):

        # Definition of the endposition and endorientation /startposition and startorientation from the beginning/ the startorientation is the current orientation plus the rotation angle
        end_pos = [self.start_pos_rot[0], self.start_pos_rot[1], self.start_pos_rot[2]]
        eulerStart = [self.b0[0], self.b0[1], self.b0[2]]
        eulerGoal = [self.b0[0]+self.xx, self.b0[1]+self.yy, self.b0[2]+self.zz]
        
        # Calculation of the coefficients for the trajectory
        for j in range(3):
            self.a3[j] = (10 * (end_pos[j] - self.a0[j])) / (self.duration ** 3)
            self.a4[j] = -(15 * (end_pos[j] - self.a0[j])) / (self.duration ** 4)
            self.a5[j] = (6 * (end_pos[j] - self.a0[j])) / (self.duration ** 5)
            
            self.b3[j] = (10 * (eulerGoal[j] - eulerStart[j])) / (self.duration ** 3)
            self.b4[j] = -(15 * (eulerGoal[j] - eulerStart[j])) / (self.duration ** 4)
            self.b5[j] = (6 * (eulerGoal[j] - eulerStart[j])) / (self.duration ** 5)
    
    # Calculation of the goal position and orientation of the TCP
    def position1(self):

        # Calculation of the transformation matrix from the base to the TCP
        x_quat_rot = [self.start_pos_rot[3], self.start_pos_rot[4], self.start_pos_rot[5], self.start_pos_rot[6]]
        x_euler_rot = euler_from_quaternion(x_quat_rot)

        transformation_base_tcp = euler_matrix(x_euler_rot[0], x_euler_rot[1], x_euler_rot[2])
        transformation_base_tcp [0][3] = self.start_pos_rot[0]
        transformation_base_tcp [1][3] = self.start_pos_rot[1]
        transformation_base_tcp [2][3] = self.start_pos_rot[2]

        # Calculation of the transformation matrix from the TCP to the object
        transformation_tcp_object = np.eye(4)
        transformation_tcp_object [0][3] = self.distance / 2           
        transformation_base_object = transformation_base_tcp @ transformation_tcp_object
        
        # Calculation of the transformation matrix from the object to the endposition
        transformation_object_endpos = euler_matrix(-self.yy,-self.xx,-self.zz)
        transformation_object_endpos_2  = np.eye(4)
        transformation_object_endpos_2 [0][3] = -(self.distance / 2)
        transformation_object_endpos = transformation_object_endpos @ transformation_object_endpos_2
        
        # Calculation of the transformation matrix from the base to the endposition
        x_endpos = transformation_base_object @ transformation_object_endpos

        # Definition of the endposition and endorientation
        end_pos = [x_endpos[0][3] , x_endpos[1][3] ,x_endpos[2][3]]
        eulerStart = [self.b0[0], self.b0[1], self.b0[2]]
        eulerGoal = [self.b0[0]-self.xx, self.b0[1]-self.yy,self.b0[2]-self.zz]
        
        # Calculation of the coefficients for the trajectory
        for j in range(3):
            self.a3[j] = (10 * (end_pos[j] - self.a0[j])) / (self.duration ** 3)
            self.a4[j] = -(15 * (end_pos[j] - self.a0[j])) / (self.duration ** 4)
            self.a5[j] = (6 * (end_pos[j] - self.a0[j])) / (self.duration ** 5)
            
            self.b3[j] = (10 * (eulerGoal[j] - eulerStart[j])) / (self.duration ** 3)
            self.b4[j] = -(15 * (eulerGoal[j] - eulerStart[j])) / (self.duration ** 4)
            self.b5[j] = (6 * (eulerGoal[j] - eulerStart[j])) / (self.duration ** 5)
        
if __name__=="__main__":
    inv_diff_kinematics()