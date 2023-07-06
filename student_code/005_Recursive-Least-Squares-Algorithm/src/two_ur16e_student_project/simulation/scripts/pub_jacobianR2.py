#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

class PubJacobian(object):

    def __init__(self, dim=(6,6)):
       
        self.layout = MultiArrayLayout()
        self.layout.dim.append(MultiArrayDimension())
        self.layout.dim[0].label = "0"
        self.layout.dim[0].size = dim[0]
        self.layout.dim.append(MultiArrayDimension())
        self.layout.dim[1].label = "1"
        self.layout.dim[1].size = dim[1]


        self.group = moveit_commander.MoveGroupCommander("UR16e_arm2")
        rospy.Subscriber('/robot2/joint_states', JointState, self.joint_states_cb)
        self.pub_jacobi = rospy.Publisher("/robot2/jacobian", Float64MultiArray, queue_size=10)

    def joint_states_cb(self, data):  
        joint_angular = data
        joint_states = [joint_angular.position[2], joint_angular.position[1], joint_angular.position[0], joint_angular.position[3], joint_angular.position[4], joint_angular.position[5]]
        
        jm = np.array(self.group.get_jacobian_matrix(joint_states), dtype=np.float64)
        jm_flattend = jm.reshape(-1)


        data_to_send = Float64MultiArray()
        data_to_send.data = jm_flattend
        data_to_send.layout = self.layout

        self.pub_jacobi.publish(data_to_send)
        print(data_to_send)
        
if __name__ == "__main__":
    rospy.init_node("pub_jacobian_urR2")
    jac = PubJacobian()
    rospy.spin()
