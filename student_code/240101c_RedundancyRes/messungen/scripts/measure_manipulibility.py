from typing import Optional

import numpy as np
import rospy
# import moveit_commander
from match_lib.match_robots import Joints
from std_msgs.msg import Float64
from match_lib.robot_mats.jacobians.jacobian_ur_16_eef import getJacobianUr16_base_link_inertiaUr16_wrist_3_link as jacobian_ur_16_eef


def calc_manipulability(q, group = None):
    if group:
        jacob = group.get_jacobian_matrix(list(q))
        jacob[0:2,:]=-1*jacob[0:2,:]
        jacob[3:5,:]=-1*jacob[3:5,:]
    else:
        jacob = jacobian_ur_16_eef(q)
        
    m = np.linalg.det(jacob @ jacob.T)
    return m
    

if __name__ == "__main__":
    rospy.set_param('/use_sim_time', True)
    rospy.init_node('measure_manipulibility')

    pub = rospy.Publisher("manipulability", Float64, queue_size=1)
    ns="/mur"
    description="/mur/ur/robot_description"
    group_name="manipulator"
    prefix = rospy.get_param("prefix_ur", default="ur/")
    ns_prefix = ns+"/"+prefix

    # if not from bag:
    # group = moveit_commander.MoveGroupCommander(group_name, ns=ns, robot_description=description,wait_for_servers=5.0)
    group=None
    
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    j = Joints(ns=ns_prefix, joint_names=joint_names)
    # q = j.q

    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        q = j.q
        m = calc_manipulability(q, group)
        pub.publish(m)
        rate.sleep()

