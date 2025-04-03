#!/usr/bin/env python3
# coding=latin-1

# see also: http://docs.ros.org/en/hydro/api/orocos_kdl/html/classKDL_1_1ChainDynParam.html#a616876214cb26793cedeed01e5093b3a

from typing import Optional
import PyKDL as kdl
from pykdl_utils.kdl_kinematics import joint_list_to_kdl
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import Robot
import numpy as np
# from simple_pid import PID

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

import rospy

class RobotKinDyn(object):
    def __init__(self, base_link: str, end_link: str, urdf_file_name: Optional[str], urdf_string: Optional[str]) -> None:
        if urdf_file_name is not None:
            with open(urdf_file_name, 'r') as f:
                urdf_string = f.read()
        else:
            if urdf_string is None:
                raise ValueError('urdf_file_name and urdf_string are None')

        robot = Robot.from_xml_string(urdf_string)
        self.kdl_tree = kdl_tree_from_urdf_model(robot)
        self.chain = self.kdl_tree.getChain(base_link, end_link)
        self.dyn_kdl = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())
        self.num_joints = self.chain.getNrOfJoints()
        self.kdl_inertia = kdl.JntSpaceInertiaMatrix(self.num_joints)
        self.coriolis = kdl.JntArray(self.num_joints)
        
    def get_jacobian(self, joint_positions: np.ndarray) -> np.ndarray:
        """ Returns the jacobian of the end effector in the base frame. TODO: check if this is correct """
        jacobian = kdl.Jacobian(self.chain.getNrOfJoints())
        joint_positions_kdl = joint_list_to_kdl(joint_positions)
        self.kdl_tree.getChain(self.base_link, self.end_link).JntToJac(joint_positions_kdl, jacobian)
        return jacobian.data
    
    def calc_mass_matrix(self, joint_positions: np.ndarray) -> np.ndarray:
        """ Returns the mass matrix of the robot in the base frame """
        if len(joint_positions) != self.num_joints:
            raise ValueError(f'joint_positions has wrong size! Correct size is {self.num_joints}')
        self.dyn_kdl.JntToMass(joint_list_to_kdl(joint_positions), self.kdl_inertia)
        mass_mat = np.array([[self.kdl_inertia[row, column] for row in range(self.kdl_inertia.rows())]
                             for column in range(self.kdl_inertia.columns())])
        return mass_mat
    
    def calc_coriolis_matrix(self, joint_positions: np.ndarray, joint_velocities: np.ndarray) -> np.ndarray:
        """ Returns the coriolis matrix of the robot in the base frame """
        if len(joint_positions) != self.num_joints or len(joint_velocities) != self.num_joints:
            raise ValueError(f'joint_positions or velocities has wrong size! Correct size is {self.num_joints}')
        
        self.dyn_kdl.JntToCoriolis(joint_list_to_kdl(joint_positions), joint_list_to_kdl(joint_velocities), self.coriolis)
        return np.array([self.coriolis[row] for row in range(self.coriolis.rows())])
    
class VelocityObserverMiR:
    def __init__(self, k_p: np.ndarray, k_d: np.ndarray, init_pose: Pose = Pose(), max_vel=1.0) -> None:
        self.max_vel = max_vel
        self.old_time = None
        self.dt = 0.0 # is set by setting v_odom
        self.pose_by_integration = self.pose_to_np(init_pose) #x,y,theta
        self._v_hat = np.zeros(3) #x',y',theta'
        self.k_p = np.array(k_p)
        self.k_d = np.array(k_d)
        self.pose_error_odom_last = np.zeros(3)
        
        # set before calc_velocity is called
        self._v_odom = np.zeros(3) #x',y',theta'
        self._pose_actual = self.pose_by_integration  #x,y,theta
    
    def _calc_velocity(self) -> np.ndarray:
        """calculates the velocity v_hat of the robot based on the odometry and the actual pose.
        velocity is rotated by -theta of actual pose and integrated to get the pose.
        v_hat is then calculated by using the difference between the actual pose and the pose by integration additionally to v_odom.
        """
        # or use IntegralTrapez?
        # self.pose_by_integration = self.pose_by_integration + self.dt * self._v_hat
        th = self._pose_actual[2] # oder self.pose_by_integration[2]?
        delta_x = (self._v_hat[0] * np.cos(th) - self._v_hat[1] * np.sin(th)) * self.dt
        delta_y = (self._v_hat[0] * np.sin(th) + self._v_hat[1] * np.cos(th)) * self.dt
        delta_th = self._v_hat[2] * self.dt
        self.pose_by_integration += np.array([delta_x, delta_y, delta_th])
        pose_error = self._pose_actual - self.pose_by_integration
        rospy.logdebug(f'VelObserver pose_error: {pose_error}')
        # in odom frame
        pose_error_odom = np.array([pose_error[0] * np.cos(th) + pose_error[1] * np.sin(th),
                                    -pose_error[0] * np.sin(th) + pose_error[1] * np.cos(th),
                                    pose_error[2]])
        # Todo: add damped influence of v_odom: v_hat_odom = v_odom + k_d * (v_odom_last - v_odom) / dt
        v = self._v_odom + self.k_p*pose_error_odom + self.k_d * (pose_error_odom - self.pose_error_odom_last) / self.dt
        v_max = v.max()
        if v_max<self.max_vel:
            self._v_hat = v
        else:
            self._v_hat = self.max_vel * v / v_max
    
    def pose_to_np(self, pose: Pose) -> np.ndarray:
        # th = transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
        theta = -2*np.arccos(pose.orientation.w)
        return np.array([pose.position.x, pose.position.y, theta])
     
    @property
    def v_hat(self) -> np.ndarray:
        # return self._v_hat[[0,2]] #x',theta'
        return self._v_hat
        
    @property
    def pose_actual(self) -> np.ndarray:
        return self._pose_actual
    
    @pose_actual.setter
    def pose_actual(self, pose_actual: Pose) -> None:
        self._pose_actual = self.pose_to_np(pose_actual)
        
        
    @property
    def v_odom(self) -> np.ndarray:
        return self._v_odom
    
    @v_odom.setter
    def v_odom(self, v_odom: Odometry) -> None:
        msg_time = v_odom.header.stamp
        try:
            self.dt = (msg_time - self.old_time).to_sec()
            self.old_time = msg_time
            # umrechnen in x',y',theta':
            self._v_odom = np.array([v_odom.twist.twist.linear.x,0,v_odom.twist.twist.angular.z])
            self._calc_velocity()
        except TypeError: # first time
            self.old_time = msg_time
        
    
    
if __name__ == '__main__':
    base_link, end_link = "base_footprint", "UR16/wrist_3_link"
    urdf_file_name = "/home/rosmatch/Hee/catkin_hee/src/nullraum_sim/urdf/mur_with_mir_joints.urdf"
    f = open(urdf_file_name, 'r')
    robot = Robot.from_xml_string(f.read())
    f.close()
    
    q = np.array([0.0]*8)
    q_dot = np.array([1.0]*8) # aus joint_states: velocity und u_p
    
    dyn = RobotKinDyn(base_link, end_link, urdf_file_name, None)
    # print(dyn.get_jacobian(q))
    print(dyn.calc_mass_matrix(q))
    print(dyn.calc_coriolis_matrix(q, q_dot))
    
    # kdl_tree = kdl_tree_from_urdf_model(robot)
    # chain = kdl_tree.getChain("base_footprint", "UR16/wrist_3_link")


    # dyn_kdl = kdl.ChainDynParam(chain, kdl.Vector.Zero())
    # h_kdl = kdl.JntSpaceInertiaMatrix(8)
    # dyn_kdl.JntToMass(joint_list_to_kdl(q), h_kdl)

    # print(f"MassMatrix: {h_kdl}")
    # # print(kdl_to_mat(h_kdl)) # kdl_to_mat wahrscheinlich sehr langsam programmiert
    # mat = np.array([[h_kdl[row, column] for row in range(h_kdl.rows())] for column in range(h_kdl.columns())])
    # print(mat)

    # coriolis = kdl.JntArray(8)
    # dyn_kdl.JntToCoriolis(joint_list_to_kdl(q), joint_list_to_kdl(q_dot), coriolis)
    # print(f"Coriolis: {coriolis}")