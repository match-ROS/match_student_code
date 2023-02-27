#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Callable, Optional, Tuple
from cmath import isnan
import numpy as np
from scipy.optimize import minimize
# from match_lib.robot_mats.jacobians.jacobian_ur_16_eef import getJacobianUr16_base_link_inertiaUr16_wrist_3_link as getJacobianManipulator
# from match_lib.robot_mats.jacobians.jacobian_jacobian_T import jacobian_jt
from match_lib.robot_mats.jacobians.jacobian_platform import getJacobianPlatform, getJacobianPlatformWithEEF, getRotationMatrixS3
from match_lib.robot_mats.transformations.transform_ur16_base_link_eef import getTransform, getVector_eef
from match_lib.match_robots import Joints
from match_lib.robot_kin_dyn import RobotKinDyn, VelocityObserverMiR
import numdifftools as nd
from math import pi

import sys
#from match_lib.gazebo import GazeboVelocity, GazeboPose
# from match_lib.match_geometry import rotateVector
from match_lib.filter import MovingAvg
from geometry_msgs.msg import Twist, TwistStamped, TwistWithCovarianceStamped, Vector3, Vector3Stamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
import moveit_commander
import tf
from tf import transformations
# from simple_pid import PID 

Twist.__neg__ = lambda self: Twist(linear=Vector3(self.linear.x * -1, self.linear.y * -1, self.linear.z * -1),angular=Vector3(self.angular.x * -1, self.angular.y * -1, self.angular.z * -1))
Vector3.__add__ = lambda self, other: Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
Vector3.__sub__ = lambda self, other: Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

class UR():
    def __init__(self,ns="",description="mur216/robot_description", prefix="UR16/", tf=None, T=getTransform, base_name="/mur216/base_footprint", ur_base_name="/mur216/UR16/base_link", group_name="UR_arm"):
        # group_name = 'UR_arm'
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(group_name, ns=ns, robot_description=description,wait_for_servers=5.0)
        if ns is not None:
            prefix = ns + "/"+ prefix
        self.pub = rospy.Publisher('cooperative_manipulation/cartesian_velocity_command', Twist, queue_size=1)
        self.T = T

        if tf==None:
            tf_listener = tf.TransformListener()
        else:
            tf_listener = tf
        tf_listener.waitForTransform(base_name, ur_base_name, rospy.Time(), rospy.Duration(5.0))
        self.ur_base_pos = Vector3(*tf_listener.lookupTransform(base_name, ur_base_name, rospy.Time(0))[0])

    def getTransformation(self, q):
        return self.T(q)

    def setCartesianVelocity(self, v: Optional[np.ndarray] = None):
        v = np.zeros(6) if v is None else v
        if max(abs(v)) > 0.2:
            rospy.logerr("Velocity too high!. Max velocity is 0.2 m/s but got {}".format(v))
            rospy.signal_shutdown("Cartewsian Velocity was too high")
        self.pub.publish(Twist(linear=Vector3(*v[0:3]), angular=Vector3(*v[3:6])))
        # rospy.logdebug("setCartesianVelocity: {}".format(v))

class MiR():
    def __init__(self, gazebo_model="mur216", prefix_mir="mir/"):
        self.v_odom = Twist()
        # self.v_odom_stamped = TwistWithCovarianceStamped()
        # self.pub = rospy.Publisher(prefix_mir+'mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.pub = rospy.Publisher(prefix_mir+'cmd_vel', Twist, queue_size=1)
        
        init_pose=rospy.wait_for_message(prefix_mir+'robot_pose', Pose)
        rospy.logdebug("init_pose: {}".format(init_pose))
        rospy.loginfo("init_pose.position: {}".format(init_pose.position))

        # self.velocity_observer = VelocityObserverMiR([0.01,0.01,0.01], [0.01,0.01,0.01], init_pose=init_pose)
        self.velocity_observer = VelocityObserverMiR([0.0,0.0,0.0], [0.0,0.0,0.0], init_pose=init_pose) #TEST
        self.odom_sub = rospy.Subscriber(prefix_mir+'odom', Odometry, self.odom_callback)
        self.robot_pose_sub = rospy.Subscriber(prefix_mir+'robot_pose', Pose, self.pose_callback)

        self.theta = 2*np.arccos(init_pose.orientation.w)
        self.rotM = getRotationMatrixS3(0.0)
        self.quaternion = (0.0, 0.0, 0.0, 1.0)
        rospy.Subscriber(prefix_mir+'amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
    
    def set_vel_cmd(self, v=np.array([0.0, 0.0])):
        """Publishes velocity command to the robot. Restricts the velocity to the maximum velocity.

        Args:
            v (np.array[2]): [v_x_translational, v_z_angular]. Defaults to [0.0, 0.0].
        """
        # if v[0]>v_max[0]:
        #     v*=v_max[0]/v[0]
        #     rospy.loginfo("v_x was too high. Restricting to v_x_max = {}".format(v_max[0]))
        # if v[1]>v_max[1]:
        #     v*=v_max[1]/v[1]
        #     rospy.loginfo("v_theta was too high. Restricting to v_theta_max = {}".format(v_max[1]))
        self.pub.publish(Twist(linear=Vector3(v[0], 0.0, 0.0), angular=Vector3(0.0, 0.0, v[1])))

    def restrict_vel(self, v: np.ndarray, v_min: np.ndarray, v_max: np.ndarray):
        v_abs=abs(v)
        if v_abs[0] < v_min[0] and v_abs[1] < v_min[1]:
                v[0], v[1] = 0.0, 0.0
        
        if v_abs[0]>v_max[0]:
            v*=v_max[0]/v_abs[0]
            rospy.loginfo("v_x was too high. Restricting to v_x_max = {}".format(v_max[0]))            
        if v_abs[1]>v_max[1]:
            v*=v_max[1]/v_abs[1]
            rospy.loginfo("v_theta was too high. Restricting to v_theta_max = {}".format(v_max[1]))
        return v

    def get_cur_velocity(self):
        """in base_footprint frame (odom or rotated map)

        Returns:
            Twist:
        """
        return self.v_odom

    def get_cur_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """Gets velocity from odom and velocity observer

        Returns:
            Tuple[np.ndarray[2], np.ndarray[3]]: [v_odom, v_hat]
        """
        cur_vel = self.get_cur_velocity()
        v_odom = np.array([cur_vel.linear.x, cur_vel.angular.z])
        return (v_odom, self.velocity_observer.v_hat)

    def odom_callback(self, msg=Odometry()):
        self.v_odom = msg.twist.twist
        self.velocity_observer.v_odom = msg
        
    def pose_callback(self, msg=Pose()):
        self.velocity_observer.pose_actual = msg
        self.calc_rotM(msg.orientation)

    def get_gazebo_velocity(self):
        return self.gazebo_velocity.velocity_stamped.twist

    def calc_rotM(self, quaternion):
        self.quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.theta = transformations.euler_from_quaternion(self.quaternion)[2]
        self.rotM = getRotationMatrixS3(self.theta)
        rospy.logdebug("theta: {}".format(self.theta))

    def amcl_callback(self, msg=PoseWithCovarianceStamped()):
        self.calc_rotM(msg.pose.pose.orientation)


def manipulability_pos(q, group):
    """For positional manipulability (splitted in position and orientation)

    Args:
        q (np.ndarray): joint positions

    Returns:
        float: det(jacob_p@jacob_p.T)
    """
    jacob = group.get_jacobian_matrix(list(q))[:3,:]
    jacob[0:2,:]=-1*jacob[0:2,:]
    # jacob[3:5,:]=-1*jacob[3:5,:]
    m = np.linalg.det(jacob @ jacob.T)
    return m

def manipulability_o(q, group):
    """For orientational manipulability (splitted in position and orientation)

    Args:
        q (np.ndarray): joint positions

    Returns:
        float: det(jacob_o@jacob_o.T)
    """
    jacob = group.get_jacobian_matrix(list(q))[3:,:]
    # jacob[0:2,:]=-1*jacob[0:2,:]
    jacob[3:5,:]=-1*jacob[3:5,:]
    m = np.linalg.det(jacob @ jacob.T)
    return m

def manipulability(q, group):
    """For overall manipulability

    Args:
        q (np.ndarray): joint positions

    Returns:
        float: det(jacob@jacob.T)
    """
    jacob = group.get_jacobian_matrix(list(q))
    jacob[0:2,:]=-1*jacob[0:2,:]
    jacob[3:5,:]=-1*jacob[3:5,:]
    m = np.linalg.det(jacob @ jacob.T)
    return m

def manipulability_min_val(q, group):
    """For disatance to nearest singularity

    Args:
        q (np.ndarray): joint positions

    Returns:
        float: smallest singular value
    """
    jacob = group.get_jacobian_matrix(list(q))
    jacob[0:2,:]=-1*jacob[0:2,:]
    jacob[3:5,:]=-1*jacob[3:5,:]
    jjt = jacob @ jacob.T
    s_min = np.linalg.svd(jjt, compute_uv=False, hermitian=True).min()
    return s_min

def distance_toq_goal(q, q_goal):
    """Calculates the distance between the current joint configuration and the goal joint configuration.

    Args:
        q (np.array): current joint configuration
        q_goal (np.array): goal joint configuration. np.nan if no goal is set.

    Returns:
        float: Norm of q-q_goal
    """
    dist = q-q_goal
    dist[isnan(dist)]=0
    return -np.linalg.norm(dist)

def distance_toq_goal_grad(q, q_goal):
    """Calculates the gradient of the distance to the goal.

    Args:
        q (np.array): current joint configuration
        q_goal (np.array): goal joint configuration. np.nan if no goal is set.

    Returns:
        np.array: gradient of distance to goal
    """
    grad=(q_goal-q)
    grad[np.isnan(grad)]=0
    return grad

class JointLimits:
    """https://doi.org/10.1007/s10846-018-0964-8: Incorporation of Physical Constraints
    """
    def __init__(self, q_min: np.ndarray, q_max: np.ndarray, gamma: Optional[np.ndarray]=None):
        """Cost function for joint limits. And modification of MassMatrix.

        Args:
            q_min (np.ndarray): minimal joint limits
            q_max (np.ndarray): maximal joint limits
            gamma (Optional[np.ndarray], optional): Weighting of the joint limits. Smaller values mean more importance. Defaults to [1,...].
        """
        self.q_min = q_min
        self.q_max = q_max
        self.q_range = q_max-q_min
        if gamma is None:
            self.gamma = np.ones_like(q_min)
        else:
            self.gamma = gamma
        self.cost = np.zeros_like(q_min)

    def check_limits(self, q):
        if np.any(q<self.q_min) or np.any(q>self.q_max):
            return False
        return True

    def check_limits_grad(self, q):
        grad = np.zeros(q.shape)
        grad[q<self.q_min] = -1
        grad[q>self.q_max] = 1
        return grad
    def cost_joint_limits(self, q: np.ndarray)->np.ndarray:
        """Calculates the cost in dependence of the joint limits. Cost rises to infinity near joint limits.

        Args:
            q (np.array): current joint configuration

        Returns:
            np.array: cost of the joint limits
        """
        cost = self.q_range*(2*q-self.q_min-self.q_max)/((self.q_max-q)**2*(q-self.q_min)**2*self.gamma)
        # cost[np.isnan(cost)] = 0
        cost=np.nan_to_num(cost)
        # rospy.logdebug("cost_joint_limits: {}".format(cost))
        return cost

    def cost_joint_limits_pos_grad(self, q: np.ndarray)->np.ndarray:
        """Returns the cost only if positive gradient in cost. Otherwise 0. Uses cost_joint_limits.

        Args:
            q (np.array): current joint configuration

        Returns:
            np.array: cost of the joint limits
        """
        cost_new = self.cost_joint_limits(q)
        cost_out = np.zeros_like(cost_new)
        cost_out[cost_new>self.cost] = cost_new[cost_new>self.cost]
        self.cost = cost_new
        return cost_out

    def modifyMassMatrix(self, M: np.ndarray, q: np.ndarray)->np.ndarray:
        """Modifies the mass matrix to consider constraints.

        Args:
            M (np.array): mass matrix
            q (np.array): current joint configuration

        Returns:
            np.array: modified mass matrix
        """
        #if delta_cost>0, else = M_ii 
        M[np.diag_indices_from(M)] += np.abs(self.cost_joint_limits_pos_grad(q)) #hierdurch werden Bewegung dieser Joints langsamer
        return M


class Derivative:
    def __init__(self, dt: float, num_values: int = 8) -> None:
        self.dt = dt #1/Hz (s) # TODO: get time_diff instead
        self.last_value = np.array([0.0]*num_values)
        self.derivative = np.array([0.0]*num_values)

    def calc_derivative(self, value: np.array) -> np.array:
        self.derivative = (value - self.last_value)/self.dt
        self.last_value = value
        return self.derivative

class IntegralTrapez:
    def __init__(self, dt: float, num_values: int = 8) -> None:
        self.dt = dt #Hz (1/dt) # TODO: get time_diff instead
        self.last_value = np.array([0.0]*num_values)
        self.integral = np.array([0.0]*num_values)

    def calc_integral(self, value: np.ndarray, last_integral: Optional[np.ndarray]=None) -> np.ndarray:   #TODO:use measured value for self.integral+=
        if last_integral is not None:
            # rospy.logdebug(f"{last_integral=}")
            self.integral = last_integral
        self.integral += (value + self.last_value)*self.dt/2
        self.last_value = value
        return self.integral

class OptimizationFunction(object):
    """This class is used to calculate the objective function and its gradient. If no gradient is given, the gradient is calculated numerically.

    Args:
        func (function): objective function
        gradFunc (function, optional): gradient of objective function. Defaults to None.
        alpha (float, optional): weight for line-search. Defaults to 1.0.
    """
    def __init__(self, func, gradFunc=None, alpha=1.0):
        self.func = func
        self.gradFunc = gradFunc
        self.alpha = alpha

        if gradFunc is None:
            self.gradFunc = nd.Gradient(self.func)

    def calc_gradient(self, q):
        """Calculates the gradient of the objective function.
        
        Args:
            q (np.array): current joint configuration"""
        return self.gradFunc(q)

    def calc_gradient_weighted(self, q):
        """Calculates the gradient of the objective function weighted with alpha.
        
        Args:
            q (np.array): current joint configuration"""
        return self.alpha*self.calc_gradient(q)

class Optimization():
    def __init__(self, gradient_funcs: Optional[List[Callable]] = None, min_funcs: Optional[List[Callable]] = None, alpha=1e-7):
        self.rate = rospy.Rate(100)
        
        self.filter_up = MovingAvg(50,2)
        self.filter_mir = MovingAvg(10,3)
        self.alpha = alpha
        # * Get namespace for topics from launch file
        ns_group=rospy.get_namespace() # slashed e.g. "/mur216/"
        try:
            self.ns = rospy.get_param("ur_ns")
        except KeyError:
            self.ns = ns_group[1:-1] # remove slashes
            if not self.ns:
                self.ns = "mur"
        if self.ns:
            self.ns_slashed = "/" + self.ns + "/"
        else:
            self.ns_slashed = "/"
            
        self.prefix_mir = rospy.get_param("prefix_mir", default="mir/")
        self.ns_prefix_mir = self.ns + "/" + self.prefix_mir
        
        self.prefix = rospy.get_param("prefix_ur", default="ur/")
        self.ns_prefix = self.ns_slashed+self.prefix
        self.base_name = self.ns_prefix_mir+"base_footprint"
        self.mir=MiR(gazebo_model=self.ns, prefix_mir=self.prefix_mir)
        self.tf_listener = tf.TransformListener()
        # self.tf_broadcaster = tf.TransformBroadcaster() # TODO: only for debugging
        rospy.loginfo("self.ns: {}".format(self.ns))
        rospy.loginfo("ns_group: {}".format(ns_group))
        # HW:
        group_name="manipulator"
        self.ur = UR(ns="/mur/ur", description="/mur/ur/robot_description", prefix=self.prefix, tf=self.tf_listener, base_name=self.base_name, ur_base_name="/mur/ur/base_link", group_name=group_name)
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.j = Joints(ns=self.ns_prefix, joint_names=joint_names)
        
        self.v_command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._gradient_funcs = gradient_funcs if gradient_funcs is not None else []
        self._min_funcs = min_funcs if min_funcs is not None else []
        self.min_func = self.get_total_min_func()
        
        rospy.Subscriber("velocity_command", Twist, self.cb_vel_command)
        
    @property
    def gradient_funcs(self):
        return self._gradient_funcs
    
    @gradient_funcs.setter
    def gradient_funcs(self, gradient_funcs: List[Callable[[np.ndarray], np.ndarray]]):
        if not isinstance(gradient_funcs[0], Callable):
            raise ValueError("gradient_funcs must be a list of functions")
        self.gradient_funcs = gradient_funcs
        
    def add_gradient_func(self, gradient_func: Callable[[np.ndarray], np.ndarray]):
        if not isinstance(gradient_func, Callable):
            raise ValueError("opt_func must be a function")
        self._gradient_funcs.append(gradient_func)
    
    def calc_gradient(self, q, funcs : Optional[List[Callable]] = None):
        """Accumulates the result of the gradient functions. (gradient of the objective function, weighted by the corresponding alpha)
        
        Args:
            q (np.array): current joint configuration"""
        dim_q_m = 6
        grad = np.zeros(dim_q_m)
        funcs = self.gradient_funcs if funcs is None else funcs
        for func in funcs:
            grad += func(q)
        return grad
    
    @property
    def min_funcs(self):
        return self._min_funcs
    
    @min_funcs.setter
    def min_funcs(self, min_funcs: List[Callable[[np.ndarray], np.ndarray]]):
        if not isinstance(min_funcs[0], Callable):
            raise ValueError("min_funcs must be a list of functions")
        self.min_funcs = min_funcs
        
    def add_min_func(self, min_func: Callable[[np.ndarray], np.ndarray]):
        if not isinstance(min_func, Callable):
            raise ValueError("min_func must be a function")
        self._min_funcs.append(min_func)
    
    def get_total_min_func(self):
        """Accumulates the result of the min functions.
        
        Args:
            q (np.array): current joint configuration"""
        func = lambda q: np.sum([func(q) for func in self.min_funcs])
        return func
    
    def minimize_to_distance(self, q: np.ndarray, maxiter: int = 10):
        """uses scipy.minimize

        Args:
            q (np.ndarray): current joint configuration
            maxiter (int, optional): maximum number of iterations. Defaults to 10.

        Returns:
            np.ndarray: the distance to the local minimum of the objective function
        """
        # min_q = minimize(self.min_func, q, method='BFGS', jac=self.calc_gradient, options={'disp': True, 'maxiter': 10})
        res = minimize(self.min_func, q, method='BFGS', options={'disp': True, 'maxiter': maxiter})
        min_q = res.x
        # rospy.logdebug("min_q: {}".format(min_q))
        dist = min_q - q # rate beachten
        return dist

    def get_G(self, r0E=Vector3()):
        """get Matrix G from r0E. For velocity in [x,theta] (v_odom)

        Args:
            r0E (Vector3): Vector from mur_base_link to end effector (T+ur_base_pos)

        Returns:
            np.array(3,2): transform velocity of MiR to EEF velocity (x,w_z zu x,y,w_z)
        """
        G = np.zeros((3, 2))
        G[0,0] = 1
        G[0,1] = -r0E.y
        G[1,1] = r0E.x
        G[2,1] = 1
        return G

    def get_G_xy(self, r0E=Vector3()) -> np.ndarray:
        """get Matrix G from r0E. Like get_G, but for velocity in [x,y,theta]

        Args:
            r0E (Vector3): Vector from mur_base_link to end effector (T+ur_base_pos)

        Returns:
            np.array(3,3): transform velocity of MiR to EEF velocity (x,y,w_z zu x,y,w_z)
        """
        G = np.zeros((3, 3))
        G[0,0] = 1  #x=x
        G[0,2] = -r0E.y     #x=ry*th
        G[1,1] = 1  #y=y
        G[1,2] = r0E.x    #y=rx*th
        G[2,2] = 1  #th=th
        return G

    def calc_G(self, q, xy=False):
        """Calculates G by calculating r0E and then using get_G

        Args:
            q (list): joint angles of manipulator

        Returns:
            np.array(3,2): transform velocity of MiR to EEF velocity (x,w_z zu x,y,w_z)
        """
        r0E = getVector_eef(q) + self.ur.ur_base_pos # ur_base_frame = base_footprint frame
        if xy:
            return self.get_G_xy(r0E)
        else:
            return self.get_G(r0E)


    def calcPlatformOptimal(self, j_m, j_p, dH, matG=np.ones((3,2)), alpha=1.0):
        """Calc the platform velcoity from gradient

        Args:
            j_m (np.ndarray): jacobian manipulator
            j_p (np.ndarray): jacobian platform
            dH (np.ndarray): gradient
            matG (np.ndarray, optional): q_p'=G@u_p. Defaults to np.ones((3,2)).
            alpha (float, optional): Schrittweitenfaktor. Defaults to 1.0.

        Returns:
            np.ndaray: u_p platform velocities
        """
        dim_q_m = 6
        dim_q_p = j_p.shape[1]
        dim_dH = dH.shape[0]
        optMat = np.zeros((dim_q_p, dH.shape[0]))
        optMat[:, 0:dim_q_m] = -(np.linalg.inv(j_m) @ j_p).T
        optMat[:, dim_q_m:] = np.eye(dim_q_p, dim_dH-dim_q_m)
        q_p = optMat @ dH   # x',y',w_z
        u_p = matG.T @ q_p  # x_p',w_z
        u_p= u_p*alpha
        return u_p

    def check_min(self, q, u_p, j_p, matG, dt):
        """Checks if the new position has smaller value for OptFunc than the old one"""
        q_ur=q+j_p @ matG @ np.array([u_p[0], u_p[1]])*dt
        if self.get_total_min_func(q_ur)>self.get_total_min_func(q):
            #k=1
            return False
        else:
            return True

    def cb_vel_command(self, msg=Twist()):
        vel = msg
        
        self.v_command = np.array([vel.linear.x, vel.linear.y, vel.linear.z, vel.angular.x, vel.angular.y, vel.angular.z])

    def control_loop(self):
        self.filter_up.filter_len = 10 # for use with minimize instead of JUST gradient
        rospy.logdebug("control loop")
        rospy.on_shutdown(self.__shutdown)
        if not self._min_funcs: #empty or None
            rospy.signal_shutdown("min_funcs must be set")
            raise ValueError("min_funcs must be set")
        
        while not rospy.is_shutdown():
            dt = self.rate.sleep_dur.to_sec()
            q=self.j.q
            gradient = self.calc_gradient(q)
            
            #via minimize:
            # dist = self.minimize_to_distance(q, maxiter=10)
            # gradient = dist*dt
            # gradient*=1.0 # kann auch hoeher sein, da nur zum Ausrechnen von up benutzt -> gradient  gegen 0
            
            j_m = self.ur.group.get_jacobian_matrix(list(q))
            # negate x and y of jacobian j_m
            # j_m[0,:] = -j_m[0,:]
            # j_m[1,:] = -j_m[1,:]
            #also rotation:
            j_m[0:2,:]=-j_m[0:2,:]
            j_m[3:5,:]=-j_m[3:5,:]
            
            
            j_p = getJacobianPlatformWithEEF() # zu MiRbase mit x',y',w_z
            matG = self.calc_G(q)
            u_p=self.calcPlatformOptimal(j_m, j_p, gradient, matG, alpha=self.alpha)
            
            # if not self.check_min(q, u_p, j_p, matG, dt): u_p = np.zeros(2)

            # rospy.logdebug("gradient in q: {}".format(gradient) + "for q: {}".format(q)+"\n u_p: {}".format(u_p))
            
            u_p = self.mir.restrict_vel(u_p, (0.01, 0.105), (0.3, 0.2))#1.0)) # v_min = 0.01, 0.105 # 0.105 =ca. 6°/s
            # TEST disable rotation:
            #u_p = self.mir.restrict_vel(u_p, (0.01, 0.3), (0.3, 0.2))#1.0)) # v_min = 0.01, 0.105 # 0.105 =ca. 6°/s
            
            u_p = self.filter_up.update(u_p)
            self.mir.set_vel_cmd(u_p) # up glaetten

            rotM = self.mir.rotM

            # cur_vel = self.mir.get_cur_velocity()   #.twist   # in MiRbase Coordinates
            # # rospy.loginfo("cur_vel: {}".format(cur_vel))
            # v_ur_nullspace = j_p @ matG @ np.array([cur_vel.linear.x, cur_vel.angular.z]) # in MiRbase Coordinates
            # stattdessen mit vel aus pose_diff:
            matG_xy = self.calc_G(q, xy=True)
            vel_mir = self.mir.velocity_observer.v_hat
            vel_mir  = self.filter_mir.update(vel_mir)            
            
            v_ur_nullspace = j_p @ matG_xy @ vel_mir # in MiRbase Coordinates
            ur_vel = np.linalg.inv(rotM) @ self.v_command - v_ur_nullspace # in MiRbase Coordinates (v_command is in worldCoordinates)
            rospy.logdebug(f"{ur_vel=}")

            self.ur.setCartesianVelocity(ur_vel)
            self.rate.sleep()
    
    def __shutdown(self):
        """set velocities to zero"""
        self.ur.setCartesianVelocity()
        self.mir.set_vel_cmd()

if __name__ == "__main__":
    # q = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    # theta = pi / 2

    # for testing:
    import rospy
    import moveit_commander
    
    rospy.init_node('optimization', log_level=rospy.DEBUG)
    f_opts = []
    o=Optimization(f_opts, alpha=1.0)
    q_min = np.array([-pi, -pi, -pi, -pi, -pi, -pi])
    q_max = np.array([pi, pi, pi, pi, pi, pi])

    q_goal = np.array([np.nan]*6)
    q_goal[0] = 0.0
    k_manip=0.8 #0.2
    k_dist=0.04 #0.01

    f_opt_manip_p = OptimizationFunction(lambda q: manipulability_pos(q, o.ur.group), alpha=0.8)
    f_opt_manip_o = OptimizationFunction(lambda q: manipulability_o(q, o.ur.group), alpha=0.01)

    # q0 near 0:
    f_opt_distq1 = OptimizationFunction(lambda q: distance_toq_goal(q[0], 0),lambda q: distance_toq_goal_grad(q, q_goal), alpha=k_dist)
    
    # f_opts = [f_opt_manip..calc_gradient, f_opt_distq1.calc_gradient]
    # f_opts = [f_opt_distq1.calc_gradient]
    
    o.add_gradient_func(f_opt_manip_p.calc_gradient_weighted)
    o.add_min_func(lambda q: f_opt_manip_p.func(q)*(-1))
    
    # o.add_gradient_func(f_opt_manip_o.calc_gradient_weighted)
    # o.add_min_func(lambda q: f_opt_manip_o.func(q)*(-1))
    
    # o.add_gradient_func(f_opt_distq1.calc_gradient_weighted)
    # o.add_min_func(lambda q: f_opt_manip_p.func(q))

    o.control_loop()