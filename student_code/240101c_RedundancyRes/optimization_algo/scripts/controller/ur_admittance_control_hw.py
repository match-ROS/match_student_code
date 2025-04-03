#!/usr/bin/env python3

# /***************************************************************************

# **************************************************************************/

"""
    see cooperative_manipulation_controllers/scripts/ur16e_admittance_control_simulation.py
    Description...
    
    Admittance controller
    
    Input: 
    * Desired cartesian velocity of the EE: desired_velocity (In 'world' frame)
        "cooperative_manipulation/cartesian_velocity_command"
    * External wrench from the f/t sensor: wrench_ext (In 'tool0_controller' frame)
        "/mur/ur/wrench"
    
    Output: 
    * Target joint velocity: self.target_joint_velocity (In 'base_link' frame)
        "/" + self.namespace + "/joint_group_vel_controller/command"
"""
import sys, copy, math, rospy, tf, tf2_ros, moveit_commander
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Vector3, Twist, TransformStamped, PoseStamped
from std_msgs.msg import Float64MultiArray, Float32, Bool
from optimization_algo.msg import SingularityAvoidance, WorkspaceViolation
from match_lib.match_geometry import MyPose, MyOrient


class ur_admittance_controller():
    
    def config(self):
        # Control thread publish rate [Hz]
        self.publish_rate = 100 #max Rate (measured iin control loop to be more robust)
        # self.ft_sensor_frame = self.ns_prefix+"ft_sensor_link" # without gripper
        self.ft_sensor_frame = self.ns_prefix+"tool0" #_controller"
        self.command_in_frame = self.ns_prefix_mir+'base_footprint'
        self.command_out_frame = None #self.ns_prefix_mir+'base_footprint'  # if None: self.group.get_planning_frame()
        self.eef_frame_w_o_ns = "tool0" #self.prefix+"tool0" #"wrist_3_link" #without namespace
        # self.jacobian_frame = None #self.ns_prefix+"base_link" # if None: self.group.get_planning_frame()
        #* Admittance controler values
        # Array with the contact forces/torques
        self.wrench_contact = np.array([1.0,1.0,1.0,0.5,0.5,0.5], dtype=np.float64)
        #self.wrench_contact = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Array to store the wrench difference
        self.wrench_diff = np.array([0.0,0.0,0.0,0.0,0.0,0.0], dtype=np.float64)
        # Admittance gains
        self.A_trans_x = 200
        self.A_trans_y = 200
        self.A_trans_z = 200
        self.A_rot_x = 100
        self.A_rot_y = 100
        self.A_rot_z = 100
        # Kp gains
        self.Kp_trans_x = 1.25
        self.Kp_trans_y = 1.25
        self.Kp_trans_z = 1.25
        self.Kp_rot_x = 0.125
        self.Kp_rot_y = 0.125
        self.Kp_rot_z = 0.3
        
        # set arrays
        self.A = np.array([self.A_trans_x, self.A_trans_y, self.A_trans_z, self.A_rot_x, self.A_rot_y, self.A_rot_z], dtype=np.float64)
        self.Kp = np.array([self.Kp_trans_x, self.Kp_trans_y, self.Kp_trans_z, self.Kp_rot_x, self.Kp_rot_y, self.Kp_rot_z], dtype=np.float64)
        
        #* Min and max limits for the cartesian velocity (trans/rot) [m/s]
        self.cartesian_velocity_trans_max_limit = 0.1
        self.cartesian_velocity_rot_max_limit = 0.1
        #* Bandpass filter
        # # Force threshold 
        # self.wrench_filter_force_x = 1.0
        # self.wrench_filter_force_y = 1.0
        # self.wrench_filter_force_z = 1.0
        # # Torque treshold 
        # self.wrench_filter_torque_x = 0.1
        # self.wrench_filter_torque_y = 0.1
        # self.wrench_filter_torque_z = 0.1
        # Force threshold 
        self.wrench_filter_force_x = 0.0
        self.wrench_filter_force_y = 0.0
        self.wrench_filter_force_z = 0.0
        # Torque treshold 
        self.wrench_filter_torque_x = 0.0
        self.wrench_filter_torque_y = 0.0
        self.wrench_filter_torque_z = 0.0
        self.wrench_deadband = np.array([self.wrench_filter_force_x, self.wrench_filter_force_y, self.wrench_filter_force_z, self.wrench_filter_torque_x, self.wrench_filter_torque_y, self.wrench_filter_torque_z])
        
        #* Average filter
        # Lists to store the wrench values
        self.average_filter_list_force_x = []
        self.average_filter_list_force_y = []
        self.average_filter_list_force_z = []
        self.average_filter_list_torque_x = []
        self.average_filter_list_torque_y = []
        self.average_filter_list_torque_z = []
        # Wrench average list length
        self.average_filter_list_length = 100
        # Wrench average list lengthself.singularity_stop == False
        self.average_wrench_ext_filtered_array = np.array([0.0,0.0,0.0,0.0,0.0,0.0], dtype=np.float64)
        #* Set gripper offset
        self.ur16e_gripper_offset = 0.01 # [m]
        # * Initialize the needed velocity data types
        # Initialize desired velocity transformed form 'wrorld' frame to 'base_link' frame (xdot_desired_base_link)
        self.base_link_desired_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.base_link_cartesian_desired_velocity_trans =Vector3Stamped()
        self.world_cartesian_velocity_trans = Vector3Stamped()
        self.world_cartesian_velocity_trans.header.frame_id = self.command_in_frame
        self.world_cartesian_velocity_rot = Vector3Stamped()
        self.world_cartesian_velocity_rot.header.frame_id = self.command_in_frame

        # Initialize velocity from admittance (xdot_a_base_link)
        self.admittance_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Initialize target cartesian velocity array (xdot_target_base_link)
        self.target_cartesian_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        # Declare target joint velocity msgs (qdot_target_base_ink) (unit: [radian/s])
        self.target_joint_velocity = Float64MultiArray()
        #* Array to store the desired endeffector pose
        self.trajectory_EE_pose = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        #* Franka workspace violation
        self.workspace_violation = False
        #* Singulariy avoidance: OLMM
        self.singularity_velocity_world = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        self.singularity_stop = False
        self.singularity_stop_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
         
         
        self.singularity_entry_threshold = 0.05
        self.singularity_exit_threshold = 0.06
        self.singularity_min_threshold = 0.015
        self.singularity_stop_threshold = 0.01
        
        self.adjusting_scalar = 0.0
        
        self.singularity_velocity_cmd = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.singularity_avoidance_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.singularity_velocity_msg = SingularityAvoidance()
        self.bool_singularity = False
        self.bool_reduce_singularity_offset = False
        self.singularity_trans_offset_accuracy = 0.001 # [m]
        self.singularity_rot_offset_accuracy = 0.0174 # [rad] ~ 1 [°]
        self.last_target_trans_cartesian_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0]) 
        self.last_target_rot_cartesian_velocity = np.array([0.0,0.0,0.0,0.0,0.0,0.0]) 
        #* Initialize shutdown joint velocity, called on shutdown 
        self.shutdown_joint_velocity = Float64MultiArray()
        self.shutdown_joint_velocity.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        
        
    def __init__(self):
        
        # * Initialize node
        rospy.init_node('admittance_controller_node', anonymous=True, log_level=rospy.INFO)
        
        
        # * Get namespace for topics from launch file
        self.namespace = rospy.get_param("ur_ns",default="mur")
        self.prefix = rospy.get_param("prefix_ur", default="ur/")
        self.ns_prefix = self.namespace + "/" + self.prefix
        self.prefix_mir = rospy.get_param("prefix_mir", default="mir/")
        self.ns_prefix_mir = self.namespace + "/" + self.prefix_mir

        # * Load config parameters
        self.config()

        # * Initialize on_shutdown clean up
        rospy.on_shutdown(self.shutdown)
        
        
        # * Initialize tf TransformBroadcaster
        self.brodacaster = tf2_ros.StaticTransformBroadcaster()
        # * Initialize tf TransformListener
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("Wait for transformation 'wrist_3_link' to 'base_link'.")
        self.tf_listener.waitForTransform(self.ns_prefix+"wrist_3_link",self.ns_prefix+"base_link", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation 'mirbase_footprint' to 'wrist_3_link'.")
        self.tf_listener.waitForTransform(self.ns_prefix_mir+"base_footprint",self.ns_prefix+"wrist_3_link", rospy.Time(), rospy.Duration(5.0))
        rospy.loginfo("Wait for transformation 'mirbase_footprint' to 'base_link'.")
        self.tf_listener.waitForTransform(self.ns_prefix_mir+"base_footprint", self.ns_prefix+"base_link", rospy.Time(), rospy.Duration(5.0))
        
        # TODO: just test
        self. tf_broadcaster = tf.TransformBroadcaster()
        # Initialize the 'ur16e_gripper' frame in tf tree
        self.set_gripper_offset()
        
        # Wait for transformations from 'world' to 'ur16e_gripper' and 'world' to 'panda_EE'
        rospy.loginfo("Wait for transformation 'mirbase_footprint' to 'tool0'.")
        self.tf_listener.waitForTransform(self.ns_prefix_mir+"base_footprint",self.ns_prefix+"tool0", rospy.Time(), rospy.Duration(10.0))
        # rospy.loginfo("Wait for transformation 'world' to 'panda_EE'.")
        # self.tf_listener.waitForTransform("world","panda_EE", rospy.Time(), rospy.Duration(60.0))

        
        # * Initialize move_it
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            group_name = 'manipulator'
            print("Initialize movit_commander. Group name: ",group_name)
            self.group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0, ns="/mur/ur", robot_description="/mur/ur/robot_description")
            # self.group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=5.0, ns="", robot_description="robot_description")
        except Exception as e: 
            print(e)
            raise Exception("Could not initialize moveit_commander. Group name: ",group_name)

        # Set endeffector link
        self.group.set_end_effector_link("tool0")
        rospy.logdebug(f"Endeffektor is: {self.group.get_end_effector_link()}")
        if self.command_out_frame is None:
            self.command_out_frame = self.ns_prefix+self.group.get_planning_frame()
            rospy.logdebug(f"{self.command_out_frame=}")
        
        # * Initialize publisher:
        # Publish final joint velocity to "/mur/joint_group_vel_controller/command"
        self.joint_velocity_pub = rospy.Publisher(
            "/" + self.ns_prefix + "joint_group_vel_controller/command",
            Float64MultiArray,
            queue_size=1)
        
        # Publish singularity velocity
        self.singularity_velocity_pub = rospy.Publisher(
            "/cooperative_manipulation/ur16e/singularity_velocity",
            SingularityAvoidance,
            queue_size=1)
        
        
        
        
        # Publisher and Subscriber for measurements-------------------------------------------------------------
        # * Initialize subscriber:

        self.delta_pos_msg = Float64MultiArray()
        self.delta_ori_msg = Float64MultiArray()
        self.sigma_msg = Float32()
        self.singularity_msg = Bool()
        self.offset_msg = Bool()
        self.wrench_msg = Float64MultiArray()
        self.admittance_msg = Float64MultiArray()
        self.cartesian_vel_msg = Float64MultiArray()
        
        self.delta_pos_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/delta_pos',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        self.delta_ori_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/delta_ori',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        self.sigma_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/sigma',
                Float32,
                tcp_nodelay=True,
                queue_size=1)
        
        self.sigularity_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/sigularity',
                Bool,
                tcp_nodelay=True,
                queue_size=1)
        
        self.offset_publisher = rospy.Publisher(
                '/' + self.namespace + '/measurement/reduce_offset',
                Bool,
                tcp_nodelay=True,
                queue_size=1)
        
        self.wrench_pub = rospy.Publisher(
                '/' + self.namespace + '/measurement/wrench',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        # average_wrench_array...
        
        self.admittance_pub = rospy.Publisher(
                '/' + self.namespace + '/measurement/admittance_vel',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)
        
        self.cart_vel_pub = rospy.Publisher(
                '/' + self.namespace + '/measurement/cartesian_vel',
                Float64MultiArray,
                tcp_nodelay=True,
                queue_size=1)

        
        
        
        # Publisher and Subscriber for measurements-------------------------------------------------------------
        
        
        # * Initialize subscriber:
        # Subscriber to "/ur/wrench"
        self.wrench_ext_sub = rospy.Subscriber(
            "/" + self.ns_prefix + "wrench",
            WrenchStamped,
            self.wrench_callback,queue_size=1)
        
        # Subscriber to "/ur/cooperative_manipulation/cartesian_velocity_command"
        self.cartesian_velocity_command_sub = rospy.Subscriber(
            "cooperative_manipulation/cartesian_velocity_command",
            Twist,
            self.cartesian_velocity_command_callback,queue_size=1)
        
        
        # Subscriber to workspace violation of the Franka Emika
        self.workspace_msg_sub = rospy.Subscriber(
            "/cooperative_manipulation/workspace",
            WorkspaceViolation,
            self.workspace_violation_callback, 
            queue_size=1,
            tcp_nodelay=True)
        
        
        
        # Converse input_vector rotation from np.array to vector3
        self.world_cartesian_velocity_rot.header.frame_id = self.command_in_frame
        self.world_cartesian_velocity_rot.header.stamp = rospy.Time()
        self.world_cartesian_velocity_rot.vector.x = 0.0
        self.world_cartesian_velocity_rot.vector.y = 0.0
        self.world_cartesian_velocity_rot.vector.z = 0.0
        
        # * Get the start EE pose
        # Get start EE pose in 'base_link' frame (command_out_frame / jacobian_frame)
        self.pose_trajectory = self.get_eef_pose()
        
        # * Run control_thread
        rospy.loginfo("Recieved messages; Launch ur16e Admittance control.")
        self.control_thread()
        
        rospy.spin()
        
    def set_gripper_offset(self):
        """
            Set the gripper offset from 'wirst_3_link' frame.
        """
        static_gripper_offset = TransformStamped()
        static_gripper_offset.header.stamp = rospy.Time.now()
        static_gripper_offset.header.frame_id = self.ns_prefix+"wrist_3_link"
        static_gripper_offset.child_frame_id = self.ns_prefix+"tool0"
        static_gripper_offset.transform.translation.x = 0.0
        static_gripper_offset.transform.translation.y = 0.0
        static_gripper_offset.transform.translation.z = self.ur16e_gripper_offset
        static_gripper_offset.transform.rotation.x = 0.0
        static_gripper_offset.transform.rotation.y = 0.0
        static_gripper_offset.transform.rotation.z = 0.0
        static_gripper_offset.transform.rotation.w = 1.0

        self.brodacaster.sendTransform(static_gripper_offset) 
    
    def transform_pose_stamped(self,source_frame: str,target_frame: str,source_pose: np.array):
        """ Transforms a cartesian PoseStamped array from the source frame to the target frame. 

        Args:
            source_frame (str): The frame to transform from 
            target_frame (str): The frame to transform to
            source_pose (np.array): The PoseStamped in source frame as array

        Returns:
            np.array: The PoseStamped in target frame as array
        """
        now = rospy.Time()
        
        source_position = PoseStamped()
        source_position.header.frame_id = source_frame
        source_position.header.stamp = now
        source_position.pose.position.x = source_pose[0]
        source_position.pose.position.y = source_pose[1]
        source_position.pose.position.z = source_pose[2]
        
        source_frame_quaternion = quaternion_from_euler(source_pose[3],source_pose[4],source_pose[5])
                
        source_position.pose.orientation.x = source_frame_quaternion[0]
        source_position.pose.orientation.y = source_frame_quaternion[1]
        source_position.pose.orientation.z = source_frame_quaternion[2]
        source_position.pose.orientation.w = source_frame_quaternion[3]
        
        target_frame_position = self.tf_listener.transformPose(target_frame,source_position)
        
        target_frame_quaternion =  (target_frame_position.pose.orientation.x,
                                    target_frame_position.pose.orientation.y,
                                    target_frame_position.pose.orientation.z,
                                    target_frame_position.pose.orientation.w)
        
        target_frame_euler = euler_from_quaternion(target_frame_quaternion)
        
        target_pose = np.array([target_frame_position.pose.position.x,
                                    target_frame_position.pose.position.y,
                                    target_frame_position.pose.position.z,
                                    target_frame_euler[0],
                                    target_frame_euler[1],
                                    target_frame_euler[2]])  
        return target_pose
    
    def scalar_adjusting_function(self,current_sigma):
        """Compute the adjusting scalar for the OLMM. 3 varaitions to calculate the adjusting scalar are presented.

            Source: QIU, Changwu; CAO, Qixin; MIAO, Shouhong. An on-line task modification method for singularity avoidance of robot manipulators. Robotica, 2009, 27. Jg., Nr. 4, S. 539-546.

        Args:
            current_sigma (float): The current singular value

        Returns:
            float: The adjusting_scalar
        """
        
        
        # 0.
        # adjusting_scalar = (1-current_sigma)
        # 1.
        # adjusting_scalar = (1-(current_sigma/self.singularity_entry_threshold))
        # 2.
        # adjusting_scalar = (1-(current_sigma/self.singularity_entry_threshold)**(1/2))
        # 3.
        # adjusting_scalar = (1-(current_sigma/self.singularity_entry_threshold)**(3/2))
        # 4.
        adjusting_scalar = 0.5 * (1 + math.cos(math.pi*((current_sigma-self.singularity_min_threshold)/(self.singularity_entry_threshold - self.singularity_min_threshold))))


        
        return adjusting_scalar
    
    def transform_vector(self,source_frame: str,target_frame: str,input_vector: np.array):
        """ 
            Transforms a vector from source frame to target frame.

        Args:
            source_frame (str): The frame to transform from 
            target_frame (str): The frame to transform to
            input_array (np.array): The vector in source frame as 6 dim array (trans+rot)

        Returns:
            np.array: The vector in target frame as array
        """
        source_frame_cartesian_velocity_trans = Vector3Stamped()
        source_frame_cartesian_velocity_rot = Vector3Stamped()
        
        # Get current time stamp
        now = rospy.Time()
 
        # Converse input_vector translation from np.array to vector3
        source_frame_cartesian_velocity_trans.header.frame_id = source_frame
        source_frame_cartesian_velocity_trans.header.stamp = now
        source_frame_cartesian_velocity_trans.vector.x = input_vector[0]
        source_frame_cartesian_velocity_trans.vector.y = input_vector[1]
        source_frame_cartesian_velocity_trans.vector.z = input_vector[2]
        
        # Transform input_vector translation from 'wrist_3_link' frame to 'base_link' frame
        target_frame_cartesian_velocity_trans = self.tf_listener.transformVector3(target_frame,source_frame_cartesian_velocity_trans)
        
        # Converse input_vector rotation from np.array to vector3
        source_frame_cartesian_velocity_rot.header.frame_id = source_frame
        source_frame_cartesian_velocity_rot.header.stamp = now
        source_frame_cartesian_velocity_rot.vector.x = input_vector[3]
        source_frame_cartesian_velocity_rot.vector.y = input_vector[4]
        source_frame_cartesian_velocity_rot.vector.z = input_vector[5]
        
        # Transform input_vector rotation from 'wrist_3_link' frame to 'base_link' frame
        target_frame_cartesian_velocity_transrot = self.tf_listener.transformVector3(target_frame,source_frame_cartesian_velocity_rot)
        
        # Converse input_vector from vector3 to np.array
        output_vector = np.array([
            target_frame_cartesian_velocity_trans.vector.x,
            target_frame_cartesian_velocity_trans.vector.y,
            target_frame_cartesian_velocity_trans.vector.z,
            target_frame_cartesian_velocity_transrot.vector.x,
            target_frame_cartesian_velocity_transrot.vector.y,
            target_frame_cartesian_velocity_transrot.vector.z
            ])
        
        return output_vector
    
        
    def workspace_violation_callback(self,workspace_violation):
        """_summary_

        Args:
            workspace_violation (_type_): _description_
        """
        self.workspace_violation = workspace_violation.workspace_violation
    
    def cartesian_velocity_command_callback(self,desired_velocity=Twist()):
        """
            Get the cartesian velocity command and transform it from from the 'world' frame to the 'base_link' and 'wrist_link_3' frame.
            
            Send example velocity:
            rostopic pub -r 10 cooperative_manipulation/cartesian_velocity_command geometry_msgs/Twist "linear:
            x: 0.0
            y: 0.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0" 
        """
        # Get current time stamp
        now = rospy.Time()

        # # Calculate the trajectory velocity of the manipulator for a rotation of the object in extra node   

        # Transform the velocity from 'world' frame to 'base_link' frame------------------------------------------------
        # Converse cartesian_velocity translation to vector3
        # self.world_cartesian_velocity_trans.header.frame_id = self.command_in_frame
        self.world_cartesian_velocity_trans.header.stamp = now
        self.world_cartesian_velocity_trans.vector.x = desired_velocity.linear.x
        self.world_cartesian_velocity_trans.vector.y = desired_velocity.linear.y
        self.world_cartesian_velocity_trans.vector.z = desired_velocity.linear.z
        
        # Transform cartesian_velocity translation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_trans = self.tf_listener.transformVector3(self.command_out_frame,self.world_cartesian_velocity_trans)
        
        
        # Converse cartesian_velocity rotation to vector3
        # self.world_cartesian_velocity_rot.header.frame_id = self.command_in_frame
        self.world_cartesian_velocity_rot.header.stamp = now
        self.world_cartesian_velocity_rot.vector.x = desired_velocity.angular.x
        self.world_cartesian_velocity_rot.vector.y = desired_velocity.angular.y
        self.world_cartesian_velocity_rot.vector.z = desired_velocity.angular.z
        
        # Transform cartesian_velocity rotation from 'world' frame to 'base_link' frame
        self.base_link_cartesian_desired_velocity_rot = self.tf_listener.transformVector3(self.command_out_frame,self.world_cartesian_velocity_rot)
        
        # Converse cartesian_velocity from vector3 to np.array
        self.base_link_desired_velocity = np.array([
            self.base_link_cartesian_desired_velocity_trans.vector.x,
            self.base_link_cartesian_desired_velocity_trans.vector.y,
            self.base_link_cartesian_desired_velocity_trans.vector.z,
            self.base_link_cartesian_desired_velocity_rot.vector.x,
            self.base_link_cartesian_desired_velocity_rot.vector.y,
            self.base_link_cartesian_desired_velocity_rot.vector.z
            ])
    
    def wrench_callback(self,wrench_ext):
        """ 
            Get external wrench in 'tool0' frame.
            
        """
        # * Average filter
        # Fill the empty lists with wrench values
        if len(self.average_filter_list_force_x) < self.average_filter_list_length:
            # 2. Add the new wrench to the list 
            self.average_filter_list_force_x.append(wrench_ext.wrench.force.x) 
            self.average_filter_list_force_y.append(wrench_ext.wrench.force.y) 
            self.average_filter_list_force_z.append(wrench_ext.wrench.force.z) 
            self.average_filter_list_torque_x.append(wrench_ext.wrench.torque.x)
            self.average_filter_list_torque_y.append(wrench_ext.wrench.torque.y)
            self.average_filter_list_torque_z.append(wrench_ext.wrench.torque.z )
            # 3. Calculate the average 
            self.average_force_x = sum(self.average_filter_list_force_x)/len(self.average_filter_list_force_x)
            self.average_force_y = sum(self.average_filter_list_force_y)/len(self.average_filter_list_force_y)
            self.average_force_z = sum(self.average_filter_list_force_z)/len(self.average_filter_list_force_z)
            self.average_torque_x = sum(self.average_filter_list_torque_x)/len(self.average_filter_list_torque_x)
            self.average_torque_y = sum(self.average_filter_list_torque_y)/len(self.average_filter_list_torque_y)
            self.average_torque_z = sum(self.average_filter_list_torque_z)/len(self.average_filter_list_torque_z)
            
        # If the lists reached the length of self.average_filter_list_length
        elif len(self.average_filter_list_force_x) == self.average_filter_list_length:
            # 1. Delete the first element in the list 
            self.average_filter_list_force_x.pop(0)
            self.average_filter_list_force_y.pop(0)
            self.average_filter_list_force_z.pop(0)
            self.average_filter_list_torque_x.pop(0)
            self.average_filter_list_torque_y.pop(0)
            self.average_filter_list_torque_z.pop(0)
            # 2. Add the new wrench to the list 
            self.average_filter_list_force_x.append(wrench_ext.wrench.force.x)
            self.average_filter_list_force_y.append(wrench_ext.wrench.force.y)
            self.average_filter_list_force_z.append(wrench_ext.wrench.force.z)
            self.average_filter_list_torque_x.append(wrench_ext.wrench.torque.x)
            self.average_filter_list_torque_y.append(wrench_ext.wrench.torque.y)
            self.average_filter_list_torque_z.append(wrench_ext.wrench.torque.z)
            # 3. Calculate the average 
            self.average_force_x = sum(self.average_filter_list_force_x)/self.average_filter_list_length
            self.average_force_y = sum(self.average_filter_list_force_y)/self.average_filter_list_length
            self.average_force_z = sum(self.average_filter_list_force_z)/self.average_filter_list_length
            self.average_torque_x = sum(self.average_filter_list_torque_x)/self.average_filter_list_length
            self.average_torque_y = sum(self.average_filter_list_torque_y)/self.average_filter_list_length
            self.average_torque_z = sum(self.average_filter_list_torque_z)/self.average_filter_list_length
            
            
        self.average_wrench_array = (self.average_force_x,
                                     self.average_force_y,
                                     self.average_force_z,
                                     self.average_torque_x,
                                     self.average_torque_y,
                                     self.average_torque_z)

        # * Transform wrench from 'tool0_controller' frame to 'base_link' frame
        self.average_wrench_array_transformed = self.transform_vector(self.ft_sensor_frame,self.command_out_frame,self.average_wrench_array)
        #* Band-passfilter
        
        if np.linalg.norm(self.base_link_desired_velocity) <= 1e-6:
            filtered_wrench = self.average_wrench_array_transformed - np.sign(self.average_wrench_array_transformed)*self.wrench_deadband
            filtered_wrench[abs(self.average_wrench_array_transformed) <= self.wrench_deadband] = 0.0
            self.average_wrench_ext_filtered_array = filtered_wrench
        else:
            #print("no bandsperre")
            self.average_wrench_ext_filtered_array = self.average_wrench_array_transformed
        
        
    def get_eef_pose(self):
        """ Get the current pose of the end-effector

        Returns:
            MyPose(geometry_msgs.msg.Pose): position=Point(), orientation=Quaternion()
        """
        return MyPose(self.group.get_current_pose(self.eef_frame_w_o_ns).pose)
    
    def control_thread(self):
        """ 
            This thread calculates and publishes the target joint velocity using an admittance controller.
        """
        rate = rospy.Rate(self.publish_rate)
        dt = 1.0/self.publish_rate # set later by measuring the time between two control steps (more robust)
        last_time = rospy.get_time()-dt
        while not rospy.is_shutdown():
            dt=rospy.get_time()-last_time
            last_time = rospy.get_time()
            #*Get current EE pose in 'base_link' frame (command_out / jacobian(self.group) frame)
            pose_cur = self.get_eef_pose()
            pose_delta = self.pose_trajectory - pose_cur
            # orient_diff_euler = euler_from_quaternion(pose_delta.orientation.asArray())
            # pose_trans_diff = pose_delta.position.asArray()
            pose_diff = np.concatenate((pose_delta.position.asArray()[:3],euler_from_quaternion(pose_delta.orientation.asArray())))
            
            # For measurements------------------------------------------------------------------------------------------
            self.delta_pos_msg.data = pose_diff[:3]
            self.delta_ori_msg.data = pose_diff[3:]
            self.delta_pos_publisher.publish(self.delta_pos_msg)
            self.delta_ori_publisher.publish(self.delta_ori_msg)
            # For measurements------------------------------------------------------------------------------------------
        
            #* Compute the wrench difference
                    
            # TODO: if abs(average_wrench_ext_filtered_array) < wrench_contact, then set wrench_diff to 0?
            # wrench_delta = np.abs(self.average_wrench_ext_filtered_array)-self.wrench_contact
            # wrench_delta[wrench_delta<0] = 0
            # self.wrench_diff = np.sign(self.average_wrench_ext_filtered_array)*wrench_delta
            
            sign_wrench = np.sign(self.average_wrench_ext_filtered_array)
            sign_wrench[sign_wrench==0] = 1 #else sign_wrench is 0
            self.wrench_diff = sign_wrench * (np.abs(self.average_wrench_ext_filtered_array)-self.wrench_contact)
            
            # * Calculate velocity from wrench difference and admittance in 'wrist_3_link' frame
            
            self.admittance_velocity = (self.wrench_diff / self.A) + pose_diff * self.Kp 

            # * Add the desired_velocity in 'base_link' frame and admittance velocity in 'base_link' frame            
            self.target_cartesian_velocity = self.base_link_desired_velocity + self.admittance_velocity
            
            # rospy.logdebug(f"{self.admittance_velocity=}")
            self.admittance_msg.data = self.admittance_velocity
            self.admittance_pub.publish(self.admittance_msg)
            
            # self.wrench_msg.data = self.average_wrench_array_transformed
            # self.wrench_msg.data = self.average_wrench_ext_filtered_array
            self.wrench_msg.data = self.wrench_diff
            self.wrench_pub.publish(self.wrench_msg)

            # ### Change new pose after giving the velocity commands -------------------------------------------------------------------------------
            #*  Calculate desired EE trajectory pose

            # Transform cartesian_velocity rotation from 'world' frame to 'tool0' frame da sonst Drehung um EEF Achse
            self.tool0_velocity_rot = self.tf_listener.transformVector3(self.ns_prefix+self.eef_frame_w_o_ns,self.world_cartesian_velocity_rot)
            
            tool0_velocity_rot_array = np.array([self.tool0_velocity_rot.vector.x,
                                                    self.tool0_velocity_rot.vector.y,
                                                    self.tool0_velocity_rot.vector.z])
            

            #*  Calculate desired EE pose with MyPose class
            des_vel = MyPose(pos=self.base_link_desired_velocity[:3]*dt, quatern=quaternion_from_euler(*tool0_velocity_rot_array*dt))
            self.pose_trajectory += des_vel

            # # TODO: just for TEST
            self.tf_broadcaster.sendTransform(tuple(self.pose_trajectory.position.asArray()[0:3]), self.pose_trajectory.orientation.asArray(),
                                                rospy.Time.now(),
                                                self.ns_prefix+'trajectory_pose',
                                                self.command_out_frame)


            # # * Check self.target_cartesian_velocity for the max velocity limits
            # # Calculate the norm of target_cartesian_velocity (trans and rot)
            # target_cartesian_trans_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[0],2) + pow(self.target_cartesian_velocity[1],2) + pow(self.target_cartesian_velocity[2],2))
            
            # target_cartesian_rot_velocity_norm = math.sqrt(pow(self.target_cartesian_velocity[3],2) + pow(self.target_cartesian_velocity[4],2) + pow(self.target_cartesian_velocity[5],2))
            
            # #  Check for cartesian velocity max limit and set to max limit, if max limit is exceeded
            # if target_cartesian_trans_velocity_norm > self.cartesian_velocity_trans_max_limit:
            #     for i in range(3):
            #         self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_trans_velocity_norm) * self.cartesian_velocity_trans_max_limit
                    
            # if target_cartesian_rot_velocity_norm > self.cartesian_velocity_rot_max_limit:
            #     for i in range(3,6):
            #         self.target_cartesian_velocity[i] = (self.target_cartesian_velocity[i]/target_cartesian_rot_velocity_norm) * self.cartesian_velocity_rot_max_limit
                    
            # * Get the current joint states 
            self.current_joint_states_array = self.group.get_current_joint_values() 
            
            # * Calculate the jacobian-matrix and transform in same KS as target_cartesian_velocity (base_link bzw. command_out)
            self.jacobian = self.group.get_jacobian_matrix(self.current_joint_states_array)
            # _,rot=self.tf_listener.lookupTransform(self.command_out_frame,self.jacobian_frame, rospy.Time())
            # rotMat = tf.transformations.quaternion_matrix(rot)
            # self.jacobian[:3,:] = rotMat[:3,:3]@self.jacobian[:3,:]
            # self.jacobian[3:,:] = rotMat[:3,:3]@self.jacobian[3:,:]
            # rospy.logdebug("Jacobian rotiert um: {}".format(rotMat[:3,:3])) #IdentityMat for base_footprint to UR16/base_link
            
# * Singulartiy avoidance: OLMM-----------------------------------------------------------------------------------------
            # QIU, Changwu; CAO, Qixin; MIAO, Shouhong. An on-line task modification method for singularity avoidance of robot manipulators. Robotica, 2009, 27. Jg., Nr. 4, S. 539-546.
            
            # Do a singular value decomposition of the jacobian
            u,s,_ = np.linalg.svd(self.jacobian,full_matrices=True)
            
            sigma_min = min(s) 

            # For measurements------------------------------------------------------------------------------------------
            self.sigma_msg.data = sigma_min 
            self.sigma_publisher.publish(self.sigma_msg)

            # For measurements------------------------------------------------------------------------------------------


            for sigma in range(len(s)):
                    # If a singularity is detected
                    if s[sigma] < self.singularity_entry_threshold:
                        if self.bool_singularity == False:
                            self.bool_singularity = True
                            rospy.loginfo("Activate OLMM")
                            
                        if s[sigma] < self.singularity_min_threshold:
                            rospy.loginfo("Singularity stop activated! Sigma %f is smaller then threshol %f",sigma_min,self.singularity_min_threshold)
                            self.singularity_stop = True
                        else:
                            if s[sigma] > self.singularity_min_threshold:
                                self.singularity_avoidance_velocity = np.dot(self.scalar_adjusting_function(s[sigma]),np.dot(np.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))
                            else:
                                self.singularity_avoidance_velocity = np.dot(1,np.dot(np.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))
            # if  self.singularity_entry_threshold < sigma_min < self.singularity_exit_threshold and self.bool_singularity == True:
            #     self.singularity_avoidance_velocity = np.dot(self.scalar_adjusting_function(s[sigma]),np.dot(np.dot(u[:,sigma],self.target_cartesian_velocity),u[:,sigma]))

                
            # If a singularity was detected but the singularity_counter is null, 
            if sigma_min > self.singularity_entry_threshold and self.bool_singularity == True:

                #* Reset the bool_singularity and acitivate the bool_reduce_singularity_offset
                self.bool_singularity = False
                self.bool_reduce_singularity_offset = True
                rospy.loginfo("Deactivate OLMM")
                #* Reset the singularity velocity world to zero
                self.singularity_avoidance_velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
                
            #* Transform the singularity avoidance velocity into 'world' frame (from jacobian / command_output frame)
            self.singularity_velocity_world = self.transform_vector(self.command_out_frame,self.command_in_frame,self.singularity_avoidance_velocity)
            #* Publish the singularity avoidance velocity to rosmaster
            self.singularity_velocity_msg.singularity_stop = self.singularity_stop
            self.singularity_velocity_msg.singularity_velocity = self.singularity_velocity_world
            self.singularity_velocity_pub.publish(self.singularity_velocity_msg)  
            
            #* Check for trajectory differences:
            if self.bool_reduce_singularity_offset == True:
                self.singularity_velocity_world = np.array([0.0,0.0,0.0,0.0,0.0,0.0]) 
                if np.linalg.norm(pose_diff[:3]) < self.singularity_trans_offset_accuracy and np.linalg.norm(pose_diff[3:]) < self.singularity_rot_offset_accuracy:
                    
                    self.bool_reduce_singularity_offset = False
                    
                    rospy.loginfo("Endeffector trans difference %f [m] is samller then threshold %f [m]",np.linalg.norm(pose_diff[:3]),self.singularity_trans_offset_accuracy)
                    rospy.loginfo("Endeffector rot difference %f [rad] is samller then threshold %f [rad]",np.linalg.norm(pose_diff[3:]),self.singularity_rot_offset_accuracy)



            # For measurements------------------------------------------------------------------------------------------
            self.singularity_msg.data = self.bool_singularity 
            self.sigularity_publisher.publish(self.singularity_msg)
            
            
            
            self.offset_msg.data = self.bool_reduce_singularity_offset
            self.offset_publisher.publish(self.offset_msg)

            # For measurements------------------------------------------------------------------------------------------
# * Singulartiy avoidance: OLMM-----------------------------------------------------------------------------------------
            #* Subract the target cartesian velocity with the singularity avoidance velocity
            if self.bool_singularity == True and self.singularity_stop == False:
                rospy.logdebug("target_cartesian_velocity - singularity_avoidance_velocity")
                self.target_cartesian_velocity = self.target_cartesian_velocity - self.singularity_avoidance_velocity
                
            elif (self.bool_singularity == True and self.singularity_stop == True) or self.workspace_violation == True:
                self.target_cartesian_velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
            
            # * Calculate the inverse of the jacobian-matrix
            self.inverse_jacobian = np.linalg.inv(self.jacobian)
            
            # * Calculate the target joint velocity with the inverse jacobian-matrix and the target cartesain velociy
            # self.target_joint_velocity.data = self.inverse_jacobian@self.target_cartesian_velocity
            joint_velocity= self.inverse_jacobian@self.target_cartesian_velocity
            
            # # TODO: test: max_vel
            # max_vel = np.max(joint_velocity)
            # if max_vel >0.08:
            #     joint_velocity=joint_velocity/max_vel*0.08
            self.target_joint_velocity.data = joint_velocity
            
            # * Publish the target_joint_velocity
            self.joint_velocity_pub.publish(self.target_joint_velocity)
            
            self.cartesian_vel_msg.data = self.target_cartesian_velocity
            self.cart_vel_pub.publish(self.cartesian_vel_msg)
            
            # * Sleep for publish_rate
            rate.sleep()

    def shutdown(self):
        """ 
            This function is called by rospy.on_shutdown!
        """
        print("Shutdown amittcance controller:")
        print("Shutdown publisher joint velocity!")
        self.joint_velocity_pub.publish(self.shutdown_joint_velocity)
        print("Unregister from joint_velocity_pub!")
        self.joint_velocity_pub.unregister()
        print("Unregister from wrench_ext_sub!")
        self.wrench_ext_sub.unregister()
        
    
if __name__ == '__main__':
    ur_admittance_controller()
    