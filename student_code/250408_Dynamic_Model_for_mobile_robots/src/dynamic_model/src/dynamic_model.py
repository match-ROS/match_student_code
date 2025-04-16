#!/usr/bin/env python3

"""
Dynamic Model for Mobile Robots

This module implements a dynamic model for mobile robots that:
  - Initializes a ROS node and loads robot configuration parameters.
  - Subscribes to various sensor topics (IMU, wheel encoders) to collect real-time data.
  - Uses sensor data buffers and interpolation to estimate the current state.
  - Integrates a dynamic model to compute state evolution and publishes the estimated state.
  - Computes forces and moments (including chassis dynamics, suspension, and wheel forces) acting on the robot.

The code structure includes:
  - Utility functions for ROS connectivity and coordinate transformations.
  - A configuration class (RobotConfig) that loads robot parameters from JSON.
  - A main dynamics class (DynModell) that processes sensor data, updates kinematics, and computes dynamics.
"""

import rospy
import rospkg

#Ros Messages
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from custom_msgs.msg import StateVector

#Check Ros Running
from xmlrpc.client import ServerProxy
from urllib.parse import urlparse

#Basic Imports
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import math
from scipy.spatial.transform import Rotation as R
from scipy.signal import resample
import json
from collections import deque
import threading
import csv #shutdown handler

#Solver
from scipy.integrate._ivp.bdf import BDF
from scipy.integrate import Radau
from scipy.integrate import solve_ivp

from ahrs.filters import Mahony

#Klassen
from odometrie_mecanum import mecanum_kinematic
from odometrie_tracked import tracked_kinematic
from odometrie_differential import differential_kinematic
from rls import SpringDamperRLS
from oneDKalman import ZKalmanFilter 
from savgol import  savgol_filter
from CausalButterworth import CausalButterworth

def is_ros_master_running():
    """
    Check if the ROS master is running by attempting a connection and calling a test method.
    Returns:
        True if the ROS master is reachable, False otherwise.
    """
    try:
        # Get the ROS master URI
        master_uri = rospy.get_param('/ros_master_uri', 'http://localhost:11311')
        parsed_uri = urlparse(master_uri)

        # Connect to the master
        master = ServerProxy(f"http://{parsed_uri.hostname}:{parsed_uri.port}")
        # Call the getSystemState method to test connectivity
        master.getSystemState('/')
        return True
    except Exception as e:
        rospy.logwarn(f"Cannot connect to ROS Master: {e}")
        return False


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion (x, y, z, w) into Euler angles (roll, pitch, yaw).
    Parameters:
        x, y, z, w: Quaternion components.
    Returns:
        A tuple (roll, pitch, yaw).
    """
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))     # Roll (X-axis rotation)
    pitch = np.arcsin(2 * (w * y - z * x))     # Pitch (Y-axis rotation)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2)) # Yaw (Z-axis rotation) 
    return roll, pitch, yaw


class RobotConfig:
    def __init__(self):
        self.load_robot_config()

    def load_robot_config(self):
        """
        Load the robot settings from a JSON file and assign configuration parameters.
        
        The parameters include:
          - Robot type and kinematic properties.
          - Physical dimensions and mass.
          - Sensor configurations (e.g., lidar, camera, IMU, encoders).
          - Inertia properties and wheel positions.
        """
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('dynamic_model')
        robot_settings_path = os.path.join(package_path, "src", "robot_settings.json")

        print(f"{robot_settings_path}")

        # Load the JSON file into a Python dictionary
        with open(robot_settings_path, "r") as file:
            robot_settings = json.load(file)

        self.robot_type = robot_settings["robot_typ"]
        #Dimensionierung
        self.dim_x = robot_settings['dimension']['width']   
        self.dim_y = robot_settings['dimension']['depths']
        self.dim_z = robot_settings['dimension']['heigth']
        self.track_width = robot_settings['dimension']['track_width']
        self.wheel_radius = robot_settings['dimension']['wheel_radius']
        self.wheel_base = robot_settings["dimension"]["wheel_base"]
        self.motor_count = robot_settings["motor_count"]
        self.spring_stiffness = robot_settings['dimension']["spring_stiffness"]
        self.damping = robot_settings['dimension']["damping"]

        #Rad Position
        self.front_left = robot_settings['dimension']["wheel_position"]["front_left"]
        self.front_right = robot_settings['dimension']["wheel_position"]["front_right"]
        self.rear_left = robot_settings['dimension']["wheel_position"]["rear_left"]
        self.rear_right = robot_settings['dimension']["wheel_position"]["rear_right"]
        self.spring_equilibrium = robot_settings['dimension']["wheel_position"]["rear_right"][2]
        
        #Gewicht
        self.mass_mainframe = robot_settings['mass_mainframe']
        self.wheel_mass = robot_settings["wheel_mass"]
        #Sensoren
        self.lidar = robot_settings["sensors"]['lidar']["range_meters"]
        self.lidar_resolution = robot_settings["sensors"]['lidar']["angle_resolution"]
        self.lidar_topic = robot_settings["sensors"]['lidar']["topic"]
        self.camera_resolution = robot_settings["sensors"]['camera']["resolution"]
        self.camera_framerate = robot_settings["sensors"]['camera']["frame_rate"]
        self.imu_update_rate = robot_settings["sensors"]['imu']["update_rate"]
        self.imu_topic = robot_settings["sensors"]["imu"]["topic"]
        
        self.left_encoder_rate = robot_settings["sensors"]['encoder']["left_encoder"]['update_rate']
        self.left_encoder_topic = robot_settings["sensors"]['encoder']["left_encoder"]['topic']
        self.left_encoder_unit = robot_settings["sensors"]['encoder']["left_encoder"]["unit"]
        self.left_encoder_sign = robot_settings["sensors"]['encoder']["left_encoder"]["sign"]

        self.right_encoder_rate = robot_settings["sensors"]['encoder']["right_encoder"]['update_rate']
        self.right_encoder_topic = robot_settings["sensors"]['encoder']["right_encoder"]['topic']
        self.right_encoder_unit = robot_settings["sensors"]['encoder']["right_encoder"]["unit"]
        self.right_encoder_sign = robot_settings["sensors"]['encoder']["right_encoder"]["sign"]

        #Trägheitsmomente aus Config berechnen
        self.I_x = 1/12.0 * self.mass_mainframe * (self.dim_y**2 + self.dim_z**2)
        self.I_y = 1/12.0 * self.mass_mainframe * (self.dim_x**2 + self.dim_z**2)
        self.I_z = 1/12.0 * self.mass_mainframe * (self.dim_x**2 + self.dim_y**2)
        self.I_z_wheel = 1/2.0 * self.wheel_mass * self.wheel_radius**2
        self.I_chassis = np.diag([self.I_x, self.I_y, self.I_z])

        # Wheel positions in chassis frame (front-left, front-right, rear-left, rear-right)
        # 1 Column per Wheel
        self.wheel_positions = np.array([
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right
        ])


class DynModell(): #RobotBase
    def __init__(self, config):
        """
        Initialize the dynamic model node.
        
        Performs:
          - ROS master check and node initialization.
          - Initialization of sensor buffers and state variables.
          - Selection of the appropriate kinematic model based on robot type.
          - Subscription to sensor topics and setup of publishers.
          - Integrating the state forward.
        """

        if is_ros_master_running():
            print("RosMaster Found\nStarting Dynamic Model Node")   
            rospy.init_node('tracked_mobile_robot')
            rospy.sleep(0.01)
            startup_time = rospy.Time().now().to_sec()      

            #Sensor Variablen
            print("startup_time: ", startup_time)
            self.imu_time = startup_time
            self.imu_time_LU = deque(maxlen=2)
            self.imu_time_LU.append(startup_time)

            self.last_solver_time = deque(maxlen=2) # LU = Zuletzt vom Dyn Modell verwendet
            self.last_solver_time.append(startup_time)
            self.front_right_wheel_time_LU = deque(maxlen=2) # LU = Zuletzt vom Dyn Modell verwendet
            self.front_right_wheel_time_LU.append(startup_time)
            self.rear_left_wheel_time_LU = deque(maxlen=2) # LU = Zuletzt vom Dyn Modell verwendet
            self.rear_left_wheel_time_LU.append(startup_time)
            self.rear_right_wheel_time_LU = deque(maxlen=2) # LU = Zuletzt vom Dyn Modell verwendet
            self.rear_right_wheel_time_LU.append(startup_time)
            
        self.config = config

        
        if config.robot_type == "omnidirectional":            
            self.kinematic = mecanum_kinematic(config.wheel_radius, config.wheel_base, config.track_width)
        elif config.robot_type == "tracked":
            self.kinematic = tracked_kinematic(config.wheel_radius, config.wheel_base, config.track_width)
        elif config.robot_type == "differential":
            self.kinematic = differential_kinematic(config.wheel_radius, config.wheel_base, config.track_width) 
       
        self.update_last_used = False

        self.freq_encoder = 1
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.v_z = 0.0
        self.phi = 0.0  # roll
        self.theta = 0.0  # pitch
        self.psi = 0.0  # yaw
        self.linear_velocity = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = deque(maxlen=2)
        self.angular_velocity.append((0,0,0))
        self.linear_acceleration = deque(maxlen=2) # Initialize linear acceleration
        self.linear_acceleration.append((0,0,0))
        self.imu_data = 0.0

        self.buffer_lock = threading.Lock()
        self.imu_acc_buffer = deque(maxlen=100)
        self.imu_acc_buffer.append((startup_time, 0, 0, 0))
        self.imu_angular_buffer = deque(maxlen=100)
        self.imu_angular_buffer.append((startup_time, 0, 0, 0))
        self.imu_acc_buffer = deque(maxlen=100)
        self.imu_acc_buffer.append((startup_time, 0, 0, 0))
        self.imu_angular_buffer = deque(maxlen=100)
        self.imu_angular_buffer.append((startup_time, 0, 0, 0))
        self.imu_z_savgol_que = deque(maxlen=20)

        self.imu_quat_buffer = deque(maxlen=100)
        self.imu_quat_buffer.append((startup_time, 0, 0, 0, 1))

        self.z_wheels = config.wheel_radius

        self.front_left_buffer = deque(maxlen=100)
        self.front_left_buffer.append((startup_time, 0))
        self.front_left_wheel_angle_LU = deque(maxlen=2) # LU = Zuletzt vom Dyn Modell verwendet
        self.front_left_wheel_angle_LU.append(0)
        self.front_left_wheel_velocity = 0.0
        self.front_left_wheel_velocity_LU = 0.0 # LU = Zuletzt vom Dyn Modell verwendet

        self.front_right_buffer = deque(maxlen=100)
        self.front_right_buffer.append((startup_time, 0))
        self.front_right_wheel_angle_LU = deque(maxlen=2) # LU = Zuletzt vom Dyn Modell verwendet
        self.front_right_wheel_angle_LU.append(0)
        self.front_right_wheel_velocity = 0.0
        self.front_right_wheel_velocity_LU = 0.0 # LU = Zuletzt vom Dyn Modell verwendet
 
        self.rear_left_buffer = deque(maxlen=100)
        self.rear_left_buffer.append((startup_time, 0))
        self.rear_left_wheel_angle_LU = deque(maxlen=2)
        self.rear_left_wheel_angle_LU.append(0)
        self.rear_left_wheel_velocity = 0.0
        self.rear_left_wheel_velocity_LU = 0.0

        self.rear_right_buffer = deque(maxlen=100)
        self.rear_right_buffer.append((startup_time, 0))
        self.rear_right_wheel_angle_LU = deque(maxlen=2)
        self.rear_right_wheel_angle_LU.append(0)
        self.rear_rigth_wheel_velocity = 0.0
        self.rear_right_wheel_velocity_LU = 0.0

        self.diff_acc = deque(maxlen=100)
        self.diff_acc.append((0,0,0))

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.left_wheel_time_diff = 0 

        if config == []:
            self.wheel_mass = 0.0
            self.wheel_radius = 0.0
            self.track_width = 0.0
            self.dim_x = 0.0
            self.dim_y = 0.0
            self.dim_z = 0.0
            self.I_x = 0.0
            self.I_y = 0.0
            self.I_z = 0.0
            self.I_z_wheel = 0.0
        
        self.g = 9.81

        # Allgemeine Größen
        self.c_fric = 50.0

        # ---------------------------------------------------------------------
        # State Vector Definition:
        # [x, y, z, qx, qy, qz, qw, vx, vy, vz, p, q, r, alpha1, alpha2, alpha3, alpha4, dalpha1, dalpha2, dalpha3, dalpha4]
        # where:
        #   - x, y, z: position in inertial frame
        #   - qx, qy, qz, qw: orientation quaternion (body frame)
        #   - vx, vy, vz: linear velocities in inertial frame
        #   - p, q, r: angular velocities in body frame
        #   - alpha1...alpha4: wheel angular positions/velocities (model-specific)        
        self.state = np.zeros(21)
        self.state[0] = 0
        self.state[1] = 0
        self.state[2] = 0.100998
        self.state[6] = 1  # quaternione x,y,z = 0 ; w = 1
        
        self.dt = 0.1
        self.max_step = self.dt/10

        self.t = 0.0
    
        self.solver_t = 0.0 
        
        # Param Identification Methods Values
        self.springDamperRLS = SpringDamperRLS(config.mass_mainframe)
        self.zKalman = ZKalmanFilter(dt=0.1)
        self.est = []
        self.z_disp_prev = 0.0
        self.z_disp_dot_prev = 0.0
        
        #Filter
        self.savgol = savgol_filter(11, 3)
        self.bwFilter = CausalButterworth()


        # -----------------------------------------------------------------------------
        # ROS Subscribers and Publishers Initialization
        # ---------------------------------------------------------------------------
        if is_ros_master_running():
            print("\nStarting Subscriber and Publisher")   
            self.t0 = rospy.Time().now().to_sec() 
            print("t0: ", self.t0)
            rospy.Subscriber(config.imu_topic, Imu, self.imu_callback,queue_size=10)

            if config.motor_count >= 2:
                rospy.Subscriber(config.right_encoder_topic, JointState, self.fr_wheel_callback,queue_size=10)
                rospy.Subscriber(config.left_encoder_topic, JointState, self.fl_wheel_callback,queue_size=10)
            if config.motor_count >=4 :
                rospy.Subscriber(config.right_encoder_topic, JointState, self.rr_wheel_callback,queue_size=10)
                rospy.Subscriber(config.left_encoder_topic, JointState, self.rl_wheel_callback,queue_size=10)
            
            self.state_publisher = rospy.Publisher('/robot_state', StateVector, queue_size=10)
            self.force_publisher = rospy.Publisher('/dynModForce', StateVector, queue_size=10)
            self.moment_publisher = rospy.Publisher('/dynModMoment', PoseStamped, queue_size=10)
            self.acc_pub = rospy.Publisher("/lin_acc", PoseStamped, queue_size=10)
            self.acc_filt_pub = rospy.Publisher("/lin_acc_filt", PoseStamped, queue_size=10)

            rospy.on_shutdown(self.shutdown_handler)
           
            #Iteration solver for state calculation
            rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
            rospy.loginfo("Stiff solver started.")


    # ---------------------------------------------------------------------
    # Callback Functions
    # ---------------------------------------------------------------------    
    def timer_callback(self, event):
        """
        Called by ROS every self.dt seconds. 
        Advance the solver up to (self.t + self.dt).
        Calculates newest state and publishes it.
        """
        
        # Integrate forward 
        self.solver_t = self.solver_t + self.dt

        dstate, dt = self.robot_dynamics(self.solver_t, self.state)

        # Update global time and state from the solver
        self.t = self.solver_t
        self.state = self.state + dstate * dt

        # Log or publish the result
        state_msg = StateVector()
        state_msg.header.stamp = rospy.Time.from_sec(self.t0 + self.t)
        state_msg.state = self.state
        self.state_publisher.publish(state_msg)
        self.update_last_used = True
        rospy.loginfo(f"[StiffSolver] t={self.t:.4f}, y={self.state[0:3]}")
        
        
    def imu_callback(self, msg):
        """
        Callback for incoming IMU messages.
        Updates time stamps and appends linear acceleration, angular velocity,
        and quaternion data to the respective buffers.
        """
        self.imu_time = msg.header.stamp.to_sec()
        self.imu_acc_buffer.append((self.imu_time, msg.linear_acceleration.x, msg.linear_acceleration.y , msg.linear_acceleration.z))
        self.imu_angular_buffer.append((self.imu_time, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        self.imu_quat_buffer.append((self.imu_time, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))


    def fl_wheel_callback(self, msg):
        """
        Front Left Wheel Encoder Callback.
        Processes the JointState message and appends the wheel angle to the buffer.
        """
        msg_time = msg.header.stamp.to_sec()
        if self.config.left_encoder_sign == "plus":
            fl_angle = msg.position[0] #JointState MSG     
        elif self.config.left_encoder_sign == "minus":
            fl_angle = -1* msg.position[0] #JointState MSG     
        else:
            raise NameError("Error in Config: Choose 'plus' ord 'minus' as sign of enocer") 
        if msg.velocity:
            self.front_left_buffer.append((msg_time, fl_angle))


    def fr_wheel_callback(self, msg):
        """
        Front Right Wheel Encoder Callback.
        Processes the JointState message and appends the wheel angle to the buffer.
        """       
        msg_time = msg.header.stamp.to_sec()
        if self.config.right_encoder_sign == "plus":
            fr_angle = msg.position[1] #JointState MSG     
        elif self.config.right_encoder_sign == "minus":
            fr_angle = -1* msg.position[1] #JointState MSG     
        else:
            raise NameError("Error in Config: Choose 'plus' ord 'minus' as sign of enocer") 
        if msg.velocity:
            self.front_right_buffer.append((msg_time, fr_angle))


    def rl_wheel_callback(self, msg):
        """
        Rear left Wheel Encoder Callback.
        Processes the JointState message and appends the wheel angle to the buffer.
        """        
        msg_time = msg.header.stamp.to_sec()
        if self.config.left_encoder_sign == "plus":
            rl_angle = msg.position[2] #JointState MSG     
        elif self.config.left_encoder_sign == "minus":
            rl_angle = -1* msg.position[2] #JointState MSG     
        else:
            raise NameError("Error in Config: Choose 'plus' ord 'minus' as sign of enocer") 
        if msg.velocity:
            self.rear_left_buffer.append((msg_time, rl_angle))
        

    def rr_wheel_callback(self, msg):
        """
        Rear Right Wheel Encoder Callback.
        Processes the JointState message and appends the wheel angle to the buffer.
        """
        msg_time = msg.header.stamp.to_sec()
        if self.config.right_encoder_sign == "plus":
            rr_angle = msg.position[3] #JointState MSG     
        elif self.config.right_encoder_sign == "minus":
            rr_angle = -1* msg.position[3] #JointState MSG     
        else:
            raise NameError("Error in Config: Choose 'plus' ord 'minus' as sign of enocer") 
        if msg.velocity:
            self.rear_right_buffer.append((msg_time, rr_angle))


    def shutdown_handler(self):
        """This function is called automatically when the node shuts down.
            To be saved data can be declared here.
        """
        
        rospy.loginfo("Shutdown detected. Saving buffer to CSV...")
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('dynamic_model')
        robot_settings_path = os.path.join(package_path, "src/")
        est_df = pd.concat(self.est, ignore_index=True)
        est_df.to_csv(robot_settings_path+"est.csv",index=False)
        rospy.loginfo("Saved")

    def save_buffer_to_csv(self, filename):
        """Saves the entire buffer to a CSV file.       
        This function locks the buffer during file writing to avoid concurrent modifications."""

        with self.buffer_lock:
            local_buffer = list(self.front_left_buffer)

        # Write to CSV
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Optional header
                writer.writerow(["time", "imu_phi"])
                for (t, psi) in local_buffer:
                    writer.writerow([t, psi])
            rospy.loginfo(f"Buffer saved to {filename} with {len(local_buffer)} entries.")
        except Exception as e:
            rospy.logerr(f"Failed to write buffer to CSV: {e}")

    @staticmethod
    def save_list_to_csv(data, filename):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('dynamic_model')
        filepath = os.path.join(package_path, "src/", filename)

        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            for item in data:
                writer.writerow([item])
                
                
    # ---------------------------------------------------------------------
    # Interpolate Functions
    # ---------------------------------------------------------------------  
    @staticmethod
    def interpolate_sensor_value(sensor_data_buffer, query_time):
        """
        Interpolate a 1D sensor value from a buffer of (time, data) pairs.

        Parameters:
            sensor_data_buffer: List/array of (t, d) in ascending order.
            query_time: The time at which to estimate the sensor value.

        Returns:
            The interpolated sensor reading as a float.
        """
        if not sensor_data_buffer:
            return None  # or some default/fallback

        # Extract time and data arrays
        times = np.array([p[0] for p in sensor_data_buffer], dtype=float)
        values = np.array([p[1] for p in sensor_data_buffer], dtype=float)

        # If query_time is out of bounds, handle edge cases
        if query_time <= times[0]:
            return values[0]  # earliest sensor value
        if query_time >= times[-1]:
            return values[-1] # latest sensor value

        # Perform 1D linear interpolation
        interpolated_value = np.interp(query_time, times, values)
        return interpolated_value
    

    @staticmethod
    def interpolate_sensor_value_3D(sensor_data_buffer_3D, query_time):
        """
        Interpolate a 3D sensor value from a buffer of (time, x, y, z) tuples.

        Parameters:
            sensor_data_buffer_3D: List/array of (t, x, y, z) in ascending order.
            query_time: The time at which to estimate the sensor value.
        
        Returns:
            A NumPy array with interpolated [x, y, z] values.
        """
        if not sensor_data_buffer_3D:
            return None  # or some default/fallback
        
        # Extract time and data arrays
        times = np.array([p[0] for p in sensor_data_buffer_3D], dtype=float)
        values_x = np.array([p[1] for p in sensor_data_buffer_3D], dtype=float)
        values_y = np.array([p[2] for p in sensor_data_buffer_3D], dtype=float)
        values_z = np.array([p[3] for p in sensor_data_buffer_3D], dtype=float)
        #print(f"query_time: {query_time}, buffer[0]: {times[0]}, buffer[-1]: {times[-1]}")


        # If query_time is out of bounds, handle edge cases
        if query_time <= times[0]:
            return np.array([values_x[0], values_y[0] ,values_z[0]])   # earliest sensor value
        if query_time >= times[-1]:
            return np.array([values_x[-1], values_y[-1] ,values_z[-1]]) # latest sensor value
        
            
        # Perform 1D linear interpolation
        interpolated_value_x = np.interp(query_time, times, values_x)
        interpolated_value_y = np.interp(query_time, times, values_y)
        interpolated_value_z = np.interp(query_time, times, values_z)
        return np.array([interpolated_value_x, interpolated_value_y ,interpolated_value_z])


    @staticmethod
    def interpolate_sensor_value_4D(sensor_data_buffer_4D, query_time):
        """
        Interpolate a 4D sensor value from a buffer of (time, x, y, z, w) tuples.

        Parameters:
            sensor_data_buffer_4D: List/array of (t, x, y, z, w) in ascending order.
            query_time: The time at which to estimate the sensor value.
        
        Returns:
            A NumPy array with interpolated [x, y, z, w] values.
        """
        if not sensor_data_buffer_4D:
            return None  # or some default/fallback
        
        # Extract time and data arrays
        times = np.array([p[0] for p in sensor_data_buffer_4D], dtype=float)
        values_x = np.array([p[1] for p in sensor_data_buffer_4D], dtype=float)
        values_y = np.array([p[2] for p in sensor_data_buffer_4D], dtype=float)
        values_z = np.array([p[3] for p in sensor_data_buffer_4D], dtype=float)
        values_w = np.array([p[4] for p in sensor_data_buffer_4D], dtype=float)

        #print(f"query_time: {query_time}, buffer[0]: {times[0]}, buffer[-1]: {times[-1]}")

        # If query_time is out of bounds, handle edge cases
        if query_time <= times[0]:
            return np.array([values_x[0], values_y[0] ,values_z[0], values_w[0]])   # earliest sensor value
        if query_time >= times[-1]:
            return np.array([values_x[-1], values_y[-1] ,values_z[-1], values_w[-1]]) # latest sensor value
        
        # Perform 1D linear interpolation
        interpolated_value_x = np.interp(query_time, times, values_x)
        interpolated_value_y = np.interp(query_time, times, values_y)
        interpolated_value_z = np.interp(query_time, times, values_z)
        interpolated_value_w = np.interp(query_time, times, values_w)

        return np.array([interpolated_value_x, interpolated_value_y ,interpolated_value_z, interpolated_value_w])


    # ---------------------------------------------------------------------
    # Helper Functions
    # ---------------------------------------------------------------------

    def euler_to_rotation_matrix(self, phi, theta, psi):
        """Convert roll-pitch-yaw (phi,theta,psi) to rotation matrix from body to inertial."""
        # Rotation about x-axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi),  np.cos(phi)]
        ])
        # Rotation about y-axis (pitch)
        R_y = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        # Rotation about z-axis (yaw)
        R_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0, 0, 1]
        ])
        R = R_z @ R_y @ R_x
        return R

    def body_rates_to_euler_rates(self, phi, theta, psi, p, q, r):
        """Transform body angular rates p,q,r to Euler angle rates phi_dot, theta_dot, psi_dot."""
        # Transformation matrix T depends on current angles
        T = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])
        #print("thste: ", theta)
        euler_dots = T @ np.array([p, q, r])
        return euler_dots

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix."""
        if np.linalg.norm(q) == 0:
            x, y, z, w = np.array([0, 0, 0, 0])
        else:
            x, y, z, w = q / np.linalg.norm(q)

        R = np.array([
            [1 - 2*(y**2 + z**2),    2*(x*y - z*w),      2*(x*z + y*w)],
            [2*(x*y + z*w),          1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),          2*(y*z + x*w),      1 - 2*(x**2 + y**2)]
        ])

        return R

    def quaternion_kinematics(self, q, omega):
        """
        Compute the derivative of a quaternion given angular velocity in the body frame.

        Parameters:
            q: The current quaternion.
            omega: A tuple (p, q_rate, r) representing angular velocities.

        Returns:
            The derivative of the quaternion.
        """        
        p, q_rate, r = omega # Angular velocities in body frame
        Omega = np.array([
            [0,      r,     -q_rate,  p],
            [-r,     0,      p,       q_rate],
            [q_rate, -p,     0,       r],
            [-p,    -q_rate, -r,       0]
        ])
        q_dot = 0.5 * Omega @ q
        return q_dot

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (phi, theta, psi)."""
        if np.linalg.norm(q) == 0:
            x, y, z, w = np.array([0, 0, 0, 0])
        else:
            x, y, z, w = q / np.linalg.norm(q)
        phi   = np.arctan2(2*(w*x + y*z),      1 - 2*(x**2 + y**2))
        theta = np.arcsin(2*(w*y - z*x))
        psi   = np.arctan2(2*(w*z + x*y),      1 - 2*(y**2 + z**2))
        
        return phi, theta, psi

    def suspension_forces(self, delta, delta_dot, R, X, Y, Z):
        """
        Compute suspension forces:
        delta_i is computed as the difference between the wheel vertical position 
        along z_b and the equilibrium delta0.
        """
        F_susp = np.zeros(4)
        for i in range(4):
            #delta_dot = 0 #Räder auf Boden
            F_susp[i]  = -config.spring_stiffness * delta[i] - config.damping * delta_dot[i]
            # This force acts along z_b axis
        return F_susp

    def compute_total_inertia(self, wheel_positions):
        """
        Compute the total inertia tensor including the chassis and wheels using the Parallel Axis Theorem.
        """
        # Chassis inertia tensor (mainframe only)
        I_chassis = np.diag([config.I_x, config.I_y, config.I_z])

        # Initialize total inertia tensor
        I_total = I_chassis

        # Wheel contributions (Parallel Axis Theorem)
        for wheel_pos in wheel_positions:
            r = np.array(wheel_pos)  # Position vector of wheel relative to chassis center
            r_norm_sq = np.dot(r, r)  # Square of the distance to center of mass
            
            # Local inertia tensor for a cylindrical wheel (rotating around z-axis)
            I_wheel_local = np.diag([config.I_z_wheel, config.I_z_wheel, 0])

            # Parallel Axis Theorem: I_total += I_local + m * (r^2 * I - r*r^T)
            I_parallel = config.wheel_mass * (r_norm_sq * np.eye(3) - np.outer(r, r))
            I_total += I_wheel_local + I_parallel

        return I_total
    
    def calculate_wheel_forces(self, ax, ay, h, roll_angle):
        """
        Calculate the vertical forces on each wheel based on vehicle dynamics.

        Parameters:
            ax: Longitudinal acceleration (m/s^2).
            ay: Lateral acceleration (m/s^2).
            h:  Height of the center of gravity (m).
            roll_angle: Roll angle of the vehicle (radians).

        Returns: 4x3 Array with forces in x, y, z direction for each wheal front left, front right, rear left, rear right
        """
        trackWidth = np.abs(config.wheel_positions[0, 1] - config.wheel_positions[1, 1])  # Front track width
        axisDistance = np.abs(config.wheel_positions[0, 0])  # Distance to front/rear axle (from CoM)       
        
        # Calculate total vehicle length
        l = 2*axisDistance
        
        F_ground = np.zeros((4,3))
        
        # Calculate vertical forces on the front left and right wheels
        F_ground[0][2] = (0.5 * config.mass_mainframe * ((axisDistance / l) * self.g - (h / l) * ax)) + config.mass_mainframe * ((axisDistance / l) * self.g - (h * ay) / l)* h /(trackWidth*self.g)*ay
        F_ground[1][2] = (0.5 * config.mass_mainframe * ((axisDistance / l) * self.g - (h / l) * ax)) - config.mass_mainframe * ((axisDistance / l) * self.g - (h * ay) / l)* h /(trackWidth*self.g)*ay

        # Calculate vertical forces on the rear left and right wheels
        F_ground[2][2] = (0.5 * config.mass_mainframe * ((axisDistance / l) * self.g + (h / l) * ax)) + config.mass_mainframe * ((axisDistance / l) * self.g + (h * ay) / l)* h /(trackWidth*self.g)*ay
        F_ground[3][2] = (0.5 * config.mass_mainframe * ((axisDistance / l) * self.g + (h / l) * ax)) - config.mass_mainframe * ((axisDistance / l) * self.g + (h * ay) / l)* h /(trackWidth*self.g)*ay
        
        return F_ground
    
    def identify_parameter(self, z, dz, ddz):
        '''Updates RLS-Algorithm with new data and estimates the spring and damper coefficients.'''
        c_est, k_est = self.springDamperRLS.update(z,dz,ddz)

        df = {
            "time": self.t0+self.solver_t,
            "c": c_est,
            "k": k_est
        }
        self.est.append(pd.DataFrame([df]))
        
    @staticmethod
    def remove_gravity_from_acceleration(accel_kfs, quaternions):
        """
        Remove the gravitational acceleration component from the measured acceleration.
        
        Parameters:
            accel_kfs: Measured acceleration in the body frame.
            quaternions: The orientation quaternion.
        
        Returns:
            Corrected acceleration with gravity removed.
        """
        # Erdbeschleunigung im Weltkoordinatensystem (WKS)
        g_wks = np.array([0, 0, 9.81])
        g_wks = np.array([0, 0, 9.81])
        
        # Rotation basierend auf Quaternion erstellen
        rotation = R.from_quat(quaternions)
        
        # Erdbeschleunigung in das körperfeste Koordinatensystem rotieren
        g_kfs = rotation.inv().apply(g_wks)
        
        # Erdbeschleunigung von der gemessenen Beschleunigung abziehen
        corrected = accel_kfs - g_kfs
        
        return corrected
    
    @staticmethod
    def low_pass_filter(new_value, prev_value, alpha):
        A = np.array([0.0,0.0,0.0])
        for i in range(3):
            A[i] = alpha * new_value[i] + (1 -alpha) *prev_value[i]
            #print(f"Prev: {prev_value[i]}, New: {new_value[i]}, A[i]: {A[i]}")
        return A

    def normalize_angle(angle):
        """Wrap an angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def update_kinematic(self, linear_acceleration, ang_vel, quat):
        """
        Update the kinematic state based on sensor data.
        
        Interpolates the sensor buffers (encoder and IMU) to estimate:
          - Translational movement (dx, dy, dz)
          - Orientation (roll, pitch, yaw)
        
        Returns:
            A tuple (dx, dy, dz, roll, pitch, yaw)
        """
        update_yaw = False
        solver_time = self.t0 + self.solver_t

        # Copy buffers to local variables
        fl_buffer = list(self.front_left_buffer)
        fr_buffer = list(self.front_right_buffer)
        rl_buffer = list(self.rear_left_buffer)
        rr_buffer = list(self.rear_right_buffer)

        # Interpolate current Sensor Value
        fl_interp = self.interpolate_sensor_value(fl_buffer, solver_time )
        fr_interp = self.interpolate_sensor_value(fr_buffer, solver_time )
        rl_interp = self.interpolate_sensor_value(rl_buffer, solver_time )
        rr_interp = self.interpolate_sensor_value(rr_buffer, solver_time )

        self.imu_z_savgol_que.append(linear_acceleration[2])
        imu_z_filt = self.bwFilter.process_sample(linear_acceleration[2])
        
        self.roll, self.pitch, yaw = self.quaternion_to_euler(quat)

        if not self.last_solver_time[-1] == solver_time:
            self.last_solver_time.append(solver_time)
            update_yaw = True
        if not self.front_left_wheel_angle_LU[-1] == fl_interp:
            self.front_left_wheel_angle_LU.append(fl_interp)
        if not self.front_right_wheel_angle_LU[-1] == fr_interp:
            self.front_right_wheel_angle_LU.append(fr_interp)

        if not self.rear_right_wheel_angle_LU[-1] == rr_interp:
            self.rear_right_wheel_angle_LU.append(rr_interp)

        if not self.rear_left_wheel_angle_LU[-1] == rl_interp:
            self.rear_left_wheel_angle_LU.append(rl_interp)

        if not self.imu_time_LU[-1] == solver_time:
            self.imu_time_LU.append(solver_time)

        #Calculate traveled distance based on encoder data and the configuration of robot (omnidirectional, tracked, differential)
        dt_encoder = (solver_time - self.last_solver_time[0])
        dt_imu = (solver_time - self.imu_time_LU[0])
        #print(f"dt_enc: {dt_encoder}, dt_imu: {dt_imu}")
       
        delta_theta_fl = fl_interp - self.front_left_wheel_angle_LU[0]
        delta_theta_fr = fr_interp - self.front_right_wheel_angle_LU[0]
        delta_theta_rl = rl_interp - self.rear_left_wheel_angle_LU[0]
        delta_theta_rr = rr_interp - self.rear_right_wheel_angle_LU[0]

        dyaw = ang_vel[2] *dt_imu

        if update_yaw == True:
            self.yaw = self.yaw + dyaw
            update_yaw = False

        dx, dy, dpsi = self.kinematic.calc_odometrie(delta_theta_fl, delta_theta_fr, delta_theta_rl, delta_theta_rr, dt_encoder, self.yaw)

        # Filter z Position
        self.zKalman.predict(imu_z_filt)
        self.zKalman.update_zero_velocity()
        dz = self.zKalman.get_state()[1]*dt_imu

        return dx, dy, dz, self.roll, self.pitch, self.yaw


    # ---------------------------------------------------------------------
    # Dynamics Function
    # ---------------------------------------------------------------------

    def robot_dynamics(self, t ,state):
        """
        Calculate the derivative of the state vector using the robot's dynamics.
        
        Unpacks the state vector, updates sensor data via interpolation, computes forces,
        moments, and accelerations, and constructs the state derivative vector.
        
        Returns:
            A tuple (dstate, dt) where dstate is the derivative of the state and dt is the time step.
        """
        # Unpack states
        X, Y, Z = state[0:3]
        #phi, theta, psi = state[3:6] Wird in self.update_kinematic berechnet
        quat = state[3:7]
        #dX, dY, dZ = state[7:10] #Wird in self.update_kinematic berechnet
        p, q, r = state[10:13]
        dalpha = state[17:21]
        
        # Normalize quaternions
        quat_old = quat
        quat_old /= np.linalg.norm(quat_old)
        state[3:7] = quat_old
        
        # Linear Acceleration Interpolieren und Erdbeschleunigung abziehen
        solver_time = self.t0 +self.solver_t
        imu_acc_buffer_copy = list(self.imu_acc_buffer)
        imu_angular_buffer_copy = list(self.imu_angular_buffer)
        imu_quat_buffer_copy = list(self.imu_quat_buffer)

        ang_vel_interp = self.interpolate_sensor_value_3D(imu_angular_buffer_copy, solver_time)
        lin_acc_interp = self.interpolate_sensor_value_3D(imu_acc_buffer_copy, solver_time)
        lin_acc_interp = self.remove_gravity_from_acceleration(lin_acc_interp, quat_old)
        lin_quat_interp = self.interpolate_sensor_value_4D(imu_quat_buffer_copy, solver_time)

        external_forces = np.array([0, 0, 0])

        # Rotation matrix
        R = self.quaternion_to_rotation_matrix(lin_quat_interp)
        
        # Update Kinematic States via IMU & Encoder
        phi_old, theta_old, psi_old = self.quaternion_to_euler(quat_old)

        dX, dY, dZ, phi_new, theta_new ,psi_new = self.update_kinematic(lin_acc_interp, ang_vel_interp, lin_quat_interp)
        
        dt = solver_time - self.last_solver_time[0]
        
        # Body Rate to euler rate Transformation
        T = np.array([
            [1, np.sin(phi_new)*np.tan(theta_new),  np.cos(phi_new)*np.tan(theta_new)],
            [0, np.cos(phi_new),               -np.sin(phi_new)],
            [0, np.sin(phi_new)/np.cos(theta_new),  np.cos(phi_new)/np.cos(theta_new)]
        ])
        T_inv = np.linalg.inv(T)

        #Calculate new Euler rates
        if not dt == 0:
            dphi = (phi_new - phi_old) /dt
            dtheta = (theta_new - theta_old) /dt
            dpsi = (psi_new - psi_old) /dt
        else:
            dphi = 0
            dtheta = 0
            dpsi = 0
            
        # New Euler rate to body rate Transformation
        p, q, r = T_inv @ np.array([dphi, dtheta, dpsi])

        # Linear and angular velocity vectors in inertial and body frame
        v_chassis = np.array([dX, dY, dZ])
        omega_body = ang_vel_interp

        # ---------------------------------------------------------------------
        # Wheel and Suspension Dynamics
        # ---------------------------------------------------------------------
        wheel_static_b = np.zeros((4,3))
        for i in range(4):
            wheel_static_b[i,:] = R.T @ config.wheel_positions[i,:]
        
        # Spring Offset
        delta = np.zeros(4)
        delta_dot = np.zeros(4)
        for i in range(4):
            z_wheel_equi = Z + wheel_static_b[:,2][i] #config.wheel_positions contains relative wheel position in equilibrium
            delta[i] = z_wheel_equi - self.z_wheels 
            delta_dot[i] = dZ2
    
        # Compute forces

        F_gravity_chassis = np.array([0, 0, -config.mass_mainframe*self.g])

        # Compute ground forces for each wheel
        F_ground = np.zeros((4,3))
        if not any(d > 0.15 for d in delta):
            F_ground = self.calculate_wheel_forces(lin_acc_interp[0], lin_acc_interp[1], Z, phi_new)

        # Horizontal friction force (if at least one wheel is on ground)
        vx, vy, vz = v_chassis
        F_fric = np.array([-self.c_fric * dX, -self.c_fric * dY, 0.0])

        #Wheel Torque Modeling
        wheel_torques = np.array([0, 0, 0, 0])
        for i in range(4):
            wheel_torques[i] = ((4.0 * config.wheel_mass + config.mass_mainframe) * lin_acc_interp[0]) * config.wheel_radius

        F_wheels = np.zeros((4,3))
        for i in range(4):
            # einfache Annahme: Radkraft in Körper-x-Richtung (keine Seitenkräfte)
            F_x_wheel_b = wheel_torques[i] / config.wheel_radius
            F_w_b = np.array([F_x_wheel_b, 0, 0])
            F_w = R @ F_w_b

            F_wheels[i] = F_w

        # Suspension Forces
        F_s = self.suspension_forces(delta, delta_dot,R,X,Y,Z)
        forceVector = [F_gravity_chassis[2], np.sum(F_s) , np.sum(F_fric), F_ground[0][2], F_ground[1][2], F_ground[2][2], F_ground[3][2]]
        
        # Publish forces für Testzwecke
        Force_msg = StateVector()
        Force_msg.state = forceVector
        Force_msg.header.stamp = rospy.Time.from_sec(self.t0 + self.t)

        self.force_publisher.publish(Force_msg)

        # Calculate total forces on chassis
        F_total = F_gravity_chassis + np.array([0,0, np.sum(F_s)]) + np.sum(F_fric.reshape(1,3), axis=0) + np.sum(F_wheels, axis=0) #+np.sum(F_s)

        if external_forces is not None:
            F_total += external_forces

        # Chassis translational acceleration
        ddX, ddY, ddZ = F_total / (config.mass_mainframe + 4.0 * config.wheel_mass)

        # -----------------------------------------------------------------------------
        # Parameter Identification for Suspension Modeling
        # -----------------------------------------------------------------------------
        z_disp = Z - 0.18094  + config.wheel_base/2 * phi_new - config.track_width/2 * theta_new  # z0 = 0.18094 =1 Höhe der IMU
        z_disp_dot = (z_disp - self.z_disp_prev)/dt
        
        self.z_disp_prev = z_disp
        
        self.identify_parameter(z_disp, z_disp_dot, lin_acc_interp[2])
        
        # -----------------------------------------------------------------------------
        # Moment (Torque) Calculations about the Chassis
        # -----------------------------------------------------------------------------  

        # Gravity force on wheel in body frame:
        F_g_wheel_b = R.T @ np.array([0,0,-config.wheel_mass*self.g])


        ddz_wheels = np.zeros(4)
        # Add ground and gravity in body frame:
        for i in range(4):
            F_ground_b = R.T @ F_ground[i]
            # Sum forces in z_b direction on the wheel:

            F_z_total_wheel = F_s[i] + F_ground_b[2] + F_g_wheel_b[2]

            ddz_wheels[i] = np.clip(F_z_total_wheel / config.wheel_mass,-10,10)

        wheel_positions_dynamic = config.wheel_positions.copy()
        for i in range(4):
            # Add delta_i to the z component
            wheel_positions_dynamic[i,2] += delta[i]

        M_z_required = config.I_chassis[2][2] * lin_acc_interp[2]
        lambda_factor = M_z_required / (0.6 * 4*config.wheel_mass+config.mass_mainframe*lin_acc_interp[1])

        M_total = np.zeros(3)
        for i in range(4):
            # Radkräfte = Reibkräfte
            multiplier = 1 if i % 2 == 0 else -1
            F_w_b = (4*config.wheel_mass+config.mass_mainframe) * np.array([lin_acc_interp[0], (1+(multiplier* lambda_factor)) * lin_acc_interp[1], 0])
            F_w = R @ F_w_b

            r_w_b = wheel_positions_dynamic[i]
            M_w = np.cross(r_w_b, F_w)  # Cross product in the body frame
            
            # The suspension force on the chassis in body frame:
            F_susp_chassis_b = np.array([0,0,(config.spring_stiffness*delta[i] + config.damping * delta_dot[i] )])
            # Forces in body frame and position in body frame:
            r_w_b = wheel_positions_dynamic[i]
            F_s = R @ F_susp_chassis_b
            M_s = np.cross(r_w_b, F_s)
            M_total += (M_w +M_s)

        # Publish Moments for Testing
        Moment_Msg = PoseStamped()
        Moment_Msg.header.stamp = rospy.Time.from_sec(self.t0 + self.t)
        Moment_Msg.pose.position.x = config.I_chassis[0][0] * lin_acc_interp[0]#M_total[0]
        Moment_Msg.pose.position.y = config.I_chassis[1][1] * lin_acc_interp[1]#M_total[1]
        Moment_Msg.pose.position.z = M_z_required
        self.moment_publisher.publish(Moment_Msg)

        # -----------------------------------------------------------------
        # Rotational Dynamics of the Chassis
        # -----------------------------------------------------------------        
        
        I_total = self.compute_total_inertia(wheel_positions_dynamic)
        
        I_omega = I_total @ omega_body
        domega = np.linalg.inv(I_total) @ (M_total - np.cross(omega_body, I_omega))
        q_dot = lin_quat_interp - quat_old
    
        # Wheel rotational dynamics (for alpha) assuming no longitudinal force:
        ddalpha = np.array([wheel_torques[i]/config.I_z_wheel for i in range(4)])

        # -----------------------------------------------------------------------------
        # Construct the State Derivative Vector
        # -----------------------------------------------------------------------------
        dstate = np.zeros_like(state)
        dstate = np.zeros_like(state)
        if dt == 0:
            dstate[0:3] = [0, 0, 0]
        else:
            dstate[0:3] = [dX/dt, dY/dt, dZ/dt] #v_chassis  # Translational velocities

        dstate[3:7] = q_dot  # Quaternion derivative
        dstate[7:10] = [ddX, ddY, ddZ]  # Translational accelerations
        dstate[10:13] = domega  # Angular accelerations
        dstate[13:17] = dalpha  # Wheel angular velocities
        dstate[17:21] = ddalpha  # Wheel angular accelerations

        return dstate, dt
    

#------------------------------------------------
#----------------MAIN----------------------------
#------------------------------------------------

if __name__ == '__main__':
    # Check if ROS Master is running before proceeding
    FlagRosRunning = is_ros_master_running()

    if FlagRosRunning:
        print('Ros Running: Dyn Modell')
        #Create Config and Robot Obj

        try:
            config = RobotConfig()
            robot = DynModell(config)
            rospy.spin() # Keep the node running and processing callbacks
        except rospy.ROSInterruptException:
            pass   

    else: print("No ROS Master running!")



       

    
