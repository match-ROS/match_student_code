#!/usr/bin/env python3
import rospy
import rospkg

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Wrench
from rls4 import FourWheelRLS
from rls import SpringDamperRLS

import numpy as np
import os
import csv
import json
import pandas as pd

z = 0.0 
quat = np.array([0, 0, 0, 0])
f_z = 0

z_last = 0.0
dz_last = 0.0

phi_last = 0.0
dphi_last = 0.0
theta_last = 0.0
dtheta_last = 0.0

last_time = 0.0 
z0 = 0.18094 # z0 aus Blender aus dem Modell holen: Abstand IMU (CoM) zu Radaufnahme Lager

est = []

class RobotConfig:
    def __init__(self):
        self.load_robot_config()

    def load_robot_config(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('dynamic_model')
        robot_settings_path = os.path.join(package_path, "src", "robot_settings.json")

        print(f"{robot_settings_path}")

        # Load the JSON file into a Python dictionary
        with open(robot_settings_path, "r") as file:
            robot_settings = json.load(file)

        self.robot_typ = robot_settings["robot_typ"]
        #Dimensionierung
        self.dim_x = robot_settings['dimension']['width']   
        self.dim_y = robot_settings['dimension']['depths']
        self.dim_z = robot_settings['dimension']['heigth']
        self.track_width = robot_settings['dimension']['track_width']
        self.wheel_base = robot_settings['dimension']['wheel_base']
        self.wheel_radius = robot_settings['dimension']['wheel_radius']
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
        
def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (phi, theta, psi)."""
    if np.linalg.norm(q) == 0:
        x, y, z, w = np.array([0, 0, 0, 0])
    else:
        x, y, z, w = q / np.linalg.norm(q)
    phi   = np.arctan2(2*(w*x + y*z),      1 - 2*(x**2 + y**2))
    theta = np.arcsin(2*(w*y - z*x))
    psi   = np.arctan2(2*(w*z + x*y),      1 - 2*(y**2 + z**2))
    
    return phi, theta, psi

config = RobotConfig()

springDamperRLS = FourWheelRLS(config.wheel_base/2,config.wheel_base/2,config.track_width/2,config.mass_mainframe,2.29, 5.10)
springDamperRLS2 = SpringDamperRLS(config.mass_mainframe/4)

def callback_model_states(msg):
    global z
    global quat
    global z0

    z = msg.pose[1].position.z - z0  #-0.100998  # Z ist Feder Auslenkung
    print("z: ",z)
    quat = np.array([msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w])

def callback_wrench(msg):
    global f_z

    f_z = msg.force.z

def estimate(event):
    global last_time
    global z
    global quat
    global z_last
    global dz_last
    global phi_last
    global dphi_last
    global theta_last
    global dtheta_last

    dt = rospy.Time.now().to_sec() - last_time
    last_time = rospy.Time.now().to_sec()
    if dt == 0.0:
        dt = 1
    
    phi, theta, psi = quaternion_to_euler(quat)


    z_spring = z + config.wheel_base/2 * phi - config.track_width/2 * theta
    dz = (z_spring - z_last)/dt
    ddz =(dz - dz_last)/dt
    
    dphi = phi - phi_last
    ddphi = dphi - dphi_last
    
    dtheta = theta - theta_last
    ddtheta = dtheta - dtheta_last
       
    #k,c = springDamperRLS.rls_update(z, phi, theta, dz, dphi, dtheta, ddz, ddphi, ddtheta)
    
    c,k = springDamperRLS2.update(z, dz, ddz)

    timestamp = rospy.Time.now().to_sec()

    df = {
        "time": timestamp,
        "c": c,
        "k": k
    }

    est.append(pd.DataFrame([df]))
    
    z_last = z_spring
    dz_last = dz
    
    phi_last = phi
    dphi_last = dphi
    
    theta_last = theta
    dtheta_last = dtheta

def save_csv():
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('dynamic_model')
    robot_settings_path = os.path.join(package_path, "src/")
    est_df = pd.concat(est, ignore_index=True)
    est_df.to_csv(robot_settings_path+"est.csv",index=False)
    #save_list_to_csv(c_est, "c_est.csv")
    #save_list_to_csv(k_est, "k_est.csv")
    
def save_list_to_csv(data, filename):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('dynamic_model')
    filepath = os.path.join(package_path, "src/", filename)

    with open(filepath, mode='w', newline='') as file:
        writer = csv.writer(file)
        for item in data:
            writer.writerow([item])



if __name__ == "__main__":
    rospy.init_node('rls_test')

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback_model_states, queue_size=10)
    rospy.Subscriber("/Force", Wrench, callback_wrench, queue_size=10)

    #Live plot?
    rospy.on_shutdown(save_csv)
    if last_time == 0:
        last_time = rospy.Time.now().to_sec()
    rospy.Timer(rospy.Duration(0.01), estimate)

    rospy.spin()