#!/usr/bin/env python3
import rospy
import rospkg

#Ros Messages
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from custom_msgs.msg import StateVector
from geometry_msgs.msg import Wrench

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.dates as mdates
import os
from datetime import datetime

dynamic_model_states = []
dynamic_model_EKF_states = []
trajectory = []
lin_acc = []
lin_acc_filt = []
topic2_data = {'time': [], 'x': [], 'y': [], 'z': [], 'fl' : [], 'fr' : [], 'rl' : [], 'rr' : []}
topic3_data = {'time': [], 'F_gravity': [], 'F_ground': [], 'F_fric': []}
force_z=[]
force = []
moment = []

time_data = []

wheel_position = -0.100998

def callback_dyn_modell(msg):
    global dynamic_model_states
    timestamp = msg.header.stamp.to_sec()
    #print("callback_dyn")
    df = {
        "time": timestamp,
        "x": msg.state[0],
        "y": msg.state[1],
        "z": msg.state[2],
        "q1": msg.state[3],
        "q2": msg.state[4],
        "q3": msg.state[5],
        "q4": msg.state[6],
    }

    dynamic_model_states.append(pd.DataFrame([df])) 

    topic2_data["time"].append(timestamp)
    topic2_data['x'].append(msg.state[0])
    topic2_data['y'].append(msg.state[1])
    topic2_data['z'].append(msg.state[2])

def callback_dyn_modell_EKF(msg):
    global dynamic_model_EKF_states
    timestamp = msg.header.stamp.to_sec()
    #print("callback_dyn")
    df = {
        "time": timestamp,
        "x": msg.state[0],
        "y": msg.state[1],
        "z": msg.state[2],
    }

    dynamic_model_EKF_states.append(pd.DataFrame([df])) 

def callback_xy(msg):
    global trajectory
    timestamp = rospy.Time.now().to_sec()
    #print("callback_dyn")
    df = {
        "time": timestamp,
        "x": msg.pose[1].position.x,
        "y": msg.pose[1].position.y,
        "z": msg.pose[1].position.z,
        "q0": msg.pose[1].orientation.x,
        "q1": msg.pose[1].orientation.y,
        "q2": msg.pose[1].orientation.z,
        "q3": msg.pose[1].orientation.w,
        "dx": msg.twist[1].linear.x,
        "dy": msg.twist[1].linear.y,
        "dz": msg.twist[1].linear.z,

    }
    #print(msg.pose[1].position.x)

    trajectory.append(pd.DataFrame([df])) 
    
def callback_force(msg):
    timestamp = msg.header.stamp.to_sec()

    df = {
        "time": timestamp,
        "F_gravity": msg.state[0],
        "F_s": msg.state[1],
        "F_fric": msg.state[2],
        "F_gfl": msg.state[3],
        "F_gfr": msg.state[4],
        "F_grl": msg.state[5],
        "F_grr": msg.state[6]
        #"F_wheels": msg.state[3]
    }

    force.append(pd.DataFrame([df]))

def callback_moment(msg):
    timestamp = msg.header.stamp.to_sec()

    df = {
        "time": timestamp,
        "m_x": msg.pose.position.x,
        "m_y": msg.pose.position.y,
        "m_z": msg.pose.position.z
    }

    moment.append(pd.DataFrame([df]))

def callback_lin_acc(msg):
    timestamp = msg.header.stamp.to_sec()  
    df = {
        "time": timestamp,
        "x": msg.pose.position.x,
        "y": msg.pose.position.y,
        "z": msg.pose.position.z
    }
    lin_acc.append(pd.DataFrame([df]))
    
def callback_lin_acc_filt(msg):
    timestamp = msg.header.stamp.to_sec()  
    df = {
        "time": timestamp,
        "x": msg.pose.position.x,
    }
    lin_acc_filt.append(pd.DataFrame([df]))

def callback_wrench(msg):
    timestamp = rospy.Time.now().to_sec()
    df = {
        "time": timestamp,
        "f_z": msg.force.z
    }
    force_z.append(pd.DataFrame([df]))

        
def save_csv():
    global dynamic_model_states
    global trajectory

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('dynamic_model')
    robot_settings_path = os.path.join(package_path, "src/")
    #print(dynamic_model_states)
    try:
        dyn_df = pd.concat(dynamic_model_states,ignore_index=True)
        dyn_df.to_csv(robot_settings_path+"dynamic_model_states_single.csv",index=False)
    except:
        print("no dyn")

    try:
        tra_df = pd.concat(trajectory,ignore_index=True)
        tra_df.to_csv(robot_settings_path+"trajectory.csv",index=False)
    except:
        print("no trajectory")

    try:
        lin_acc_df = pd.concat(lin_acc, ignore_index=True)
        lin_acc_df.to_csv(robot_settings_path+"lin_acc.csv",index=False)
    except:
        print("no lin acc")

    try:
        lin_acc_filt_df = pd.concat(lin_acc_filt, ignore_index=True)
        lin_acc_filt_df.to_csv(robot_settings_path+"lin_acc_filt.csv",index=False)
    except:
        print("no lin acc")

    try:
        force_z_df = pd.concat(force_z, ignore_index=True)
        force_z_df.to_csv(robot_settings_path+"force_z.csv",index=False)
    except:
        print("no force")

    try:
        force_df = pd.concat(force, ignore_index=True)
        force_df.to_csv(robot_settings_path+"force.csv",index=False)
    except:
        print("no Forces")

    try:
        moment_df = pd.concat(moment, ignore_index=True)
        moment_df.to_csv(robot_settings_path+"moment.csv",index=False)
    except:
        print("no Moment")


if __name__ == "__main__":
    rospy.init_node('monitoring')

    rospy.Subscriber("/robot_state", StateVector, callback_dyn_modell)
    rospy.Subscriber("/dynModForce", StateVector, callback_force)
    rospy.Subscriber("/dynModMoment", PoseStamped, callback_moment)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback_xy)
    rospy.Subscriber("/ekf_state", StateVector, callback_dyn_modell_EKF)
    rospy.Subscriber("/lin_acc", PoseStamped, callback_lin_acc)
    rospy.Subscriber("/lin_acc_filt", PoseStamped, callback_lin_acc_filt)
    rospy.Subscriber("/Force", Wrench, callback_wrench, queue_size=10)
    
    #Live plot?
    rospy.on_shutdown(save_csv)


    rospy.spin()