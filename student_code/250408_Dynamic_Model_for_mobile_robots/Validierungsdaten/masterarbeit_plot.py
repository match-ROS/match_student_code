#!/usr/bin/env python3
import rospy
import rospkg

import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
import os
from datetime import datetime, timedelta
import matplotlib.gridspec as gridspec
import matplotlib.dates as mdates

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

def moving_average_filter(data, window_size=3):

    weights = np.ones(window_size) / window_size
    return np.convolve(data, weights, mode='same')

if __name__ == "__main__":
    data_1 = pd.read_csv("Trajectory/dynamic_model_states_single.csv")
    data_2 = pd.read_csv("Trajectory/trajectory.csv")
    #data_3 = pd.read_csv("Trajectory/dyn_mod_EKF.csv")
    data_4 = pd.read_csv("")

    plot_time = []
    plot_time[1] = data_1["time"]

    subplots = 1
    
    for i in range(1,subplots,1):
        plt.figure()
        plt.subplot(i,1,1)
        plt.plot(time_datetime, phi_dyn, label="Dyn. Phi")
        plt.plot(time_tra, phi_ist, color="red", linestyle="--", label="Ist Phi")
        plt.xlabel("Zeit in [s]",visible=True)
        plt.ylabel("phi(t) in [Grad]",visible=True)
        plt.ylim(-5, 5)
        plt.title("Pitch")
        plt.legend()


    plt.show
    


