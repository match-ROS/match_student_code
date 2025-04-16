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
from scipy.signal import resample

Farben = {
    "rot": (255/255, 0/255, 0/255),
    "blau": (0/255, 0/255, 255/255),
    "gruen": (0/255, 128/255, 0/255),
    "schwarz": (0/255, 0/255, 0/255),
    "magenta": (255/255, 0/255, 255/255),
    "cyan": (0/255, 255/255, 255/255),
    "orange": (255/255, 180/255, 0/255),
    "grau": (136/255, 138/255, 142/255),
    "hellrot": (255/255, 150/255, 150/255),

    "imesblau": (0/255, 80/255, 155/255),
    "imesorange": (231/255, 123/255, 41/255),
    "imesgruen": (200/255, 211/255, 23/255),
}

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


def rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2))

#folder = "Trajectory/"
folder = "ForcesMoments/"
#folder = "Ident_AX/"
#folder = "Ident_ax_New/"


if __name__ == "__main__":
    traData = pd.read_csv(folder+"trajectory.csv")
    data = pd.read_csv(folder+"dynamic_model_states_single.csv")
    
    data_est = pd.read_csv(folder+"est.csv")
    est_c = data_est["c"].to_numpy()
    est_k = data_est["k"].to_numpy()
    data_est["time"] = data_est["time"] - data_est["time"][0]
    est_time = data_est["time"].to_numpy()

    #print(data_EKF)
    traData["time"]= traData["time"] -data['time'][0]
    time_tra = traData['time'].to_numpy()
    time_vx = (traData["time"]-traData["time"][0]).to_numpy()
    time_tra = time_tra +0.78
    v_x = traData["dx"].to_numpy()
    v_x = moving_average_filter(v_x, 500)

    accData = pd.read_csv(folder+"lin_acc.csv")
    accData["time"] = accData["time"] - accData["time"][0]
    accData_z = np.convolve(accData["z"],np.ones(5)/5, mode='same')

    accData_filt = pd.read_csv(folder+"lin_acc_filt.csv")
    accData_filt["time"] = accData_filt["time"] - accData_filt["time"][0]
    accData_z_filt = np.convolve(accData_filt["x"],np.ones(5)/5, mode='same')

    try:
        force_data = pd.read_csv(folder+"force_z.csv")
        force_z = force_data["f_z"].to_numpy()
        force_data["time"] = force_data["time"] - force_data["time"][0]
        force_z_time = force_data["time"].to_numpy()
    except:
        print("no force z")

    data["time"] = data["time"] - data["time"][0]
    #time_datetime = [datetime.fromtimestamp(ts) for ts in data["time"]]
    time_datetime = data["time"].to_numpy()
    #print(data['x'])
    first_change_inedx = np.where(data["x"].to_numpy() != data["x"].to_numpy()[0])[0][0]
    #print(first_change_inedx)

    # Pitch Roll Yaw
    q0, q1, q2, q3 = data["q1"].to_numpy(), data["q2"].to_numpy(), data["q3"].to_numpy(), data["q4"].to_numpy() 
    phi_dyn = []
    theta_dyn = []
    psi_dyn = []
    phi_ist = []
    theta_ist = []
    psi_ist = []
    for i in range(len(q0)):
        quat = np.array([q0[i], q1[i], q2[i], q3[i]])
        x,y,z = quaternion_to_euler(quat)
        phi_dyn.append(x/(2*math.pi)*360)
        theta_dyn.append(y/(2*math.pi)*360)
        psi_dyn.append(z/(2*math.pi)*360)

    try:
        q0_ist, q1_ist, q2_ist, q3_ist = traData["q0"].to_numpy(), traData["q1"].to_numpy(), traData["q2"].to_numpy(), traData["q3"].to_numpy()
    except:
        q0_ist, q1_ist, q2_ist, q3_ist = np.array(([0],[0],[0],[0]))
    for i in range(len(q0_ist)):
        quat = np.array([q0_ist[i], q1_ist[i], q2_ist[i], q3_ist[i]])
        x,y,z = quaternion_to_euler(quat)
        phi_ist.append(x/(2*math.pi)*360)
        theta_ist.append(y/(2*math.pi)*360)
        psi_ist.append(z/(2*math.pi)*360)
        
    phi_ist = phi_ist - phi_ist[0]
    theta_ist = theta_ist - theta_ist[0]
    psi_ist = psi_ist -psi_ist[0]

    phi_ist = moving_average_filter(phi_ist, 20)
    theta_ist = moving_average_filter(theta_ist, 20)
    
    ##########Forces Moments #################
    try:
        window = 20
        dataForces = pd.read_csv(folder+"force.csv")
        dataForces["time"] = dataForces["time"] - dataForces["time"][0]
        dataForcesTime = dataForces["time"].to_numpy()
        F_gravity = dataForces["F_gravity"].to_numpy()
        F_s = dataForces["F_s"].to_numpy()
        F_fric = moving_average_filter(dataForces["F_fric"].to_numpy(), window)
        F_gfl = np.convolve(dataForces["F_gfl"].to_numpy(),np.ones(window)/window,"valid")
        F_gfr = np.convolve(dataForces["F_gfr"].to_numpy(),np.ones(window)/window,"valid")
        F_grl = np.convolve(dataForces["F_grl"].to_numpy(),np.ones(window)/window,"valid")
        F_grr = np.convolve(dataForces["F_grr"].to_numpy(),np.ones(window)/window,"valid")
        time_fg = resample(dataForcesTime, len(F_gfl))
        

        dataMoments = pd.read_csv(folder+"moment.csv")
        dataMoments["time"] = dataMoments["time"] - dataMoments["time"][0]
        dataMomentsTime = dataMoments["time"].to_numpy()
        M_x = moving_average_filter(dataMoments["m_x"].to_numpy(), window)
        M_y = moving_average_filter(dataMoments["m_y"].to_numpy(), window)
        M_z = moving_average_filter(dataMoments["m_z"].to_numpy(), window)
    except:
        print("no forces or moments")

    ########## Flags ############

    FlagOrientation = False
    FlagIdentification = False
    FlagTrajectory = False
    FlagForcesMoments = True



    if FlagOrientation == True:
        ##### Error Berechnung #####
        phi_ist_interp = np.interp(time_datetime, time_tra, phi_ist)
        theta_ist_interp = np.interp(time_datetime, time_tra, theta_ist)
        psi_ist_interp = np.interp(time_datetime, time_tra, psi_ist)
        psi_diff = psi_dyn - psi_ist_interp
        for i in range(len(psi_diff)):
            if abs(psi_diff[i]) > 50:
                psi_diff[i] = psi_diff[i-1]

        ''' Orientation '''
        ########### Phi #############
        plt.figure(figsize=(6.3, 3.94))
        plt.subplot(2,1,1)
        plt.plot(time_tra, phi_ist, color=Farben['imesblau'], label='$\phi_\mathrm{Ist}$')
        plt.plot(time_datetime, phi_dyn, color=Farben['imesorange'], linestyle='--', label='$\hat{\phi}_\mathrm{Modell}$')
        plt.ylabel('$\phi\,(t)$ in [Grad]',visible=True)
        plt.ylim(-2, 2)
        plt.grid()
        plt.legend()

        plt.subplot(2,1,2)
        plt.plot(time_datetime, phi_dyn-phi_ist_interp, color=Farben['imesblau'], label='$\Delta\phi$')
        plt.xlabel('Zeit in [s]',visible=True)
        plt.ylabel('$\Delta\phi\,(t)$ in [Grad]',visible=True)
        #plt.ylim(-5, 5)
        plt.grid()
        plt.legend()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.11, hspace=0.3)
        plt.savefig("Roll.pdf_tex", format="pgf")

        rmse_phi = rmse(phi_ist_interp, phi_dyn)
        sigma_phi = np.std(phi_dyn-phi_ist_interp)
        print(f"RMSE Phi: {rmse_phi}, STD: {sigma_phi}")

        ########### Theta #############
        plt.figure(figsize=(6.3, 3.94))
        plt.subplot(2,1,1)
        plt.plot(time_tra, theta_ist, color=Farben['imesblau'], label='$\\theta_\mathrm{Ist}$')
        plt.plot(time_datetime, theta_dyn, color=Farben['imesorange'], linestyle='--', label='$\hat\\theta_\mathrm{Modell}$')
        plt.ylabel('$\\theta\,(t)$ in [Grad]',visible=True)
        plt.ylim(-2, 2)
        plt.grid()
        plt.legend()

        plt.subplot(2,1,2)
        plt.plot(time_datetime, theta_dyn-theta_ist_interp, color=Farben['imesblau'], label='$\Delta\\theta$')
        plt.xlabel('Zeit in [s]',visible=True)
        plt.ylabel('$\Delta\\theta\,(t)$ in [Grad]',visible=True)
        #plt.ylim(-5, 5)
        plt.grid()
        plt.legend()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.11, hspace=0.3)
        plt.savefig("Pitch.pdf_tex", format="pgf")

        rmse_theta = rmse(theta_ist_interp,theta_dyn)
        sigma_theta = np.std(theta_dyn-theta_ist_interp)
        print(f"RMSE Theta: {rmse_theta}, STD: {sigma_theta}")

        ########### Psi #############
        plt.figure(figsize=(6.3, 3.94))
        plt.subplot(2,1,1)
        plt.plot(time_tra, psi_ist, color=Farben['imesblau'], label='$\psi_\mathrm{Ist}$')
        plt.plot(time_datetime, psi_dyn, color=Farben['imesorange'], linestyle='--', label='$\hat{\psi}_\mathrm{Modell}$')
        plt.ylabel('$\psi\,(t)$ in [Grad]',visible=True)
        #plt.ylim(-2, 2)
        plt.grid()
        plt.legend()
        #plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))

        plt.subplot(2,1,2)
        plt.plot(time_datetime, psi_diff, color=Farben['imesblau'], label='$\Delta\psi$')
        plt.xlabel('Zeit in [s]',visible=True)
        plt.ylabel('$\Delta\psi\,(t)$ in [Grad]',visible=True)
        #plt.ylim(-5, 5)
        plt.grid()
        plt.legend()
        plt.subplots_adjust(left=0.15, right=0.9, top=0.9, bottom=0.11, hspace=0.3)
        plt.savefig("Yaw.pdf_tex", format="pgf")
        plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))


        rmse_psi = rmse(2*psi_diff,psi_diff)
        sigma_psi = np.std(psi_diff)
        print(f"RMSE Psi: {rmse_psi}, STD: {sigma_psi}")
        print(time_tra)

    if FlagIdentification == True:
        ######################
        ''' Identifikation '''
        ######################
        #v_x_interp = np.interp(est_time, time_tra, v_x)

        plt.figure(figsize=(6.3,5.3))
        plt.subplot(3,1,1)
        plt.plot(est_time, est_c, color=Farben["imesblau"], label="c-Ident")
        plt.axhline(y=250, color=Farben["imesorange"], linestyle="--", label="c-Ist")
        plt.ylabel("$c$ in [Ns/m]")
        plt.ylim(-200, 500)
        plt.xlim(-0.5,10)
        plt.legend(loc="lower right")
        plt.grid()

        plt.subplot(3,1,2)
        plt.plot(est_time, est_k, color=Farben["imesblau"], label="k-Ident")
        plt.axhline(y=15000, color=Farben["imesorange"], linestyle="--", label="k-Ist")
        #    ax7.set(ylim=(22000, 27000))
        plt.ylabel("$k$ in [N/m]")
        plt.ylim(-500,25000)
        plt.xlim(-0.5,10)
        plt.legend()
        plt.grid()
        plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))

        plt.subplot(3,1,3)
        plt.plot(time_vx,v_x, color=Farben["imesblau"], label="$v_\mathrm{x}$")
        #plt.plot(force_z_time,force_z, label="Force_Z")

        plt.xlabel("Zeit in [s]",visible=True)
        plt.ylabel("$v_\mathrm{x}$ in [m/s]",visible=True)
        plt.grid()
        plt.xlim(-0.5,10)
        plt.legend()

        '''
        plt.subplot(3,1,3)
        plt.plot(force_z_time, force_z, label="Dyn. Psi")
        plt.xlabel("Zeit in [s]",visible=True)
        plt.ylabel("F_z in [N]",visible=True)
        plt.title("F_z")
        plt.xlim(0,5)
        plt.legend()'''
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1, hspace=0.5)
        plt.savefig("IdentVX.pdf_tex", format="pgf")

    if FlagTrajectory == True:
        z_ist_interp = np.interp(time_datetime, time_tra, traData["z"].to_numpy() )

        plt.figure(figsize=(6.3, 2.3))
        #plt.subplot(2,1,1)
        data_z = data["z"].to_numpy()
        data_z[0:first_change_inedx] = data_z[first_change_inedx+1]
        #ata_z = data_z*4
        data_z = data_z - (data_z[0]-traData["z"][0])
        plt.plot(time_tra, traData["z"].to_numpy() , color=Farben["imesblau"], label="$z_\mathrm{Ist}$")
        plt.plot(time_datetime, data_z, color=Farben["imesorange"], linestyle="--", label="$\hat{z}_\mathrm{Modell}$")
        plt.ylim(0,0.3)
        plt.ylabel("z(t) in [m]")
        plt.xlabel("Zeit in [s]", visible=True)

        plt.legend(loc="upper left")
        plt.grid()

        plt.twinx()
        plt.plot(time_datetime, data_z-z_ist_interp, color=Farben["grau"], linestyle="-.", label="$\Delta z$")
        plt.xlabel("Zeit in [s]", visible=True)
        plt.ylabel("$\Delta z\,(t)$ in [m]")
        
        plt.grid(linestyle="-.")
        plt.legend(loc="upper right")
        plt.subplots_adjust(left=0.1, right=0.85, top=0.9, bottom=0.2, hspace=0.3)
        plt.savefig("z.pdf_tex", format="pgf")

        rmse_z = rmse(z_ist_interp, data_z)
        sigma_z = np.std(data_z-z_ist_interp)
        print(f"RMSE Z: {rmse_z*100} cm, STD:{sigma_z*100} cm")

        '''plt.subplot(2,1,2)
        plt.plot(time_datetime, data_z-z_ist_interp,  label="$\Delta z$")
        plt.xlabel("Zeit in [s]")
        plt.ylabel("$\Delta z\,(t)$ in [m]")
        plt.grid()
        plt.legend()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.11, hspace=0.3)
        plt.savefig("z.pdf_tex", format="pgf")
        '''

        ############# XY ################

        plt.figure(figsize=(6.3/1.3, 3.94/1.3))
        plt.plot(traData["x"].to_numpy(), traData["y"].to_numpy(), color=Farben["imesblau"], label="Ist-Trajektorie")
        plt.plot(data["x"].to_numpy(), data["y"].to_numpy(), color=Farben["imesorange"],  label="Modell Trajektorie",  linestyle='--')
        #plt.plot(data_EKF["x"].to_numpy(), data_EKF["y"].to_numpy(), color='green', label="Dynamisches Modell EKF", linestyle="--")
        plt.legend(loc="upper right")
        plt.grid()
        plt.xlim(-3,5)
        plt.ylim(-3,4)
        plt.xlabel("x in [m]",visible=True)
        plt.ylabel("y in [m]")
        #plt.xaxis.set_tick_params(labelbottom=True)
        plt.subplots_adjust(left=0.13, right=0.9, top=0.9, bottom=0.15, hspace=0.3)
        plt.savefig("XY.pdf_tex", format="pgf")

        data_x_interp = np.interp(time_datetime, time_tra, traData["x"].to_numpy())
        data_y_interp = np.interp(time_datetime, time_tra, traData["y"].to_numpy())

        rmse_x = rmse(data_x_interp, data["x"].to_numpy())
        sigma_x = np.std(data["x"].to_numpy()-data_x_interp)
        print(f"RMSE X: {rmse_x*100} cm, STD: {sigma_x*100} cm")

        rmse_y = rmse(data_y_interp, data["y"].to_numpy())
        sigma_y = np.std(data["y"].to_numpy()- data_y_interp)
        print(f"RMSE Y: {rmse_y*100} cm, STD: {sigma_y*100} cm")


    if FlagForcesMoments == True:
        plt.figure(figsize=(6.8, 6.8))
        plt.subplot(3,2,1)
        plt.plot(time_fg, F_gfl, color=Farben["imesblau"], label="$\hat{F}_\mathrm{z,fl}$")
        plt.plot(time_fg, F_gfr, color=Farben["imesorange"], label="$\hat{F}_\mathrm{z,fr}$")
        plt.plot(time_fg, F_grl, color=Farben["imesgruen"], linestyle= "--", label="$\hat{F}_\mathrm{z,rl}$")
        plt.plot(time_fg, F_grr, color=Farben["grau"], linestyle= "--", label="$\hat{F}_\mathrm{z,rr}$")
        #plt.ylim(140, 170)
        plt.ylabel("Normalkräfte in [N]")
        plt.grid()
        plt.legend(loc="upper right")

        plt.subplot(3,2,3)
        plt.plot(dataForcesTime, F_s, color=Farben["imesblau"], label="$\hat{F}_\mathrm{S}$")
        plt.ylabel("Federkräfte in [N]")
        plt.grid()
        plt.legend()

        plt.subplot(3,2,5)
        plt.plot(dataForcesTime, F_fric, color=Farben["imesblau"], label="$\hat{F}_\mathrm{fric}$")
        plt.ylabel("Reibkräfte in [N]")
        plt.xlabel("Zeit in [s]")

        plt.grid()
        plt.legend()

        plt.subplot(3,2,2)
        plt.plot(dataMomentsTime, M_x, color=Farben["imesblau"], label="$\hat{M}_\mathrm{x}$")
        plt.ylabel("Moment um X in [Nm]")
        plt.grid()
        plt.legend(loc="upper right")

        plt.subplot(3,2,4)
        plt.plot(dataMomentsTime, M_y, color=Farben["imesblau"], label="$\hat{M}_\mathrm{y}$")
        plt.ylabel("Moment um Y in [Nm]")
        plt.grid()
        plt.legend(loc="upper right")

        plt.subplot(3,2,6)
        plt.plot(dataMomentsTime, M_z, color=Farben["imesblau"], label="$\hat{M}_\mathrm{z}$")
        plt.ylabel("Moment um Z in [Nm]")
        plt.xlabel("Zeit in [s]")
        plt.grid()
        plt.legend(loc="upper right")
        plt.subplots_adjust(left=0.1, right=0.98, top=0.9, bottom=0.11, hspace=0.3, wspace=0.4)
        plt.ticklabel_format(style='sci', axis='y', scilimits=(-5,5))


        plt.savefig("ForceMoments.pdf_tex", format="pgf")


    plt.show()


