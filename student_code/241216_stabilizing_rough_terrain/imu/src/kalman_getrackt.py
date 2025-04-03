#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Imu
import math
import matplotlib.pyplot as plt
from collections import deque

# Variablen für die Orientierung
orientation_estimate = (0.0, 0.0, 0.0) 
P_orientation = [1.0, 1.0, 1.0]  

# Variablen für die Geschwindigkeiten
angular_velocity_estimate = (0.0, 0.0, 0.0)  
P_angular_velocity = [1.0, 1.0, 1.0]  

# Prozessrauschen
Q_orientation = 0.01  
Q_angular_velocity = 0.01  

# Messrauschen
R_orientation = 0.5  
R_angular_velocity = 0.5  

# Plotten 
plot_length = 5000
raw_orientation = {"roll": deque(maxlen=plot_length), "pitch": deque(maxlen=plot_length), "yaw": deque(maxlen=plot_length)}
filtered_orientation = {"roll": deque(maxlen=plot_length), "pitch": deque(maxlen=plot_length), "yaw": deque(maxlen=plot_length)}
raw_angular_velocity = {"x": deque(maxlen=plot_length), "y": deque(maxlen=plot_length), "z": deque(maxlen=plot_length)}
filtered_angular_velocity = {"x": deque(maxlen=plot_length), "y": deque(maxlen=plot_length), "z": deque(maxlen=plot_length)}

# Publisher
filtered_imu_publisher = None

# Update Funktion
def kalman_update(est, P, measurement, Q, R):
    P += Q  
    K = P / (P + R)  
    est = est + K * (measurement - est)  
    P = (1 - K) * P  
    return est, P

# Quterionen zu Euler
def quaternion_to_euler(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw

# Callback funktion
def imu_callback(msg):
    global orientation_estimate, P_orientation, angular_velocity_estimate, P_angular_velocity
    global raw_orientation, filtered_orientation, raw_angular_velocity, filtered_angular_velocity

    
    roll, pitch, yaw = quaternion_to_euler(msg.orientation)

    # Daten holen
    raw_orientation["roll"].append(roll)
    raw_orientation["pitch"].append(pitch)
    raw_orientation["yaw"].append(yaw)

    # Update 
    roll, P_orientation[0] = kalman_update(orientation_estimate[0], P_orientation[0], roll, Q_orientation, R_orientation)
    pitch, P_orientation[1] = kalman_update(orientation_estimate[1], P_orientation[1], pitch, Q_orientation, R_orientation)
    yaw, P_orientation[2] = kalman_update(orientation_estimate[2], P_orientation[2], yaw, Q_orientation, R_orientation)
    orientation_estimate = (roll, pitch, yaw)

    #Speichern der Filtered Daten
    filtered_orientation["roll"].append(roll)
    filtered_orientation["pitch"].append(pitch)
    filtered_orientation["yaw"].append(yaw)

    #Daten holen
    raw_angular_velocity["x"].append(msg.angular_velocity.x)
    raw_angular_velocity["y"].append(msg.angular_velocity.y)
    raw_angular_velocity["z"].append(msg.angular_velocity.z)

    # Update 
    ang_vel_x, P_angular_velocity[0] = kalman_update(
        angular_velocity_estimate[0], P_angular_velocity[0], msg.angular_velocity.x, Q_angular_velocity, R_angular_velocity)
    ang_vel_y, P_angular_velocity[1] = kalman_update(
        angular_velocity_estimate[1], P_angular_velocity[1], msg.angular_velocity.y, Q_angular_velocity, R_angular_velocity)
    ang_vel_z, P_angular_velocity[2] = kalman_update(
        angular_velocity_estimate[2], P_angular_velocity[2], msg.angular_velocity.z, Q_angular_velocity, R_angular_velocity)
    angular_velocity_estimate = (ang_vel_x, ang_vel_y, ang_vel_z)

    # Speichern der filtered Daten
    filtered_angular_velocity["x"].append(ang_vel_x)
    filtered_angular_velocity["y"].append(ang_vel_y)
    filtered_angular_velocity["z"].append(ang_vel_z)

    # Loggen
    rospy.loginfo(f"Filtered Orientation: Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")
    rospy.loginfo(f"Filtered Angular Acceleration: X: {ang_vel_x:.3f}, Y: {ang_vel_y:.3f}, Z: {ang_vel_z:.3f}")

    # publishen der Daten
    filtered_imu_msg = Imu()
    filtered_imu_msg.header = msg.header
    filtered_imu_msg.orientation = msg.orientation  
    filtered_imu_msg.angular_velocity.x = ang_vel_x
    filtered_imu_msg.angular_velocity.y = ang_vel_y
    filtered_imu_msg.angular_velocity.z = ang_vel_z
    filtered_imu_msg.linear_acceleration = msg.linear_acceleration  
    filtered_imu_publisher.publish(filtered_imu_msg)

# Plotten der Daten
def plot_data():
    plt.ion()
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))
    fig.suptitle("IMU Data (Raw vs Filtered)")

    while not rospy.is_shutdown():
        for i, key in enumerate(["roll", "pitch", "yaw"]):
            axs[i].cla()
            axs[i].plot(raw_orientation[key], label=f"Raw {key.capitalize()}", linestyle="--", color="red")
            axs[i].plot(filtered_orientation[key], label=f"Filtered {key.capitalize()}", color="blue")
            axs[i].legend(loc="upper right")
            axs[i].set_ylabel(key.capitalize())

        for i, key in enumerate(["x", "y", "z"], start=3):
            axs[i].cla()
            axs[i].plot(raw_angular_velocity[key], label=f"Raw Angular Velocity {key.upper()}", linestyle="--", color="green")
            axs[i].plot(filtered_angular_velocity[key], label=f"Filtered Angular Velocity {key.upper()}", color="purple")
            axs[i].legend(loc="upper right")
            axs[i].set_ylabel(f"Angular Vel {key.upper()}")

        plt.pause(0.01)

def main():
    global filtered_imu_publisher

    rospy.init_node('imu_kalman_filter', anonymous=False)

    # Publisher
    filtered_imu_publisher = rospy.Publisher('/mur620/filtered_imu_data', Imu, queue_size=10)

    # IMU Daten abbonieren
    rospy.Subscriber('/mur620/imu_data', Imu, imu_callback)

    # Plottfunktion 
    from threading import Thread
    plot_thread = Thread(target=plot_data)
    plot_thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()



