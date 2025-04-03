#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import csv
import os
import time

# Initialisiere ROS-Node
rospy.init_node("stabilize_end_effector", anonymous=False)
rospy.loginfo("Node initialized successfully.")

# Parameter
x_ib = 0.5  # Abstand IMU von Basis x
y_ib = 0.4  # Abstand IMU von Basis y
z_ib = 0.9  # Abstand IMU von Basis z
INITIAL_Z = 1.4  # Zielhöhe des Endeffektors im Raum (m)
TOLERANCE_ANGLE = 0.001  # Toleranz für Roll-/Pitch-Differenzen (Rad)
TOLERANCE_Z = 0.001  # Toleranz für Z-Differenz (m)
TOLERANCE_VELOCITY = 0.002  # Toleranz für Winkelgeschwindigkeiten (Rad/s)
tcp_roll = 0.0
tcp_pitch = 0.0
tcp_yaw = 0

dt = 0.1  #Schrittweite

ACCELERATION_THRESHOLD = 0.02  # Schwelle für große Beschleunigungsänderung

def calculate_velocity(acc_x, acc_z, previous_velocity):
    """
    Hier habe Ich unterschiedliche Ansätzte ausprobiert, jedoch hat keiner zu einer korrekten Geschwindigkewitsberechgnung gefühjrt, sodass ich hier ein Großteil wieder auskommentiert habe
    """
    acc_z_kompensiert = acc_z -9.81

    # Berechne die Gesamtbeschleunigung
    #acc_total = np.sqrt(acc_x**2 + acc_z_kompensiert**2)
    acc_total = acc_x 

    #return acc_total * dt
    # Überprüfe, ob eine große Änderung der Beschleunigung vorliegt
    if acc_total > ACCELERATION_THRESHOLD:
        # Geschwindigkeit neu berechnen (z. B. durch Reset)
        rospy.loginfo("Große Beschleunigungsänderung erkannt. Geschwindigkeit wird neu berechnet.")
        return  acc_total * dt  # Geschwindigkeit basierend auf aktueller Beschleunigung
    
    else:
        # Normale Inkrementierung der Geschwindigkeit
        return  previous_velocity + acc_total * dt


def calculate_distance(velocity):
    
    return velocity * dt

def calculate_height_difference(distance, pitch):
   
    return -distance * np.sin(pitch)


imu_data = None
reference_roll = 0.0
reference_pitch = 0.0
reference_z = 1.4

current_tcp_pose = np.zeros(3)
current_velocity = 0.0
tcp_global_pose = np.zeros(3)

# Logger
logger = {"time": [], "angular_x": [], "angular_y": [], "linear_z": [], "berechnetehohe": [], "globale_hoehe": []}

# Publisher
twist_pub = rospy.Publisher('/mur620/UR10_r/twist_fb_command', Twist, queue_size=10)

# Callback 
def imu_callback(msg):
    global imu_data
    imu_data = msg

# Callback
def TCP_pose(msg):
    global current_tcp_pose, tcp_roll, tcp_pitch, tcp_yaw
    # Aktuelle TCP-Position
    current_tcp_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    # Quaternion 
    tcp_quaternion = [
        msg.pose.orientation.x,  # x
        msg.pose.orientation.y,  # y
        msg.pose.orientation.z,  # z
        msg.pose.orientation.w   # w
    ]
    
    # Quaternion in Euler-Winkel
    tcp_roll, tcp_pitch, tcp_yaw = tf.transformations.euler_from_quaternion(tcp_quaternion)

    #rospy.loginfo(f"TCP Pose - Roll: {tcp_roll}, Pitch: {tcp_pitch}")

def TCP_global_pose(msg):
    global tcp_global_pose
    tcp_global_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

# IMU-Daten abonnieren
rospy.Subscriber('/mur620/filtered_imu_data', Imu, imu_callback)
rospy.Subscriber('/mur620/UR10_r/tcp_pose', PoseStamped, TCP_pose)
rospy.Subscriber('/mur620/UR10_r/global_tcp_pose', PoseStamped,TCP_global_pose)

# Update Referenz
def update_reference(roll, pitch, current_z):
    global reference_roll, reference_pitch, reference_z
    reference_roll = roll
    reference_pitch = pitch
    reference_z = current_z

#PID Regler
Kp_angle = 5
Ki_angle = 0.5
Kd_angle = 0.1

Kp_z = 5
Ki_z = 0.5
Kd_z = 0.1

previous_error_roll = 0.0
previous_error_pitch = 0.0
previous_error_z = 0.0

error_sum_roll = 0.0
error_sum_pitch = 0.0
error_sum_z = 0.0

def calculate_stable_twist(roll, pitch, current_z, height_difference):
    global previous_error_roll, previous_error_pitch, previous_error_z
    global error_sum_roll, error_sum_pitch, error_sum_z
    
    twist = Twist()

    # Berechnung der Differenzen
    error_roll = roll + tcp_roll - np.pi / 2
    error_pitch = pitch + tcp_pitch
    error_z = current_z - INITIAL_Z + height_difference

    # P-Anteil
    p_roll = Kp_angle * error_roll
    p_pitch = Kp_angle * error_pitch
    p_z = Kp_z * error_z

    # I-Anteil 
    error_sum_roll += error_roll * dt
    error_sum_pitch += error_pitch * dt
    error_sum_z += error_z * dt

    i_roll = Ki_angle * error_sum_roll
    i_pitch = Ki_angle * error_sum_pitch
    i_z = Ki_z * error_sum_z

    # D-Anteil 
    d_roll = Kd_angle * (error_roll - previous_error_roll) / dt
    d_pitch = Kd_angle * (error_pitch - previous_error_pitch) / dt
    d_z = Kd_z * (error_z - previous_error_z) / dt

    # PID-Regelung
    twist.angular.x = -(p_roll + i_roll + d_roll)
    twist.angular.y = -(p_pitch + i_pitch + d_pitch)
    twist.linear.z = -(p_z + i_z + d_z)

    # Begrenzung der Geschwindigkeiten
    max_linear_speed = 1.0
    max_angular_speed = 1.0
    twist.linear.z = np.clip(twist.linear.z, -max_linear_speed, max_linear_speed)
    twist.angular.x = np.clip(twist.angular.x, -max_angular_speed, max_angular_speed)
    twist.angular.y = np.clip(twist.angular.y, -max_angular_speed, max_angular_speed)

    # Update der vorherigen Fehler
    previous_error_roll = error_roll
    previous_error_pitch = error_pitch
    previous_error_z = error_z

    return twist


# Funktion zum Plotten 
def plot_and_save_log():
    # Erstelle den Plot
    plt.figure(figsize=(10, 6))
    plt.plot(logger["time"], logger["angular_x"], label="Twist Befehl Orientierung X", color="blue")
    plt.plot(logger["time"], logger["angular_y"], label="Twist Befehl Orientierung Y", color="green")
    plt.plot(logger["time"], logger["linear_z"], label="Twist Befehl Z (Height)", color="red")
    plt.plot(logger["time"], logger["berechnetehohe"], label="Höhe absolut", color="black")
    plt.plot(logger["time"], logger["globale_hoehe"], label="Höhe Raumkoordinatensystem", color="yellow")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Effort")
    plt.title("Regler-Eingriffe über die Zeit")
    plt.legend()
    plt.grid()
    plt.savefig("control_effort_plot.png")
    plt.show()

    # Speichern der LoggerDaten 
    csv_filename = "control_effort_log.csv"
    csv_filepath = os.path.join("/home/maximilianlippold/catkin_ws", csv_filename)

    with open(csv_filepath, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Time", "Angular X", "Angular Y", "Linear Z", "Höhendifferenz"])
        for i in range(len(logger["time"])):
            writer.writerow([logger["time"][i], logger["angular_x"][i], logger["angular_y"][i], logger["linear_z"][i], logger["berechnetehohe"][i],logger["globale_hoehe"][i]])

    rospy.loginfo(f"CSV-Datei gespeichert unter: {csv_filepath}")


# Main control loop
def main_loop():
    global current_velocity

    rate = rospy.Rate(1/dt)  # 10 Hz loop rate

    start_time = time.time()

    while not rospy.is_shutdown():
        if imu_data is not None:
            try:
                #Daten abrufen
                roll, pitch, yaw = tf.transformations.euler_from_quaternion([ 
                    imu_data.orientation.x, 
                    imu_data.orientation.y, 
                    imu_data.orientation.z, 
                    imu_data.orientation.w 
                ])

                
                acc_x = imu_data.linear_acceleration.x
                acc_z = imu_data.linear_acceleration.z

                # Geschwindigkeit berechnen
                current_velocity = calculate_velocity(acc_x, acc_z, current_velocity)

                # Distanze berechnen (nur in x)
                distance = calculate_distance(current_velocity)

                # Höhendifferenz aufgrund pitch und distance berechnen 
                height_difference = calculate_height_difference(distance, pitch)

                #TODO: height difference aufgrund von Neigung und Hebelarm um den Kippunktes fehlt

                # Endeffektorhöhe extrahieren
                current_z = z_ib + current_tcp_pose[2]

                # Winkelbeschleunigungen
                roll_rate = imu_data.angular_velocity.x
                pitch_rate = imu_data.angular_velocity.y

                # Nur bei signifikanter Änderung
                if (
                    abs(roll_rate) < TOLERANCE_VELOCITY and
                    abs(pitch_rate) < TOLERANCE_VELOCITY and
                    abs(current_z - reference_z) < TOLERANCE_Z):
                     update_reference(roll, pitch, current_z)

                
                stable_twist = calculate_stable_twist(roll, pitch, current_z, height_difference)

                # Loggen
                current_time = time.time() 
                logger["time"].append(current_time)
                logger["angular_x"].append(roll)
                logger["angular_y"].append(pitch)
                logger["linear_z"].append(stable_twist.linear.z)
                logger["berechnetehohe"].append(height_difference+current_z)
                logger["globale_hoehe"].append(tcp_global_pose[2])

                # Publizieren
                twist_pub.publish(stable_twist)
                rospy.loginfo(f"Published Twist: {stable_twist}")
            except Exception as e:
                rospy.logerr(f"Error in processing IMU data: {e}")
        else:
            rospy.logwarn("IMU data is not available yet.")

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.loginfo("Starting stabilization main loop...")
        main_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stabilization node terminated.")
    finally:
        rospy.loginfo("Erstelle Plots der Logger-Daten...")
        plot_and_save_log()
