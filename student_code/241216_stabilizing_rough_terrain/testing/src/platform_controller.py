#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi

# Liste Wegpunkten
waypoints = [
    {"x": 0, "y": 0, "yaw": 0},  # Startpunkt
    {"x": 25, "y": 0, "yaw": 0},  # Startpunkt
    {"x": 25, "y": 0, "yaw": 3.316},  # Wegpunkt 1
    {"x": 0, "y": 0, "yaw": 0}, # Wegpunkt 2

]

current_waypoint_index = 0

def quaternion_to_yaw(orientation):
    """Konvertiert Quaternion zu Yaw-Winkel"""
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw

def angular_difference(current_yaw, target_yaw):
    
    diff = target_yaw - current_yaw
    while diff > pi:
        diff -= 2 * pi
    while diff < -pi:
        diff += 2 * pi
    return diff

def distance_to_target(current_x, current_y, target_x, target_y):
    """Berechnet die Distanz zum Ziel."""
    dx = target_x - current_x
    dy = target_y - current_y
    return sqrt(dx**2 + dy**2)

def mir_pose_callback(data, pub):
    
    global current_waypoint_index

    # Aktuelle Position und Orientierung
    current_x = data.position.x
    current_y = data.position.y
    current_orientation = data.orientation
    current_yaw = quaternion_to_yaw(current_orientation)

    # Aktueller Zielwegpunkt
    if current_waypoint_index >= len(waypoints):
        rospy.loginfo("Alle Wegpunkte abgefahren!")
        return

    target = waypoints[current_waypoint_index]
    target_x, target_y, target_yaw = target["x"], target["y"], target["yaw"]

    # Berechne Distanz und Winkel zum Ziel
    distance = distance_to_target(current_x, current_y, target_x, target_y)
    target_angle = atan2(target_y - current_y, target_x - current_x)
    angle_to_target = angular_difference(current_yaw, target_angle)
    angle_to_orientation = angular_difference(current_yaw, target_yaw)

    cmd_vel_msg = Twist()

    # Zielwegpunkt erreicht?
    if distance < 0.5 and abs(angle_to_orientation) < 0.8:  # Toleranz für Position und Orientierung
        rospy.loginfo(f"Wegpunkt {current_waypoint_index + 1} erreicht!")
        current_waypoint_index += 1  # Zum nächsten Wegpunkt
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
    else:
        if distance >= 0.1:  # Bewegung zur Zielposition
            cmd_vel_msg.angular.z = 0.4 * angle_to_target  # P-Regler für Drehung
            if abs(angle_to_target) < 0.4:  # Nur vorwärts fahren, wenn der Winkel passt
                cmd_vel_msg.linear.x = 1 # P-Regler für Vorwärtsbewegung
        else:  # Position erreicht, nur Orientierung anpassen
            cmd_vel_msg.angular.z = 0.4 * angle_to_orientation

    # Veröffentliche die Geschwindigkeit
    pub.publish(cmd_vel_msg)

    #Informationen
    rospy.loginfo(f"Aktuelle Position: x={current_x}, y={current_y}, yaw={current_yaw:.2f}")
    rospy.loginfo(f"Zielwegpunkt: x={target_x}, y={target_y}, yaw={target_yaw:.2f}")
    rospy.loginfo(f"Distanz: {distance:.2f}, Winkel zur Zielposition: {angle_to_target:.2f}")
    rospy.loginfo(f"Winkel zur Zielorientierung: {angle_to_orientation:.2f}")
    rospy.loginfo(f"Geschwindigkeit: linear.x={cmd_vel_msg.linear.x:.2f}, angular.z={cmd_vel_msg.angular.z:.2f}")

def platform_controller():
    """Initialisiert die Node und startet den Prozess."""
    rospy.init_node('platform_controller', anonymous=True)

    # Publisher für cmd_vel
    pub = rospy.Publisher('/mur620/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    # Subscriber für die aktuelle Position des Roboters
    rospy.Subscriber('/mur620/mir_pose_simple', Pose, mir_pose_callback, pub)

    rospy.loginfo("Platform Controller gestartet. Wegpunkte werden abgefahren.")

    
    rospy.spin()

if __name__ == '__main__':
    try:
        platform_controller()
    except rospy.ROSInterruptException:
        pass

