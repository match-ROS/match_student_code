#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def send_twist_commands():
    # Initialisiere die ROS-Node
    rospy.init_node('twist_command_publisher', anonymous=True)

    # Publisher für das Twist-Command
    twist_pub = rospy.Publisher('/mur620/twist_fb_command', Twist, queue_size=10)

    # Publish-Frequenz (z.B. 50 Hz passend zu deinem Controller)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        # Erstelle eine Twist-Nachricht
        twist_msg = Twist()

        # Beispielwerte für lineare und angulare Geschwindigkeit:
        # Diese können angepasst werden, um spezifische Bewegungen zu erzielen
        twist_msg.linear.x = 0.0  # Bewegung entlang der X-Achse (m/s)
        twist_msg.linear.y = 0.0  # Bewegung entlang der Y-Achse
        twist_msg.linear.z = 0.0  # Bewegung entlang der Z-Achse

        twist_msg.angular.x = 0.5  # Rotation um die X-Achse
        twist_msg.angular.y = 0.0  # Rotation um die Y-Achse
        twist_msg.angular.z = 0.0  # Rotation um die Z-Achse (rad/s)

        # Nachricht veröffentlichen
        twist_pub.publish(twist_msg)

        # Debug-Ausgabe
        rospy.loginfo(f"Published Twist Command: linear=[{twist_msg.linear.x}, {twist_msg.linear.y}, {twist_msg.linear.z}], "
                      f"angular=[{twist_msg.angular.x}, {twist_msg.angular.y}, {twist_msg.angular.z}]")

        # Warte bis zur nächsten Iteration
        rate.sleep()

if __name__ == '__main__':
    try:
        send_twist_commands()
    except rospy.ROSInterruptException:
        pass


