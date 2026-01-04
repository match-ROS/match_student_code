#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("move_along_wall")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(20)

    speed = rospy.get_param("~speed", 0.05)       # m/s
    duration = rospy.get_param("~duration", 60.0) # s

    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = 0.0

    t0 = rospy.Time.now().to_sec()
    rospy.loginfo("Starting forward motion for %.1f s", duration)

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        if t > duration:
            break
        pub.publish(twist)
        rate.sleep()

    # Stop
    twist.linear.x = 0.0
    pub.publish(twist)
    rospy.loginfo("Stopping robot.")

if __name__ == "__main__":
    main()
