#!/usr/bin/env python

import rospy 
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3, Point 
import math

if __name__ == "__main__":
    rospy.init_node("apply_alternating_force")
    # Wait for the Gazebo service to be available
    rospy.wait_for_service("/gazebo/apply_body_wrench")
    pub = rospy.Publisher("/Force",Wrench,queue_size=10 )
    try:
        apply_wrench_service = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        

    # Parameters you may want to tune
    body_name = "scout/::base_link"    # Change to match the robot and link in your simulation
    force_magnitude = 800.0            # Newtons
    apply_duration = 0.1              # Duration in seconds for each wrench application
    switch_period = 0.4               # How often (seconds) we switch the sign of the force

    rate = rospy.Rate(10)  # 10 Hz loop
    start_time = rospy.Time(0)

    # For toggling the sign of the force
    force_sign = -1
    last_switch_time = rospy.Time.now().to_sec()
    force_magnitude2 = 0
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()

        # Check if it's time to switch the sign
        if (current_time - last_switch_time) > switch_period:
            force_sign *= -1
            last_switch_time = current_time
            if force_magnitude * force_sign > 0:
                force_magnitude2 = 0
            else:
                force_magnitude2 = force_magnitude * force_sign

        # Construct the wrench
        wrench = Wrench()
        #wrench.force = Vector3(0.0, 0.0, force_sign * force_magnitude)
        wrench.force = Vector3(0.0, 0.0, force_magnitude2)

        wrench.torque = Vector3(0.0, 0.0, 0.0)
        pub.publish(wrench)
        # Apply the wrench
        try:
            apply_wrench_service(
                body_name=body_name,
                reference_frame="world",           # apply force in world frame
                reference_point=Point(0.0, 0.0, 0.0),
                wrench=wrench,
                start_time=start_time,
                duration=rospy.Duration(apply_duration)
            )
        except rospy.ServiceException as e:
            rospy.logerr("Failed to apply wrench: {}".format(e))

        rate.sleep()
