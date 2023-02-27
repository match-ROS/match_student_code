#!/usr/bin/env python3
# # coding=latin-1

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped,Pose
from gazebo_msgs.msg import ModelStates
from .filter import LowPassFilter

class GazeboPose:
    def __init__(self, model_name='mur216'):
        """Gets the current pose of the model and calculates the velocity

        Args:
            model_name (str, optional): Defaults to 'mur216'.
        """        
        self.pose_stamped = None
        self.velocity_stamped = None
        self.velocity_stamped_filtered = None
        self.model_name = model_name
        self.filter_t = LowPassFilter((1,3), 0.9)
        self.filter_r = LowPassFilter((1,3), 0.9)
    
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_gazebo_pose)

    def cb_gazebo_pose(self, msg):
        """get pose and velocity from gazebo"""
        pose = msg.pose[msg.name.index(self.model_name)]
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        velocity_stamped = self.get_velocity_from_poses(self.pose_stamped, pose_stamped)
        if velocity_stamped is not None:
            self.velocity_stamped = velocity_stamped
            self.velocity_stamped_filtered = self.filter_velocity(velocity_stamped) # no filtering for now
        self.pose_stamped = pose_stamped

    def get_velocity_from_poses(self, pose_old=PoseStamped(), pose_new=PoseStamped()):
        """get velocity from two poses"""

        if pose_old is None or pose_new is None:
            rospy.loginfo("pose_old or pose_new is None")
            return

        position_old = np.array([pose_old.pose.position.x, pose_old.pose.position.y, pose_old.pose.position.z])
        position_new = np.array([pose_new.pose.position.x, pose_new.pose.position.y, pose_new.pose.position.z])
        orientation_old = np.array([pose_old.pose.orientation.x, pose_old.pose.orientation.y, pose_old.pose.orientation.z, pose_old.pose.orientation.w])
        orientation_new = np.array([pose_new.pose.orientation.x, pose_new.pose.orientation.y, pose_new.pose.orientation.z, pose_new.pose.orientation.w])
        # quaternion_old = transformations.quaternion_matrix(orientation_old)
        # quaternion_new = transformations.quaternion_matrix(orientation_new)

        delta_t=pose_new.header.stamp.secs-pose_old.header.stamp.secs
        if delta_t==0:
            return

        velocity_lin = (position_new - position_old) / delta_t
        velocity_ang = (orientation_old - orientation_new) / delta_t

        velocity = TwistStamped()
        velocity.twist.linear.x = velocity_lin[0]
        velocity.twist.linear.y = velocity_lin[1]
        velocity.twist.linear.z = velocity_lin[2]
        velocity.twist.angular.x = velocity_ang[0]
        velocity.twist.angular.y = velocity_ang[1]
        velocity.twist.angular.z = velocity_ang[2]
        velocity.header.stamp = pose_new.header.stamp

        return velocity
    
    def filter_velocity(self, velocity_stamped):
        return filter_velocity(self.filter_t, self.filter_r, velocity_stamped)

class GazeboVelocity:
    def __init__(self, model_name='mur216'):
        """Gets the current velocity of the model

        Args:
            model_name (str, optional): Defaults to 'mur216'.
        """        
        self.velocity_stamped = None
        self.velocity_stamped_filtered = None
        self.model_name = model_name
        self.filter_t = LowPassFilter((1,3), 0.9)
        self.filter_r = LowPassFilter((1,3), 0.9)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_gazebo_velocity)

    def cb_gazebo_velocity(self, msg):
        """get velocity from gazebo"""
        velocity = msg.twist[msg.name.index(self.model_name)]
        velocity_stamped = TwistStamped(header=Header(stamp=rospy.Time.now()), twist=velocity)
        self.velocity_stamped = velocity_stamped
        self.velocity_stamped_filtered = self.filter_velocity(velocity_stamped)

    def filter_velocity(self, velocity_stamped):
        return filter_velocity(self.filter_t, self.filter_r, velocity_stamped)

def filter_velocity(filter_t, filter_r, velocity_stamped=TwistStamped()):
    """filter velocity"""
    velocity_stamped_filtered = TwistStamped()
    velocity_stamped_filtered.header = velocity_stamped.header
    vel_t = filter_t.filter([velocity_stamped.twist.linear.__reduce__()[2]])
    vel_r = filter_r.filter([velocity_stamped.twist.angular.__reduce__()[2]])
    velocity_stamped_filtered.twist.linear.x = vel_t[0,0]
    velocity_stamped_filtered.twist.linear.y = vel_t[0,1]
    velocity_stamped_filtered.twist.linear.z = vel_t[0,2]
    velocity_stamped_filtered.twist.angular.x = vel_r[0,0]
    velocity_stamped_filtered.twist.angular.y = vel_r[0,1]
    velocity_stamped_filtered.twist.angular.z = vel_r[0,2]
    return velocity_stamped_filtered
        

if __name__ == '__main__':
    rospy.init_node('gazebo_pose')
    gazebo_pose = GazeboPose()
    gazebo_velocity = GazeboVelocity()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if gazebo_velocity.velocity_stamped and gazebo_pose.pose_stamped is not None:
            break
        rate.sleep()
    while not rospy.is_shutdown():
        print(f"Pose: {gazebo_pose.pose_stamped}")
        print(f"velocity:  {gazebo_velocity.velocity_stamped}")
        rate.sleep()