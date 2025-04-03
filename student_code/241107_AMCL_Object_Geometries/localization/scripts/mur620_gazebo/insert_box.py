#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def load_box_gazebo(width, height, depth, initial_pose_x, initial_pose_y):

    # Wait for the SpawnModel service to be available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        # Create a proxy for the SpawnModel service
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # Define the box model in SDF format with the given dimensions
        box_sdf = f"""
        <sdf version="1.6">
          <model name="box">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
              <pose>0 0 0.5 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{width} {depth} {height}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{width} {depth} {height}</size>
                  </box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """

        # Define the initial position of the box
        initial_pose = Pose(Point(initial_pose_x, initial_pose_y, 0), Quaternion(0, 0, 0, 0))

        # Call the service to insert the model into the simulation
        spawn_model_prox("box", box_sdf, "", initial_pose, "world")

        rospy.loginfo("Box inserted into Gazebo")

    except rospy.ServiceException as e:
        rospy.logerr("Spawn service failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('load_box_node', anonymous=True)

    # Get parameters from the parameter server
    box_width = rospy.get_param('~width')
    box_height = rospy.get_param('~height')
    box_depth = rospy.get_param('~depth')
    initial_pose_x = rospy.get_param('~center_object_x')
    initial_pose_y = rospy.get_param('~center_object_y')



    try:
        load_box_gazebo(box_width, box_height, box_depth, initial_pose_x, initial_pose_y)
    except rospy.ROSInterruptException:
        pass
