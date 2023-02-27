#!/usr/bin/env python  
import rospy
from typing import List
from match_lib.topics import PosePublisher, RotatePosePublisher
from match_lib.match_geometry import MyPose
from geometry_msgs.msg import Vector3Stamped, Vector3, PoseStamped
import tf

def main():
    # source_frame = 'map'
    # target_frame_ur = 'mur/ur/wrist_3_link'
    # target_frame_mir = "mur/mir/base_footprint"
    
    # source_frames = [source_frame, source_frame]
    # target_frames = [target_frame_mir, target_frame_ur]
    
    listener = tf.TransformListener()
    
    source_frames = ["map", "map", "map", "mur/ur/base_link", "mur/mir/base_footprint","mur/ur/base_link"]
    target_frames = ["mur/mir/base_footprint", "mur/ur/wrist_3_link", "mur/ur/base_link", "map", "mur/ur/base_link", "mur/ur/wrist_3_link"]
    names = ["mir_to_map", "map_to_eef", "ur_base_to_map","ur_map_to_base", "mir_to_ur", "ur_to_eef"]
    # rotate velocities of ur to map frame:
    rot_publisher = RotatePosePublisher("map", "mur/ur/base_link", names[-1], "ur_to_eef_in_map", tf_timeout=3)
    # rotate velocities of mir to base frame:
    RotatePosePublisher("mur/ur/base_link","map", names[2], "ur_base_to_map_in_base", tf_timeout=3)
    # ur_base_to_map_in_base
    pose_publisher = PosePublisher(target_frames, source_frames, pub_topics=names)
    pose_publisher.publish_poses()

if __name__ == '__main__':
    rospy.set_param('/use_sim_time', True)
    rospy.init_node('measurements_eef')
    main()

# for pose and manipulability plot:
#rosbag record /ur_to_eef_in_map /ur_base_to_map /mur/velocity_command /map_to_eef /ur_base_to_map_in_base /ur_to_eef /mur/ur/joint_states
