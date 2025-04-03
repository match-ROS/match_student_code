import rospy

# from match_lib.topics import PosePublisher
from move_for_distance import MoveForDistance
from measurements_eef import main as measure

# use z=+-0.05 for the measurements
if __name__ == '__main__':
    
    frame_target = "/map"
    frame_source = "/mur/ur/wrist_3_link"
    rospy.init_node('move_and_measure')
    
    move = MoveForDistance(1.1, frame_target, frame_source)

    # ## Measurements:
    # target_frame = 'mur/ur/wrist_3_link'
    # source_frame = 'mur/ur/base_link'
    # source_frame_mir = "mur/mir/base_footprint"
    # target_frame_mir = "map"
    # source_frames = [source_frame, source_frame_mir]
    # target_frames = [target_frame, target_frame_mir]

    # pose_publisher = PosePublisher(target_frames, source_frames)
    # pose_publisher.publish_poses()
    measure()
    #rosbag record /mur/ur/joint_states /mur/velocity_command /tf /tf_static
