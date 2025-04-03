#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Vector3Stamped, QuaternionStamped
from .match_geometry import MyPose, rotateVector
from typing import List

class PosePublisher():

    def __init__(self, target_frames: List[str], source_frames: List[str], rate: float = 10.0, pub_topics: List[str] = None, tf_timeout=10):
        # check if node is already running
        if rospy.get_name() == "/unnamed":
            rospy.init_node("pose_publisher")
        if len(target_frames) != len(source_frames):
            raise ValueError("target_frames and source_frames must have the same length")
        if pub_topics is None:
            pub_topics = [f"/tf_pose_{source}_TO_{target}" for source, target in zip(source_frames, target_frames)]
        if len(pub_topics) != len(target_frames):
            raise ValueError("pub_topics must have the same length as target_frames")
        self.target_frames = target_frames
        self.source_frames = source_frames
        
        self.pubs = [rospy.Publisher(topic, geometry_msgs.msg.Pose, queue_size=10) for topic in pub_topics]
        self.listener = tf.TransformListener(tf_timeout)
        self.rate = rospy.Rate(rate)
        self.poses = [None] * len(target_frames)

    def publish_poses(self):
        while not rospy.is_shutdown():
            self.publish_poses_once()
            self.rate.sleep()
            
    def publish_poses_once(self):
        for pub, source, target in zip(self.pubs, self.source_frames, self.target_frames):
            try:
                t, r = self.listener.lookupTransform(source, target, rospy.Time(0))
                p = MyPose(t, r)
                pub.publish(p)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
    def get_poses(self):
        for i, (source, target) in enumerate(zip(self.source_frames, self.target_frames)):
            try:
                t, r = self.listener.lookupTransform(source, target, rospy.Time(0))
                p = MyPose(t, r)
                self.poses[i]=p
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return self.poses

class RotatePosePublisher():

    def __init__(self, target_frame, source_frame, topic_sub, topic_pub, tf_timeout=10):
        # super().__init__(tf_timeout) # if we want to inherit from TfListener
        # check if node is already running
        if rospy.get_name() == "/unnamed":
            rospy.init_node("pose_publisher")

        self.listener = tf.TransformListener(tf_timeout)
        self.target_frame = target_frame
        self.source_frame = source_frame

        self.pub = rospy.Publisher(topic_pub, Pose, queue_size=10)
        self.sub = rospy.Subscriber(topic_sub, Pose, self.callback)
        self.vector = Vector3Stamped()
        self.vector.header.frame_id = self.source_frame
        self.orient = QuaternionStamped()
        self.orient.header.frame_id = self.source_frame



    def callback(self, msg: Pose):
        if msg is None:
            return
        self.vector.header.stamp = rospy.Time()
        self.orient.header.stamp = rospy.Time()

        pos = self.transform_vec(msg.position)
        rot = self.transform_orientation(msg.orientation)
        if pos is None or rot is None:
            return
        msg.position = pos
        # msg.orientation = rot
        self.pub.publish(msg)


    def transform_vec(self, msg):
        """doesnt work if frame is also moving?!
        """
        self.vector.vector.x = msg.x
        self.vector.vector.y = msg.y
        self.vector.vector.z = msg.z
        try:
            vector_out = self.listener.transformVector3(self.target_frame, self.vector)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            return
        msg.x, msg.y, msg.z = vector_out.vector.x, vector_out.vector.y, vector_out.vector.z
        return msg
    
    def transform_orientation(self,msg):
        self.orient.quaternion.x = msg.x
        self.orient.quaternion.y = msg.y
        self.orient.quaternion.z = msg.z
        self.orient.quaternion.w = msg.w
        try:
            out = self.listener.transformQuaternion(self.target_frame, self.orient)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            return
        msg.x, msg.y, msg.z, msg.w = out.quaternion.x, out.quaternion.y, out.quaternion.z, out.quaternion.w
        return msg

    def rotate_xyz_by_match_geometry(self, msg):
        try:
            _,r = self.listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
            # transform = self.listener.fromTranslationRotation((0,0,0),r)        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            return
        p=rotateVector((msg.x, msg.y, msg.z, 1.0), r, transpose=False)
        msg.x, msg.y, msg.z = p.x, p.y, p.z
        return msg
        

if __name__ == '__main__':
    # source_frame = 'map'
    rospy.init_node('tf_listener')

    rospy.set_param('/use_sim_time', True)

    # target_frame = 'mur/ur/wrist_3_link'
    # source_frame = 'mur/ur/base_link'
    # source_frame_mir = "mur/mir/base_footprint"
    # target_frame_mir = "map"
    # source_frames = [source_frame, source_frame_mir]
    # target_frames = [target_frame, target_frame_mir]

    # pose_publisher = PosePublisher(target_frames, source_frames)
    # rotPos = RotatePosePublisher("/map", pose_publisher.source_frames[0], pose_publisher.pubs[0].name, "/mur/ur/target_pose_rotated")
    # pose_publisher.publish_poses()

    RotatePosePublisher("/mur/ur/base_link","/map", "/ur_base_to_map", "/ur_base_to_map_in_base_rot", tf_timeout=3)
    rospy.spin()