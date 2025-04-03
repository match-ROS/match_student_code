#!/usr/bin/env python
# coding=latin-1

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import std_msgs.msg
import geometry_msgs.msg as geom_msg

import time
import copy
import tf
import tf2_ros

import match_geometry
from match_geometry import MyPoint, MyPointStamped


class CameraHandler(object):
    """Handler to interact with Camera from other node
    """

    def __init__(self, listener=None, toFrame="map"):
        self.pub_runCam = rospy.Publisher("/cam/run", std_msgs.msg.Bool, queue_size=1)
        self.pub_gripP = rospy.Publisher("/cam/gripPosEstimate", MyPointStamped, queue_size=10)
        self.subGripP = rospy.Subscriber("/points_grasp", PointCloud2, self.cb_graspP, queue_size=10)
        self.graspP = []

        self.listener = listener
        self.toFrame = toFrame

        # sync time between miranda and panda
        self.syncTime = rospy.Publisher("/syncTime", std_msgs.msg.Bool, queue_size=1)
    
        self.headerPubPoseEstimate = std_msgs.msg.Header()
        self.headerPubPoseEstimate.frame_id = 'camera_depth_optical_frame'

    def sendGripPoseEstimate(self, p=(0,0,0)):
        """Sends grip pose estimate for camera. p in frame 'map'

        Args:
            p (tuple(3)): (x,y,z)
        """
        self.headerPubPoseEstimate.stamp = rospy.Time.now()
        self.headerPubPoseEstimate.seq += 1
        # msg = point_cloud2.create_cloud_xyz32(self.headerPubPoseEstimate, [p])
        msg = MyPointStamped(p, 'camera_depth_optical_frame')
        self.pub_gripP.publish(msg)

    def run(self, boolVal=True):
        """Starts/Stops gripPoints detection of camera script

        Args:
            boolVal (bool, optional): Starts cam if True, else stops. Defaults to True.
        """
        msg = std_msgs.msg.Bool()
        msg.data = boolVal
        self.pub_runCam.publish(msg)

        if not boolVal: self.graspP = []  # reset

    def cb_graspP(self, msg=PointCloud2()):
        assert isinstance(msg, PointCloud2)

        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.graspP = []
        for p in gen:
            if self.listener is not None:  # transform points to map-coordinates
                p_msg = geom_msg.PointStamped()
                p_msg.header = msg.header
                p_msg.point.x, p_msg.point.y, p_msg.point.z = p

                p = self.transformPointMsgToFrame(self.toFrame, p_msg)

            self.graspP.append(MyPoint(p))

    def transformPointMsgToFrame(self, frame, p_msg=geom_msg.PointStamped()):
        """transforms point from PointStamped to frame_id.

        Args:
            frame (string): Frame to trabsform into
            p_msg (geom_msg.PointStamped()): Point to transform with frameid in header.

        Returns:
            [Point]: 
        """
        # Transform point to toFrame:
        return match_geometry.transformPointMsgToFrame(self.listener, self.syncTime, frame, p_msg)

    def getGraspP(self):
        """Starts/Stops detection and returns the 2 graspPoints (transformed to "toFrame")

        Returns:
            list(MyPoint): Grasp points from edge detection
        """
        while len(self.graspP) != 2 and not rospy.is_shutdown():
            self.run()
            time.sleep(1)
        graspP = copy.copy(self.graspP)
        self.run(False)
        return graspP

    def getValidGraspP(self, minH=0.05, minD=0.0):
        """Checks grasp Points if in front of hand (cam.toFrame) and above ground (map)

        Args:
            minH (float, optional): Minimum Height above z=0 in map Frame. Defaults to 0.05.
            minD (float, optional): Minimum Distance to z=0 in self.toFrame (panda_hand). Defaults to 0.0

        Returns:
            MyPoint p: actual graspPoint in self.toFrame (hand)
            MyPoint p_hand_pre: p with Z=0 (no movement in Z for self.toFrame (hand: Z=grip))
            MyPoint p_map: p_hand_pre in map frame


        """
        while not rospy.is_shutdown():
            graspPoints = self.getGraspP()  # in panda_hand-frame
            # Z zeigt aus Greifer raus
            p = MyPoint(((graspPoints[0] + graspPoints[1]).asArray()[:3]) / 2.0)
            if p.z < minD:
                rospy.loginfo("Z Value is smaller 0: GraspPoint is behind gripper!")
                continue
            # change position: graspPosition in front of gripper (dont move in Z)
            p_hand_pre = MyPointStamped(copy.copy(p), self.toFrame)
            p_hand_pre.point.z = 0

            p_map = self.transformPointMsgToFrame("map", p_hand_pre)
            if p_map.z < minH:
                rospy.loginfo("Z Value is small: GraspPoint is near floor")
                continue

            # found valid grasp point
            return p, MyPoint(p_hand_pre.point), MyPoint(p_map)


if __name__ == '__main__':
    rospy.init_node("TEST_cam_otherNode")
    time.sleep(1)

    tf_caster = tf2_ros.StaticTransformBroadcaster()
    trans = (1.0, 0.0, 0.0)
    rot = (0.0, 0.0, 0.0, 1.0)
    tf_msg = geom_msg.TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "map"
    tf_msg.child_frame_id = "camera_depth_optical_frame"
    tf_msg.transform.translation.x = trans[0]
    tf_msg.transform.translation.y = trans[1]
    tf_msg.transform.translation.z = trans[2]
    tf_msg.transform.rotation.x = rot[0]
    tf_msg.transform.rotation.y = rot[1]
    tf_msg.transform.rotation.z = rot[2]
    tf_msg.transform.rotation.w = rot[3]
    tf_caster.sendTransform(tf_msg)

    listener = tf.TransformListener()

    cam = CameraHandler(listener, "map")
    time.sleep(1)
    points = cam.getGraspP()

    print(points)
