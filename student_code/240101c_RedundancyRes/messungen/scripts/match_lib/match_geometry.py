#!/usr/bin/env python
# # coding=latin-1
from geometry_msgs.msg import Point, Quaternion, Pose, PointStamped, PoseStamped
from actionlib_msgs.msg import *
import std_msgs.msg as std_msg
import numpy as np
import math
from scipy.spatial.transform import Rotation as rot
from tf import transformations

import copy
import time
import rospy


class MyPoint(Point):
    def __init__(self, pos=(0.0, 0.0, 0.0)):
        if isinstance(pos, Point) or type(pos).__name__ == '_geometry_msgs__Point':
            self._asArray = np.array(pos.__reduce__()[2])
        else:
            self._asArray = np.array(pos)
        super(MyPoint, self).__init__(*(self._asArray))

        # 1 hinten anfuegen
        self._asArray = np.append(self._asArray, 0)

    def __add__(self, p2):
        p_out = MyPoint()
        p_out.x = self.x + p2.x
        p_out.y = self.y + p2.y
        p_out.z = self.z + p2.z

        return p_out

    def __sub__(self, p2):
        p_out = MyPoint()
        p_out.x = self.x - p2.x
        p_out.y = self.y - p2.y
        p_out.z = self.z - p2.z

        return p_out

    def asArray(self):
        return np.array([self.x, self.y, self.z, 0])


class MyPointStamped(PointStamped):
    def __init__(self, pos=(0.0, 0.0, 0.0), frame_id=""):
        super(MyPointStamped, self).__init__()
        self.point = MyPoint(pos)
        self.header.frame_id = frame_id
        self.header.stamp = rospy.Time.now()

    def __add__(self, p2):
        p = self.point + p2.point
        return MyPointStamped(p, self.header.frame_id)


class MyOrient(Quaternion):
    def __init__(self, quatern=(0.0, 0.0, 0.0, 1.0)):
        if isinstance(quatern, Quaternion) or type(quatern).__name__ == '_geometry_msgs__Quaternion':
            self._asArray = np.array(quatern.__reduce__()[2])
        else:
            self._asArray = np.array(quatern)
        super(MyOrient, self).__init__(*(self._asArray))

    def __add__(self, o2):
        return MyOrient(transformations.quaternion_multiply(self.asArray(), o2.asArray()))

    def __sub__(self, o2):
        q_inv = transformations.quaternion_conjugate(o2.asArray())
        return MyOrient(transformations.quaternion_multiply(self.asArray(), q_inv))

    def asArray(self):
        return np.array([self.x, self.y, self.z, self.w])


class MyPose(Pose):
    def __init__(self, pos=(0.0, 0.0, 0.0), quatern=(0.0, 0.0, 0.0, 1.0)):
        if isinstance(pos, Pose):
            point = MyPoint(pos.position)
            orient = MyOrient(pos.orientation)
        else:
            point = MyPoint(pos)
            orient = MyOrient(quatern)
        super(MyPose, self).__init__(point, orient)
        self.position = point
        self.orientation = orient

    def __add__(self, p2):
        p_out = MyPose()
        p_out.position = self.position + p2.position
        p_out.orientation = self.orientation + p2.orientation
        return p_out

    def __sub__(self, p2):
        p_out = MyPose()
        p_out.position = self.position - p2.position
        p_out.orientation = self.orientation - p2.orientation
        return p_out

    def rotateVector(self, vec=None, rot=None):
        if vec is None:
            vec = self.position.asArray()
        if rot is None:
            rot = self.orientation.asArray()
        return rotateVector(vec, rot)


def rotateVector(vec=(0.0, 0.0, 0.0, 1.0), rot=(0.0, 0.0, 0.0, 1.0), transpose=False):
    if transpose:
        rot_conj = rot
        rot = transformations.quaternion_conjugate(rot_conj)
    else:
        rot_conj = transformations.quaternion_conjugate(rot)
    trans = transformations.quaternion_multiply(transformations.quaternion_multiply(rot_conj, vec), rot)[:3]
    return MyPoint(trans)


def rotationDiffRotated(rot_diff=(0.0, 0.0, 0.0, 1.0), rot=(0.0, 0.0, 0.0, 1.0), transpose=False):
    """Express orientation in different frame

    Args:
        rot_diff (tuple, optional): Rotation difference to rotate.
        rot (tuple): Rotation from frame of rot_diff to goalFrame.

    Returns:
        tuple (quaternion): rot_diff rotated by rot
    """
    if transpose:
        rot_conj = rot
        rot = transformations.quaternion_conjugate(rot)
    else:
        rot_conj = transformations.quaternion_conjugate(rot)
    rot_diff_rotated = transformations.quaternion_multiply(transformations.quaternion_multiply(rot_conj, rot_diff), rot)
    return rot_diff_rotated


def rotateToXAxis(points, axis=(0, 1), transpose=False):
    """Rotate list of points to from X-Axis to new axis

    Args:
        points ([[[x1, y1]], [[x2,y2]],...]): list of points to rotate
        axis (tuple, optional): new x-Axis. Defaults to (0,1).
        transpose (bool, optional): reverse action. Defaults to False.

    Returns:
        points: rotated
    """
    if points[0] is None: return None

    axis = axis / np.linalg.norm(axis)
    R = np.array([[axis[0], axis[1]],
                  [-axis[1], axis[0]]])
    if transpose: R = R.T
    contour_new = np.array(np.matmul(points, R))
    return contour_new


def getOrientDiffNoZ(goalOrient=MyOrient(), preGripOrient=MyOrient(), withoutAxisIdx=2):
    """Calculates OrientDiff and sets rotation around axis to 0

    Args:
        goalOrient (MyOrient): goalPose. Defaults to MyOrient().
        preGripOrient (MyOrient, optional): initial Pose. If default: result=goalOrient without Axis. Defaults to MyOrient().
        withoutAxisIdx (int, optional): Axis to disregard rotation. Defaults to 2.

    Returns:
        MyOrient(withoutAxis), MyOrient(onlyAxis): For default: Orientation without Z and only Z
    """

    orient_delta = MyOrient((goalOrient - preGripOrient).asArray())
    return getOrientationNoZ(orient_delta, withoutAxisIdx)


def getOrientationNoZ(orient_delta=MyPose, withoutAxisIdx=2):
    """Sets rotation around axis to 0

    Args:
        orient_delta (MyOrient): Defaults to MyPose.
        withoutAxisIdx (int, optional): Axis to disregard rotation. Defaults to 2.

    Returns:
        MyOrient(withoutAxis), MyOrient(onlyAxis): For default: Orientation without Z and only Z
    """
    rot_delta = rot.from_quat(orient_delta.asArray())
    rot_delta_euler = rot_delta.as_euler("xyz")

    # set Z to 0
    rot_noZ_euler = copy.copy(rot_delta_euler)
    rot_noZ_euler[withoutAxisIdx] = 0
    rot_noZ = rot.from_euler("xyz", rot_noZ_euler)

    rot_onlyZ_euler = [0, 0, 0]
    rot_onlyZ_euler[withoutAxisIdx] = rot_delta_euler[withoutAxisIdx]
    rot_onlyZ = rot.from_euler("xyz", rot_onlyZ_euler)

    return MyOrient(rot_noZ.as_quat()), MyOrient(rot_onlyZ.as_quat())


def getNearestOrientation(goalOrient=MyOrient(), preGripOrient=[MyOrient()]):
    """Gets smallest orientation difference between goal and preGripOrientations
    by disregarding rotation around Z-Axis

    Args:
        goalOrient (MyOrient): Goal Orientation. Defaults to MyOrient().
        preGripOrient (list[MyOrient]): possible preGripOrientations. Defaults to [MyOrient()].

    Returns:
       idxNearest: int, Idx of nearest orientation
       deltaOrientOut: MyOrient, Orientation difference without Z
       deltaZOut: MyOrient, Orientation difference in Z
    """
    smallestAngle = np.Inf
    for i, gripOrient in enumerate(preGripOrient):
        deltaOrient, deltaZ = getOrientDiffNoZ(goalOrient, gripOrient)
        if math.acos(deltaOrient.w) < smallestAngle:
            deltaOrientOut = deltaOrient
            deltaZOut = deltaZ
            idxNearest = i
    return idxNearest, deltaOrientOut, deltaZOut


def getOrientationDiffList(goalOrient=MyOrient(), preGripOrient=[MyOrient()], maxDiff=math.pi*2):
    """Gets orientation difference between goal and preGripOrientations
    by disregarding rotation around Z-Axis

    Args:
        goalOrient (MyOrient): Goal Orientation. Defaults to MyOrient().
        preGripOrient (list[MyOrient]): possible preGripOrientations. Defaults to [MyOrient()].

    Returns:
        list[
           idx: int, Idx of orientation
           deltaOrient: MyOrient, Orientation difference without Z
           deltaZ: MyOrient, Orientation difference in Z
       ]
    """
    sortedPreGrip = []
    for i, gripOrient in enumerate(preGripOrient):
        deltaOrient, deltaZ = getOrientDiffNoZ(goalOrient, gripOrient)
        if math.acos(deltaOrient.w) < maxDiff:
            sortedPreGrip.append((i, deltaOrient, deltaZ))

    sortedPreGrip.sort(key=lambda x: math.acos(x[1].w))
    return sortedPreGrip

def getTransformation(listener, fromFrame, toFrame, syncTimePublisher):
    """Express fromFrame in toFrame (transform fromFrame to toFrame)

    Returns:
        pos, rot: output of listener.lookupTransform(toFrame, fromFrame, now)
    """
    try:
        now = rospy.Time.now()
        listener.waitForTransform(toFrame, fromFrame, now, rospy.Duration(4.0))
        (pos, rot) = listener.lookupTransform(toFrame, fromFrame, now)
    except:  # ExtrapolationException:
        syncTimePublisher.publish(std_msg.Bool(True))
        time.sleep(0.5)
        now = rospy.Time.now()
        listener.waitForTransform(toFrame, fromFrame, now, rospy.Duration(4.0))
        (pos, rot) = listener.lookupTransform(toFrame, fromFrame, now)

    return pos, rot


def _transformMsgToFrame(listener, syncTimePublisher, frame, p_msg, func_transf=None):
    """Transform Stamped Msg to frame

    Args:
        listener (tf): listener for transformation
        syncTimePublisher: publisher to syncTime
        frame (string): Frame to transform into
        p_msg: msg to Transform with frameid in header
        func_transf (function): i.e. listener.transformPoint

    Returns:
        p_msg_new: transformed msg
    """
    if func_transf is None:
        if type(p_msg) == PointStamped:
            func_transf = listener.transformPoint
        elif type(p_msg) == PoseStamped:
            func_transf = listener.transformPose
        else:
            return None
    try:
        now = rospy.Time.now()
        listener.waitForTransform(p_msg.header.frame_id, frame, now, rospy.Duration(4.0))
        p_msg_new = func_transf(frame, p_msg)
    except:  # ExtrapolationException:
        syncTimePublisher.publish(std_msg.Bool(True))
        time.sleep(0.5)
        now = rospy.Time.now()
        listener.waitForTransform(p_msg.header.frame_id, frame, now, rospy.Duration(4.0))
        p_msg_new = func_transf(frame, p_msg)
    return p_msg_new


def transformPointMsgToFrame(listener, syncTimePublisher, frame, p_msg=PointStamped()):
    """transforms point from PointStamped to frame.

    Args:
        listener (tf): listener for transformation
        syncTimePublisher: publisher to syncTime
        frame (string): Frame to transform into
        p_msg (geom_msg.PointStamped()): Point to transform with frameid in header.

    Returns:
        [Point]: 
    """
    # Transform point to toFrame: TODO: replace with _transformMsgToFrame
    try:
        now = rospy.Time.now()
        listener.waitForTransform(p_msg.header.frame_id, frame, now, rospy.Duration(4.0))
        p_msg_new = listener.transformPoint(frame, p_msg)
    except:  # ExtrapolationException:
        syncTimePublisher.publish(std_msgs.msg.Bool(True))
        time.sleep(0.5)
        now = rospy.Time.now()
        listener.waitForTransform(p_msg.header.frame_id, frame, now, rospy.Duration(4.0))
        p_msg_new = listener.transformPoint(frame, p_msg)

    p = p_msg_new.point
    return p


def transformPoseMsgToFrame(listener, syncTimePublisher, frame, p_msg=PoseStamped()):
    """transforms point from PointStamped to frame.

    Args:
        listener (tf): listener for transformation
        syncTimePublisher: publisher to syncTime
        frame (string): Frame to transform into
        p_msg (geom_msg.PoseStamped()): pose to transform with frameid in header.

    Returns:
        [Pose]
    """
    p_msg_new = _transformMsgToFrame(listener, syncTimePublisher, frame, p_msg, listener.transformPose)
    p = MyPose(p_msg_new.pose)
    return p

def inverseTransformationMat(mat):
    """Inverts Transformation Matrix

    Args:
        mat (np.array): Transformation Matrix

    Returns:
        np.array: Inverted Transformation Matrix
    """
    matInv = np.eye(4)
    matInv[:3, :3] = mat[:3, :3].T
    matInv[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return matInv

if __name__ == '__main__':
    rospy.init_node("Test_geom")

    p1 = MyPoint((1, 2, 3))
    p2 = MyPoint((1, 2, 3))

    p3 = p1 + p2

    print(p3)

    pose1 = MyPose((1, 2, 3))
    pose2 = MyPose((1, 2, 3))

    pose3 = MyPose()

    pose3.position = pose1.position + pose2.position

    print(pose3)

    # MyPose from Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 1, 2, 3
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 1, 0, 0

    pose_my = MyPose(pose)
    print(pose_my)

    p1 = MyPose((0.656036424314, -0.0597577841713, -0.103558385398), (-0.909901224555, 0.41268467068,
                                                                      -0.023065127793, 0.0352011934197))
    a1 = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
          2.762937863667806, 0.815807519980951]

    # panda_goal = PandaGoals(p1, a1)
    # print(panda_goal)

    p_stamped = MyPointStamped((1, 2, 3))
    p_stamped2 = MyPointStamped((1, 2, 3))

    p_stamped_res = p_stamped + p_stamped2

    print(p_stamped_res)

    # Orientation:
    o_diff = MyOrient((0, 0, 0.7071068, 0.7071068))
    print(f"{rotateVector((1, 0, 0, 1), o_diff.asArray()) = }")

    o = MyOrient()  # 0°
    o_diff_rot = rotationDiffRotated(o.asArray(), o_diff.asArray())
    print(f"{o_diff_rot = }")  # 0°

    o = MyOrient((0.7071068, 0, 0.0, 0.7071068))  # 45° um x
    o_diff_rot = rotationDiffRotated(o.asArray(), o_diff.asArray())
    print(o_diff_rot)

    diff_list = getOrientationDiffList(o, [p1.orientation, pose1.orientation, pose2.orientation, pose3.orientation])
    print(diff_list)
