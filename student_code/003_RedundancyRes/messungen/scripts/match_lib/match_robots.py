#!/usr/bin/env python
# # coding=latin-1
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, Pose
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import *
import std_msgs.msg as std_msg
import numpy as np
# import quaternion
from .panda_grasping import *
from .match_geometry import *
from .robot_mats.jacobians.jacobian_ur_16_eef import getJacobianUr16_base_link_inertiaUr16_wrist_3_link as getJacobianManipulator
from .robot_mats.transformations.transform_ur16_base_link_eef import getTransform as getTransformManipulator, getVector_eef
from .robot_mats.jacobians.jacobian_platform import getJacobianPlatform, getJacobianPlatformWithEEF, getRotationMatrixS3
import tf
from tf import transformations
from tf import ExtrapolationException

class Joints():
    def __init__(self, T=getTransformManipulator, J=getJacobianManipulator, joint_names=None, ns=""):
        """Class to get the joint angles, transformation and jacobian of a robot

        Args:
            T (func, optional): Defaults to getTransformManipulator.
            J (func, optional): Defaults to getJacobianManipulator.
            joint_names (list(string), optional): Defaults to ['UR16/shoulder_pan_joint', 'UR16/shoulder_lift_joint', 'UR16/elbow_joint', 'UR16/wrist_1_joint', 'UR16/wrist_2_joint', 'UR16/wrist_3_joint'].
            ns (str, optional): Robot namespace if not launched in group (i.e. "/mur216/"). Defaults to "".
        """
        topic = ns+"joint_states"
        rospy.logdebug(f"Joints class subscribed to {topic}")
        rospy.Subscriber(topic, JointState, self.cb_joint_states)
        self.rate = rospy.Rate(10)
        self.T = T
        self.J = J
        if joint_names is None:
            self.joint_names = ['UR16/shoulder_pan_joint', 'UR16/shoulder_lift_joint', 'UR16/elbow_joint', 'UR16/wrist_1_joint', 'UR16/wrist_2_joint', 'UR16/wrist_3_joint']
        else:
            self.joint_names = joint_names
        self.q = np.zeros(len(self.joint_names))
        self.q_dot = np.zeros(len(self.joint_names))

    def cb_joint_states(self, msg=JointState()):
        for i in range(len(self.joint_names)):
            idx=msg.name.index(self.joint_names[i])
            self.q[i] = msg.position[idx]
            self.q_dot[i] = msg.velocity[idx]

    def getTransformation(self):
        return self.T(self.q)

    def getJacobian(self):
        return self.J(self.q)

    def getPose_eef(self):
        R=np.zeros((4,4))
        R[:3,:3]=self.getTransformation()[:3,:3]
        R[3,3]=1
        quaternions = transformations.quaternion_from_matrix(R)
        return Pose(position=Point(*self.getTransformation()[:3,3]), orientation=Quaternion(*quaternions))

class PandaGoals(object):
    def __init__(self, pose_relative_pre=MyPose(), axis_goal=[], pose_relative_grip=MyPose()):
        if len(axis_goal) < 7:
            print("using standard for Axis_goal because len<7")
            axis_goal = [-0.09165325995045537, -0.1307664982896102, -0.08691672911214791, -1.2039535559629443,
                         -0.058938511593474276, 1.7850536203251945, -1.5727488613542584]
        # self.pose_relative = pose_relative
        self.pose_relative_pre = pose_relative_pre
        self.pose_relative_grip = pose_relative_grip

        self.axis_goal = axis_goal
        self.movement = None

    def calcRelGoal(self, base_pose=MyPose(), grab_pose=None):
        # rel_pose = myPose()
        # pose_current = base_pose+self.pose_relative
        # rel_pose = grab_pose-pose_current
        if grab_pose is None:
            grab_pose = self.pose_relative
        rel_pose = grab_pose - base_pose

        return rel_pose

    def transfPoseBase(self, pose=MyPose()):
        q = pose.orientation.asArray()
        q_conj = transformations.quaternion_conjugate(q)
        t = pose.position.asArray()
        trans = transformations.quaternion_multiply(transformations.quaternion_multiply(q_conj, t), q)[:3]
        # trans = q_conj*t*q

        return MyPose(trans, q_conj)


class PandaMove(object):
    def __init__(self, group_name="panda_arm", ns='', robot_description="robot_description", listener=None):
        moveit_commander.roscpp_initialize([])
        self.ns = ns
        self.finger_length = 0.05
        self.maxWidthGrip = 0.1
        self.syncTime = rospy.Publisher("/syncTime", std_msg.Bool, queue_size=1)
        self.robot = moveit_commander.RobotCommander(ns=ns, robot_description=robot_description)
        # interface to a planning group (group of joints). used to plan and execute motions
        try:
            self.move_group = moveit_commander.MoveGroupCommander(group_name, ns=ns,
                                                                  robot_description=robot_description)
        except RuntimeError:
            rospy.logerr("group name " + group_name + " is not available. Available Planning Groups: ")
            rospy.logerr(self.robot.get_group_names())
            raise RuntimeError

        # remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        # Displays trajectory in RVIZ
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.move_group.set_end_effector_link("panda_hand")

        if listener is None:
            self.listener = tf.TransformListener()
        else:
            self.listener = listener

        # Add Gripper:
        self.gripper = PandaGripper(ns)

    def velocity_scale(self, plan, scale):
        for i in range(len(plan.joint_trajectory.points)):
            plan.joint_trajectory.points[i].time_from_start /= scale
            vel = list(plan.joint_trajectory.points[i].velocities)
            acc = list(plan.joint_trajectory.points[i].accelerations)
            for j in range(len(plan.joint_trajectory.points[i].velocities)):
                vel[j] *= scale
                acc[j] *= scale * scale
                plan.joint_trajectory.points[i].velocities = vel
                plan.joint_trajectory.points[i].accelerations = acc

        return plan

    def movePose(self, pose=MyPose(), vel=1):
        pose = Pose(pose.position, pose.orientation)
        self.move_group.set_pose_target(pose)
        # call the planner to compute the plan and execute it.
        # plan = move_group.go(wait=True)
        plan = self.move_group.plan()  # plan can get altered
        plan = self.velocity_scale(plan, vel)
        self.move_group.execute(plan, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def movePoseLin(self, pose=MyPose(), vel=1):
        waypoints = []  # ggf in 90 deg zu Orientierung EEF und dann Rest nur in x-Richtung aus EEF heraus
        waypoints.append(copy.deepcopy(self.move_group.get_current_pose().pose))  # current pose
        wpose = Pose(pose.position, pose.orientation)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        # plan = self.velocity_scale(plan, vel)
        self.move_group.execute(plan, wait=True)

    def movePoseTotal(self, pose=MyPose(), linear=False):
        """ Not working properly use movePoseTotalViaHand instead"""
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(self.ns + "/panda_link0", "map", now, rospy.Duration(4.0))
            (pos, rot) = self.listener.lookupTransform(self.ns + "/panda_link0", "map", now)
        except:  # ExtrapolationException:
            self.syncTime.publish(std_msg.Bool(True))
            time.sleep(0.5)
            now = rospy.Time.now()
            self.listener.waitForTransform(self.ns + "/panda_link0", "map", now, rospy.Duration(4.0))
            (pos, rot) = self.listener.lookupTransform(self.ns + "/panda_link0", "map", now)

        poseRel = MyPose(tuple(pos), tuple(rot))
        poseRel = pose - poseRel

        if linear:
            self.movePoseLin(poseRel)
        else:
            self.movePose(poseRel)

    def movePoseTotalViaHand(self, pose=MyPose(), toFrame="map", v=0.5):
        """using transformation from hand to toFrame(map) to move remaining distance

        Args:
            pose (MyPose()): Pose to move to in toFrame frame.
            toFrame (str, optional): Reference Frame. Defaults to "map".
            v (float, optional): Speed scale
            
        Returns: True if movement successful, False if not or no plan found
        """
        pos,rot = getTransformation(self.listener, self.ns+"/panda_hand", toFrame, self.syncTime)

        poseRel = MyPose(tuple(pos), tuple(rot))
        poseRel = pose - poseRel  # pose diff in toFrame frame

        return self.moveLin(poseRel, vel=v, inFrame=toFrame)
            
    def moveLin(self, pose=MyPose(), vel=1, inFrame="/miranda/panda/panda_hand"):
        """moveLinear in inFrame (panda_hand)

        Args:
            pose (MyPose, optional): PoseDifference to move. Defaults to MyPose().
            vel (int, optional): Velocity with which to move. Defaults to 1.
            inFrame (str, optional): Frame of PoseDifference. Defaults to "/miranda/panda/panda_hand".

        Returns:
            [type]: [description]
        """

        # Transform Pose difference to panda_link0 (only rotate vector)
        _, rot_0_hand = getTransformation(self.listener, self.ns + "/panda_link0", inFrame,
                                          self.syncTime)
        diffVec = rotateVector(pose.position.asArray(), rot_0_hand)
        diffRot = rotationDiffRotated(pose.orientation.asArray(), rot_0_hand)
        pose = MyPose(diffVec, diffRot)

        waypoints = []
        # wpose is in panda_link0-frame (planning frame)
        wpose = MyPose(self.move_group.get_current_pose().pose.position,
                       self.move_group.get_current_pose().pose.orientation)
        waypoints.append(copy.deepcopy(wpose))  # current pose
        wpose += pose  # append relative pose to current pose
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        plan = self.velocity_scale(plan, vel)
        if len(plan.joint_trajectory.points):
            return self.move_group.execute(plan, wait=True)
        rospy.loginfo("No Plan found for lin movement")
        return False


class MirNav2Goal(object):

    def __init__(self, mir_prefix=""):
        use_poseSub = False
        self.listener = tf.TransformListener()
        self.status = None  # if robot has no status/goal := 10
        self.id = 0
        self.ready = True

        # rospy.init_node('my_moveGoal', anonymous=False)
        self.pub = rospy.Publisher(mir_prefix + '/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

        sub_status = rospy.Subscriber(mir_prefix + '/move_base/status', GoalStatusArray, self.status_callback)

        # sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback) # get the messages of the robot pose in frame
        if use_poseSub:
            self.mirPose = MyPose()
            sub_odom = rospy.Subscriber(mir_prefix + '/robot_pose', Pose, self.odom_callback)

    def status_callback(self, msg=GoalStatusArray()):
        if len(msg.status_list):
            id = msg.status_list[0].goal_id.id
            self.status = msg.status_list[0].status
            if id != self.id:
                self.id = id
                self.ready = True
        else:
            self.status = 10

        ############ -- get the current pose of the robot -- #################

    def odom_callback(self, msg=Pose()):
        pos = msg.position
        self.mirPose.position.x = float(pos.x)
        self.mirPose.position.y = float(pos.y)
        self.mirPose.position.z = float(pos.z)

        ori = msg.orientation
        self.mirPose.orientation.w = float(ori.w)
        self.mirPose.orientation.x = float(ori.x)
        self.mirPose.orientation.y = float(ori.y)
        self.mirPose.orientation.z = float(ori.z)

    def getMirPose(self):
        (pos, rot) = self.listener.lookupTransform('/map', '/miranda/mir/base_link', rospy.Time(0))
        mirPose = MyPose()
        mirPose.position = MyPoint(tuple(pos))
        mirPose.orientation = MyOrient(tuple(rot))
        return mirPose

    def sendGoalPos(self, pose):
        self.ready = False
        poseMsg = PoseStamped()
        poseMsg.header.frame_id = "map"
        poseMsg.header.stamp = rospy.Time.now()
        poseMsg.pose = pose

        self.pub.publish(poseMsg)

    def getSendGoal(self):
        pose = self.getGoalCommandLine()
        self.sendGoalPos(pose)

    def getGoalCommandLine(self, deg=True):
        print("Position: ")
        x = float(input("X= "))
        y = float(input("Y= "))

        if deg:
            print("Orientation: 0-360")
            z = float(input("Z= "))
            z = (z % 360) / 360 * 2 - 1
        else:
            print("Orientation: 0-1")
            z = float(input("Z= "))
        w = math.sqrt(1 - z ** 2)

        pose = MyPose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        return pose

    def is_ready(self):
        if self.ready and self.status >= 3:
            return True
        return False

    # def goalReached(self):
    #     #  * /mobile_base_controller/cmd_vel [geometry_msgs/Twist]
    #     # * /move_base/feedback [move_base_msgs/MoveBaseActionFeedback]
    #     # * /move_base/goal [move_base_msgs/MoveBaseActionGoal]
    #     # * /move_base/result [move_base_msgs/MoveBaseActionResult]
    #     # * /move_base/status [actionlib_msgs/GoalStatusArray] == status_list --> letzter Status
    #     # GoalStatusArray.status_list.text=="Goal reached" oder .status == 3?
    #     # "Failed to find a valid plan. Even after executing recovery behaviors." .status==4
    #     # while .status < 3: sleep
    #     # sonst: dist(soll-ist < accuracy) via /robot_pose
    #     pass


if __name__ == '__main__':
    p1 = MyPoint((1, 2, 3))
    p2 = MyPoint((1, 2, 3))

    p3 = p1 + p2

    print(p3)

    pose1 = MyPose((1, 2, 3))
    pose2 = MyPose((1, 2, 3))

    pose3 = MyPose()

    pose3.position = pose1.position + pose2.position

    print(pose3)

    p1 = MyPose((0.656036424314, -0.0597577841713, -0.103558385398), (-0.909901224555, 0.41268467068,
                                                                      -0.023065127793, 0.0352011934197))
    a1 = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
          2.762937863667806, 0.815807519980951]

    panda_goal = PandaGoals(p1, a1)
    print(panda_goal)

    # TEST PANDA:
    rospy.init_node("match_Robots")

    mir_prefix = "/miranda/mir"
    panda_prefix = "/miranda/panda"
    panda_description = "/miranda/panda/robot_description"
    try:
        panda = PandaMove(group_name="panda_arm", ns=panda_prefix, robot_description=panda_description, listener=None)
    except RuntimeError:
        mir_prefix = ""
        panda_prefix = ""
        panda_description = "robot_description"
        panda = PandaMove(group_name="panda_arm", ns=panda_prefix, robot_description=panda_description, listener=None)

    panda.move_group.set_pose_reference_frame("/miranda/panda/panda_link0")
    target = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
              2.762937863667806, 0.815807519980951]
    panda.move_group.set_joint_value_target(target)
    plan = panda.move_group.plan()
    res = panda.move_group.execute(plan, wait=True)

    # panda.move_group.set_pose_reference_frame("panda_link0")
    print("============ ROBOT POSE")
    p = panda.move_group.get_current_pose()
    print(p)
