#!/usr/bin/env python
# coding=latin-1

import sys
import rospy
import time

from franka_gripper.msg import GraspActionGoal, GraspActionResult, HomingActionGoal, HomingActionResult
from actionlib_msgs.msg import GoalID


class PandaGripper(object):
    def __init__(self, panda_prefix="/miranda/panda", homing=True):

        # try:
        #     self.pub = rospy.Publisher(panda_prefix + '/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
        # except RuntimeError:     # try with no namespace
        #     print("Runtime Errror. Trying with no namespace...")
        #     panda_prefix=""
        #     self.pub = rospy.Publisher(panda_prefix + '/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
        self.pub = rospy.Publisher(panda_prefix + '/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
        print(self.pub.name)

        self.pubCancel = rospy.Publisher(panda_prefix + '/franka_gripper/grasp/cancel', GoalID, queue_size=1)
        self.pubHoming = rospy.Publisher(panda_prefix + '/franka_gripper/homing/goal', HomingActionGoal, queue_size=1)

        sub = rospy.Subscriber(panda_prefix + '/franka_gripper/grasp/result', GraspActionResult, self.graspResult)
        sub_homing = rospy.Subscriber(panda_prefix + '/franka_gripper/homing/result', HomingActionResult,
                                      self.homingResult)

        self.goalId = GoalID()
        self.success = False
        self.status = None
        self.gotResult = False
        self.homingSuccess = False

        while self.pubHoming.get_num_connections() == 0:
            print("Waiting for Subscribers to connect")
            time.sleep(0.5)
        if homing:
            self.homing()

    def graspResult(self, msg=GraspActionResult()):
        self.gotResult = True
        self.success = msg.result.success
        self.status = msg.status

    def move(self, width=0.0, eps_out=0.2, eps_in=0.01, force=0.0):
        """Moves to width. eps for success

        Args:
            width (float, optional): goal width. Defaults to 0.0.
            eps_out (float, optional): outer epsilon. Defaults to 0.2.
            eps_in (float, optional): inner epsilon. Defaults to 0.01.
            force (float, optional): Force in N. Defaults to 0.0.
        """
        rospy.loginfo("Grasping")
        msg = GraspActionGoal()
        msg.goal.force = force
        msg.goal.speed = 0.1
        msg.goal.width = width
        msg.goal.epsilon.inner = eps_in
        msg.goal.epsilon.outer = eps_out

        msg.goal_id.stamp = rospy.Time.now()

        self.pub.publish(msg)

        self.goalId = msg.goal_id
        self.success = False
        self.gotResult = False

    def graspObj(self):
        """closes gripper and checks if object inside

        Returns:
            bool: success
        """
        self.move(0.00, 0.001, 0.01, force=0.01)
        while not self.gotResult:
            time.sleep(0.1)
        if not self.success:  # gripper.success:
            rospy.loginfo("Object found!")
            # ToDO: /franka_gripper/joint_states .position for comparison if obj lost
            self.move(0.0, 0.2, 0.2, force=50)
            return True
        return False

    def graspCancel(self):
        rospy.loginfo("Cancel Grasping")
        self.pubCancel.publish(self.goalId)

    def homing(self):
        """Homing of gripper
        """
        rospy.loginfo("Homing")
        self.homingSuccess = False
        msg = HomingActionGoal()
        self.pubHoming.publish(msg)
        while not self.homingSuccess:
            time.sleep(0.1)

    def homingResult(self, msg=HomingActionResult()):
        self.homingSuccess = msg.result.success
        rospy.loginfo("Homing success: " + self.homingSuccess.__str__())


if __name__ == "__main__":
    rospy.init_node("PandaGripperTest")
    gripper = PandaGripper()

    if not gripper.graspObj():
        print("No Object")

    print("opening in 5s")
    time.sleep(5)
    gripper.move(0.20, 0.01)
    # gripper.graspCancel()
