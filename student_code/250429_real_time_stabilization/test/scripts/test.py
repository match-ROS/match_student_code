#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import Wrench, Vector3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
import threading
import os

class TorqueExperiment:
    def __init__(self):
        rospy.init_node('torque_experiment_node')

        # CSV setup
        data_dir = os.path.expanduser('~/catkin_ws/src/data')
        os.makedirs(data_dir, exist_ok=True)
        self.csv_path = os.path.join(data_dir, 'torque_experiment.csv')
        self.csv_file = open(self.csv_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp','tcp_x','tcp_y','tcp_z','mir_x','mir_y','mir_z'])
        self.csv_file.flush()
        rospy.loginfo(f"Logging to {self.csv_path}")

        # Buffers for latest poses
        self.lock = threading.Lock()
        self.latest_tcp = None
        self.latest_mir = None

        # Subscribe to TCP PoseStamped
        rospy.Subscriber('/mur620/UR10_l/global_tcp_pose', PoseStamped, self.callback_tcp, queue_size=1)
        # Subscribe to MIR Pose (PoseStamped or Odometry)
        rospy.Subscriber('/mur620/mir_pose_simple', PoseStamped, self.callback_mir_ps, queue_size=1)
        rospy.Subscriber('/mur620/mir_pose_simple', Odometry, self.callback_mir_odom, queue_size=1)
        rospy.loginfo('Subscribed to TCP and MIR topics (PoseStamped + Odometry)')

        # Allow callbacks to start
        rospy.sleep(1.0)
        rospy.loginfo('Initial buffers: tcp=%s, mir=%s', self.latest_tcp, self.latest_mir)

        # Wait for Gazebo service
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.loginfo('ApplyBodyWrench service ready')

    def callback_tcp(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.latest_tcp = (p.x, p.y, p.z)
        rospy.logdebug(f"TCP update: {self.latest_tcp}")

    def callback_mir_ps(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.latest_mir = (p.x, p.y, p.z)
        rospy.logdebug(f"MIR PoseStamped update: {self.latest_mir}")

    def callback_mir_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        with self.lock:
            self.latest_mir = (p.x, p.y, p.z)
        rospy.logdebug(f"MIR Odometry update: {self.latest_mir}")

    def apply_torque(self, torque, duration_s):
        req = ApplyBodyWrenchRequest(
            body_name='mur620::base_footprint',
            reference_frame='world',
            reference_point=Vector3(0,0,0),
            wrench=Wrench(force=Vector3(0,0,0), torque=Vector3(torque,0,0)),
            start_time=rospy.Time(0),
            duration=rospy.Duration(duration_s)
        )
        try:
            self.apply_wrench(req)
            rospy.loginfo(f"Applied torque {torque} Nm for {duration_s}s")
        except rospy.ServiceException as e:
            rospy.logerr(f"Wrench service failed: {e}")

    def log_sample(self, label):
        with self.lock:
            if self.latest_tcp is None or self.latest_mir is None:
                rospy.logwarn(f"{label}: missing tcp={self.latest_tcp}, mir={self.latest_mir}")
                return
            tx, ty, tz = self.latest_tcp
            mx, my, mz = self.latest_mir
        ts = rospy.Time.now().to_sec()
        self.csv_writer.writerow([ts, tx, ty, tz, mx, my, mz])
        self.csv_file.flush()
        rospy.loginfo(f"{label}: logged TCP({tx:.3f},{ty:.3f},{tz:.3f}) MIR({mx:.3f},{my:.3f},{mz:.3f}) at {ts:.2f}")

    def run(self):
        rate = rospy.Rate(0.5)  # 2s cycle
        while not rospy.is_shutdown():
            self.log_sample('baseline')
            self.apply_torque(2000, 0.3)
            rospy.sleep(0.0)
            self.log_sample('after_plus')
            rospy.sleep(0.0)
            self.apply_torque(1900, 0.8)
            rospy.sleep(1.5)
            self.log_sample('after_minus')
            rospy.sleep(1.0)
            rate.sleep()

    def close(self):
        self.csv_file.close()

if __name__ == '__main__':
    exp = TorqueExperiment()
    try:
        exp.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        exp.close()