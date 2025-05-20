#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, PoseStamped
import tf
import matplotlib.pyplot as plt
import csv
import numpy as np

# LowPassFilter class: Smoothes incoming data.
class LowPassFilter:                                          # Not used
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

# PIDController class: Implements a simple PID controller.
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# TCPData class
class TCPData:
    def __init__(self):
        self.pose = None            
        self.angles = None          
        self.reference_pose = None  # Reference nozzle pose when base is level
        self.reference_angles = None  # Reference nozzle orientation

    def update(self, pose, angles):
        self.pose = pose
        self.angles = angles
        if self.reference_pose is None:
            self.reference_pose = pose.copy()
        if self.reference_angles is None:
            self.reference_angles = angles.copy()

# nozzle stabilization for the left arm.
class NozzleStabilizerLeft:
    def __init__(self):
        rospy.init_node('nozzle_stabilizer_left', anonymous=False)
        
        # Processing rate and deadband
        self.dt = rospy.get_param("~dt", 0.005)  # 10 Hz
        self.deadband = rospy.get_param("~deadband", 0.005)
        self.last_time = 0.0
        
        # Low-pass filters for IMU data
        alpha = rospy.get_param("~alpha", 0.05)
        self.roll_filter = LowPassFilter(alpha)
        self.pitch_filter = LowPassFilter(alpha)
        self.yaw_filter = LowPassFilter(alpha)
        
        # Reference IMU orientation (set on first filtered measurement)
        self.reference_imu = None  # Tuple: (roll, pitch, yaw)
        
        # TCP data manager (for nozzle pose)
        self.tcp_data = TCPData()
        
        # PID controllers for angular corrections
        self.pid_roll = PIDController(
            rospy.get_param("~Kp_roll", 5.0),
            rospy.get_param("~Ki_roll", 0.5),
            rospy.get_param("~Kd_roll", 0.1),
            self.dt
        )
        self.pid_pitch = PIDController(
            rospy.get_param("~Kp_pitch", 5.0),
            rospy.get_param("~Ki_pitch", 0.5),
            rospy.get_param("~Kd_pitch", 0.1),
            self.dt
        )
        self.pid_yaw = PIDController(
            rospy.get_param("~Kp_yaw", 5.0),
            rospy.get_param("~Ki_yaw", 0.5),
            rospy.get_param("~Kd_yaw", 0.1),
            self.dt
        )
        
        # Gains for TCP (nozzle) linear (position) corrections
        self.Kp_tcp_x = rospy.get_param("~Kp_tcp_x", 2.0)
        self.Kp_tcp_y = rospy.get_param("~Kp_tcp_y", 2.0)
        self.Kp_tcp_z = rospy.get_param("~Kp_tcp_z", 2.0)
        
        # Maximum velocities for clamping (change it if needed)
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity", 1.0)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity", 1.0)
        
        # Lever arm vector: nominal relative position of the nozzle from the base pivot when level.
        # For example, [1.0, 0.0, 0.0] means the nozzle is 1 m ahead.
        self.lever_arm_vector = np.array(rospy.get_param("~lever_arm_vector", [-0.6, -1.2, -1.4])) #Change in ref to the position
        
        # Publisher and Subscribers
        self.twist_pub = rospy.Publisher('/mur620/UR10_l/twist_fb_command', Twist, queue_size=10)
        #rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mur620/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/mur620/UR10_l/tcp_pose', PoseStamped, self.tcp_callback)
        
        # Logger dictionary for analysis
        self.logger = {
            "time": [],
            "imu_roll": [],
            "imu_pitch": [],
            "imu_yaw": [],
            "tcp_roll": [],
            "tcp_pitch": [],
            "tcp_yaw": [],
            "combined_error_roll": [],
            "combined_error_pitch": [],
            "combined_error_yaw": [],
            "tcp_error_x": [],
            "tcp_error_y": [],
            "tcp_error_z": [],
            "correction_angular_x": [],
            "correction_angular_y": [],
            "correction_angular_z": [],
            "correction_linear_x": [],
            "correction_linear_y": [],
            "correction_linear_z": []
        }
        
        rospy.on_shutdown(self.plot_and_save_log)
        rospy.loginfo("NozzleStabilizerLeft initialized, waiting for IMU and TCP data...")

    def imu_callback(self, msg):
        current_time = rospy.get_time()
        if (current_time - self.last_time) < self.dt:
            return
        self.last_time = current_time
        
        # Convert raw IMU quaternion to Euler angles (IMU is mounted in the base)
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        raw_roll, raw_pitch, raw_yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        # Filter the IMU measurements
        imu_roll = self.roll_filter.update(raw_roll)
        imu_pitch = self.pitch_filter.update(raw_pitch)
        imu_yaw = self.yaw_filter.update(raw_yaw)
        
        # Set reference IMU orientation on first measurement
        if self.reference_imu is None:
            self.reference_imu = (imu_roll, imu_pitch, imu_yaw)
            rospy.loginfo("Reference IMU set: roll=%.3f, pitch=%.3f, yaw=%.3f", imu_roll, imu_pitch, imu_yaw)
            return
        
        # Compute combined orientation error 
        combined_error_roll = raw_roll + self.tcp_data.angles[0] - np.pi / 2
        combined_error_pitch = raw_pitch + self.tcp_data.angles[1]
        combined_error_yaw = 0
        
        # Update PID controllers for angular corrections
        output_roll = self.pid_roll.update(combined_error_roll)
        output_pitch = self.pid_pitch.update(combined_error_pitch)
        output_yaw = self.pid_yaw.update(combined_error_yaw)
        
        # Compute angular correction (negative feedback)
        angular_corr = Twist()
        angular_corr.angular.x = -output_roll
        angular_corr.angular.y = -output_pitch
        angular_corr.angular.z = -output_yaw
        
        # Process TCP (nozzle) position errors for linear corrections
        if self.tcp_data.pose is not None and self.tcp_data.reference_pose is not None:
            tcp_measured = np.array(self.tcp_data.pose)
            tcp_ref = np.array(self.tcp_data.reference_pose)
            # Raw TCP error (difference from reference when level)
            tcp_error = tcp_measured - tcp_ref
            
            # Correction using a lever arm model in 3D
            # Compute the rotation matrix from the filtered IMU roll and pitch.
            # ignore yaw since tilt-induced displacement is mainly due to roll and pitch.
            R = tf.transformations.euler_matrix(raw_roll, raw_pitch, 0)[:3, :3]
            # The predicted displacement of the nozzle due to base tilt is:
            # predicted_disp = R * lever_arm_vector - lever_arm_vector.
            predicted_disp = np.dot(R, self.lever_arm_vector) - self.lever_arm_vector
            
            # Correct the TCP error by subtracting the predicted displacement.
            tcp_error_corrected = tcp_error - predicted_disp
            tcp_error_x, tcp_error_y, tcp_error_z = tcp_error_corrected
        else:
            tcp_error_x = tcp_error_y = tcp_error_z = 0.0
        
        linear_corr = Twist()
        linear_corr.linear.x = -self.Kp_tcp_x * tcp_error_x
        linear_corr.linear.y = -self.Kp_tcp_y * tcp_error_y
        linear_corr.linear.z = -self.Kp_tcp_z * tcp_error_z
        
        # Compose final Twist command with both angular and linear corrections.
        command = Twist()
        command.angular.x = angular_corr.angular.x
        command.angular.y = angular_corr.angular.y
        command.angular.z = angular_corr.angular.z
        command.linear.x = linear_corr.linear.x
        command.linear.y = linear_corr.linear.y
        command.linear.z = linear_corr.linear.z
        
        # Clamp velocities.
        command.linear.x = np.clip(command.linear.x, -self.max_linear_velocity, self.max_linear_velocity)
        command.linear.y = np.clip(command.linear.y, -self.max_linear_velocity, self.max_linear_velocity)
        command.linear.z = np.clip(command.linear.z, -self.max_linear_velocity, self.max_linear_velocity)
        command.angular.x = np.clip(command.angular.x, -self.max_angular_velocity, self.max_angular_velocity)
        command.angular.y = np.clip(command.angular.y, -self.max_angular_velocity, self.max_angular_velocity)
        command.angular.z = np.clip(command.angular.z, -self.max_angular_velocity, self.max_angular_velocity)
        
        self.twist_pub.publish(command)
        rospy.loginfo("Published correction Twist: %s", command)
        
        # Log data for analysis.
        self.logger["time"].append(current_time)
        self.logger["imu_roll"].append(imu_roll)
        self.logger["imu_pitch"].append(imu_pitch)
        self.logger["imu_yaw"].append(imu_yaw)
        if self.tcp_data.angles is not None:
            self.logger["tcp_roll"].append(self.tcp_data.angles[0])
            self.logger["tcp_pitch"].append(self.tcp_data.angles[1])
            self.logger["tcp_yaw"].append(self.tcp_data.angles[2])
        else:
            self.logger["tcp_roll"].append(0.0)
            self.logger["tcp_pitch"].append(0.0)
            self.logger["tcp_yaw"].append(0.0)
        self.logger["combined_error_roll"].append(combined_error_roll)
        self.logger["combined_error_pitch"].append(combined_error_pitch)
        self.logger["combined_error_yaw"].append(combined_error_yaw)
        if self.tcp_data.pose is not None:
            self.logger["tcp_error_x"].append(tcp_error_x)
            self.logger["tcp_error_y"].append(tcp_error_y)
            self.logger["tcp_error_z"].append(tcp_error_z)
        else:
            self.logger["tcp_error_x"].append(0.0)
            self.logger["tcp_error_y"].append(0.0)
            self.logger["tcp_error_z"].append(0.0)
        self.logger["correction_angular_x"].append(command.angular.x)
        self.logger["correction_angular_y"].append(command.angular.y)
        self.logger["correction_angular_z"].append(command.angular.z)
        self.logger["correction_linear_x"].append(command.linear.x)
        self.logger["correction_linear_y"].append(command.linear.y)
        self.logger["correction_linear_z"].append(command.linear.z)

    def tcp_callback(self, msg):
        # (TCP) pose position and orientation.
        pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        angles = list(tf.transformations.euler_from_quaternion(quaternion))
        self.tcp_data.update(pose, angles)
        if self.tcp_data.reference_pose is None:
            rospy.loginfo("Reference TCP pose set: x=%.3f, y=%.3f, z=%.3f", 
                          self.tcp_data.pose[0], self.tcp_data.pose[1], self.tcp_data.pose[2])
            rospy.loginfo("Reference TCP orientation set: roll=%.3f, pitch=%.3f, yaw=%.3f",
                          self.tcp_data.angles[0], self.tcp_data.angles[1], self.tcp_data.angles[2])

    def plot_and_save_log(self):
        # ombined orientation errors over time.
        plt.figure(figsize=(10, 6))
        plt.plot(self.logger["time"], self.logger["combined_error_roll"], label="Combined Roll Error")
        plt.plot(self.logger["time"], self.logger["combined_error_pitch"], label="Combined Pitch Error")
        plt.plot(self.logger["time"], self.logger["combined_error_yaw"], label="Combined Yaw Error")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (radians)")
        plt.title("Combined Orientation Errors Over Time (Left Arm)")
        plt.legend()
        plt.grid(True)
        plt.savefig("combined_orientation_errors_left.png")
        plt.show()
        
        # angular correction commands over time.
        plt.figure(figsize=(10, 6))
        plt.plot(self.logger["time"], self.logger["correction_angular_x"], label="Angular Correction X")
        plt.plot(self.logger["time"], self.logger["correction_angular_y"], label="Angular Correction Y")
        plt.plot(self.logger["time"], self.logger["correction_angular_z"], label="Angular Correction Z")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Correction (rad/s)")
        plt.title("Angular Correction Commands Over Time (Left Arm)")
        plt.legend()
        plt.grid(True)
        plt.savefig("angular_corrections_left.png")
        plt.show()
        
        # TCP (nozzle) pose errors over time.
        plt.figure(figsize=(10, 6))
        plt.plot(self.logger["time"], self.logger["tcp_error_x"], label="TCP Error X")
        plt.plot(self.logger["time"], self.logger["tcp_error_y"], label="TCP Error Y")
        plt.plot(self.logger["time"], self.logger["tcp_error_z"], label="TCP Error Z")
        plt.xlabel("Time (s)")
        plt.ylabel("TCP Error (m)")
        plt.title("TCP Pose Errors Over Time (Left Arm)")
        plt.legend()
        plt.grid(True)
        plt.savefig("tcp_errors_left.png")
        plt.show()
        
        # linear correction commands over time.
        plt.figure(figsize=(10, 6))
        plt.plot(self.logger["time"], self.logger["correction_linear_x"], label="Linear Correction X")
        plt.plot(self.logger["time"], self.logger["correction_linear_y"], label="Linear Correction Y")
        plt.plot(self.logger["time"], self.logger["correction_linear_z"], label="Linear Correction Z")
        plt.xlabel("Time (s)")
        plt.ylabel("Linear Correction (m/s)")
        plt.title("Linear Correction Commands Over Time (Left Arm)")
        plt.legend()
        plt.grid(True)
        plt.savefig("linear_corrections_left.png")
        plt.show()
        
        # log data to CSV for further analysis.
        csv_filename = "nozzle_stabilizer_left_log.csv"
        with open(csv_filename, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["time", "imu_roll", "imu_pitch", "imu_yaw",
                             "tcp_roll", "tcp_pitch", "tcp_yaw",
                             "combined_error_roll", "combined_error_pitch", "combined_error_yaw",
                             "tcp_error_x", "tcp_error_y", "tcp_error_z",
                             "correction_angular_x", "correction_angular_y", "correction_angular_z",
                             "correction_linear_x", "correction_linear_y", "correction_linear_z"])
            for i in range(len(self.logger["time"])):
                writer.writerow([
                    self.logger["time"][i],
                    self.logger["imu_roll"][i],
                    self.logger["imu_pitch"][i],
                    self.logger["imu_yaw"][i],
                    self.logger["tcp_roll"][i],
                    self.logger["tcp_pitch"][i],
                    self.logger["tcp_yaw"][i],
                    self.logger["combined_error_roll"][i],
                    self.logger["combined_error_pitch"][i],
                    self.logger["combined_error_yaw"][i],
                    self.logger["tcp_error_x"][i],
                    self.logger["tcp_error_y"][i],
                    self.logger["tcp_error_z"][i],
                    self.logger["correction_angular_x"][i],
                    self.logger["correction_angular_y"][i],
                    self.logger["correction_angular_z"][i],
                    self.logger["correction_linear_x"][i],
                    self.logger["correction_linear_y"][i],
                    self.logger["correction_linear_z"][i]
                ])
        rospy.loginfo("Log data saved to %s", csv_filename)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    stabilizer = NozzleStabilizerLeft()
    stabilizer.tcp_data = TCPData()
    stabilizer.run()
