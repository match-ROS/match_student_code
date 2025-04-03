#! /usr/bin/env python
import rospy
import subprocess
import actionlib
import math
import os
import threading
import time
from pynput import keyboard  # Make sure you are using pynput, not keyboard
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from odf.opendocument import OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell
from odf.text import P
# Global variables for pause and cancel

# Ctrl + P Pause
# Ctrl + L Resume
# Ctrl + N Cancel
paused = False
cancelled = False
ctrl_pressed = False  # Variable to detect when 'Ctrl' is pressed

# Function to handle key presses
def on_press(key):
    global paused, cancelled, ctrl_pressed
    try:
        if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
            ctrl_pressed = True  # Detect that Ctrl is pressed

        if ctrl_pressed:
            if key.char == 'p':  # Ctrl+P
                paused = True
                rospy.loginfo("Script paused")
            elif key.char == 'l':  # Ctrl+L
                paused = False
                rospy.loginfo("Script resumed")
            elif key.char == 'n':  # Ctrl+N
                cancelled = True
                rospy.loginfo("Script cancelled")
                return False  # Exit the listener
    except AttributeError:
        pass

# Function to handle key releases
def on_release(key):
    global ctrl_pressed
    if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        ctrl_pressed = False  # Detect that Ctrl has been released

# Thread to monitor the keyboard
def monitor_keyboard():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

def callback_ground_truth(pose_ground_truth):
     global ground_truth
     ground_truth = pose_ground_truth

def callback_amcl(pose_amcl):
     global amcl
     amcl = pose_amcl

if __name__ == "__main__":
    rosbag_directory = "/home/rafass/Documents/Bachelorarbeit/Posiciones/mur620/Rosbag"
    rospy.init_node("collect_data", anonymous=True)
    rospy.sleep(0.1)
    client = actionlib.SimpleActionClient('/mur620/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()
    goal_client = MoveBaseGoal()

    rospy.sleep(1.0)

    listener1 = rospy.Subscriber("/mur620/amcl_pose", PoseWithCovarianceStamped, callback_amcl)
    listener2 = rospy.Subscriber("/mur620/ground_truth", Odometry, callback_ground_truth)

    # Start the thread to monitor keys
    keyboard_thread = threading.Thread(target=monitor_keyboard)
    keyboard_thread.daemon = True
    keyboard_thread.start()

    # Desired positions
    config = [
        [5, -3, 0],
        [-6, -5, 20],
        [2, -8, 40],
        [-4, 3, 60],
        [1, -9, 80]
    ]

    doc = OpenDocumentSpreadsheet()
    table = Table(name="Table1")
    
    columns = ["Cycle", "Configuration number", "Amcl Pose x", "Amcl Pose y", "Amcl Pose Orientation",
               "Ground Truth x", "Ground Truth y", "Ground Truth Orientation", "Secs Rosbag"]
    header_row = TableRow()
    for column in columns:
        cell = TableCell()
        cell.addElement(P(text=str(column)))
        header_row.addElement(cell)
    table.addElement(header_row)

    goal = PoseStamped()
    for j in range(1, 31):
        # Check if the script is cancelled
        if cancelled:
            rospy.loginfo("Cancelling execution and saving data...")
            break

        rosbag_filename = os.path.join(rosbag_directory, f"Ciclo_{j}.bag")
        rosbag_process = subprocess.Popen(["rosbag", "record", "-O", rosbag_filename, "/mur620/amcl_pose", "/mur620/ground_truth"])
        rospy.loginfo(f"Started recording rosbag: {rosbag_filename}")
        start_time = rospy.Time.now()

        for i in range(5):
            # Check if the script is cancelled
            if cancelled:
                rospy.loginfo("Cancelling execution and saving data...")
                break

            # Check if the script is paused
            while paused:
                rospy.loginfo("Paused, waiting to resume...")
                time.sleep(1)

            angle_degrees = config[i][2]
            angle_radians = math.radians(angle_degrees)
            q = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
            rospy.sleep(0.1)

            goal_client.target_pose.header.frame_id = 'map'
            goal_client.target_pose.pose.orientation.x = q[0]
            goal_client.target_pose.pose.orientation.y = q[1]
            goal_client.target_pose.pose.orientation.z = q[2]
            goal_client.target_pose.pose.orientation.w = q[3]
            goal_client.target_pose.pose.position.x = config[i][0]
            goal_client.target_pose.pose.position.y = config[i][1]
            rospy.sleep(0.1)

            # Retry attempts if aborted
            attempt_count = 0
            max_attempts = 5

            while attempt_count < max_attempts:
                client.send_goal(goal_client)
                rospy.loginfo(f"Cycle {j}, Configuration {i+1}, Attempt {attempt_count + 1}, going to point")
                state = client.wait_for_result()

                if state:
                    result_state = client.get_state()

                    if result_state == 4:  # Aborted
                        rospy.loginfo(f"Cycle {j} Configuration {i+1} aborted, retrying after 2 seconds")

                        # Rotate in current position 90 degrees
                        angle_radians = math.radians(90)  
                        q = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
                        rotate_goal = MoveBaseGoal()
                        rotate_goal.target_pose.header.frame_id = 'map'
                        rotate_goal.target_pose.pose.orientation.x = q[0]
                        rotate_goal.target_pose.pose.orientation.y = q[1]
                        rotate_goal.target_pose.pose.orientation.z = q[2]
                        rotate_goal.target_pose.pose.orientation.w = q[3]
                        rotate_goal.target_pose.pose.position.x = amcl.pose.pose.position.x  # Current robot position
                        rotate_goal.target_pose.pose.position.y = amcl.pose.pose.position.y  # Current robot position

                        # Send goal to rotate
                        client.send_goal(rotate_goal)
                        client.wait_for_result() 
                        rospy.loginfo("Rotation complete, retrying the original goal")

                        attempt_count += 1
                        client.cancel_goal() 
                        rospy.sleep(2)  

                    elif result_state == 3:  # Point reached
                        rospy.loginfo("Point reached")
                        current_time = rospy.Time.now()
                        secs = (current_time - start_time).to_sec()
                        rospy.sleep(1)

                        # Transform quaternion to Euler
                        amcl_orientation = tf.transformations.euler_from_quaternion([
                            amcl.pose.pose.orientation.x,
                            amcl.pose.pose.orientation.y,
                            amcl.pose.pose.orientation.z,
                            amcl.pose.pose.orientation.w
                        ])
                        ground_truth_orientation = tf.transformations.euler_from_quaternion([
                            ground_truth.pose.pose.orientation.x,
                            ground_truth.pose.pose.orientation.y,
                            ground_truth.pose.pose.orientation.z,
                            ground_truth.pose.pose.orientation.w
                        ])

                        row = TableRow()

                        cell = TableCell()
                        cell.addElement(P(text=str(j)))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(i+1)))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(amcl.pose.pose.position.x)))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(amcl.pose.pose.position.y)))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(amcl_orientation[2])))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(ground_truth.pose.pose.position.x)))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(ground_truth.pose.pose.position.y)))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(ground_truth_orientation[2])))
                        row.addElement(cell)

                        cell = TableCell()
                        cell.addElement(P(text=str(secs)))
                        row.addElement(cell)

                        table.addElement(row)
                        break  # Exit the while loop if point is reached
                    else:
                        rospy.loginfo(f"Unknown state: {result_state}")
                        break

            if attempt_count >= max_attempts:
                rospy.loginfo(f"Failed to reach point after {max_attempts} attempts")
                break  

        rosbag_process.terminate()
        rosbag_process.wait()  # Wait for the process to close completely
        rospy.loginfo(f"Stopped recording rosbag: {rosbag_filename}")

        if rosbag_process.returncode is not None and rosbag_process.returncode != 0:
            rospy.logwarn(f"Error while recording rosbag: {rosbag_filename}")

    # Save collected information even if the script is cancelled
    doc.spreadsheet.addElement(table)
    doc.save("positions.ods")
    rospy
