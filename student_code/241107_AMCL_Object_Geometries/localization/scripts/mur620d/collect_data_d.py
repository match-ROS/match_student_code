#! /usr/bin/env python3
import rospy
import subprocess
import actionlib
import math
import os
import threading
import time
from pynput import keyboard  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from odf.opendocument import OpenDocumentSpreadsheet
from odf.table import Table, TableRow, TableCell
from odf.text import P

#Ctrl + P Pausieren
#Ctrl + L Forsetzen
#Ctrl + N Abbrechen
paused = False
cancelled = False
ctrl_pressed = False 

def on_press(key):
    global paused, cancelled, ctrl_pressed
    try:
        if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
            ctrl_pressed = True  #Ctrl detected

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
                return False  
    except AttributeError:
        pass

def on_release(key):
    global ctrl_pressed
    if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        ctrl_pressed = False  ##Ctrl released


def monitor_keyboard():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()



def callback_ground_truth(pose_ground_truth):
     global ground_truth
     ground_truth= pose_ground_truth

def callback_amcl(pose_amcl):
     global amcl
     amcl=pose_amcl


if __name__=="__main__":
    rosbag_directory="/home/rosmatch/catkin_ws_amcl/src/Bachelorarbeit_match/localization/rosbags"
    rospy.init_node("collect_data", anonymous=True)
    rospy.sleep(0.1)
    client = actionlib.SimpleActionClient('mur620d/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()
    goal_client=MoveBaseGoal()

    rospy.sleep(1.0)
    listener1=rospy.Subscriber("/mur620d/robot_pose",Pose, callback_amcl)
    listener2=rospy.Subscriber("qualisys_map/mur620d/pose",PoseStamped, callback_ground_truth)
    
    keyboard_thread = threading.Thread(target=monitor_keyboard)
    keyboard_thread.daemon = True
    keyboard_thread.start()

    #Desired positions
    config=[]  
    config.append([38.0 ,28.5 , 180])
    config.append([29.7 ,28.7, 25])  
    config.append([38.0 ,32.8, 105])
    config.append([30.1 ,32.95, 300])
    config.append([33.0, 26.68, 15]) 

    doc = OpenDocumentSpreadsheet()
    table = Table(name="Table1")
    
    #Columns for table
    columns = ["Cycle","Configuration number","Amcl Pose x", "Amcl Pose y", "Amcl Pose Orientation", "Ground Truth x", "Ground Truth y", "Ground Truth Orientation",  "Secs Rosbag"]
    header_row = TableRow()
    for column in columns:
        cell = TableCell()
        cell.addElement(P(text=str(column)))
        header_row.addElement(cell)
    table.addElement(header_row)
    
    goal = PoseStamped()
    n=100  #Number of cycles
    for j in range(1,n):
          rosbag_filename = os.path.join(rosbag_directory, f"Cycle_{j}.bag")
          rosbag_process = subprocess.Popen(["rosbag", "record", "-O", rosbag_filename, "/qualisys_map/mur620d/pose", "/mur620d/robot_pose","/mur620d/odom", "/mur620d/scan"])
          rospy.loginfo(f"Started recording rosbag: {rosbag_filename}")
          start_time = rospy.Time.now()         
          
          if cancelled:
               rospy.loginfo("Cancelling execution and saving data...")
               break

          while paused:
               rospy.loginfo("Paused, waiting to resume...")
               time.sleep(5)         
         
         
          for i in range (0,5):
                         
               angle_degrees= config[i][2]
               angle_radians=math.radians(angle_degrees)
               q = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
               rospy.sleep(0.1)

               goal_client.target_pose.header.frame_id = 'map' 
               goal_client.target_pose.pose.orientation.x = q[0]
               goal_client.target_pose.pose.orientation.y = q[1]
               goal_client.target_pose.pose.orientation.z = q[2]
               goal_client.target_pose.pose.orientation.w = q[3]
               goal_client.target_pose.pose.position.x =config[i][0]
               goal_client.target_pose.pose.position.y =config[i][1]          
               rospy.sleep(0.1)
               
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
                              
                              #Rotate in current position 45 grades
                              angle_radians = math.radians(45) 
                              amcl_orientation = tf.transformations.euler_from_quaternion([
                                   amcl.orientation.x,
                                   amcl.orientation.y,
                                   amcl.orientation.z,
                                   amcl.orientation.w
                              ])
                              rotation_angle= amcl_orientation[2] + angle_radians
                              q = tf.transformations.quaternion_from_euler(0, 0, rotation_angle)
                              rotate_goal = MoveBaseGoal()
                              rotate_goal.target_pose.header.frame_id = 'map'
                              rotate_goal.target_pose.pose.orientation.x = q[0]
                              rotate_goal.target_pose.pose.orientation.y = q[1]
                              rotate_goal.target_pose.pose.orientation.z = q[2]
                              rotate_goal.target_pose.pose.orientation.w = q[3]
                              rotate_goal.target_pose.pose.position.x = amcl.position.x  # Current Position x
                              rotate_goal.target_pose.pose.position.y = amcl.position.y  # Current Position y                           
                              client.send_goal(rotate_goal)     
                              
                              #Send goal rotate
                              client.send_goal(rotate_goal)
                              client.wait_for_result() 
                              rospy.loginfo("Rotation complete, retrying the original goal")
                              
                              
                              attempt_count += 1
                              rospy.sleep(2)  # Wait 2 seconds before retrying
                              client.cancel_goal() 
                         elif result_state == 3 :  # Point reached
                              rospy.loginfo("Point reached")
                              rospy.loginfo("Waiting for ground truth to be updated")
                              rospy.wait_for_message("/qualisys_map/mur620d/pose", PoseStamped)
                              current_time = rospy.Time.now()
                              secs = (current_time - start_time).to_sec()
                              

                              amcl_orientation = tf.transformations.euler_from_quaternion([
                                   amcl.orientation.x,
                                   amcl.orientation.y,
                                   amcl.orientation.z,
                                   amcl.orientation.w
                              ])
                              ground_truth_orientation = tf.transformations.euler_from_quaternion([
                                   ground_truth.pose.orientation.x,
                                   ground_truth.pose.orientation.y,
                                   ground_truth.pose.orientation.z,
                                   ground_truth.pose.orientation.w
                              ])

                              row = TableRow()

                              cell = TableCell()
                              cell.addElement(P(text=str(j)))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(i+1)))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(amcl.position.x)))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(amcl.position.y)))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(amcl_orientation[2])))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(ground_truth.pose.position.x)))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(ground_truth.pose.position.y)))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(ground_truth_orientation[2])))
                              row.addElement(cell)

                              cell = TableCell()
                              cell.addElement(P(text=str(secs)))
                              row.addElement(cell)

                              table.addElement(row)
                              rospy.sleep(2)
                              break  # Exit the while loop if point is reached
                         else:
                              rospy.loginfo(f"Unknown state: {result_state}")
                              break

               if attempt_count >= max_attempts:
                    rospy.loginfo(f"Failed to reach point after {max_attempts} attempts")
                    break  

          rosbag_process.terminate()
          rosbag_process.wait()  
          rospy.loginfo(f"Stopped recording rosbag: {rosbag_filename}")

          if rosbag_process.returncode is not None and rosbag_process.returncode != 0:
               rospy.logwarn(f"Error while recording rosbag: {rosbag_filename}")




    doc.spreadsheet.addElement(table)
    doc.save("positions.ods")
    rospy.loginfo("Data saved successfully.")