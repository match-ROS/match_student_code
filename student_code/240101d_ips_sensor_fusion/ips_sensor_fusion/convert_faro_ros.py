#! /usr/bin/env python3
"""convert all faro csv files in the faro folder to bag files using specified time offset

   this code expects faro data to be in the following csv format:
   x,y,z,timestamp with points as decimal point and space as delimiter.
   the faro tracker provides a comma as decimal point and a comma + space as delimiter. This can be corrected using search
   and replace in a text editor: first replace ", " (comma+space) with " " (space) and afterwards replace "," with "."

   time_offset is measured manually using plotjuggler as the difference between the faro data and the recorded ros localization data
   start faro_trasnform_launch.launch before running this script seperately"""

import rospy
import csv
import tf2_ros
import tf2_geometry_msgs
import rospkg
import rosbag
from geometry_msgs.msg import PoseStamped
from pathlib import Path
from os import listdir

def faro_to_bag(package_path, filename, time_offset):
    """converts a faro csv file to a bag file while performing transformation
    
        subtracts given time_offset from timestamp for synchronization with recorded data"""

    filename_noext = Path(filename).stem # get filename without extension
    
    #input path
    full_path = package_path + "/data/raw_faro_data/" + filename
    print('writing faro positions to bag from:', full_path,"...", end='')

    #target path and filename
    bag_path = rospack.get_path("ips_sensor_fusion") + '/data/transformed_faro_data/'
    bag_name = filename_noext + ".bag"
    
    #setup transform listener
    tfBuffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    
    #create bag file and open csv
    with rosbag.Bag(bag_path + bag_name, 'w') as bag:
        with open(full_path, 'r') as csvfile:
            datareader = csv.reader(csvfile, delimiter=' ',quoting=csv.QUOTE_NONNUMERIC)

            sequential_counter = 0
            
            #for every faro datapoint transform it to map frame and write to bag
            for row in datareader:
                if rospy.is_shutdown() == True:
                    break
                timestamp = rospy.Time.from_sec(row[3] - time_offset)
                print("t:",timestamp, "csv_t:",row[3])
                faro_pose = PoseStamped()

                #header
                faro_pose.header.frame_id = 'faro_frame'
                faro_pose.header.seq = sequential_counter
                faro_pose.header.stamp = timestamp

                #pose
                faro_pose.pose.position.x = row[0]
                faro_pose.pose.position.y = row[1]
                faro_pose.pose.position.z = row[2]
                
                #get transform
                tf_faro_map = tfBuffer.lookup_transform("map",
                                    'faro_frame', #source frame
                                    rospy.Time(0), #get the tf at first available time
                                    rospy.Duration(1.0))
                
                #apply transform and write to bag
                faro_pose_transformed = tf2_geometry_msgs.do_transform_pose(faro_pose, tf_faro_map)
                bag.write('/faro_pos',faro_pose_transformed, timestamp)
                sequential_counter += 1
            print('  ...done')
    return

if __name__=='__main__':
    rospy.init_node("faro_csv_to_bag")
    rospack = rospkg.RosPack()
    package_path  = rospack.get_path("ips_sensor_fusion")

    filelist = listdir(package_path + "/data/raw_faro_data/") #get all files in faro folder

    faro_time_offset = 10687.337 #manually measured time offset using plotjuggler

    for filename in filelist:
        if filename.endswith(".csv"):
            faro_to_bag(package_path, filename, faro_time_offset)