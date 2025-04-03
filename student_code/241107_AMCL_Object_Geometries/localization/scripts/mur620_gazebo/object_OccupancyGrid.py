#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import copy

def map_callback(msg):
    resolution = msg.info.resolution
    
    # Deep copy of the OccupancyGrid message to modify it
    updated_map = copy.deepcopy(msg)

    updated_map.header.stamp = rospy.Time.now()
    updated_map.header.frame_id = msg.header.frame_id

    # Object dimensions in meters (obtained from ROS parameters)
    length_x = rospy.get_param('~width','')
    length_y = rospy.get_param('~depth')

    # Get necessary information from the OccupancyGrid
    width = updated_map.info.width
    height = updated_map.info.height
    origin_x = updated_map.info.origin.position.x
    origin_y = updated_map.info.origin.position.y

    # Coordinates of the object's center in meters (obtained from ROS parameters)
    x_center = rospy.get_param('~center_object_x')
    y_center = rospy.get_param('~center_object_y')

    # Convert object dimensions from meters to number of cells in the map
    cells_x = int(length_x / resolution)
    cells_y = int(length_y / resolution)

    # Convert the object's center coordinates to cell indices in the map
    x_index_center = int((x_center - origin_x) / resolution)
    y_index_center = int((y_center - origin_y) / resolution)

    # Calculate the range of cells for the object based on its size
    half_cells_x = int(cells_x / 2)
    half_cells_y = int(cells_y / 2)

    # Check if the coordinates are within the map boundaries
    if (x_index_center - half_cells_x >= 0 and x_index_center + half_cells_x < width and
            y_index_center - half_cells_y >= 0 and y_index_center + half_cells_y < height):
        # Convert updated_map.data to a mutable list to modify it
        updated_data_list = list(updated_map.data)

        # Iterate over the object's cells and mark them as occupied (100)
        for i in range(-half_cells_x, half_cells_x + 1):
            for j in range(-half_cells_y, half_cells_y + 1):
                index = (y_index_center + j) * width + (x_index_center + i)
                updated_data_list[index] = 100

        # Assign the modified list back to updated_map.data
        updated_map.data = tuple(updated_data_list)

        # Publish the updated OccupancyGrid on the /map topic
        map_publisher.publish(updated_map)
        rospy.loginfo("OccupancyGrid modified")
        
    else:
        rospy.logwarn("The object's coordinates are out of the map boundaries.")                     
    
    # Unsubscribe from the /map topic and stop the node
    map_subscriber.unregister()
    rospy.signal_shutdown("Map update completed")

if __name__ == '__main__':

    rospy.init_node('map_updater', anonymous=True)

    # Publisher on the same /map topic to update the map
    map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    # Subscribe to the /map topic
    map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    rospy.spin()
