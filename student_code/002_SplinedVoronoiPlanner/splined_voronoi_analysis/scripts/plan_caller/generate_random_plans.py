#!/usr/bin/env python3
from typing import List, Tuple

import cv2
import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float64, Float64MultiArray, Int64


def mapToWorld(
    point_map: Tuple[float, float], map_origin: Tuple[float, float], resolution: float
):
    """
    converts point from map coordinates (pixel) to world coordinates (meters).

    Args:
        point_map: 2D point in map coordinates.
        map_origin: 2D origin of map as specified in costmap.
        resolution: resolution of map in meter per pixel (typically 0.05).

    Returns:
        2D point in world coordinates.
    """
    half_a_cell = 0.5
    point_world_x = map_origin[0] + (point_map[0] + half_a_cell) * resolution
    point_world_y = map_origin[1] + (point_map[1] + half_a_cell) * resolution
    return (point_world_x, point_world_y)


def write_plans_to_rosbag(
    start_poses: List[PoseStamped],
    goal_poses: List[PoseStamped],
    max_curvature: float,
    map_data: OccupancyGrid,
):
    """Saves randomly generted start and goal poses in bagfile for future use.

    Args:
        start_poses: List of start poses
        goal_poses: List of goal poses
        max_curvature: maximum allowed curvature for which map was altered.
        map_data: map from which poses are generated.
    """
    bag = rosbag.Bag(f"random_plans.bag", "w")
    formation_max_curvature = Float64(max_curvature)
    path_start_poses = Path()
    path_start_poses.header.frame_id = "map"
    path_start_poses.poses = start_poses
    path_goal_poses = Path()
    path_goal_poses.header.frame_id = "map"
    path_goal_poses.poses = goal_poses

    try:
        bag.write("start_poses", path_start_poses)
        bag.write("goal_poses", path_goal_poses)
        bag.write("formation_max_curvature", formation_max_curvature)
        bag.write("map_data", map_data)
    finally:
        bag.close()


class RandomPlanGenerator:
    """Class for randomly generating starts and goals for testing path planning."""
    # voronoi_data: OccupancyGrid = None

    def __init__(self, formation_radius: float = 1.0) -> None:
        """Waits for map on ROS-topic and generates free points on it based on specified formation radius."""
        self.formation_radius = formation_radius

        print("ROS connection set up and running!")
        self.map_data: OccupancyGrid = rospy.wait_for_message(
            "/map", OccupancyGrid, 2.0
        )
        self.map_callback(self.map_data)
        print("Got a map")
        self.get_free_points()
        # np.random.seed(0)
        # while True:
        #     self.get_random_plan()
        # rospy.spin()

    def map_callback(self, data: OccupancyGrid):
        """callbakc for message of ROS-topic with map; creates image from map"""
        costmap_size_y = data.info.height
        costmap_size_x = data.info.width
        self.map_origin = (data.info.origin.position.x, data.info.origin.position.y)
        self.map_resolution = data.info.resolution
        map_img = np.zeros((costmap_size_x, costmap_size_y), dtype=np.uint8)
        for i in range(costmap_size_x):
            for j in range(costmap_size_y):
                occupancy_index = j * costmap_size_x + i
                if data.data[occupancy_index] == 0:
                    map_img[i, j] = 255
        # cv2.imwrite("debug_img0.png", map_img)
        distance_img = cv2.distanceTransform(
            map_img, cv2.DIST_L2, 3, dstType=cv2.CV_8UC1
        )
        _, self.free_space_img = cv2.threshold(
            distance_img,
            (self.formation_radius + 0.6) / self.map_resolution,
            255,
            cv2.THRESH_BINARY,
        )
        # cv2.imwrite("debug_img1.png", self.free_space_img)
        # pdb.set_trace()

    def get_free_points(self):
        """get all free points on map."""
        scale_percent = 10
        width = int(self.free_space_img.shape[1] * scale_percent / 100)
        height = int(self.free_space_img.shape[0] * scale_percent / 100)
        dim = (width, height)

        # resize image
        self.resized_img = cv2.resize(
            self.free_space_img, dim, interpolation=cv2.INTER_AREA
        )
        # cv2.imwrite("debug_img2.png", self.resized_img)
        self.free_points = cv2.findNonZero(self.free_space_img).squeeze()

    def get_random_plan(
        self, low_dist_thresh_m: float = 10.0, upper_dist_thresh_m: float = 75.0
    ) -> Tuple[PoseStamped, PoseStamped]:
        """get a single random plan; first slect free point randomly as start; then select goal with specified distance limitations to start."""
        random_start_idx = np.random.choice(self.free_points.shape[0])
        start = self.free_points[random_start_idx]
        distances = np.linalg.norm((self.free_points - start).T, axis=0, ord=1)
        # low_dist_thresh = (self.formation_radius + 10.0) / self.map_resolution
        # upper_dist_thresh = 75.0 / self.map_resolution
        low_dist_thresh = low_dist_thresh_m / self.map_resolution
        upper_dist_thresh = upper_dist_thresh_m / self.map_resolution
        available_goals = self.free_points[
            np.bitwise_and(distances > low_dist_thresh, distances < upper_dist_thresh)
        ]
        random_goal_idx = np.random.choice(available_goals.shape[0])
        goal = available_goals[random_goal_idx]
        print(f"Start: {start}, goal: {goal}")
        start_list = tuple(start.tolist())
        start_world = mapToWorld(
            (start_list[1], start_list[0]), self.map_origin, self.map_resolution
        )
        goal_list = tuple(goal.tolist())
        goal_world = mapToWorld(
            (goal_list[1], goal_list[0]), self.map_origin, self.map_resolution
        )
        # cv2.circle(self.free_space_img, start_list, 3, 127, -1)
        # cv2.circle(self.free_space_img, goal_list, 3, 150, -1)
        # cv2.imwrite("debug_img3.png", self.free_space_img)
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.pose.orientation.w = 1.0
        start_pose.pose.position.x = start_world[0]
        start_pose.pose.position.y = start_world[1]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.position.x = goal_world[0]
        goal_pose.pose.position.y = goal_world[1]
        return (start_pose, goal_pose)


def generate_multiple_plans():
    """generates multiple random plans and saves them to a bagfile."""
    rospy.init_node("random_plan_generator")
    formation_radius = 1.0
    random_plan_generator = RandomPlanGenerator(formation_radius=formation_radius)
    num_iterations = 100
    start_poses = []
    goal_poses = []
    for idx in range(num_iterations):
        print("#############")
        print(f"Plan {idx+1}/{num_iterations}:")
        start_pose, goal_pose = random_plan_generator.get_random_plan()
        print(
            f"Start Position: {start_pose.pose.position.x}, {start_pose.pose.position.y}"
        )
        print(
            f"Goal Position: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}"
        )
        start_poses.append(start_pose)
        goal_poses.append(goal_pose)

    write_plans_to_rosbag(
        start_poses, goal_poses, formation_radius, random_plan_generator.map_data
    )


class PlanningBagContents:
    """Class for reading out contents of a bagfile which contains results of path planning."""
    def __init__(
        self, filename: str = "plan_00000.bag", skip_maps: bool = False
    ) -> None:
        self.skip_maps = skip_maps
        self.bag_contents = {}
        self.read_rosbag(filename)

        self.start_poses: Path = self.bag_contents["start_poses"]
        self.goal_poses: Path = self.bag_contents["goal_poses"]
        self.formation_max_curvature: float = self.bag_contents[
            "formation_max_curvature"
        ].data
        self.map_data: OccupancyGrid = self.bag_contents["map_data"]

    def read_rosbag(self, filename: str):
        bag = rosbag.Bag(filename)
        for topic, msg, t in bag.read_messages(topics=[]):
            # assuming there is only one message per topic!
            self.bag_contents[topic] = msg
        bag.close()


def main():
    generate_multiple_plans()


if __name__ == "__main__":
    main()
