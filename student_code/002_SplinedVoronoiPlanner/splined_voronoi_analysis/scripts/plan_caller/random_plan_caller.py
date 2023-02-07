#!/usr/bin/env python3
import json
import time
from typing import List, Tuple

import cv2
import dynamic_reconfigure.client
import numpy as np
import rosbag
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from single_path_planner import (
    VoronoiListener,
    call_path_planner_with_stats,
    write_rosbag_with_stats,
)
from std_msgs.msg import Float64, Float64MultiArray, Int64


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


def random_plan_caller():
    """calls path planner with random starts and goals from bagfile.

    Can be used for different algorithms by changing the algorithm variable. And changing the algorithm in the path planner optimization.
    """
    rospy.init_node("random_plan_caller")
    algorithms = ["BOBYQA", "COBYLA", "Nelder-Mead", "NEWUOA", "PRAXIS", "SBPLX"]
    algorithm = algorithms[5]
    # algorithm = "NEWUOA_new"
    # algorithm = f"testarena2/{algorithm}"
    filename = "random_plans.bag"
    bag_contents = PlanningBagContents(filename)
    voronoi_listener = VoronoiListener("SplinedVoronoiPlanner")

    failed_idx = []
    invalid_idx = []
    success_idx = []
    for idx, (start_pose, goal_pose) in enumerate(
        zip(bag_contents.start_poses.poses, bag_contents.goal_poses.poses)
    ):
        print("#############")
        print(f"Plan {idx}:")
        start_time = time.time()
        print(
            f"Start Position: {start_pose.pose.position.x}, {start_pose.pose.position.y}"
        )
        print(
            f"Goal Position: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}"
        )
        voronoi_listener.reset_voronoi_received()
        service_response = call_path_planner_with_stats(
            start_pose, goal_pose, bag_contents.formation_max_curvature
        )
        end_time = time.time()
        print(
            f"Got Path with length {len(service_response.path)} in {end_time-start_time} s"
        )
        print("wait for voronoi map")
        if service_response.plan_found >= 0:
            while not voronoi_listener.voronoi_received:
                time.sleep(0.1)
            print("Voronoi received")
        write_rosbag_with_stats(
            idx,
            algorithm,
            service_response,
            start_pose,
            goal_pose,
            bag_contents.formation_max_curvature,
            bag_contents.map_data,
            voronoi_listener.voronoi_data,
        )
        if service_response.plan_found >= 0:
            success_idx.append(idx)
        elif service_response.plan_found == -1 or service_response.plan_found < -6:
            failed_idx.append(idx)
        elif service_response.plan_found < -1:
            invalid_idx.append(idx)

    with open(f"{algorithm}/random_plans_overview.json", "w") as json_file:
        json.dump(
            {"success": success_idx, "failed": failed_idx, "invalid": invalid_idx},
            json_file,
        )


def compare_voronoi_planning():
    """Feeds plans from bagfile in PureVoronoiPlanner and compares result between different used algorithms.

    Can be used for dijkstra vs astar analysis.
    Can be used for freespace inclusion analysis
    """
    rospy.init_node("random_plan_generator")

    client = dynamic_reconfigure.client.Client(
        "move_base_flex/PureVoronoiPlanner",
        timeout=5,
    )

    filename = "random_plans.bag"
    bag_contents = PlanningBagContents(filename)
    voronoi_listener = VoronoiListener("PureVoronoiPlanner")

    for idx, (start_pose, goal_pose) in enumerate(
        zip(bag_contents.start_poses.poses, bag_contents.goal_poses.poses)
    ):
        print("#############")
        print(f"Plan {idx}:")
        print(
            f"Start Position: {start_pose.pose.position.x}, {start_pose.pose.position.y}"
        )
        print(
            f"Goal Position: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}"
        )
        voronoi_listener.reset_voronoi_received()
        # activate astar
        client.update_configuration(
            {
                "use_dijkstra": False,
                "min_radius": 1.0,
                "free_space_factor": 4.0,
                "large_free_spaces": True,
            }
        )
        start_time_astar = time.time()
        service_response_astar = call_path_planner_with_stats(
            start_pose, goal_pose, bag_contents.formation_max_curvature
        )
        end_time_astar = time.time()
        print(
            f"Got A*-Path with length {len(service_response_astar.path)} in {end_time_astar-start_time_astar} s"
        )
        # activate dijkstra
        client.update_configuration(
            {
                "use_dijkstra": True,
                "min_radius": 1.0,
                "free_space_factor": 4.0,
                "large_free_spaces": True,
            }
        )
        start_time_dijkstra = time.time()
        service_response_dijkstra = call_path_planner_with_stats(
            start_pose, goal_pose, bag_contents.formation_max_curvature
        )
        end_time_dijkstra = time.time()
        print(
            f"Got Dijkstra-Path with length {len(service_response_dijkstra.path)} in {end_time_dijkstra-start_time_dijkstra} s"
        )
        print("wait for voronoi map")
        while not voronoi_listener.voronoi_received:
            time.sleep(0.1)
        print("Voronoi received")
        write_rosbag_with_stats(
            idx,
            "dijkstra",
            service_response_dijkstra,
            start_pose,
            goal_pose,
            bag_contents.formation_max_curvature,
            bag_contents.map_data,
            voronoi_listener.voronoi_data,
        )
        write_rosbag_with_stats(
            idx,
            "astar",
            service_response_astar,
            start_pose,
            goal_pose,
            bag_contents.formation_max_curvature,
            bag_contents.map_data,
            voronoi_listener.voronoi_data,
        )


def main():
    random_plan_caller()


if __name__ == "__main__":
    main()
