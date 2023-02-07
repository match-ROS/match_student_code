#!/usr/bin/env python3
import glob
import pdb
import time
from typing import List, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from single_path_planner import (
    call_path_planner,
    call_path_planner_with_stats,
    write_rosbag,
    write_rosbag_with_stats,
)


def pose_from_str(
    pose_str: str = "[5.97142, 1.02662], -1.76848",
) -> Tuple[float, float, float]:
    """Read Pose from string of specific composition."""
    pos_x_str, pos_y_str, yaw_angle_str = pose_str.split(", ")
    yaw_angle = float(yaw_angle_str)
    pos_x = float(pos_x_str[1:])
    pos_y = float(pos_y_str[:-1])
    return (pos_x, pos_y, yaw_angle)


def pose_from_tuple(pose_tuple: Tuple[float, float, float]) -> PoseStamped:
    """transforms Pose from Tuple of floats (x,y,angle) to ROS-Message type."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = pose_tuple[0]
    pose.pose.position.y = pose_tuple[1]
    pose.pose.orientation.w = np.cos(pose_tuple[2] / 2.0)
    pose.pose.orientation.z = np.sin(pose_tuple[2] / 2.0)
    return pose


def get_poses_from_line(
    line: str = "[5.97142, 1.02662], -1.76848 to [7.91423, 6.09365], 3.16417",
) -> Tuple[PoseStamped, PoseStamped]:
    """Read start and goal pose from line of file."""
    # line.startswith("Planning from ")
    pose_strs = line.split(" to ")
    start = pose_from_str(pose_strs[0])
    goal = pose_from_str(pose_strs[1])
    start_pose = pose_from_tuple(start)
    goal_pose = pose_from_tuple(goal)
    return (start_pose, goal_pose)


def get_plans_from_file(filename: str) -> List[Tuple[PoseStamped, PoseStamped]]:
    """Iterate file and read plans from each line."""
    res_plans = []
    line = "- Planning from [5.97142, 1.02662], -1.76848 to [7.91423, 6.09365], 3.16417"
    lines = []
    with open(filename, "r", encoding="utf-8") as inp_file:
        lines = inp_file.readlines()
    for line in lines:
        if line.startswith("- Planning from "):
            poses = get_poses_from_line(line[len("- Planning from ") :])
            res_plans.append(poses)
    return res_plans


def specific_plan_caller(plans: List[Tuple[PoseStamped, PoseStamped]]):
    """calls splined_voronoi and splined_relaxed_astar for a list of plans

    Args:
        plans (List[Tuple[PoseStamped, PoseStamped]]): list of path starts and goals to plan
    """

    formation_radius = 0.4
    rospy.init_node("specific_plan_caller")
    map_data: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid, 2.0)
    num_iterations = len(plans)
    for idx, (start_pose, goal_pose) in enumerate(plans[:], start=0):
        print("#############")
        print(f"Plan {idx + 1}/{num_iterations}:")
        print(
            f"Start Position: {start_pose.pose.position.x}, {start_pose.pose.position.y}"
        )
        print(
            f"Goal Position: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}"
        )
        start_time = time.time()
        voronoi_response = call_path_planner_with_stats(
            start_pose, goal_pose, 1.0 / formation_radius
        )
        end_time = time.time()
        print(f"Got Voronoi Path in {end_time-start_time} s")
        start_time = time.time()
        sra_response = call_path_planner(
            start_pose, goal_pose, "SplinedRelaxedAStar", "splined_planner"
        )
        end_time = time.time()
        sra_duration = end_time - start_time
        print(f"Got SplinedRelaxedAStar Path in {end_time-start_time} s")
        write_rosbag_with_stats(
            idx,
            "voronoi",
            voronoi_response,
            start_pose,
            goal_pose,
            1.0 / formation_radius,
            map_data,
            map_data,
        )
        write_rosbag(
            idx,
            "sra",
            sra_response,
            start_pose,
            goal_pose,
            sra_duration,
            formation_radius,
            map_data,
        )


def main():
    plans = get_plans_from_file("List.md")
    specific_plan_caller(plans)


if __name__ == "__main__":
    main()
