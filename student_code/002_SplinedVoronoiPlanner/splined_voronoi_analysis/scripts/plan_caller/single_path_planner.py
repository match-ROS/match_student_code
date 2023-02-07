#!/usr/bin/env python3
import glob
import pdb
import time
from typing import List, Tuple

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from navfn.srv import MakeNavPlan, MakeNavPlanRequest, MakeNavPlanResponse
from splined_voronoi.srv import (
    MakePlanWithStats,
    MakePlanWithStatsRequest,
    MakePlanWithStatsResponse,
)
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int64


def pose_from_tuple(pose_tuple: Tuple[float, float, float]) -> PoseStamped:
    """transforms Pose from Tuple of floats (x,y,angle) to ROS-Message type."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = pose_tuple[0]
    pose.pose.position.y = pose_tuple[1]
    pose.pose.orientation.w = np.cos(pose_tuple[2] / 2.0)
    pose.pose.orientation.z = np.sin(pose_tuple[2] / 2.0)
    return pose


def write_rosbag_with_stats(
    index: int,
    algorithm: str,
    service_response: MakePlanWithStatsResponse,
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    max_curvature: float,
    map_data: OccupancyGrid,
    voronoi_data: OccupancyGrid,
):
    """Writes rosbag with result from path planning via MakePlanWithStats interface."""
    bag = rosbag.Bag(f"{algorithm}/plan_{str(index).zfill(5)}.bag", "w")
    planning_status_code = Int64(service_response.plan_found)
    total_time = Float64(service_response.time_taken)
    formation_max_curvature = Float64(max_curvature)
    path_on_voronoi = Path()
    path_on_voronoi.header.frame_id = "map"
    path_on_voronoi.poses = service_response.astar_path
    sparse_path = Path()
    sparse_path.header.frame_id = "map"
    sparse_path.poses = service_response.sparse_path
    optimized_sparse_path = Path()
    optimized_sparse_path.header.frame_id = "map"
    optimized_sparse_path.poses = service_response.optimized_sparse_path
    path = Path()
    path.header.frame_id = "map"
    path.poses = service_response.path
    spline_tangent_lengths = Float64MultiArray(
        data=service_response.spline_tangent_lengths
    )
    try:
        bag.write("start_pose", start_pose)
        bag.write("goal_pose", goal_pose)
        bag.write("formation_max_curvature", formation_max_curvature)
        bag.write("planning_status_code", planning_status_code)
        bag.write("total_time_taken_for_planning", total_time)
        bag.write("path_on_voronoi", path_on_voronoi)
        bag.write("sparse_path", sparse_path)
        bag.write("optimized_sparse_path", optimized_sparse_path)
        bag.write("spline_tangent_lengths", spline_tangent_lengths)
        bag.write("path", path)
        bag.write("map_data", map_data)
        bag.write("voronoi_data", voronoi_data)
    finally:
        bag.close()


def write_rosbag(
    index: int,
    suffix: str,
    service_response: MakeNavPlanResponse,
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    time_taken: float,
    max_curvature: float,
    map_data: OccupancyGrid,
):
    """Writes rosbag with result from path planning via MakePlan interface."""
    bag = rosbag.Bag(f"plan_{str(index).zfill(5)}_{suffix}.bag", "w")
    total_time = Float64(time_taken)
    success = Bool(service_response.plan_found)
    formation_max_curvature = Float64(max_curvature)
    path = Path()
    path.header.frame_id = "map"
    path.poses = service_response.path
    try:
        bag.write("start_pose", start_pose)
        bag.write("goal_pose", goal_pose)
        bag.write("formation_max_curvature", formation_max_curvature)
        bag.write("time_taken", total_time)
        bag.write("path", path)
        bag.write("success", success)
        bag.write("map_data", map_data)
    finally:
        bag.close()


def call_path_planner_with_stats(
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    max_curvature: float,
    planner_name: str = "SplinedVoronoiPlanner",
) -> MakePlanWithStatsResponse:
    """Call make_plan_with_stats service of global planner."""
    service_name = f"/move_base_flex/{planner_name}/make_plan_with_stats"
    try:
        rospy.wait_for_service(service_name, 2)
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
    try:
        make_plan_with_stats = rospy.ServiceProxy(
            service_name,
            MakePlanWithStats,
        )
        req = MakePlanWithStatsRequest()
        req.goal = goal_pose
        req.start = start_pose
        req.max_curvature = max_curvature
        resp: MakePlanWithStatsResponse = make_plan_with_stats(req)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return MakePlanWithStatsResponse()
    print(f"{resp.plan_found=}")
    print(f"{resp.time_taken=}")
    if resp.plan_found < 0:
        rospy.logerr("No valid path found")
    return resp


def call_path_planner(
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    planner_name: str = "SplinedVoronoiPlanner",
    namespace: str = "",
) -> MakeNavPlanResponse:
    """Call make_plan service of global planner."""
    if len(namespace) > 0:
        namespace = f"/{namespace}"
    service_name = f"{namespace}/move_base_flex/{planner_name}/make_plan"
    try:
        rospy.wait_for_service(service_name, 2)
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
    print("Service available")
    try:
        make_plan = rospy.ServiceProxy(service_name, MakeNavPlan)
        req = MakeNavPlanRequest()
        req.goal = goal_pose
        req.start = start_pose
        print("build request")
        resp: MakeNavPlanResponse = make_plan(req)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return []
    print("service finished")
    if not resp.plan_found:
        rospy.logerr("No path found")
    print("just returning response")
    return resp


class VoronoiListener:
    """Class for getting voronoi map from topic and saving it."""
    voronoi_data: OccupancyGrid = None

    def __init__(self, planner_type: str = "SplinedVoronoiPlanner") -> None:
        """Initializes class with planner name; creates subscriber on voronoi topic."""
        self.voronoi_received = False

        # rospy.init_node("voronoi_listener")
        rospy.Subscriber(
            f"/move_base_flex/{planner_type}/voronoi_map",
            OccupancyGrid,
            self.voronoi_callback,
        )

    def voronoi_callback(self, data: OccupancyGrid):
        """Callback if voornoi map has been received; creates image from map."""
        print("Got voronoi map")
        self.voronoi_data = data
        voronoi_size_y = data.info.height
        voronoi_size_x = data.info.width
        self.voronoi_img = np.zeros((voronoi_size_x, voronoi_size_y), dtype=np.uint8)
        # pdb.set_trace()
        for i in range(voronoi_size_x):
            for j in range(voronoi_size_y):
                occupancy_index = j * voronoi_size_x + i
                if data.data[occupancy_index] == 0:
                    self.voronoi_img[i, j] = 255
                elif data.data[occupancy_index] == 50:
                    self.voronoi_img[i, j] = 127
                elif data.data[occupancy_index] == 100:
                    self.voronoi_img[i, j] = 0
        # cv2.imwrite("debug_img0.png", self.voronoi_img)
        self.voronoi_received = True

    def reset_voronoi_received(self):
        """reset if voronoi map has been received."""
        self.voronoi_received = False


def single_plan_caller():
    """waits for start and goal on topics, sends them to planner and saves results in bagfile."""
    rospy.init_node("single_plan_caller")
    # bag 0
    # start = [9.825, 10.575, 0]
    # goal = [-12.074, 0.225, 0]
    # bag 10
    max_curve = 1.0
    start = [5.975, 2.125, 0]
    goal = [-7.875, 4.775, 0]
    start_pose = pose_from_tuple(start)
    goal_pose = pose_from_tuple(goal)

    start_pose = rospy.wait_for_message("/start_pose", PoseStamped)
    goal_pose = rospy.wait_for_message("/goal_pose", PoseStamped)

    map_data: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid, 2.0)

    voronoi_listener = VoronoiListener("SplinedVoronoiPlanner")

    print(f"Start Pose: {start_pose.pose.position.x}, {start_pose.pose.position.y}")
    print(f"Goal Pose: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")
    start_time = time.time()
    response = call_path_planner(start_pose, goal_pose, "SplinedVoronoiPlanner")

    voronoi_listener.reset_voronoi_received()
    service_response = call_path_planner_with_stats(start_pose, goal_pose, max_curve)
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
        4,
        "single_plan",
        service_response,
        start_pose,
        goal_pose,
        max_curve,
        map_data,
        voronoi_listener.voronoi_data,
    )

    end_time = time.time()
    duration = end_time - start_time
    print(f"Got Path in {duration} s")
    if not response.plan_found:
        print("Not plan found")


def main():
    single_plan_caller()


if __name__ == "__main__":
    main()
