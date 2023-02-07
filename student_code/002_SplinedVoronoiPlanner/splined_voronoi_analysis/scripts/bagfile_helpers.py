import rosbag


def get_contents_from_bagfile(filename: str):
    """Reads all contents from specified bagfile."""
    bag = rosbag.Bag(filename)
    bag_contents = {}
    for topic, msg, t in bag.read_messages(topics=[]):
        # assuming there is only one message per topic!
        bag_contents[topic] = msg
    bag.close()
    return bag_contents


class PlanningBagContents:
    """Class to wrap contents of a bagfile containing results of a path planning and smoothing."""
    def __init__(
        self, filename: str = "plan_00000.bag", skip_maps: bool = False
    ) -> None:
        """Intializes class by reading bagfile.

        Args:
            filename: name of bagfile as string
            skip_maps: if maps should be skipped in reading; can improve time taken but reduces available information.
        """
        self.skip_maps = skip_maps
        self.bag_contents = {}
        self.read_rosbag(filename)

        self.start_pose = self.bag_contents["start_pose"]
        self.goal_pose = self.bag_contents["goal_pose"]
        self.formation_max_curvature: float = self.bag_contents[
            "formation_max_curvature"
        ].data
        self.path = self.bag_contents["path"]
        if not self.skip_maps:
            self.map_data = self.bag_contents["map_data"]

    def read_rosbag(self, filename: str):
        """Reads data from specified bagfile."""
        bag = rosbag.Bag(filename)
        for topic, msg, t in bag.read_messages(
            topics=[], connection_filter=self.filter_map_topics
        ):
            # assuming there is only one message per topic!
            self.bag_contents[topic] = msg
        bag.close()

    def filter_map_topics(self, topic, datatype, md5sum, msg_def, header):
        """Filters out topics in bagfile with map type if skip_maps is specified."""
        # print(datatype)
        if self.skip_maps and datatype == "nav_msgs/OccupancyGrid":
            # print("skipping topic with occupancy grid")
            return False
        return True


class MakeNavPlanBagContents(PlanningBagContents):
    """Class for wrapping bagfile contents from planning with service of type MakeNavPlan."""
    def __init__(
        self, filename: str = "plan_00000.bag", skip_maps: bool = False
    ) -> None:
        super().__init__(filename, skip_maps)
        self.planning_success: int = self.bag_contents["success"].data
        self.time_taken: float = self.bag_contents["time_taken"].data


class MakeNavPlanWithStatsBagContents(PlanningBagContents):
    """Class for wrapping bagfile contents from planning with service of type MakeNavPlanWithStats."""
    def __init__(
        self, filename: str = "plan_00000.bag", skip_maps: bool = False
    ) -> None:
        super().__init__(filename, skip_maps)
        self.planning_status_code: int = self.bag_contents["planning_status_code"].data
        self.planning_success = self.planning_status_code >= 0
        self.total_time: float = self.bag_contents["total_time_taken_for_planning"].data
        self.time_taken = self.total_time
        self.path_on_voronoi = self.bag_contents["path_on_voronoi"]
        self.sparse_path = self.bag_contents["sparse_path"]
        self.optimized_sparse_path = self.bag_contents["optimized_sparse_path"]
        self.spline_tangent_lengths = self.bag_contents["spline_tangent_lengths"].data
        if not self.skip_maps:
            self.voronoi_data = self.bag_contents["voronoi_data"]
