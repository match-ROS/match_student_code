import cv2
import numpy as np


def points_from_path(inp_path):
    """type conversion from PoseStampd to numpy array

    Args:
        inp_path (list of PoseStamped): input path as list of PoseStamped

    Returns:
        np.NDArray: output path as array
    """
    out_path = []
    # pdb.set_trace()
    for pose in inp_path.poses:
        point = [pose.pose.position.x, pose.pose.position.y]
        out_path.append(point)
    return np.asarray(out_path, dtype=np.float64).T


def calc_derivative(samples):
    """Calculate numerical derivative of a timeseries.

    Args:
        samples (np.NDArary): input samples of shape (2, n)

    Returns:
        np.NDArray: derivation for each value (2, n-1)
    """
    der = np.diff(samples)
    der_lengths = np.sqrt(np.sum(der**2, axis=0))
    der /= der_lengths
    return der


def calc_angles(points):
    """Calculate Angles for each point in path as angle to next point.

    Args:
        points (np.NDArray): path of shape (2, n)

    Returns:
        _type_: angles of shape (n,)
    """
    deltas = np.diff(points)
    angles = np.arctan2(deltas[1], deltas[0])
    new_angles = []
    for i in range(len(angles)):
        if i == 0:
            new_angles.append(angles[i])
            continue
        delta_angle = angles[i] - new_angles[-1]
        angle_diff = np.arctan2(np.sin(delta_angle), np.cos(delta_angle))
        # if delta_angle < 0:
        #     delta_angle = -(-delta_angle % np.pi)
        # else:
        #     delta_angle = delta_angle % np.pi
        new_angles.append(new_angles[-1] + angle_diff)
    return np.array(new_angles)


def offset_path(path, angles, offset):
    """transforms each point of input path by the offset

    Args:
        path (np.NDArray): input path of shape (2, n)
        angles (np.NDArray): angles for each point (n,)
        offset (Tuple[float, float]): desired relative offset to path (x, y)

    Returns:
        np.NDArray: path with offset
    """
    res_path_x = []
    res_path_y = []
    for point, angle in zip(path.T, angles):
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        tf_point_x = cos_angle * offset[0] - sin_angle * offset[1] + point[0]
        tf_point_y = sin_angle * offset[0] + cos_angle * offset[1] + point[1]
        res_path_x.append(tf_point_x)
        res_path_y.append(tf_point_y)
    res_path = np.asarray([res_path_x, res_path_y], dtype=np.float64)
    return res_path


def world_to_map(point_world, map_origin, map_size_x, map_size_y, map_resolution):
    """transforms point from world frame to map frame.

    Args:
        point_world: 2d point in m
        map_origin: 2d map origin
        map_size_x: x dimension of map in px
        map_size_y: y dimension of map in px
        map_resolution: resolution of map in m/px

    Returns:
        2d point in map frame (px)
    """
    point_map = [0, 0]
    if point_world[0] < map_origin[0] or point_world[1] < map_origin[1]:
        return point_map

    point_map[0] = int((point_world[0] - map_origin[0]) / map_resolution)
    point_map[1] = int((point_world[1] - map_origin[1]) / map_resolution)

    if point_map[0] < map_size_x and point_map[1] < map_size_y:
        return point_map

    return [map_size_x - 1, map_size_y - 1]


class PathAnalyzer:
    """Class for analyzing a single path. Has functions for calculating path length and distance to obstacles."""
    def __init__(self, path) -> None:
        """
        Intializes Class with path from ROS-Message and converts it to Array.

        Args:
            path: path as ROS message nav_msgs/Path.
        """
        self.path = path
        self.points = points_from_path(self.path)
        self.angles = calc_angles(self.points)
        self.angles = np.append(self.angles, self.angles[-1])

    def calc_path_length(self):
        """Calculates length of path."""
        differences = np.diff(self.points.T, axis=0)
        if len(differences) < 1:
            return 0.0
        distances = np.linalg.norm(differences, axis=1, ord=2)
        # pdb.set_trace()
        path_length = np.sum(distances)
        return path_length

    def calc_distances_to_obstacles(self, map_data):
        """
        Calculates distance to obstacle in given map for every point in path.

        Args:
            map_data: map as ROS message OccupancyGrid

        Returns:
            distances as array
        """
        costmap_size_y = map_data.info.height
        costmap_size_x = map_data.info.width
        map_origin = (
            map_data.info.origin.position.x,
            map_data.info.origin.position.y,
        )
        map_resolution = map_data.info.resolution
        map_img = np.zeros((costmap_size_x, costmap_size_y), dtype=np.uint8)
        for i in range(costmap_size_x):
            for j in range(costmap_size_y):
                occupancy_index = j * costmap_size_x + i
                if map_data.data[occupancy_index] == 0:
                    map_img[i, j] = 255
        distance_img = cv2.distanceTransform(
            map_img, cv2.DIST_L2, 3, dstType=cv2.CV_8UC1
        )
        map_img_bgr = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
        # cv2.imwrite("map_img.png", map_img)
        # cv2.imwrite("distance_img.png", distance_img)
        distances = []
        for point in self.points.T:
            point_map = world_to_map(
                point, map_origin, costmap_size_x, costmap_size_y, map_resolution
            )
            map_img_bgr[point_map[0], point_map[1]] = [0, 255, 0]
            distances.append(distance_img[point_map[0], point_map[1]] * map_resolution)
        # cv2.imwrite("map_with_path.png", map_img_bgr)
        return np.asarray(distances, dtype=np.float64)
