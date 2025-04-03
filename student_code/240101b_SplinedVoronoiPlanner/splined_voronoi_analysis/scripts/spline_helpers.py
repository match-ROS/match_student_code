import pdb

import cv2
import numpy as np
from path_helpers import (
    calc_angles,
    calc_derivative,
    offset_path,
    points_from_path,
    world_to_map,
)

# all functions regarding tinyspline without dependency on tinyspline lib


def calc_connection_points(sparse_path):
    """Calculates connection points in the middle of given path; start and goal are not modified.

    Args:
        sparse_path (np.NDArray): sparse path of shape (2, n)

    Returns:
        np.NDArray: Connection points of shape (2, n-1)
    """
    prev_point = sparse_path.T[0]
    connection_points = []
    eps = 0.0001
    for i, point in enumerate(sparse_path.T):
        if abs(point[0] - prev_point[0]) < eps and abs(point[1] - prev_point[1]) < eps:
            continue
        if i == 1:
            # first point
            connection_point = prev_point.tolist()
        elif i == len(sparse_path.T) - 1:
            # last point
            connection_point = point.tolist()
        else:
            connection_point = [
                (prev_point[0] + point[0]) / 2,
                (prev_point[1] + point[1]) / 2,
            ]
        connection_points.append(connection_point)
        prev_point = point
    return np.asarray(connection_points, dtype=np.float64).T


def calc_control_points_for_path(sparse_path, lengths):
    """Calculate control points for every segment of the given path.

    Args:
        sparse_path (np.NDArray): sparse path of shape (2, n)
        lengths (np.NDArray): desired lengths for the tangents of each segment shape (n-1,)

    Returns:
        np.NDArray: spline control points of shape (n-1, 2)
    """
    connection_points = calc_connection_points(sparse_path)
    tangents = np.diff(sparse_path.T, axis=0)
    tangents /= np.sqrt(np.sum(tangents**2, axis=1)).reshape(-1, 1)
    tangents *= lengths.reshape(-1, 1)

    distances = np.sqrt(np.sum(np.diff(connection_points.T, axis=0) ** 2, axis=1))
    d_AB = distances[:-1]
    d_BC = distances[1:]
    alphas = (d_BC / (d_AB + d_BC)).reshape(-1, 1)
    betas = (d_AB / (d_AB + d_BC)).reshape(-1, 1)
    As = connection_points.T[:-2]
    Bs = connection_points.T[1:-1]
    Cs = connection_points.T[2:]
    tangents_A = tangents[:-2]
    tangents_B = tangents[1:-1]
    tangents_C = tangents[2:]
    intermediate_accelerations = alphas * (
        6 * As + 2 * tangents_A + 4 * tangents_B - 6 * Bs
    ) + betas * (-6 * Bs - 4 * tangents_B - 2 * tangents_C + 6 * Cs)
    first_acc = (
        (-6 * Bs[0] - 4 * tangents_B[0] - 2 * tangents_C[0] + 6 * Cs[0])
        .reshape(-1, 1)
        .T
    )
    last_acc = (
        (6 * As[-1] + 2 * tangents_A[-1] + 4 * tangents_B[-1] - 6 * Bs[-1])
        .reshape(-1, 1)
        .T
    )
    accelerations = np.concatenate((first_acc, intermediate_accelerations, last_acc))
    p_0s = connection_points.T[:-1]
    p_5s = connection_points.T[1:]
    p_1s = 0.2 * tangents[:-1] + p_0s  # * 5.0 * 0.5 * ratio
    p_4s = p_5s - 0.2 * tangents[1:]  # * 5.0 * 0.5 * ratio
    p_2s = 0.05 * accelerations[:-1] + 2 * p_1s - p_0s
    p_3s = 0.05 * accelerations[1:] + 2 * p_4s - p_5s
    control_points = np.stack((p_0s, p_1s, p_2s, p_3s, p_4s, p_5s), axis=1)
    return control_points


def calc_control_points_for_path_sra(sparse_path, lengths):
    """Calculate control points for every segment of the given path. Uses sparse path directly.

    Args:
        sparse_path (np.NDArray): sparse path of shape (2, n)
        lengths (np.NDArray): desired lengths for the tangents of each segment shape (n-1,)

    Returns:
        np.NDArray: spline control points of shape (n-1, 2)
    """
    first_tangent = (sparse_path.T[1] - sparse_path.T[0]) * 1.0
    tangents = [first_tangent]
    for idx, point in enumerate(sparse_path.T[1:-1]):
        ab = point - sparse_path.T[idx]
        ac = sparse_path.T[idx + 2] - point
        ab_norm = ab / np.linalg.norm(ab)
        ac_norm = ac / np.linalg.norm(ac)
        anglular_bisector = ab_norm + ac_norm
        anglular_bisector /= np.linalg.norm(anglular_bisector)
        length_diff = np.linalg.norm(ac)
        tangent = length_diff * anglular_bisector * 0.5
        tangents.append(tangent)
    last_tangent = (sparse_path.T[-1] - sparse_path.T[-2]) * 0.5
    tangents.append(last_tangent)
    tangents = np.asarray(tangents)
    tangents *= lengths.reshape(-1, 1)

    distances = np.sqrt(np.sum(np.diff(sparse_path.T, axis=0) ** 2, axis=1))
    d_AB = distances[:-1]
    d_BC = distances[1:]
    alphas = (d_BC / (d_AB + d_BC)).reshape(-1, 1)
    betas = (d_AB / (d_AB + d_BC)).reshape(-1, 1)
    As = sparse_path.T[:-2]
    Bs = sparse_path.T[1:-1]
    Cs = sparse_path.T[2:]
    tangents_A = tangents[:-2]
    tangents_B = tangents[1:-1]
    tangents_C = tangents[2:]
    intermediate_accelerations = alphas * (
        6 * As + 2 * tangents_A + 4 * tangents_B - 6 * Bs
    ) + betas * (-6 * Bs - 4 * tangents_B - 2 * tangents_C + 6 * Cs)
    first_acc = (
        (-6 * Bs[0] - 4 * tangents_B[0] - 2 * tangents_C[0] + 6 * Cs[0])
        .reshape(-1, 1)
        .T
    )
    last_acc = (
        (6 * As[-1] + 2 * tangents_A[-1] + 4 * tangents_B[-1] - 6 * Bs[-1])
        .reshape(-1, 1)
        .T
    )
    accelerations = np.concatenate((first_acc, intermediate_accelerations, last_acc))
    p_0s = sparse_path.T[:-1]
    p_5s = sparse_path.T[1:]
    p_1s = 0.2 * tangents[:-1] + p_0s  # * 5.0 * 0.5 * ratio
    p_4s = p_5s - 0.2 * tangents[1:]  # * 5.0 * 0.5 * ratio
    p_2s = 0.05 * accelerations[:-1] + 2 * p_1s - p_0s
    p_3s = 0.05 * accelerations[1:] + 2 * p_4s - p_5s
    control_points = np.stack((p_0s, p_1s, p_2s, p_3s, p_4s, p_5s), axis=1)
    return control_points


def get_coefficient_matrices(control_points):
    """Calculates coefficients for quintic bezier spline for a set of control points.

    Args:
        control_points (np.NDArray): control points of spline.

    Returns:
        np.NDArray: matrix with coefficients for each bezier-curve.
    """
    C_QBS = []
    for cps in control_points:
        C_QB = np.array(
            [
                -cps[0] + 5 * cps[1] - 10 * cps[2] + 10 * cps[3] - 5 * cps[4] + cps[5],
                5 * cps[0] - 20 * cps[1] + 30 * cps[2] - 20 * cps[3] + 5 * cps[4],
                -10 * cps[0] + 30 * cps[1] - 30 * cps[2] + 10 * cps[3],
                10 * cps[0] - 20 * cps[1] + 10 * cps[2],
                -5 * cps[0] + 5 * cps[1],
                cps[0],
            ]
        )
        C_QBS.append(C_QB)
    return np.asarray(C_QBS)


def get_sample(C_QB, frac):
    """Interpolates quintic bezier-spline at a given fraction.

    Args:
        C_QB: coefficient matrix as of @get_coefficient_matrices
        frac: fraction for interpolation, between 0 and 1.

    Returns:
        Tuple: 2d value of spline, derivative, second derivative and curvature at this point
    """
    T = np.array([frac**5, frac**4, frac**3, frac**2, frac, 1])
    T_der = np.array([5 * frac**4, 4 * frac**3, 3 * frac**2, 2 * frac, 1, 0])
    T_der2 = np.array([20 * frac**3, 12 * frac**2, 6 * frac, 2, 0, 0])
    sample = T @ C_QB
    sample_der = T_der @ C_QB
    sample_der2 = T_der2 @ C_QB
    curvature = (sample_der[0] * sample_der2[1] - sample_der2[0] * sample_der[1]) / (
        (sample_der[0] ** 2 + sample_der[1] ** 2) ** 1.5
    )
    return sample, sample_der, sample_der2, curvature


def sample_spline_path(control_points, num_samples=1000):
    """Sample spline path from its control points

    Args:
        control_points (np.NDArray): output of @calc_control_points_for_path
        num_samples (int, optional): Number of samples per segment. Defaults to 1000.

    Returns:
        np.NDArray: output samples of shape (2, n)
    """
    samples = []
    samples_der = []
    samples_der2 = []
    curvatures = []
    fracs = np.linspace(0.0, 1.0, num_samples)

    C_QBS = get_coefficient_matrices(control_points)
    for C_QB in C_QBS:
        for frac in fracs[:-1]:
            sample, sample_der, sample_der2, curvature = get_sample(C_QB, frac)
            samples.append(sample)
            samples_der.append(sample_der)
            samples_der2.append(sample_der2)
            curvatures.append(curvature)
    sample, sample_der, sample_der2, curvature = get_sample(C_QBS[-1], fracs[-1])
    samples.append(sample)
    samples_der.append(sample_der)
    samples_der2.append(sample_der2)
    curvatures.append(curvature)
    samples = np.asarray(samples).T
    samples_der = np.asarray(samples_der).T
    samples_der2 = np.asarray(samples_der2).T
    return samples, samples_der, samples_der2, curvatures


class SplineAnalyzer:
    """Class for analysing a specific spline. Also contins interface for offsetting spline path to get path of a robot in formation."""
    def __init__(
        self, optimized_sparse_path, spline_tangent_lengths, num_samples=1000
    ) -> None:
        """
        Initializes class; Calculates and interpolates spline from points; calculates orientation at each point and derivation of path.

        Args:
            optimized_sparse_path: waypoints from which spline is generated.
            spline_tangent_lengths: tangent lengths at the connection points of bezier curves.
            num_samples: number of samples per bezier curve.
        """
        self.optimized_sparse_path = optimized_sparse_path
        self.spline_tangent_lengths = np.asarray(spline_tangent_lengths)
        self.points = points_from_path(self.optimized_sparse_path)
        self.connection_points = calc_connection_points(self.points)
        self.control_points = calc_control_points_for_path(
            self.points, self.spline_tangent_lengths
        )
        (
            self.samples,
            self.samples_der,
            self.samples_der2,
            self.curvatures,
        ) = sample_spline_path(self.control_points, num_samples)
        self.sample_distances = np.sqrt(
            np.diff(self.samples[0]) ** 2 + np.diff(self.samples[1]) ** 2
        )
        self.path_distance = np.cumsum(self.sample_distances)
        self.path_distance = np.append(self.path_distance, self.path_distance[-1])
        self.angles = calc_angles(self.samples)
        self.angles = np.append(self.angles, self.angles[-1])
        # self.angles_der = calc_derivative(self.angles)
        self.angles_der = np.diff(self.angles) / self.sample_distances
        self.angles_der2 = calc_derivative(self.angles_der)
        self.offsets = []

    def offset(self, offset):
        """
        Offset spline by a given amount to generate path of a single robot in the formation.

        Args:
            offset: Tuple of x, y offset of robot relative to formation center.

        Returns:
            samples, angles, derivative, second derivative
        """
        off_samples = offset_path(self.samples, self.angles, offset)
        off_angles = calc_angles(off_samples)
        off_angles = np.append(off_angles, off_angles[-1])
        off_sample_distances = np.sqrt(
            np.diff(off_samples[0]) ** 2 + np.diff(off_samples[1]) ** 2
        )
        # off_angles_der = calc_derivative(off_angles)
        off_angles_der = np.diff(off_angles) / off_sample_distances
        off_angles_der2 = calc_derivative(off_angles_der)
        self.offsets.append((off_samples, off_angles, off_angles_der, off_angles_der2))
        return off_samples, off_angles, off_angles_der, off_angles_der2

    def calc_distances_to_obstacle(self, map_data):
        """
        Calculates distance to obstacle of formation path at every point in path to a given map.

        Args:
            map_data: Map as ROS message type OccupancyGrid

        Returns:
            distance for each sample in spline.
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
        for point in self.samples.T:
            point_map = world_to_map(
                point, map_origin, costmap_size_x, costmap_size_y, map_resolution
            )
            map_img_bgr[point_map[0], point_map[1]] = [0, 255, 0]
            distances.append(distance_img[point_map[0], point_map[1]] * map_resolution)
        # cv2.imwrite("map_with_path.png", map_img_bgr)
        return np.asarray(distances, dtype=np.float64)
