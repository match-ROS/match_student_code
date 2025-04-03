#!/usr/bin/env python3
import glob
import pdb

import matplotlib.pyplot as plt
import numpy as np
from bagfile_helpers import MakeNavPlanBagContents, MakeNavPlanWithStatsBagContents
from path_helpers import PathAnalyzer, calc_angles, points_from_path
from spline_helpers import SplineAnalyzer


def get_stats_from_bag(filename: str, min_dist_thresh: float = 1.5):
    """
    Reads results of path planning and smoothing from bagfile.

    Args:
        filename: name of bagfile as string
        min_dist_thresh: minimum distance to obstacles for this map. (1.5 for testarena; 0.9 for spine).

    Returns:
        Tuple of planning success, time, path length, distances to obstacles, min distance to obstacles, max dist to obstacles, median distance to obstacles, path.
    """
    if "voronoi" in filename:
        bag_contents = MakeNavPlanWithStatsBagContents(filename, skip_maps=False)
    else:
        bag_contents = MakeNavPlanBagContents(filename, skip_maps=False)
    planning_time = bag_contents.time_taken
    planning_success = bag_contents.planning_success
    path_length = 0.0
    min_dist = 0.0
    max_dist = 0.0
    mean_dist = 0.0
    median_dist = 0.0
    path = []
    distances_to_obstacles = []
    if len(bag_contents.path.poses) > 1:
        path_analyser = PathAnalyzer(bag_contents.path)
        path = path_analyser.points
        path_length = path_analyser.calc_path_length()
        # pdb.set_trace()
        # plt.show()
        distances_to_obstacles = path_analyser.calc_distances_to_obstacles(
            bag_contents.map_data
        )
        min_dist = np.min(distances_to_obstacles)
        # if min_dist < min_dist_thresh:
        #     print("Too close to obstacles!")
        #     planning_success = False
        max_dist = np.max(distances_to_obstacles)
        mean_dist = np.mean(distances_to_obstacles)
        median_dist = np.median(distances_to_obstacles)
        print(
            f"Min dist {min_dist:05f}, max dist: {max_dist:05f}, mean dist: {mean_dist:05f}, median dist: {median_dist:05f}"
        )
    return (
        planning_success,
        planning_time,
        path_length,
        distances_to_obstacles,
        min_dist,
        max_dist,
        mean_dist,
        median_dist,
        path,
    )


def get_curvatures_from_bag_with_stats(filename: str):
    """
    reads bagfile of type MakeNavPlanWithStats and calculates curvature for each point in path.

    Args:
        filename: string of bagfile name

    Returns:
        curvatures along path
    """
    bag_contents = MakeNavPlanWithStatsBagContents(filename, skip_maps=True)
    if bag_contents.planning_status_code < 0:
        return []
    spline_analyser = SplineAnalyzer(
        bag_contents.optimized_sparse_path, bag_contents.spline_tangent_lengths, 1000
    )
    return spline_analyser.curvatures


def get_curvatures_from_bag(filename: str):
    """
    reads bagfile of type MakeNavPlan and calculates curvature for each point in path.

    Args:
        filename: string of bagfile name

    Returns:
        curvatures along path
    """
    bag_contents = MakeNavPlanBagContents(filename, skip_maps=False)
    if not bag_contents.planning_success:
        return []
    samples = points_from_path(bag_contents.path)
    sample_distances = np.sqrt(np.diff(samples[0]) ** 2 + np.diff(samples[1]) ** 2)
    angles = calc_angles(samples)
    angles = np.append(angles, angles[-1])
    curvatures = np.diff(angles) / sample_distances
    return curvatures


def load_data_and_combine():
    """Compares planned paths for sra and splined voronoi by reading bagfiles and computing distance to obstacles, curvature and success rate. Saves data to npy files for later plotting."""
    maps = ["spine", "testarena"]
    maps_dist_threshs = [0.9, 1.5]
    times_voronoi = []
    times_sra = []
    distances_maps_voronoi = []
    distances_maps_sra = []
    total_successes_voronoi = 0
    total_successes_sra = 0
    total_successes_voronoi_obstacles = 0
    total_successes_sra_obstacles = 0
    total_num_plans = 0
    rel_path_lengths = []
    curvatures_voronoi = []
    curvatures_sra = []
    for map_name, dist_thresh in zip(maps, maps_dist_threshs):
        filename_struct = f"data/compare_planner/{map_name}/voronoi/plan_*.bag"
        stats_voronoi = []
        stats_sra = []
        for idx, filename_voronoi in enumerate(glob.glob(filename_struct)):
            total_num_plans += 1
            filename_sra = filename_voronoi.replace("voronoi", "sra")
            stat_voronoi = get_stats_from_bag(filename_voronoi, dist_thresh)
            stats_voronoi.append(stat_voronoi)
            times_voronoi.append(stat_voronoi[1])
            if stat_voronoi[0]:
                total_successes_voronoi += 1
            if stat_voronoi[0] and np.min(stat_voronoi[3]) >= dist_thresh:
                total_successes_voronoi_obstacles += 1
            stat_sra = get_stats_from_bag(filename_sra, dist_thresh)
            stats_sra.append(stat_sra)
            times_sra.append(stat_sra[1])
            if stat_sra[0]:
                total_successes_sra += 1
            if stat_sra[0] and np.min(stat_sra[3]) >= dist_thresh:
                total_successes_sra_obstacles += 1
            print(
                f"{map_name}: Bag {idx} voronoi / sra: {stat_voronoi[0]} / {stat_sra[0]}"
            )
            curvatures_vor = get_curvatures_from_bag_with_stats(filename_voronoi)
            curvatures_s = get_curvatures_from_bag(filename_sra)
            if len(curvatures_vor) == 0 or len(curvatures_s) == 0:
                # print(f"invalid on {map_name}")
                continue
            curvatures_voronoi.append(curvatures_vor)
            curvatures_sra.append(curvatures_s)
        success_sra = [val[0] for val in stats_sra]
        success_voronoi = [val[0] for val in stats_voronoi]
        success_both = np.bitwise_and(success_voronoi, success_sra)
        distances_flattened_voronoi = np.hstack(
            [
                val
                for val, success in zip(
                    [stat[3] for stat in stats_voronoi], success_both
                )
                if success
            ]
        )
        distances_maps_voronoi.append(distances_flattened_voronoi)
        min_dist_voronoi = np.min(distances_flattened_voronoi)
        max_dist_voronoi = np.max(distances_flattened_voronoi)
        mean_dist_voronoi = np.mean(distances_flattened_voronoi)
        median_dist_voronoi = np.median(distances_flattened_voronoi)
        print(
            f"Voronoi: Min dist {min_dist_voronoi:05f}, max dist: {max_dist_voronoi:05f}, mean dist: {mean_dist_voronoi:05f}, median dist: {median_dist_voronoi:05f}"
        )
        distances_flattened_sra = np.hstack(
            [
                val
                for val, success in zip([stat[3] for stat in stats_sra], success_both)
                if success
            ]
        )
        distances_maps_sra.append(distances_flattened_sra)
        min_dist_sra = np.min(distances_flattened_sra)
        max_dist_sra = np.max(distances_flattened_sra)
        mean_dist_sra = np.mean(distances_flattened_sra)
        median_dist_sra = np.median(distances_flattened_sra)
        print(
            f"Splined Relaxed AStar: Min dist {min_dist_sra:05f}, max dist: {max_dist_sra:05f}, mean dist: {mean_dist_sra:05f}, median dist: {median_dist_sra:05f}"
        )
        path_lengths_voronoi = np.asarray(
            [
                val
                for val, success in zip(
                    [stat[2] for stat in stats_voronoi], success_both
                )
                if success
            ]
        )
        path_lengths_sra = np.asarray(
            [
                val
                for val, success in zip([stat[2] for stat in stats_sra], success_both)
                if success
            ]
        )
        rel_path_lengths.extend(
            (path_lengths_voronoi - path_lengths_sra) / path_lengths_sra * 100
        )
    mean_time_voronoi = np.median(times_voronoi)
    mean_time_sra = np.median(times_sra)
    quota_voronoi = total_successes_voronoi / total_num_plans * 100
    quota_sra = total_successes_sra / total_num_plans * 100
    quota_voronoi_obstacles = total_successes_voronoi_obstacles / total_num_plans * 100
    quota_sra_obstacles = total_successes_sra_obstacles / total_num_plans * 100
    print(
        f"Voronoi: obstacles: {total_successes_voronoi_obstacles}/{total_num_plans} ({quota_voronoi_obstacles:.2f}%); curvature: {total_successes_voronoi}/{total_num_plans} ({quota_voronoi:.2f}%); median time: {mean_time_voronoi}"
    )
    print(
        f"sra: obstacles: {total_successes_sra_obstacles}/{total_num_plans} ({quota_sra_obstacles:.2f}%); curvature: {total_successes_sra}/{total_num_plans} ({quota_sra:.2f}%); median time: {mean_time_sra}"
    )
    # pdb.set_trace()
    np.save("data/compare_planner/rel_path_lengths.npy", rel_path_lengths)
    np.savez(
        "data/compare_planner/distances_maps_sra.npz",
        spine=distances_maps_sra[0],
        testarena=distances_maps_sra[1],
    )
    np.savez(
        "data/compare_planner/distances_maps_voronoi.npz",
        spine=distances_maps_voronoi[0],
        testarena=distances_maps_voronoi[1],
    )
    max_curves_sra = [np.max(vals) for vals in curvatures_sra]
    max_curves_voronoi = [np.max(vals) for vals in curvatures_voronoi]
    np.save("data/compare_planner/curvatures_voronoi.npy", max_curves_voronoi)
    np.save("data/compare_planner/curvatures_sra.npy", max_curves_sra)


def boxplots():
    """Plotting of comparison of sra and splined_voronoi based on data read from npy files."""
    distances_maps_sra = np.load("data/compare_planner/distances_maps_sra.npz")
    distances_maps_voronoi = np.load("data/compare_planner/distances_maps_voronoi.npz")
    rel_path_lengths = np.load(
        "data/compare_planner/rel_path_lengths.npy", allow_pickle=True
    )

    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )
    fig = plt.figure("Distances", figsize=(5, 2.5))
    ax = fig.subplots(1, 1)
    ax.boxplot(
        [
            distances_maps_sra["testarena"],
            distances_maps_voronoi["testarena"],
            distances_maps_sra["spine"],
            distances_maps_voronoi["spine"],
        ],
        labels=[
            "Splined\nRelaxed A*\nTestarena",
            "Splined-Voronoi\nTestarena",
            "Splined\nRelaxed A*\nSpine",
            "Splined-Voronoi\nSpine",
        ],
    )
    ax.set_ylabel("Distanz [m]")
    ax.grid(axis="y", which="major")
    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")
    ax.set_yticks(np.arange(1, 7, 2))
    fig.tight_layout()
    fig.savefig("data/plots/boxplot_distances.pdf")

    fig = plt.figure("Verlängerung", figsize=(5.9, 2))
    axs = fig.subplots(1, 2)
    axs[0].boxplot(rel_path_lengths, labels=["Verlängerung der Strecke"])
    axs[0].set_ylabel("Verlängerung [\%]")
    axs[0].grid(axis="y", which="major")
    axs[0].spines["top"].set_color("gray")
    axs[0].spines["right"].set_color("gray")

    max_curves_voronoi = np.load(
        "data/compare_planner/curvatures_voronoi.npy", allow_pickle=True
    )
    max_curves_sra = np.load(
        "data/compare_planner/curvatures_sra.npy", allow_pickle=True
    )
    axs[1].boxplot(
        [max_curves_sra, max_curves_voronoi],
        labels=["Splined-Relaxed-A*", "Splined-Voronoi"],
    )
    axs[1].set_ylabel("Krümmung [1/m]")
    axs[1].grid(axis="y", which="major")
    axs[1].spines["top"].set_color("gray")
    axs[1].spines["right"].set_color("gray")

    fig.tight_layout()
    fig.savefig("data/plots/boxplot_path_lengths.pdf")

    # plt.show()


def main():
    load_data_and_combine()
    boxplots()


if __name__ == "__main__":
    main()
