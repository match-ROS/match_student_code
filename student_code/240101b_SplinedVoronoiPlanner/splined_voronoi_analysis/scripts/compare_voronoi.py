#!/usr/bin/env python3
import glob
import pdb

import cv2
import matplotlib.pyplot as plt
import numpy as np
from bagfile_helpers import MakeNavPlanWithStatsBagContents
from path_helpers import PathAnalyzer


def load_times_from_txt(filename):
    """
    Loads times for voronoi generation from a text file where it has been saved by pure_voronoi_planner compare_voronoi_results.

    Args:
        filename: string name of file.

    Returns:
        Tuple of specified dimensions, times of dynamicvoronoi and times of sweepline voronoi.
    """
    dimensions = []
    times_dynamicvoronoi = []
    times_boostvoronoi = []
    print(filename, ":")
    with open(filename, "r", encoding="utf-8") as file:
        lines = file.readlines()
    for line in lines[1:]:
        vals = line[:-2].split(", ")
        dimension = int(vals[0])
        time_dynamicvoronoi = float(vals[1])
        time_boostvoronoi = float(vals[2])
        dimensions.append(dimension)
        times_dynamicvoronoi.append(time_dynamicvoronoi)
        times_boostvoronoi.append(time_boostvoronoi)
        # print(vals)
    return (dimensions, times_dynamicvoronoi, times_boostvoronoi)


def plot_voronoi_times():
    """plotting of times for voronoi generation at different resolutions."""
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )
    fig = plt.figure("Voronoi Times", figsize=(5, 3))
    ax = fig.subplots(1, 1)
    colors = ["b", "g", "r", "c", "m", "y"]
    map_names = ["Spine", "Testarena", "Säulen"]
    for idx, dirname in enumerate(glob.glob("data/voronoi/[!d]*/")):
        # if dirname.startswith("random"):
        #     continue
        (dimensions, times_dynamicvoronoi, times_boostvoronoi) = load_times_from_txt(
            dirname + "results.txt"
        )
        ax.plot(
            dimensions,
            times_dynamicvoronoi,
            f"{colors[idx%len(colors)]}--o",
            label=f"dynamicvoronoi {map_names[idx]}",
        )
        ax.plot(
            dimensions,
            times_boostvoronoi,
            f"{colors[idx%len(colors)]}-o",
            label=f"Sweepline {map_names[idx]}",
        )
    # pdb.set_trace()
    ax.set_xlabel("Skalierung [px]")
    ax.set_ylabel("Zeit [s]")
    ax.set_xticks(np.arange(1000, 5000, 1000))
    ax.legend()

    ax.grid(axis="both", which="major")
    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")

    fig.tight_layout()
    # ax.legend(bbox_to_anchor=(1.04, 0.5), loc="center left")
    # fig.subplots_adjust(right=0.6)

    fig.savefig("data/plots/voronoi_times.pdf")
    # plt.show()


def overlay_voronoi_results():
    """combine results of dynamicvoronoi and sweepline in single image for comparison.

    dynamicvoronoi: blue; sweepline: white; both: green
    """
    for dirname in ["Säulen", "spine", "testarena"]:
        base_path = f"data/voronoi/{dirname}/"
        img_boost = cv2.imread(f"{base_path}boostvoronoi_3.png")
        img_dynamic = cv2.imread(f"{base_path}dynamicvoronoi_3.png")
        # pdb.set_trace()
        img_combined = img_boost
        for row in range(img_boost.shape[0]):
            for col in range(img_boost.shape[1]):
                if img_dynamic[row, col, 0] != 255:
                    continue
                if img_boost[row, col, 0] == 255:
                    img_combined[row, col] = [0, 255, 0]
                else:
                    img_combined[row, col] = [255, 0, 0]
        cv2.imwrite(f"{base_path}combined.png", img_combined)


def get_path_length_from_bag(filename: str, simple_length: bool = False):
    """
    Reads bagfile and calculates path length and distance to obstacles.

    Args:
        filename: name of bagfile
        simple_length: if length should be calculated as number of points in path.

    Returns:
        Tuple of path length, minimal distance to obstacles, maximum distance to obstacles

    """
    bag_contents = MakeNavPlanWithStatsBagContents(filename, skip_maps=False)
    if simple_length:
        return len(bag_contents.path.poses)

    if len(bag_contents.path.poses) < 2:
        print("Not enough poses in path")
        return 0.001, 0, 0
    path_analyser = PathAnalyzer(bag_contents.path)
    # pdb.set_trace()
    path_length = path_analyser.calc_path_length()
    distances = path_analyser.calc_distances_to_obstacles(bag_contents.map_data)
    max_distance = np.max(distances)
    min_distance = np.min(distances)
    # plt.show()
    return path_length, min_distance, max_distance


def compare_freespace_inclusion():
    """
    Analyse differences in inclusion of freespaces in planning.
    Reads bagfiles and calclulates distances to obstacles, and path lengths.
    Saves results in npy files for later plotting.
    """
    path_lengths_with_freespace = []
    path_lengths_without_freespace = []
    max_dists_with_freespace = []
    max_dists_without_freespace = []
    min_dists_with_freespace = []
    min_dists_without_freespace = []
    anomal_indices = []
    anomal_indices_simple = []
    for idx in range(100):
        filename_with_freespace = (
            f"data/voronoi/dijkstra_vs_astar/with_freespace/plan_{idx:05d}_astar.bag"
        )
        filename_without_freespace = (
            f"data/voronoi/dijkstra_vs_astar/without_freespace/plan_{idx:05d}_astar.bag"
        )
        (
            path_length_with,
            min_distance_with,
            max_distance_with,
        ) = get_path_length_from_bag(filename_with_freespace)
        simple_path_length_with = get_path_length_from_bag(
            filename_with_freespace, simple_length=True
        )
        path_lengths_with_freespace.append(path_length_with)
        if max_distance_with != 0:
            max_dists_with_freespace.append(max_distance_with)
            min_dists_with_freespace.append(min_distance_with)
        # print(f"Path length with freespaces: {path_length_with}")
        (
            path_length_without,
            min_distance_without,
            max_distance_without,
        ) = get_path_length_from_bag(filename_without_freespace)
        simple_path_length_without = get_path_length_from_bag(
            filename_without_freespace, simple_length=True
        )
        path_lengths_without_freespace.append(path_length_without)
        if max_distance_without != 0:
            max_dists_without_freespace.append(max_distance_without)
            min_dists_without_freespace.append(min_distance_without)
        # print(f"Path length without freespaces: {path_length_without}")
        print(f"Bag {idx} with / without: {path_length_with} / {path_length_without}")
        print(
            f"Bag {idx} simple with / without: {simple_path_length_with} / {simple_path_length_without}"
        )
        if path_length_with > path_length_without:
            anomal_indices.append(idx)
        if simple_path_length_with > simple_path_length_without:
            anomal_indices_simple.append(idx)
    print("========")
    print(f"Indices with > without: {anomal_indices}")
    print(f"Simple Indices with > without: {anomal_indices_simple}")
    path_lengths_combined = np.array(
        [
            path_lengths_with_freespace,
            path_lengths_without_freespace,
            [i for i in range(100)],
        ]
    ).T
    path_lengths_sorted = path_lengths_combined[path_lengths_combined[:, 0].argsort()].T
    sorted_anomal_indices = np.in1d(
        path_lengths_combined[:, 0].argsort(), anomal_indices
    ).nonzero()[0]
    print(f"Sorted anomal indices: {sorted_anomal_indices}")
    path_lengths_differences = np.diff(path_lengths_sorted[:2].T, axis=1)
    path_lengths_diff_percentage = (
        (path_lengths_sorted[1] - path_lengths_sorted[0]) / path_lengths_sorted[1] * 100
    )

    mean_diff_perc = np.mean(
        path_lengths_diff_percentage[path_lengths_diff_percentage != 1.0]
    )
    mean_diff = np.mean(path_lengths_differences[path_lengths_differences != 0.0])
    median_diff_perc = np.median(
        path_lengths_diff_percentage[path_lengths_diff_percentage != 1.0]
    )
    median_diff = np.median(path_lengths_differences[path_lengths_differences != 0.0])

    print(f"Difference absolute: Mean: {mean_diff}, Median: {median_diff}")
    print(f"Difference percentual: Mean: {mean_diff_perc}, Median: {median_diff_perc}")

    print("Saving data to npy files for easier access")
    np.save("data/voronoi/min_dists_with_freespace.npy", min_dists_with_freespace)
    np.save("data/voronoi/min_dists_without_freespace.npy", min_dists_without_freespace)
    np.save("data/voronoi/max_dists_with_freespace.npy", max_dists_with_freespace)
    np.save("data/voronoi/max_dists_without_freespace.npy", max_dists_without_freespace)
    np.save(
        "data/voronoi/path_lengths_diff_percentage.npy", path_lengths_diff_percentage
    )
    print("========")


def create_freespace_boxplot():
    """Creates plots from saved data of freespace comparison."""
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )

    min_dists_with_freespace = np.load(
        "data/voronoi/min_dists_with_freespace.npy", allow_pickle=True
    )
    min_dists_without_freespace = np.load(
        "data/voronoi/min_dists_without_freespace.npy", allow_pickle=True
    )
    max_dists_with_freespace = np.load(
        "data/voronoi/max_dists_with_freespace.npy", allow_pickle=True
    )
    max_dists_without_freespace = np.load(
        "data/voronoi/max_dists_without_freespace.npy", allow_pickle=True
    )
    path_lengths_diff_percentage = np.load(
        "data/voronoi/path_lengths_diff_percentage.npy", allow_pickle=True
    )

    data = (
        min_dists_with_freespace,
        min_dists_without_freespace,
        max_dists_with_freespace,
        max_dists_without_freespace,
        path_lengths_diff_percentage,
    )

    x_labels = [
        "Min. Abstand\nmit Freiflächen",
        "Min. Abstand\nohne Freiflächen",
        "Max. Abstand\nmit Freiflächen",
        "Max. Abstand\nohne Freiflächen",
        "Einsparung der Streckenlänge",
    ]
    fig_dist = plt.figure("Abstand Boxplot", figsize=(5, 3))
    axs_dist = fig_dist.subplots(1, 1)
    axs_dist.boxplot(data[:4], labels=x_labels[:4])
    axs_dist.grid(axis="y", which="major")
    # axs_dist.grid(axis="y", which="major")
    axs_dist.set_ylabel("Distanz [m]")
    axs_dist.spines["top"].set_color("gray")
    axs_dist.spines["right"].set_color("gray")
    fig_dist.tight_layout()
    fig_dist.savefig("data/plots/boxplot_dists_obstacles.pdf")

    fig_einsp = plt.figure("Einsparung Boxplot", figsize=(4, 2))
    axs_einsp = fig_einsp.subplots(1, 1)
    axs_einsp.boxplot(
        data[4],
        labels=[x_labels[4]],
        usermedians=[np.mean(path_lengths_diff_percentage)],
    )
    # axs.yaxis.set_minor_locator(AutoMinorLocator())
    axs_einsp.grid(axis="y", which="major")
    # axs_einsp.grid(axis="y", which="major")
    axs_einsp.set_ylabel("Einsparung [\%]")
    axs_einsp.spines["top"].set_color("gray")
    axs_einsp.spines["right"].set_color("gray")
    fig_einsp.tight_layout()
    fig_einsp.savefig("data/plots/boxplot_einsparung.pdf")
    # plt.show()


def get_planning_time_from_bag(filename: str):
    """Returns planning time from specified bagfile."""
    bag_contents = MakeNavPlanWithStatsBagContents(filename, skip_maps=True)
    planning_time = bag_contents.total_time
    return planning_time


def compare_dijkstra_and_astar():
    """Compares and prints info abound astar and dijkstra planning bagfiles regarding success rate and mean time."""
    for dirname in ["without_freespace", "with_freespace"]:
        print(f"Loading bags from directory: {dirname}")
        planning_times_astar = []
        planning_times_dijkstra = []
        for idx in range(100):
            # print("================")
            filename_astar = (
                f"data/voronoi/dijkstra_vs_astar/{dirname}/plan_{idx:05d}_astar.bag"
            )
            filename_dijkstra = (
                f"data/voronoi/dijkstra_vs_astar/{dirname}/plan_{idx:05d}_dijkstra.bag"
            )
            planning_time_astar = get_planning_time_from_bag(filename_astar)
            planning_times_astar.append(planning_time_astar)
            planning_time_dijkstra = get_planning_time_from_bag(filename_dijkstra)
            planning_times_dijkstra.append(planning_time_dijkstra)
            # print(f"Loaded Bag {idx}")
        print("========")

        mean_time_astar = np.mean(planning_times_astar)
        median_time_astar = np.median(planning_times_astar)
        min_time_astar = np.min(planning_times_astar)
        max_time_astar = np.max(planning_times_astar)

        mean_time_dijkstra = np.mean(planning_times_dijkstra)
        median_time_dijkstra = np.median(planning_times_dijkstra)
        min_time_dijkstra = np.min(planning_times_dijkstra)
        max_time_dijkstra = np.max(planning_times_dijkstra)

        print(
            f"A* {dirname}: Min: {min_time_astar}, Max: {max_time_astar}, Mean: {mean_time_astar}, Median: {median_time_astar}"
        )
        print(
            f"Dijkstra  {dirname}: Min: {min_time_dijkstra}, Max: {max_time_dijkstra}, Mean: {mean_time_dijkstra}, Median: {median_time_dijkstra}"
        )


def main():
    overlay_voronoi_results()
    plot_voronoi_times()
    compare_dijkstra_and_astar()
    compare_freespace_inclusion()
    create_freespace_boxplot()


if __name__ == "__main__":
    main()
