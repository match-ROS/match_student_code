#!/usr/bin/env python3
import pdb

import cv2
import matplotlib.pyplot as plt
import numpy as np
from bagfile_helpers import get_contents_from_bagfile
from path_helpers import points_from_path
from spline_helpers import calc_connection_points


def sparsing_example_from_bag():
    """Create plot for example waypoint selection from path. Reads data from data/sparsing_example.bag."""
    bag_contents = get_contents_from_bagfile("data/sparsing_example.bag")

    original_path = bag_contents["/move_base_flex/VoronoiGlobalPlanner/original_plan"]
    sparse_path = bag_contents["/move_base_flex/VoronoiGlobalPlanner/sparse_plan"]
    voronoi_data = bag_contents["/move_base_flex/VoronoiGlobalPlanner/voronoi_map"]
    sparse_length = len(sparse_path.poses)
    original_length = len(original_path.poses)
    print(f"reduced length from {original_length} to {sparse_length}")
    sparse_plan = points_from_path(sparse_path)
    original_plan = points_from_path(original_path)

    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )

    figure = plt.figure("Sparsing", figsize=(4, 2.7))  # , dpi=300
    ax = figure.subplots(1, 1)
    ax.plot(original_plan[1], original_plan[0], "-o", label="Pfad")
    ax.plot(sparse_plan[1], sparse_plan[0], "-o", label="Wegpunkte")

    ax.plot([3.0], [-1.5], "o", color="black", label="Hindernis", zorder=1)
    circle = plt.Circle((3.0, -3.0), 2.0, color="black")
    ax.add_artist(circle)

    ax.set_aspect("equal")
    ax.invert_xaxis()
    ax.legend()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_yticks(np.arange(-3, 1, 1.0))
    ax.set_xticks(np.arange(0, 4, 1.0))
    ax.grid(axis="both", which="major")
    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")
    figure.tight_layout()
    # pdb.set_trace()
    figure.savefig("data/plots/sparsing_example.pdf")

    # plt.show()


def splining_exmple_from_bag():
    """Create plot for example splining and optimization of path. Reads data from data/splining_example.bag."""
    bag_contents = get_contents_from_bagfile("data/splining_example.bag")

    original_path = bag_contents["/move_base_flex/VoronoiGlobalPlanner/original_plan"]
    sparse_path = bag_contents["/move_base_flex/VoronoiGlobalPlanner/sparse_plan"]
    optimized_path = bag_contents["/move_base_flex/VoronoiGlobalPlanner/optimized_plan"]
    optimized_tangent_lengths = bag_contents[
        "/move_base_flex/VoronoiGlobalPlanner/optimized_lengths"
    ].data
    splined_path = bag_contents["/voronoi_planner/formation_plan"]
    voronoi_data = bag_contents["/move_base_flex/VoronoiGlobalPlanner/voronoi_map"]
    sparse_length = len(sparse_path.poses)
    original_length = len(original_path.poses)
    print(f"reduced length from {original_length} to {sparse_length}")
    # original_plan = points_from_path(original_path)
    sparse_plan = points_from_path(sparse_path)
    optimized_plan = points_from_path(optimized_path)
    splined_plan = points_from_path(splined_path)

    # optimized_tangent_lengths.data
    orig_tangent_lengths = [1] * len(optimized_tangent_lengths)
    connection_points_opt = calc_connection_points(optimized_plan)
    connection_points_org = calc_connection_points(sparse_plan)
    tangents = np.diff(optimized_plan.T, axis=0)
    tangents /= np.sqrt(np.sum(tangents**2, axis=1)).reshape(-1, 1)
    tangents_orig = np.diff(sparse_plan.T, axis=0)
    tangents_orig /= np.sqrt(np.sum(tangents_orig**2, axis=1)).reshape(-1, 1)
    # pdb.set_trace()

    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )

    figure = plt.figure("Splining", figsize=(5.9, 2.3))  # , dpi=300
    ax = figure.subplots(1, 1)
    ax.plot(sparse_plan[1], sparse_plan[0], "-o", label="Wegpunkte")
    ax.plot(
        [connection_points_opt[1, 0]],
        [connection_points_opt[0, 0]],
        "-",
        color="m",
        label="Tangenten",
    )
    ax.plot(optimized_plan[1], optimized_plan[0], "-o", label="optimierte Wegpunkte")
    ax.plot(
        [connection_points_opt[1, 0]],
        [connection_points_opt[0, 0]],
        "-",
        color="r",
        label="optimierte Tangenten",
    )
    ax.plot(splined_plan[1], splined_plan[0], "-", label="Bézier-Spline")

    for point, tangent, length in zip(
        connection_points_opt.T, tangents, optimized_tangent_lengths
    ):
        print(f"{point[::-1]}, {tangent[::-1]/4}")
        arrow = ax.arrow(
            *point[::-1],
            *tangent[::-1] / 4 * length,
            width=0.01,
            head_width=0.05,
            color="r",
            zorder=2,
        )

    for point, tangent, length in zip(
        connection_points_org.T, tangents_orig, orig_tangent_lengths
    ):
        print(f"{point[::-1]}, {tangent[::-1]/4}")
        arrow = ax.arrow(
            *point[::-1],
            *tangent[::-1] / 4 * length,
            width=0.01,
            head_width=0.05,
            color="m",
            zorder=2,
        )

    ax.plot([-3.0], [-9.0], "o", color="black", label="Hindernis", zorder=1)
    circle = plt.Circle((-3.0, -9.0), 2.2, color="black")
    ax.add_artist(circle)

    ax.set_aspect("equal")
    # ax.invert_xaxis()
    ax.legend(bbox_to_anchor=(1.04, 0.5), loc="center left")
    figure.subplots_adjust(right=0.7)
    # ax.legend()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_ylim([-11.8, -8.5])
    ax.set_yticks(np.arange(-11, -8.0, 1.0))
    ax.grid(axis="both", which="major")

    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")
    # pdb.set_trace()
    figure.tight_layout()
    figure.savefig("data/plots/splining_example.pdf")


def main():
    sparsing_example_from_bag()
    splining_exmple_from_bag()


if __name__ == "__main__":
    main()
