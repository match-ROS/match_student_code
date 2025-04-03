#!/usr/bin/env python3
import pdb

import matplotlib.pyplot as plt
import numpy as np
from bagfile_helpers import MakeNavPlanWithStatsBagContents

# from analyze_bagfile import MakeNavPlanWithStatsBagContents, SplineAnalyzer
from matplotlib.ticker import AutoMinorLocator
from spline_helpers import SplineAnalyzer


def single_formation_path():
    """
    Loads planned and smoothed path from bagfile and create robot paths for example formation.
    Plotting of paths, collisions, angles and angular velocity.

    For Usage on random_plans_algorithms/NEWUOA/plan_00002.bag
    """
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )

    filename = f"data/test_planner/single_formation_path_NEWOUA_plan_00002.bag"
    bag_contents = MakeNavPlanWithStatsBagContents(filename)
    print(f"Bag {2} - Planning status: {bag_contents.planning_status_code}")
    if len(bag_contents.optimized_sparse_path.poses) < 2:
        print("Not enough poses in path")
        return
    spline_analyser = SplineAnalyzer(
        bag_contents.optimized_sparse_path, bag_contents.spline_tangent_lengths, 2000
    )
    distances = spline_analyser.calc_distances_to_obstacle(bag_contents.map_data)
    figure = plt.figure("Distance to obstacle", figsize=(5, 2.7))
    ax = figure.subplots(1, 1)
    ax.plot(
        spline_analyser.path_distance, distances, "-", label="Distanz zu Hindernissen"
    )
    ax.set_xlabel("Pfad [m]")
    ax.set_ylabel("Abstand [m]")
    ax.set_yticks(np.arange(2.0, 4.0, 0.5))

    # ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.grid(axis="both", which="major")
    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")
    figure.tight_layout()
    figure.savefig("data/plots/formation_distance_to_obstacle.pdf")

    # pdb.set_trace()
    spline_analyser.offset((0.0, -1.0))
    spline_analyser.offset((0.0, 1.0))
    # spline_analyser.plot_paths()
    figure = plt.figure("Path", figsize=(5, 3))
    ax = figure.subplots(1, 1)
    ax.plot(
        spline_analyser.samples[1],
        spline_analyser.samples[0],
        "-",
        label="Formationszentrum",
    )
    for i, off_vals in enumerate(spline_analyser.offsets):
        ax.plot(off_vals[0][1], off_vals[0][0], "-", label=f"Roboter {i + 1}")

    ax.plot(
        [5.0, -2.0],
        [-10, -10],
        "-",
        color="black",
        linewidth=4,
        label="Hindernisse",
        zorder=1,
    )
    circle = plt.Circle((5.0, -10.0), 0.5, color="black")
    ax.add_artist(circle)
    circle = plt.Circle((-2.0, -10.0), 0.5, color="black")
    ax.add_artist(circle)
    rect = plt.Rectangle((5.0, -10.5), -7.0, 1.0, angle=0, color="black")
    ax.add_artist(rect)
    ax.plot(
        [-2.15, -6.75],
        [-3.85, -9.25],
        "-",
        color="black",
        linewidth=4,
        zorder=1,
    )
    circle = plt.Circle((-2.15, -3.85), 0.5, color="black")
    ax.add_artist(circle)
    circle = plt.Circle((-6.75, -9.25), 0.5, color="black")
    ax.add_artist(circle)
    rect = plt.Rectangle((-2.15, -4.35), -1.0, -6.75, angle=-39, color="black")
    ax.add_artist(rect)
    ax.plot(
        [-10.9, -5.8],
        [-12, -18],
        "-",
        color="black",
        linewidth=4,
        zorder=0,
    )
    rect = plt.Rectangle((-10.55, -11.65), -1.0, -6.75, angle=40, color="black")
    ax.add_artist(rect)
    ax.set_ylim(-14, -4.5)
    ax.set_xlim(-10, 6.5)
    ax.set_aspect("equal")
    ax.invert_xaxis()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(axis="both", which="major")
    ax.legend()
    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")
    figure.tight_layout()
    figure.savefig("data/plots/formation_path.pdf")
    # plt.show()
    # spline_analyser.plot_angles()
    # spline_analyser.plot_angles_der()
    figure_ang = plt.figure("Angles", figsize=(5.9, 3.15))
    ax_angs = figure_ang.subplots(2, 1, sharex=True)
    ax_angs[0].plot(
        spline_analyser.path_distance, spline_analyser.angles, label="Formationszentrum"
    )
    for i, off_vals in enumerate(spline_analyser.offsets):
        ax_angs[0].plot(
            spline_analyser.path_distance, off_vals[1], label=f"Roboter {i + 1}"
        )
    ax_angs[0].legend()
    # ax_angs[0].set_xlabel("Pfad [m]")
    ax_angs[0].set_ylabel("Orientierung [rad]")

    # figure_ang = plt.figure("Angles Derivative")
    # ax_ang = figure_ang.subplots(1, 1)

    ax_angs[1].plot(
        spline_analyser.path_distance[:-1],
        spline_analyser.angles_der,
        label="Formationszentrum",
    )
    for i, off_vals in enumerate(spline_analyser.offsets):
        ax_angs[1].plot(
            spline_analyser.path_distance[:-1], off_vals[2], label=f"Roboter {i + 1}"
        )
    # ax_angs[1].legend()
    ax_angs[1].set_xlabel("Pfad [m]")
    ax_angs[1].set_ylabel("Winkelgeschwingidkeit [rad/m]")
    ax_angs[0].grid(axis="both", which="major")
    ax_angs[1].grid(axis="both", which="major")
    ax_angs[0].spines["top"].set_color("gray")
    ax_angs[0].spines["right"].set_color("gray")
    ax_angs[1].spines["top"].set_color("gray")
    ax_angs[1].spines["right"].set_color("gray")
    figure_ang.tight_layout()
    figure_ang.savefig("data/plots/formation_angles.pdf")
    # spline_analyser.plot_angles_der2()
    # spline_analyser.plot_curvatures()
    # plt.show()
    # pdb.set_trace()


def main():
    single_formation_path()


if __name__ == "__main__":
    main()
