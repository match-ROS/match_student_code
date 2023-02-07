import pdb

import matplotlib.font_manager
import matplotlib.pyplot as plt
import numpy as np
from spline_helpers import (
    calc_connection_points,
    calc_control_points_for_path,
    calc_control_points_for_path_sra,
    sample_spline_path,
)


def single_quintic_bezier_curve():
    """
    Creates example of a quintic bezier curve as plot with tangents at start and end.
    """
    t_s = np.array([0.0, 2.0], dtype=np.float64)
    t_e = np.array([2.0, 0.0], dtype=np.float64)
    a_s = np.array([3.0, 0.0], dtype=np.float64)
    a_e = np.array([0.0, -3.0], dtype=np.float64)
    cp_0 = np.array([0, 0], dtype=np.float64)
    cp_5 = np.array([1, 1], dtype=np.float64)
    cp_1 = t_s / 5 + cp_0
    cp_4 = cp_5 - t_e / 5
    cp_2 = a_s / 20 + 2 * cp_1 - cp_0
    cp_3 = a_e / 20 + 2 * cp_4 - cp_5
    control_points = np.array(
        [
            cp_0,
            cp_1,
            cp_2,
            cp_3,
            cp_4,
            cp_5,
        ],
        dtype=np.float64,
    )

    samples, _, _, _ = sample_spline_path([control_points], 100)

    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )

    figure = plt.figure("Test Spline", figsize=(5, 3))
    ax = figure.subplots(1, 1)
    ax.plot(samples[0], samples[1], label="Bézier-Kurve")
    ax.plot(
        control_points.T[0], control_points.T[1], "o", label="Kontrollpunkte", zorder=3
    )
    ax.arrow(
        *cp_0, *t_s / 4, head_width=0.05, color="r", label="erste Ableitung", zorder=2
    )
    ax.arrow(*cp_5, *t_e / 4, head_width=0.05, color="r", zorder=2)
    ax.arrow(
        *cp_0, *a_s / 4, head_width=0.05, color="b", label="zweite Ableitung", zorder=2
    )
    ax.arrow(*cp_5, *a_e / 4, head_width=0.05, color="b", zorder=2)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    # ax.legend(bbox_to_anchor=(1.04, 0.5), loc="center left")
    ax.legend()

    ax.set_xticks(np.arange(0.0, 2.0, 0.5))
    ax.set_yticks(np.arange(0.0, 1.5, 0.5))
    ax.grid(axis="both", which="major")
    ax.spines["top"].set_color("gray")
    ax.spines["right"].set_color("gray")
    figure.tight_layout()

    figure.savefig("data/plots/example_bezier_curve.pdf")
    # plt.show()


def example_path():
    """
    Creates example path consisting of 6 points.

    Returns:
        np.NDArray of points (2,6)
    """
    points = np.array(
        [
            [1, 1],
            [1, 2],
            [3, 2],
            [3, 3],
            [5, 3],
            [5, 1],
        ],
        dtype=np.float64,
    ).T

    return points


def example_spline():
    """
    Creates example spline from example path and plot differences in used tangent length and wconection points.
    """
    num_samples = 100

    points = example_path()
    connection_points = calc_connection_points(points)

    # for optimal choice
    spline_tangent_lengths_optimal = np.array([3, 2.5, 1.5, 3, 5])
    # for everything else
    spline_tangent_lengths = np.ones(len(points.T) - 1)
    # for old version
    spline_tangent_lengths_sra = np.ones(len(points.T))

    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": "Computer Modern",
        }
    )

    # for optimal choice
    figure_opt = plt.figure("Path smooth optimal", figsize=(5.5, 2.7))
    figure_smooth = plt.figure("Path smooth", figsize=(5.5, 2.7))
    # for tangent length variation
    figure_sra = plt.figure("Path sra", figsize=(5.9, 1.8))
    figure_vars = plt.figure("Path tangent variation", figsize=(5.9, 2.2))

    ax_opt = figure_opt.subplots(1, 1)
    ax_smooth = figure_smooth.subplots(1, 1)
    ax_sra = figure_sra.subplots(1, 1)
    ax_vars = figure_vars.subplots(1, 1)

    # all plots contain path from waypoints
    for ax in [ax_opt, ax_smooth, ax_sra, ax_vars]:
        ax.plot(points[0], points[1], "-o", label="Pfad aus Wegpunkten")

    # my method introduces alternative connection points which are shown in plot
    for ax in [ax_opt, ax_smooth, ax_vars]:
        ax.plot(
            connection_points[0],
            connection_points[1],
            "o",
            label="Verknüpfungspunkte",
            zorder=3,
        )

    # calculate control points and sample for optimal tangent selection
    control_points_opt = calc_control_points_for_path(
        points, spline_tangent_lengths_optimal
    )
    samples_opt, _, _, _ = sample_spline_path(control_points_opt, num_samples)
    ax_opt.plot(
        samples_opt[0],
        samples_opt[1],
        "-",
        label=f"Bézier-Spline",
        zorder=1,
    )

    # calculate control points and sample for initial tangent selection
    control_points_smooth = calc_control_points_for_path(points, spline_tangent_lengths)
    samples_smooth, _, _, _ = sample_spline_path(control_points_smooth, num_samples)
    ax_smooth.plot(
        samples_smooth[0],
        samples_smooth[1],
        "-",
        label=f"Bézier-Spline",
        zorder=1,
    )

    # calculate control points and sample for multiple tangent factors of old version
    for tangent_length in [0.5, 1.0, 1.5, 2.0, 2.5]:
        control_points_sra = calc_control_points_for_path_sra(
            points, spline_tangent_lengths_sra * tangent_length
        )
        samples_sra, _, _, _ = sample_spline_path(control_points_sra, num_samples)
        ax_sra.plot(
            samples_sra[0],
            samples_sra[1],
            "-",
            label=f"Bézier-Spline (Faktor: {tangent_length})",
            zorder=1,
        )

    # calculate control points and sample for multiple tangent factors for my version
    for tangent_length in [0.5, 1.0, 1.5, 2.0, 2.5]:
        control_points_vars = calc_control_points_for_path(
            points, spline_tangent_lengths * tangent_length
        )
        samples_vars, _, _, _ = sample_spline_path(control_points_vars, num_samples)
        ax_vars.plot(
            samples_vars[0],
            samples_vars[1],
            "-",
            label=f"Bézier-Spline (Faktor: {tangent_length})",
            zorder=1,
        )

    # calculate tangent vectors and add them as arrows in initial smooth path
    tangents = np.diff(points.T, axis=0)
    tangents /= np.sqrt(np.sum(tangents**2, axis=1)).reshape(-1, 1)

    for conn_point, tangent in zip(connection_points.T, tangents):
        arrow = ax_smooth.arrow(
            *conn_point, *tangent / 4, width=0.01, head_width=0.05, color="r", zorder=2
        )
    arrow.set_label("Tangente")

    # formatting of plot is similar for every plot
    for ax in [ax_opt, ax_smooth, ax_sra, ax_vars]:
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_xticks(np.arange(1.0, 6.0, 1.0))
        ax.set_yticks(np.arange(1.0, 4.0, 1.0))
        ax.grid(axis="both", which="major")
        ax.spines["top"].set_color("gray")
        ax.spines["right"].set_color("gray")
    # for tangent variation the legend is on right side of plot
    for ax, figure in zip([ax_sra, ax_vars], [figure_sra, figure_vars]):
        ax.legend(bbox_to_anchor=(1.04, 0.5), loc="center left")
        figure.subplots_adjust(right=0.55)
    # for single line standard legend can be used
    for ax in [ax_opt, ax_smooth]:
        ax.legend()

    # make all figures fit tightly
    for figure in [figure_opt, figure_smooth, figure_vars, figure_sra]:
        figure.tight_layout()
    # save under different names
    figure_opt.savefig("data/plots/path_smooth_optimal_lengths.pdf")
    figure_vars.savefig("data/plots/paths_tangent_length.pdf")
    figure_smooth.savefig("data/plots/path_smooth.pdf")
    figure_sra.savefig("data/plots/path_smooth_old.pdf")

    # plt.show()


def main():
    single_quintic_bezier_curve()
    example_spline()


if __name__ == "__main__":
    main()
