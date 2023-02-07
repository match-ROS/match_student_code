#!/usr/bin/env python3
import pdb

import matplotlib.pyplot as plt
import numpy as np

# from analyze_bagfile import MakeNavPlanWithStatsBagContents, PathAnalyzer
from bagfile_helpers import MakeNavPlanWithStatsBagContents
from path_helpers import PathAnalyzer


def get_success_and_time_from_bag(filename: str):
    """
    Reads success and planning time from bagfile.

    Args:
        filename: name of bagfile with data from MakeNavPlanWithStats as string

    Returns:
        Tuple: if planning was successful and planning time
    """
    bag_contents = MakeNavPlanWithStatsBagContents(filename, skip_maps=True)
    planning_time = bag_contents.total_time
    success = bag_contents.planning_status_code >= 0
    return success, planning_time


def get_success_rate_and_time(algorithm: str):
    """
    Get success rate and mean time of specififed optimization algorithm.

    Args:
        algorithm: name of optimization algorithm (BOBYQA, COBYLA, Nelder-Mead, NEWOUA, PRAXIS, SBPLX)

    Returns:
        Tuple of success quota, mean planning time, median planning time, indeces where planning failed

    """
    planning_times = []
    success_count = 0
    num_bags = 100
    failed_idx = []
    for idx in range(num_bags):
        filename = f"data/test_planner/testarena/{algorithm}/plan_{idx:05d}.bag"
        success, planning_time = get_success_and_time_from_bag(filename)
        if success:
            planning_times.append(planning_time)
            success_count += 1
        else:
            failed_idx.append(idx)
        filename = f"data/test_planner/Säulen/{algorithm}/plan_{idx:05d}.bag"
        success, planning_time = get_success_and_time_from_bag(filename)
        if success:
            planning_times.append(planning_time)
            success_count += 1
        else:
            failed_idx.append(idx)
    success_rate = success_count / (num_bags * 2)
    mean_time = np.mean(planning_times)
    median_time = np.median(planning_times)
    return success_rate, mean_time, median_time, failed_idx


def compare_algorithms():
    """Prints stats of different optimization algorithms."""
    algorithms = ["BOBYQA", "COBYLA", "Nelder-Mead", "NEWUOA", "PRAXIS", "SBPLX"]
    failed_idxs = []
    for algorithm in algorithms:
        # print("================")
        success_rate, mean_time, median_time, failed_idx = get_success_rate_and_time(
            algorithm
        )
        failed_idxs.append(failed_idx)
        print(
            f"{algorithm}: Success Rate: {success_rate * 100}%, Mean Time: {mean_time}, Median Time: {median_time}"
        )
    print("========")


def main():
    compare_algorithms()


if __name__ == "__main__":
    main()
