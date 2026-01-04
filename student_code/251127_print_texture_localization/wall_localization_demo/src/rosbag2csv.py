#!/usr/bin/env python3
import rosbag
import argparse
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def extract_xyz(msg):
    """
    Convert PointCloud2 to Nx3 numpy array without ros_numpy.
    """
    points = []

    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])

    return np.array(points, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Export PointCloud2 from rosbag to CSV (no ros_numpy)")
    parser.add_argument("bag", help="Path to rosbag file (.bag)")
    parser.add_argument("--topic", default=None, help="PointCloud2 topic to extract")
    parser.add_argument("--out", default="cloud.csv", help="Output CSV file name")
    args = parser.parse_args()

    print(f"Opening bag: {args.bag}")
    bag = rosbag.Bag(args.bag)

    # detect PointCloud2 topics
    info = bag.get_type_and_topic_info()[1]
    available = [t for t, v in info.items() if v.msg_type == "sensor_msgs/PointCloud2"]

    if len(available) == 0:
        print("No PointCloud2 topics found in this bag.")
        return

    if args.topic is None:
        print("Available PointCloud2 topics:")
        for t in available:
            print(" ", t)
        args.topic = available[0]
        print(f"Using default topic: {args.topic}")
    else:
        if args.topic not in available:
            print(f"Topic {args.topic} is not a PointCloud2 topic.")
            return

    all_points = []

    print(f"Extracting topic: {args.topic}")
    for _, msg, _ in bag.read_messages(topics=[args.topic]):
        pts = extract_xyz(msg)
        if pts.size > 0:
            all_points.append(pts)

    bag.close()

    if len(all_points) == 0:
        print("No points extracted.")
        return

    cloud = np.vstack(all_points)
    print(f"Extracted {cloud.shape[0]} points.")

    np.savetxt(args.out, cloud, delimiter=",", header="x,y,z", comments="")
    print(f"Saved CSV to: {args.out}")


if __name__ == "__main__":
    main()
