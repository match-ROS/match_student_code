#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf_conversions


def pc2_to_numpy(msg):
    pts = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pts.append([p[0], p[1], p[2]])
    return np.asarray(pts, dtype=np.float32) if pts else np.zeros((0,3))


class WallScanRecorder:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.out_cloud = rospy.get_param("~out_cloud_csv", "wall_points_gazebo.csv")
        self.out_midpath = rospy.get_param("~out_midpath_csv", "wall_midpath_gazebo.csv")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Output buffers
        self.points_world = []
        self.midpath_world = []

        # LaserScan → PointCloud2 converter
        self.lp = LaserProjection()

        # Subscribe to LaserScan
        topic = rospy.get_param("~scan_topic", "/scan_cloud")
        self.sub = rospy.Subscriber(topic, LaserScan, self.laser_cb, queue_size=1)
        rospy.loginfo("Listening to LaserScan: %s", topic)

        rospy.on_shutdown(self.on_shutdown)

    def transform_points(self, transform, pts):
        if pts.size == 0:
            return pts
        t = transform.transform.translation
        q = transform.transform.rotation

        T = tf_conversions.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = [t.x, t.y, t.z]

        pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
        pts_tf = (T @ pts_h.T).T
        return pts_tf[:, :3]

    # -------------------
    # LaserScan callback
    # -------------------
    def laser_cb(self, scan_msg):
        rospy.loginfo_once("Laser messages are arriving.")

        # Detect LaserScan frame automatically
        laser_frame = scan_msg.header.frame_id
        if not laser_frame:
            rospy.logwarn_once("LaserScan has NO frame_id!")
            return

        # Convert LaserScan → PointCloud2
        try:
            cloud_msg = self.lp.projectLaser(scan_msg)
        except Exception as e:
            rospy.logwarn("Laser→PointCloud2 conversion failed: %s", e)
            return

        # Convert msg → numpy
        pts_scanner = pc2_to_numpy(cloud_msg)
        if pts_scanner.size == 0:
            rospy.logwarn_throttle(2, "LaserScan produced 0 points.")
            return

        # Get transforms:
        # 1. world ← laser_frame
        # 2. world ← base_link (for mid-path)
        try:
            tf_w_s = self.tf_buffer.lookup_transform(
                self.world_frame,
                laser_frame,                 # <-- THIS FIXES YOUR SCRIPT
                scan_msg.header.stamp,
                rospy.Duration(0.2),
            )

            tf_w_b = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                scan_msg.header.stamp,
                rospy.Duration(0.2)
            )
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"TF lookup failed: {e}")
            return

        # Transform points → world
        pts_world = self.transform_points(tf_w_s, pts_scanner)
        self.points_world.append(pts_world)

        # Record robot mid-path → world
        t = tf_w_b.transform.translation
        self.midpath_world.append([t.x, t.y, t.z])

        rospy.loginfo_throttle(2.0,
            f"Stored frame {len(self.points_world)}, points={pts_world.shape[0]}")

    # -------------------
    # Save CSV on shutdown
    # -------------------
    def on_shutdown(self):
        rospy.logwarn("Saving CSV files...")

        # Save point cloud
        if len(self.points_world) > 0:
            pts = np.vstack(self.points_world)
            np.savetxt(self.out_cloud, pts, delimiter=",")
            rospy.logwarn(f"Saved global pointcloud → {self.out_cloud} "
                          f"({pts.shape[0]} points)")
        else:
            rospy.logerr("NO pointcloud data received.")

        # Save mid-path
        if len(self.midpath_world) > 0:
            mp = np.asarray(self.midpath_world)
            np.savetxt(self.out_midpath, mp, delimiter=",")
            rospy.logwarn(f"Saved robot path → {self.out_midpath} "
                          f"({mp.shape[0]} samples)")
        else:
            rospy.logerr("NO robot path data recorded.")


def main():
    rospy.init_node("wall_scan_recorder")
    WallScanRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
