#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import pickle
import numpy as np
import rospy

from pathlib import Path

from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

import open3d as o3d

from wall_localization_demo.localization_core import (
    lbp_localize_query,
    sift_localize_query,
)

def reconstruct_image_from_mapping(mapping):
    """
    mapping: dict with keys (row,col) -> (gray_value, point_indices_array)
    returns: float32 image in [0,1] and the mapping itself
    """
    if not mapping:
        raise ValueError("Empty mapping provided.")
    keys = list(mapping.keys())
    rows = [int(k[0]) for k in keys]
    cols = [int(k[1]) for k in keys]
    H = max(rows) + 1
    W = max(cols) + 1
    img8 = np.zeros((H, W), dtype=np.uint8)
    for (r, c), val in mapping.items():
        gray = val[0] if isinstance(val, (tuple, list)) else val
        img8[int(r), int(c)] = np.clip(int(gray), 0, 255)
    return img8.astype(np.float32) / 255.0, mapping


def pc2_to_numpy(msg, fields=("x", "y", "z")):
    """Convert PointCloud2 to (N,3) numpy array."""
    pts = []
    for p in pc2.read_points(msg, field_names=fields, skip_nans=True):
        pts.append([p[0], p[1], p[2]])
    if not pts:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(pts, dtype=np.float32)


def transform_points(transform, pts):
    """Apply geometry_msgs/TransformStamped to Nx3 points."""
    if pts.size == 0:
        return pts

    t = transform.transform.translation
    q = transform.transform.rotation
    T = tf_conversions.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z

    pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
    pts_tf = (T @ pts_h.T).T
    return pts_tf[:, :3]


class ScannerPoseEstimator(object):
    def __init__(self):
        # localization params are now only via ROS params / YAML; no set_loc2d_config
        self.loc2d_params = rospy.get_param("~loc2d", {})

        # ---------- parameters ----------
        # Offline data
        self.offline_map_npy = rospy.get_param("~offline_map_npy", "offline_map.npy")
        self.offline_points_npy = rospy.get_param(
            "~offline_points_npy", "offline_points.npy"
        )
        self.offline_lbp_db_pkl = rospy.get_param(
            "~offline_lbp_db_pkl", "offline_lbp_db.pkl"
        )
        self.offline_sift_db_pkl = rospy.get_param(
            "~offline_sift_db_pkl", "offline_sift_db.pkl"
        )

        # Frames
        self.wall_frame = rospy.get_param("~wall_frame", "wall")
        self.scanner_frame = rospy.get_param("~scanner_frame", "vertical_profiler_link")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.icp_child_frame = rospy.get_param(
            "~icp_child_frame", "scanner_icp_refined"
        )

        # Live patch extents (in wall frame)
        self.s_min_live = float(rospy.get_param("~s_min_live", -1.0))
        self.s_max_live = float(rospy.get_param("~s_max_live", 1.0))
        self.z_min_live = float(rospy.get_param("~z_min_live", 0.0))
        self.z_max_live = float(rospy.get_param("~z_max_live", 0.6))
        self.live_patch_s_length = float(
            rospy.get_param("~live_patch_s_length", 0.20)
        )  # m

        # Method selection: "lbp", "sift" or "both"
        self.method = rospy.get_param("~method", "lbp")

        # ICP params
        self.icp_max_dist = float(rospy.get_param("~icp_max_dist", 0.05))  # m
        self.icp_max_iter = int(rospy.get_param("~icp_max_iter", 50))

        # Live image resolution (patch size for online localization)
        self.live_img_h = int(rospy.get_param("~live_img_h", 11))  # default matches YAML
        self.live_img_w = int(rospy.get_param("~live_img_w", 6))

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # ---------- load offline map ----------
        rospy.loginfo("Loading offline map from %s", self.offline_map_npy)
        mapping = np.load(self.offline_map_npy, allow_pickle=True).item()
        self.offline_img, self.pixel_to_indices = reconstruct_image_from_mapping(
            mapping
        )
        self.offline_img = self.offline_img.astype(np.float32)

        # Try to load offline 3D points (same indexing as mapping values)
        if os.path.exists(self.offline_points_npy):
            rospy.loginfo("Loading offline points from %s", self.offline_points_npy)
            self.offline_points = np.load(self.offline_points_npy)
        else:
            rospy.logwarn(
                "Offline points file %s not found. ICP will use approximate pixel grid.",
                self.offline_points_npy,
            )
            self.offline_points = None

        # ---------- build / load LBP database ----------
        if os.path.exists(self.offline_lbp_db_pkl):
            rospy.loginfo("Loading LBP-HF DB from %s", self.offline_lbp_db_pkl)
            with open(self.offline_lbp_db_pkl, "rb") as f:
                self.lbp_db = pickle.load(f)
        else:
            rospy.logwarn("LBP DB file %s not found; online node expects it to exist.", self.offline_lbp_db_pkl)
            self.lbp_db = None

        # ---------- build / load SIFT database ----------
        if os.path.exists(self.offline_sift_db_pkl):
            rospy.loginfo("Loading SIFT DB from %s", self.offline_sift_db_pkl)
            with open(self.offline_sift_db_pkl, "rb") as f:
                self.sift_db = pickle.load(f)
        else:
            rospy.logwarn("SIFT DB file %s not found; SIFT mode will be disabled.", self.offline_sift_db_pkl)
            self.sift_db = None

        # ---------- live data buffers ----------
        self.acc_pts_wall = []  # list of (N,3) in wall frame

        # ---------- subscribers ----------
        pc_topic = rospy.get_param("~pointcloud_topic", "/scan_cloud")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.scan_sub = rospy.Subscriber(pc_topic, LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self.odom_cb, queue_size=5
        )

        self.last_odom = None

        rospy.loginfo("ScannerPoseEstimator initialized.")

    # ------------------------------------------------------------------
    # Helper: LaserScan -> Nx3 points (scanner frame)
    # ------------------------------------------------------------------
    def laserscan_to_points(self, scan_msg):
        """Convert LaserScan to Nx3 array in the scanner frame."""
        # angles and ranges
        angles = scan_msg.angle_min + np.arange(len(scan_msg.ranges)) * scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges, dtype=np.float64)

        # filter valid ranges
        mask = np.isfinite(ranges) & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32)

        ranges = ranges[mask]
        angles = angles[mask]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        pts = np.stack([xs, ys, zs], axis=1).astype(np.float32)  # (N,3)
        return pts

    # ------------------------------------------------------------------
    # Odometry callback (stored mainly for debugging / future use)
    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        self.last_odom = msg

    # ------------------------------------------------------------------
    # Scan callback: accumulate patch, localize, run ICP
    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        # msg is now a LaserScan
        pts = self.laserscan_to_points(msg)
        if pts.shape[0] == 0:
            return

        # Replace any previous PointCloud2->array conversion with pts directly.
        cloud_xyz = pts

        # 1) Convert to numpy in scanner frame
        pts_scanner = cloud_xyz
        if pts_scanner.size == 0:
            return

        # 2) Transform to wall frame
        try:
            tf_s_w = self.tf_buffer.lookup_transform(
                self.wall_frame,
                self.scanner_frame,
                msg.header.stamp,
                rospy.Duration(0.2),
            )
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s", str(e))
            return

        pts_wall = transform_points(tf_s_w, pts_scanner)

        # 3) Filter by height and lateral range in wall frame.
        #    Assumption: wall x-axis = along layer, y-axis = normal, z-axis = up.
        s_vals = pts_wall[:, 0]  # "along-wall" coordinate
        z_vals = pts_wall[:, 2]  # height

        mask = (
            (s_vals >= self.s_min_live)
            & (s_vals <= self.s_max_live)
            & (z_vals >= self.z_min_live)
            & (z_vals <= self.z_max_live)
        )

        pts_wall = pts_wall[mask]
        if pts_wall.size == 0:
            return

        self.acc_pts_wall.append(pts_wall)

        # 4) Check accumulated length along s
        all_pts = np.vstack(self.acc_pts_wall)
        s_all = all_pts[:, 0]

        s_range = s_all.max() - s_all.min()
        if s_range < self.live_patch_s_length:
            # not enough coverage yet
            return

        # Define patch window [s0, s1]
        s0 = s_all.min()
        s1 = s0 + self.live_patch_s_length
        win_mask = (s_all >= s0) & (s_all <= s1)
        live_pts_wall = all_pts[win_mask]

        # Once a patch is built, clear accumulator for the next patch
        self.acc_pts_wall = []

        # 5) Convert live points to (s,z,d) for image construction
        s_live = live_pts_wall[:, 0]
        z_live = live_pts_wall[:, 2]
        d_live = live_pts_wall[:, 1]  # wall-normal offset

        # Discretization
        Delta_s = self.live_patch_s_length / float(self.live_img_w)
        Delta_z = (self.z_max_live - self.z_min_live) / float(self.live_img_h)

        # Assign to pixels
        u = np.floor((s_live - s0) / Delta_s).astype(int)
        v = np.floor((z_live - self.z_min_live) / Delta_z).astype(int)

        # Clamp to image bounds
        u = np.clip(u, 0, self.live_img_w - 1)
        v = np.clip(v, 0, self.live_img_h - 1)

        # Aggregate depths per pixel
        depth_bins = {}
        for ui, vi, di in zip(u, v, d_live):
            key = (vi, ui)  # row, col
            if key not in depth_bins:
                depth_bins[key] = []
            depth_bins[key].append(di)

        I_raw = np.zeros((self.live_img_h, self.live_img_w), dtype=np.float32)
        I_raw[:] = 0.0

        for (vi, ui), vals in depth_bins.items():
            I_raw[vi, ui] = np.median(vals)

        # Normalize depths to [0,255]
        nonzero = I_raw != 0
        if np.any(nonzero):
            vals = I_raw[nonzero]
            I_min = np.percentile(vals, 5)
            I_max = np.percentile(vals, 95)
            if I_max > I_min:
                I_norm = (I_raw - I_min) / (I_max - I_min)
            else:
                I_norm = np.zeros_like(I_raw)
        else:
            I_norm = np.zeros_like(I_raw)

        patch_img = (255.0 * np.clip(I_norm, 0.0, 1.0)).astype(np.uint8)

        # ------------------------------------------------------------------
        # NEW: Cropping logic to enforce patch size consistency
        # ------------------------------------------------------------------
        H_live, W_live = patch_img.shape

        # use configured patch size
        patch_h = self.live_img_h
        patch_w = self.live_img_w

        if H_live < patch_h or W_live < patch_w:
            rospy.logwarn(
                "Live patch (%d × %d) is smaller than required (%d × %d). Skipping patch.",
                H_live, W_live, patch_h, patch_w,
            )
            return

        # Center-crop to match offline patch size
        h0 = (H_live - patch_h) // 2
        w0 = (W_live - patch_w) // 2
        patch_img = patch_img[h0:h0 + patch_h, w0:w0 + patch_w]

        # Optional: log cropping for debugging
        rospy.loginfo(
            "Cropped live patch from (%d × %d) to (%d × %d)",
            H_live, W_live, patch_h, patch_w
        )

        rospy.loginfo("Built live patch image, starting 2D localization...")

        match_pos = None

        if self.method in ("lbp", "both") and self.lbp_db is not None:
            try:
                est_lbp = lbp_localize_query(
                    patch_img.astype(np.float32) / 255.0,
                    self.offline_img,
                    self.lbp_db,
                )
                if est_lbp is not None:
                    match_pos = est_lbp
                    rospy.loginfo("LBP-HF match at (row=%d, col=%d)", est_lbp[0], est_lbp[1])
            except Exception as e:
                rospy.logwarn("LBP localization failed: %s", str(e))

        if self.method in ("sift", "both") and self.sift_db is not None:
            try:
                est_sift = sift_localize_query(
                    patch_img.astype(np.float32) / 255.0,
                    self.offline_img,
                    self.sift_db,
                )
                if est_sift is not None:
                    match_pos = est_sift
                    rospy.loginfo("SIFT match at (row=%d, col=%d)", est_sift[0], est_sift[1])
            except Exception as e:
                rospy.logwarn("SIFT localization failed: %s", str(e))

        if match_pos is None:
            rospy.logwarn("No successful 2D localization for this patch.")
            return

        self.run_icp_and_report_pose(live_pts_wall, match_pos, msg.header.stamp)

    # ------------------------------------------------------------------
    def collect_offline_patch_points(self, match_pos):
        """
        Collect 3D points from offline map corresponding to the matched patch.
        match_pos: (row, col) of top-left corner of patch in offline_img.
        """
        row0, col0 = match_pos
        patch_h = self.live_img_h
        patch_w = self.live_img_w
        rows = range(row0, row0 + patch_h)
        cols = range(col0, col0 + patch_w)

        idxs = []
        for r in rows:
            for c in cols:
                key = (int(r), int(c))
                if key in self.pixel_to_indices:
                    _, point_indices = self.pixel_to_indices[key]
                    idxs.extend(point_indices.tolist())

        if not idxs:
            rospy.logwarn("No offline points found for matched patch; using grid proxy.")
            return None

        idxs = np.unique(np.asarray(idxs, dtype=int))

        if self.offline_points is None:
            return None

        pts = self.offline_points[idxs]
        return pts

    # ------------------------------------------------------------------
    def run_icp_and_report_pose(self, live_pts_wall, match_pos, stamp):
        """
        Align live patch (in wall frame) with offline patch using ICP and
        publish refined scanner pose relative to the wall.
        """
        # Collect offline points
        offline_pts = self.collect_offline_patch_points(match_pos)

        if offline_pts is None or offline_pts.shape[0] < 20:
            # Fallback: approximate patch points from pixel grid
            r0, c0 = match_pos
            H, W = self.offline_img.shape
            patch_h = self.live_img_h
            patch_w = self.live_img_w
            r1 = min(r0 + patch_h, H)
            c1 = min(c0 + patch_w, W)
            ys, xs = np.mgrid[r0:r1, c0:c1]
            offline_pts = np.stack(
                [xs.ravel().astype(np.float32), np.zeros(xs.size), ys.ravel().astype(np.float32)],
                axis=1,
            )

        if live_pts_wall.shape[0] < 20:
            rospy.logwarn("Too few live points for ICP.")
            return

        # Build Open3D point clouds in wall frame
        src = o3d.geometry.PointCloud()
        dst = o3d.geometry.PointCloud()
        src.points = o3d.utility.Vector3dVector(live_pts_wall.astype(np.float64))
        dst.points = o3d.utility.Vector3dVector(offline_pts.astype(np.float64))

        threshold = self.icp_max_dist
        trans_init = np.eye(4)

        reg = o3d.pipelines.registration.registration_icp(
            src,
            dst,
            threshold,
            trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.icp_max_iter
            ),
        )

        T_icp = reg.transformation  # wall->wall correction

        # Current TF-based scanner pose in wall frame
        try:
            tf_w_s = self.tf_buffer.lookup_transform(
                self.wall_frame,
                self.scanner_frame,
                stamp,
                rospy.Duration(0.2),
            )
        except Exception as e:
            rospy.logwarn(
                "TF lookup for scanner pose during ICP reporting failed: %s", str(e)
            )
            return

        # Convert TF to matrix
        t = tf_w_s.transform.translation
        q = tf_w_s.transform.rotation
        T_ws = tf_conversions.transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w]
        )
        T_ws[0, 3] = t.x
        T_ws[1, 3] = t.y
        T_ws[2, 3] = t.z

        # Refined scanner pose: apply ICP correction
        T_ws_refined = T_icp @ T_ws

        # Publish as TF
        T = T_ws_refined
        trans = TransformStamped()
        trans.header.stamp = stamp
        trans.header.frame_id = self.wall_frame
        trans.child_frame_id = self.icp_child_frame
        trans.transform.translation.x = T[0, 3]
        trans.transform.translation.y = T[1, 3]
        trans.transform.translation.z = T[2, 3]

        q_ref = tf_conversions.transformations.quaternion_from_matrix(T)
        trans.transform.rotation.x = q_ref[0]
        trans.transform.rotation.y = q_ref[1]
        trans.transform.rotation.z = q_ref[2]
        trans.transform.rotation.w = q_ref[3]

        self.tf_broadcaster.sendTransform(trans)

        rospy.loginfo(
            "ICP refined scanner pose: trans = (%.3f, %.3f, %.3f)",
            T[0, 3],
            T[1, 3],
            T[2, 3],
        )

    # ------------------------------------------------------------------


def main():
    rospy.init_node("scanner_pose_estimator")
    node = ScannerPoseEstimator()
    rospy.loginfo("ScannerPoseEstimator node running.")
    rospy.spin()


if __name__ == "__main__":
    main()
