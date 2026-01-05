#!/usr/bin/env python3
"""
pcl2texture.py  —  Build the offline texture map and databases
--------------------------------------------------------------
This script implements exactly the mathematical procedure described
in Section 4.2 “Building the Offline Database”.

Inputs:
    --cloud <cloud.csv>         3D points (x,y,z)
    --midpath <midpath.csv>     mid-path waypoints (x,y,z)
    --delta_s <float>           horizontal bin size
    --delta_z <float>           vertical bin size
    --out_map <offline_map.npy>
    --out_points <offline_points.npy>

Outputs:
    offline_map.npy      — dict[(row,col)] = (gray_value, np.ndarray(point_indices))
    offline_points.npy   — 3D points array (N,3)

This output format is fully compatible with:
 - localization_2d.py (SIFT / LBP matching)
 - main.py (online localization + ICP)
"""

import argparse
import numpy as np
from scipy.spatial import cKDTree
import pickle
import sys

def reconstruct_image_from_mapping(mapping):
    # mapping[(row, col)] = (gray, idx_array)
    rows = [r for (r, c) in mapping.keys()]
    cols = [c for (r, c) in mapping.keys()]
    H = max(rows) + 1
    W = max(cols) + 1
    img = np.zeros((H, W), dtype=np.uint8)
    for (r, c), (gray, _) in mapping.items():
        img[r, c] = np.uint8(gray)
    return img

# --------------------------------------------------------------------------
# Utility: project point to polyline (mid-path)
# --------------------------------------------------------------------------

def proj_point_to_polyline(px, py, Q):
    """
    Project point (px,py) onto the polyline Q (Nx2 array).
    Returns:
        s_star : arc-length coordinate
        idx    : index of segment used
    """
    min_dist = 1e12
    best_s = 0.0
    best_idx = 0

    # Precompute segment lengths and cumulative arc-lengths
    seg = Q[1:] - Q[:-1]                 # (#seg, 2)
    seg_len = np.linalg.norm(seg, axis=1)
    cum = np.concatenate(([0], np.cumsum(seg_len)))

    for i in range(len(seg)):
        A = Q[i]
        B = Q[i+1]
        AB = B - A
        AB2 = AB.dot(AB)
        if AB2 < 1e-12:
            continue

        t = ((px - A[0]) * AB[0] + (py - A[1]) * AB[1]) / AB2
        t_clamped = max(0.0, min(1.0, t))
        proj = A + t_clamped * AB
        dist = (proj[0] - px)**2 + (proj[1] - py)**2

        if dist < min_dist:
            min_dist = dist
            best_idx = i
            best_s = cum[i] + t_clamped * seg_len[i]

    return best_s, best_idx


# --------------------------------------------------------------------------
# Compute tangent and normal along mid-path
# --------------------------------------------------------------------------
def compute_tangent_normals(Q):
    seg = Q[1:] - Q[:-1]
    T = seg / (np.linalg.norm(seg, axis=1, keepdims=True) + 1e-12)
    # replicate last tangent for convenience
    T = np.vstack([T, T[-1]])
    # Horizontal normal (rotate +90°)
    N = np.zeros_like(T)
    N[:,0] = -T[:,1]
    N[:,1] =  T[:,0]
    return T, N


# --------------------------------------------------------------------------
# MAIN BUILD FUNCTION
# --------------------------------------------------------------------------
def build_offline_database(cloud, Q3d, delta_s, delta_z):
    """
    cloud: (N,3) array
    Q3d: (M,3) array mid-path waypoints
    Output: (map_uint8, mapping_dict)
    """
    # Mid-path projection only uses XY
    Q = Q3d[:, :2]

    # Precompute arc-lengths
    seg = Q[1:] - Q[:-1]
    seg_len = np.linalg.norm(seg, axis=1)
    cum_s = np.concatenate(([0], np.cumsum(seg_len)))
    L = cum_s[-1]

    # Precompute tangents + normals
    T, N = compute_tangent_normals(Q)

    N_pts = cloud.shape[0]
    s_star  = np.zeros(N_pts)
    z_vals  = cloud[:,2]
    depth   = np.zeros(N_pts)

    # Project each point to mid-path (with progress)
    print("[BUILD] projecting points to mid-path...")
    last_pct = -1
    for i, (x,y,z) in enumerate(cloud):
        s_i, idx = proj_point_to_polyline(x, y, Q)
        s_star[i] = s_i

        # vector from mid-path point to p
        if idx >= len(Q3d):
            idx = len(Q3d)-1
        mp = Q3d[idx]
        r = cloud[i] - mp

        # depth = r · n_hat
        depth[i] = r[0]*N[idx,0] + r[1]*N[idx,1]

        # progress every 1%
        pct = int((i + 1) * 100 / N_pts)
        if pct != last_pct and pct % 1 == 0:
            sys.stdout.write(f"\r  {pct:3d}%")
            sys.stdout.flush()
            last_pct = pct
    sys.stdout.write("\n")

    # Determine bin extents
    s_min = 0.0
    s_max = L
    z_min = np.percentile(z_vals, 5)
    z_max = np.percentile(z_vals, 95)

    Ns = int(np.floor((s_max - s_min) / delta_s))
    Nz = int(np.floor((z_max - z_min) / delta_z))

    # Create mapping structure
    mapping = {}  # (row,col): (gray, indices_array)

    # First pass: assign points to bins
    bins = {}
    for i in range(N_pts):
        si = s_star[i]
        zi = z_vals[i]
        ui = int((si - s_min) / delta_s)
        vi = int((zi - z_min) / delta_z)

        if ui < 0 or ui >= Ns or vi < 0 or vi >= Nz:
            continue

        if (ui,vi) not in bins:
            bins[(ui,vi)] = []
        bins[(ui,vi)].append(i)

    # Compute raw depths, median per pixel
    raw = np.full((Nz, Ns), np.nan, dtype=float)

    for (ui,vi), indices in bins.items():
        dvals = depth[indices]
        raw[vi, ui] = np.median(dvals)

    # Percentile normalization (clip extremes)
    valid = raw[~np.isnan(raw)]
    pmin = np.percentile(valid, 5)
    pmax = np.percentile(valid, 95)

    img = np.zeros_like(raw)
    img[:] = 0  # initialize

    for (ui,vi), indices in bins.items():
        d = raw[vi,ui]
        d_norm = (d - pmin) / (pmax - pmin + 1e-12)
        gray = int(np.clip(255 * d_norm, 0, 255))
        img[vi,ui] = gray

        # NOTE: use (row, col) = (vi, ui), and store as (gray, indices_array)
        mapping[(vi,ui)] = (gray, np.array(indices, dtype=int))

    return img.astype(np.uint8), mapping


# --------------------------------------------------------------------------
# ENTRY POINT
# --------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud", default="../data/mapping_run/wall_points_bag.csv", required=True)
    parser.add_argument("--midpath", default="../data/mapping_run/wall_midpath_bag.csv", required=True)
    parser.add_argument("--delta_s", type=float, default=0.005, required=True)
    parser.add_argument("--delta_z", type=float, default=0.005, required=True)
    parser.add_argument("--out_map", default="../data/mapping_run/offline_map.npy")
    parser.add_argument("--out_points", default="../data/mapping_run/offline_points.npy")
    args = parser.parse_args()

    print("[LOAD] cloud:", args.cloud)
    cloud = np.loadtxt(args.cloud, delimiter=",")

    print("[LOAD] midpath:", args.midpath)
    mid = np.loadtxt(args.midpath, delimiter=",")

    print("[BUILD] constructing offline database …")
    map_img, mapping = build_offline_database(
        cloud, mid,
        delta_s=args.delta_s,
        delta_z=args.delta_z
    )

    # Save mapping dict as offline_map.npy
    print("[SAVE] map (mapping dict) →", args.out_map)
    np.save(args.out_map, mapping)

    # Save full 3D cloud as offline_points.npy
    print("[SAVE] points (3D cloud) →", args.out_points)
    np.save(args.out_points, cloud)

    print("Done.")


if __name__ == "__main__":
    main()
