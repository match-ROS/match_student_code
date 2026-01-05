#!/usr/bin/env python3
"""
Simple viewer: load point cloud from CSV and visualize with Open3D.

CSV formats supported:
- x,y,z
- If header exists, columns named x,y,z will be used.
"""
import argparse
import sys
import os

import numpy as np
try:
    import pandas as pd
    _HAS_PANDAS = True
except Exception:
    _HAS_PANDAS = False

import open3d as o3d

def load_csv(path, delim):
    if _HAS_PANDAS:
        try:
            df = pd.read_csv(path, sep=delim, engine="python")
            # look for columns by name
            cols = [c.lower() for c in df.columns]
            if set(['x','y','z']).issubset(cols):
                xcol = df.columns[cols.index('x')]
                ycol = df.columns[cols.index('y')]
                zcol = df.columns[cols.index('z')]
                pts = df[[xcol,ycol,zcol]].to_numpy(dtype=float)
                return _filter_finite(pts)
            else:
                arr = df.to_numpy(dtype=float)
        except Exception:
            arr = np.genfromtxt(path, delimiter=delim)
    else:
        arr = np.genfromtxt(path, delimiter=delim)
    arr = np.asarray(arr)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 3:
        raise ValueError("CSV must contain at least 3 columns for x,y,z")
    pts = arr[:,0:3].astype(float)
    return _filter_finite(pts)

def _filter_finite(pts):
    # Remove rows that contain NaN or inf in any coordinate
    mask = np.isfinite(pts).all(axis=1)
    if not np.all(mask):
        removed = np.count_nonzero(~mask)
        # minimal informative warning to stderr
        print(f"Warning: removed {removed} rows containing NaN/inf", file=sys.stderr)
    pts = pts[mask]
    if pts.size == 0:
        raise ValueError("No valid points found after removing NaN/inf rows")
    return pts

def build_pcd(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd

def visualize(pcd, point_size=1.0, window_name="PointCloud"):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)
    vis.add_geometry(pcd)
    opt = vis.get_render_option()
    opt.point_size = float(point_size)
    opt.background_color = np.asarray([0,0,0])
    vis.run()
    vis.destroy_window()

def main():
    parser = argparse.ArgumentParser(description="Visualize point cloud from CSV using Open3D")
    parser.add_argument("csv", help="Path to CSV file")
    parser.add_argument("--delimiter", "-d", default=",", help="CSV delimiter (default ',')")
    parser.add_argument("--downsample", "-v", type=float, default=0.0,
                        help="Voxel size for voxel_down_sample (0 = no downsample)")
    parser.add_argument("--point-size", type=float, default=2.0, help="Point size for rendering")
    parser.add_argument("--save", "-s", help="Optional: save visualized point cloud to PLY")
    args = parser.parse_args()

    if not os.path.isfile(args.csv):
        print("File not found:", args.csv, file=sys.stderr)
        sys.exit(1)

    try:
        pts = load_csv(args.csv, args.delimiter)
    except Exception as e:
        print("Failed to load CSV:", e, file=sys.stderr)
        sys.exit(1)

    pcd = build_pcd(pts)
    if args.downsample and args.downsample > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size=args.downsample)

    if args.save:
        try:
            o3d.io.write_point_cloud(args.save, pcd)
            print("Saved:", args.save)
        except Exception as e:
            print("Failed to save:", e, file=sys.stderr)

    visualize(pcd, point_size=args.point_size, window_name=os.path.basename(args.csv))

if __name__ == "__main__":
    main()