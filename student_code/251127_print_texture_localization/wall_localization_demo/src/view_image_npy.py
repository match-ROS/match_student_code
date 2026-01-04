#!/usr/bin/env python3
"""
view_image_npy.py — visualize the offline map produced by pcl2texture.py

Usage:
  ./view_image_npy.py --map ../data/mapping_run/offline_map.npy
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt


def reconstruct_image_from_mapping(mapping):
    """
    mapping: dict[(row, col)] = (gray_value, indices_array)
    Returns: img (H,W) uint8
    """
    if not mapping:
        raise ValueError("Mapping dict is empty")

    rows = [rc[0] for rc in mapping.keys()]
    cols = [rc[1] for rc in mapping.keys()]
    H = max(rows) + 1
    W = max(cols) + 1

    img = np.zeros((H, W), dtype=np.uint8)
    for (r, c), (gray, _) in mapping.items():
        img[r, c] = np.uint8(gray)

    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map",
        default="../data/mapping_run/offline_map.npy",
        help="Path to offline_map.npy produced by pcl2texture.py/build.py",
    )
    args = parser.parse_args()

    print("[LOAD] mapping:", args.map)
    mapping = np.load(args.map, allow_pickle=True).item()

    img = reconstruct_image_from_mapping(mapping)

    H, W = img.shape
    print(f"[INFO] image shape: {H} x {W} (HxW)")

    plt.figure(figsize=(8, 4))
    plt.imshow(img, cmap="gray", origin="lower")
    plt.title(f"Offline map ({H} x {W})")
    plt.colorbar(label="Gray value")
    plt.xlabel("u (along s)")
    plt.ylabel("v (along z)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
