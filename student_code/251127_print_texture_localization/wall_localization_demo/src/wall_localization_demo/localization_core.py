#!/usr/bin/env python3
"""
Two independent localization modes:

  --method sift    : Use Zhang et al. style SIFT localization
  --method lbp     : Use LBP-HF + NCC localization

You can pass either:
    --image <path>       : a grayscale image file to use as the offline map
    --pixel_map <path>   : a .npy file saved by pcl2texture containing a mapping
                          dict[(row,col)] = (grayscale, np.ndarray(point_indices))
                          When provided, the script reconstructs the grayscale image
                          from the mapping and uses it as the offline map.

Example:
    python localization_2d.py --method lbp --pixel_map map_pixels.npy

If neither is provided, the script will attempt to load OFFLINE_MAP_PATH; it must be
either a raw numpy image array or the mapping dict created by pcl2texture. Synthetic
map generation is disabled.
"""

import numpy as np
import cv2
import argparse
import os
import time
import pickle
from pathlib import Path
import matplotlib.pyplot as plt
from skimage.feature import local_binary_pattern
from scipy.fft import fft
from scipy.signal import correlate2d
from sklearn.neighbors import NearestNeighbors
from sklearn.decomposition import PCA
import csv
import yaml   # <--- add for YAML loading

### ======================================================
### PARAMETERS (default placeholders, overwritten by YAML)
### ======================================================
PATCH_H, PATCH_W = 200, 160
STRIDE = 16
MAX_QUERY_KPS = 60
LBP_RADIUS = 2
LBP_POINTS = 8 * LBP_RADIUS
LBPHF_KEEP = 24
COARSE_TOPK = 8
SIFT_RATIO = 0.7
SIFT_PCA_DIM = 32
SCALE_BINS = 3
FLANN_TREES = 5
FLANN_CHECKS = 50

# ----------------------------------------------------------------------
# Global parameter synchronization (from YAML only)
# ----------------------------------------------------------------------

LOC2D_CONFIG = {}

def set_loc2d_config(params: dict):
    """
    Inject YAML-loaded parameters into this module.
    This safely overwrites the global constants used by all functions.
    """
    global LOC2D_CONFIG, PATCH_H, PATCH_W
    LOC2D_CONFIG = params.copy() if params else {}

    # Optionally patch size from YAML (if present)
    if "patch_h" in LOC2D_CONFIG:
        PATCH_H = int(LOC2D_CONFIG["patch_h"])
    if "patch_w" in LOC2D_CONFIG:
        PATCH_W = int(LOC2D_CONFIG["patch_w"])

    # Overwrite generic params
    globals()['STRIDE'] = LOC2D_CONFIG.get('stride', STRIDE)
    globals()['MAX_QUERY_KPS'] = LOC2D_CONFIG.get('max_query_kps', MAX_QUERY_KPS)

    # Overwrite LBP parameters
    globals()['LBP_RADIUS'] = LOC2D_CONFIG.get('lbp_radius', LBP_RADIUS)
    globals()['LBP_POINTS'] = LOC2D_CONFIG.get('lbp_points', 8 * globals()['LBP_RADIUS'])
    globals()['LBPHF_KEEP'] = LOC2D_CONFIG.get('lbphf_keep', LBPHF_KEEP)
    globals()['COARSE_TOPK'] = LOC2D_CONFIG.get('coarse_topk', COARSE_TOPK)

    # Overwrite SIFT parameters
    globals()['SIFT_RATIO'] = LOC2D_CONFIG.get('sift_ratio', SIFT_RATIO)
    globals()['SIFT_PCA_DIM'] = LOC2D_CONFIG.get('sift_pca_dim', SIFT_PCA_DIM)
    globals()['SCALE_BINS'] = LOC2D_CONFIG.get('scale_bins', SCALE_BINS)
    globals()['FLANN_TREES'] = LOC2D_CONFIG.get('flann_trees', FLANN_TREES)
    globals()['FLANN_CHECKS'] = LOC2D_CONFIG.get('flann_checks', FLANN_CHECKS)

# Default offline paths (used only when no image is provided)
OFFLINE_MAP_PATH = Path("/home/thach/project_ws/src/wall_localization_demo/data/mapping_run/offline_map.npy")
OFFLINE_LBP_DB_PATH = Path("/home/thach/project_ws/src/wall_localization_demo/data/mapping_run/offline_lbp_db.pkl")
OFFLINE_SIFT_DB_PATH = Path("/home/thach/project_ws/src/wall_localization_demo/data/mapping_run/offline_sift_db.pkl")
PARAM_YAML_PATH = Path("/home/thach/project_ws/src/wall_localization_demo/config/localization_params.yaml")

np.random.seed(1)

### ======================================================
### LBP-HF
### ======================================================
def lbphf_descriptor(patch):
    # local_binary_pattern expects integer images; convert from float [0,1] to uint8
    patch_u8 = (np.clip(patch, 0.0, 1.0) * 255.0).astype(np.uint8)
    lbp = local_binary_pattern(patch_u8, LBP_POINTS, LBP_RADIUS, method="uniform")
    n_bins = int(lbp.max() + 1)
    hist, _ = np.histogram(lbp.ravel(), bins=n_bins, range=(0, n_bins), density=True)
    hist_fft = fft(hist)
    mag = np.abs(hist_fft)
    mag_real = mag.real.astype(np.float32)
    # Keep the magnitudes of low-frequency components only to reduce descriptor size
    if mag_real.size >= LBPHF_KEEP:
        return mag_real[:LBPHF_KEEP].astype(np.float32)
    else:
        out = np.zeros((LBPHF_KEEP,), dtype=np.float32)
        out[:mag_real.size] = mag_real
        return out


def compute_map_lbphf_descriptors(img):
    H, W = img.shape
    descs = []
    poses = []
    for y in range(0, H - PATCH_H + 1, STRIDE):
        for x in range(0, W - PATCH_W + 1, STRIDE):
            patch = img[y:y+PATCH_H, x:x+PATCH_W]
            descs.append(lbphf_descriptor(patch))
            poses.append((y, x))
    return np.vstack(descs), poses


### ======================================================
### SIFT
### ======================================================
def sift_extract(image):
    img8 = (255 * image).astype(np.uint8)
    sift = cv2.SIFT_create()
    kps, desc = sift.detectAndCompute(img8, None)
    if desc is None:
        return [], None
    return kps, desc


def build_sift_database(img, params=None):
    """
    Build SIFT database on the given offline image.

    Uses global SIFT_PCA_DIM, SCALE_BINS, FLANN_TREES, FLANN_CHECKS that were
    already set by set_loc2d_config() from localization_params.yaml.
    Optional 'params' is ignored (kept only for backward compatibility).
    """
    # Extract SIFT features from the full offline map
    kps, desc = sift_extract(img)
    if desc is None or len(desc) == 0:
        raise RuntimeError("No SIFT features found.")

    positions = np.array([kp.pt for kp in kps], dtype=np.float32)
    scales = np.array([kp.size for kp in kps], dtype=np.float32)

    # PCA compression
    pca = PCA(n_components=SIFT_PCA_DIM, random_state=0)
    desc_pca = pca.fit_transform(desc.astype(np.float32))

    # Scale bins
    smin, smax = scales.min(), scales.max()
    bin_edges = np.linspace(smin, smax + 1e-6, SCALE_BINS + 1)
    bin_idx = np.digitize(scales, bin_edges) - 1
    bin_idx = np.clip(bin_idx, 0, SCALE_BINS - 1)

    nn_per_bin = {}
    index_params = dict(algorithm=1, trees=FLANN_TREES)  # KDTree
    search_params = dict(checks=FLANN_CHECKS)

    for b in range(SCALE_BINS):
        mask = bin_idx == b
        if not np.any(mask):
            continue

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        desc_bin = desc_pca[mask].astype(np.float32)
        flann.add([desc_bin])
        flann.train()

        nn_per_bin[b] = {
            "flann": flann,
            "desc_pca": desc_bin,
            "positions": positions[mask],
            "scales": scales[mask],
        }

    return {
        "pca": pca,
        "bin_edges": bin_edges,
        "nn_per_bin": nn_per_bin,
        "positions_all": positions,
        "scales_all": scales,
    }


# ----------------------- New helpers for SIFT DB serialization -----------------------
def make_savable_sift_db(sift_db):
    """
    Return a copy of sift_db suitable for pickle: remove the OpenCV FlannBasedMatcher objects.
    Keeps PCA, bin_edges and per-bin arrays (desc_pca, positions, scales).
    """
    sav = {
        "pca": sift_db["pca"],
        "bin_edges": sift_db["bin_edges"],
        "nn_per_bin": {},
        "positions_all": sift_db.get("positions_all", None),
        "scales_all": sift_db.get("scales_all", None),
    }
    for b, info in sift_db["nn_per_bin"].items():
        sav["nn_per_bin"][b] = {
            # ensure arrays are numpy arrays (float32) and pickle-friendly
            "desc_pca": np.array(info["desc_pca"], dtype=np.float32),
            "positions": np.array(info["positions"], dtype=np.float32),
            "scales": np.array(info["scales"], dtype=np.float32),
        }
    return sav


def reconstruct_sift_db(saved_db):
    """
    Reconstruct full sift_db from a saved (pickleable) representation by rebuilding Flann matchers.
    Uses current FLANN_TREES / FLANN_CHECKS globals.
    """
    if saved_db is None:
        return None
    sift_db = {
        "pca": saved_db.get("pca", None),
        "bin_edges": saved_db.get("bin_edges", None),
        "nn_per_bin": {},
        "positions_all": saved_db.get("positions_all", None),
        "scales_all": saved_db.get("scales_all", None),
    }
    index_params = dict(algorithm=1, trees=FLANN_TREES)
    search_params = dict(checks=FLANN_CHECKS)
    for b, info in saved_db.get("nn_per_bin", {}).items():
        desc_bin = info.get("desc_pca", None)
        positions = info.get("positions", None)
        scales = info.get("scales", None)
        nninfo = {
            "desc_pca": np.array(desc_bin, dtype=np.float32) if desc_bin is not None else np.empty((0,)),
            "positions": np.array(positions, dtype=np.float32) if positions is not None else np.empty((0,2)),
            "scales": np.array(scales, dtype=np.float32) if scales is not None else np.empty((0,)),
        }
        # build FLANN matcher if descriptors exist
        if nninfo["desc_pca"].size > 0:
            try:
                flann = cv2.FlannBasedMatcher(index_params, search_params)
                flann.add([nninfo["desc_pca"].astype(np.float32)])
                flann.train()
                nninfo["flann"] = flann
            except Exception:
                # In rare cases OpenCV flann may not be available/configured; leave flann out
                nninfo["flann"] = None
        else:
            nninfo["flann"] = None
        sift_db["nn_per_bin"][b] = nninfo
    return sift_db
# ----------------------- End new helpers -----------------------------------


### ======================================================
### NCC
### ======================================================
def ncc_score(A, B):
    A = A - A.mean()
    B = B - B.mean()
    if np.linalg.norm(A) < 1e-9 or np.linalg.norm(B) < 1e-9:
        return -1.0
    corr = correlate2d(A, B, mode="valid")
    return float(corr.max() / (np.linalg.norm(A) * np.linalg.norm(B)))


### ======================================================
### Localization Modes
### ======================================================
def lbp_localize(img, lbp_db, params=None):
    """
    Offline evaluation version of LBP-HF localization.

    Picks a random PATCH_H x PATCH_W query from img, then runs:
      - LBP descriptor
      - coarse KNN search in descriptor DB
      - NCC refinement around candidates
      - RANSAC over refined votes

    Returns:
        est: (row, col) estimated top-left in img (or None on failure)
        gt:  (row, col) ground-truth top-left used for query
        query: the query patch
        dist: localization error in pixels (or None if failed)
    """
    descs, poses, nn = lbp_db
    H, W = img.shape

    # choose random ground-truth patch
    y0 = np.random.randint(0, H - PATCH_H + 1)
    x0 = np.random.randint(0, W - PATCH_W + 1)
    query = img[y0:y0+PATCH_H, x0:x0+PATCH_W]

    # coarse KNN in descriptor space
    q_desc = lbphf_descriptor(query).reshape(1, -1)
    _, idxs = nn.kneighbors(q_desc, n_neighbors=COARSE_TOPK)
    candidates = [poses[i] for i in idxs[0]]

    if 'DEBUG' in globals() and DEBUG:
        print("[DEBUG LBP] q_desc.shape=", q_desc.shape)
        print("[DEBUG LBP] neighbor idxs=", idxs)
        print("[DEBUG LBP] candidate poses=", candidates)

    scored = []
    for (py, px) in candidates:
        if py + PATCH_H > H or px + PATCH_W > W:
            continue
        patch = img[py:py+PATCH_H, px:px+PATCH_W]
        score = ncc_score(patch, query)
        if score <= 0:
            continue
        scored.append((py, px, score))

    if len(scored) == 0:
        print(f"[LBP] no valid candidates for GT=({y0},{x0})")
        return None, (y0, x0), query, None

    # sort by NCC score, keep best few
    scored.sort(key=lambda x: x[2], reverse=True)
    top_n = min(6, len(scored))
    SR = STRIDE
    STEP = max(1, STRIDE // 2)

    votes = []
    for i in range(top_n):
        py0, px0, sc0 = scored[i]
        best_local_score = sc0
        best_local_pos = (py0, px0)

        for dy in range(-SR, SR + 1, STEP):
            for dx in range(-SR, SR + 1, STEP):
                ny = py0 + dy
                nx = px0 + dx
                if ny < 0 or nx < 0 or ny + PATCH_H > H or nx + PATCH_W > W:
                    continue
                patch = img[ny:ny+PATCH_H, nx:nx+PATCH_W]
                s = ncc_score(patch, query)
                if s > best_local_score:
                    best_local_score = s
                    best_local_pos = (ny, nx)

        if 'DEBUG' in globals() and DEBUG:
            print(f"[DEBUG LBP] refined candidate {i}: start=({py0},{px0}) -> best={best_local_pos} score={best_local_score}")

        votes.append((best_local_pos[0], best_local_pos[1], best_local_score))

    votes_arr = np.array([[v[0], v[1]] for v in votes], dtype=np.float32)
    n_votes = votes_arr.shape[0]
    if n_votes == 0:
        print(f"[LBP] no refined votes for GT=({y0},{x0})")
        return None, (y0, x0), query, None

    if n_votes == 1:
        est_y, est_x = votes_arr[0]
    else:
        best_inliers = None
        best_count = 0
        RANSAC_ITERS = 100
        RANSAC_THRESH = 8.0
        for _ in range(RANSAC_ITERS):
            idx = np.random.randint(0, n_votes)
            hyp = votes_arr[idx]
            dists = np.linalg.norm(votes_arr - hyp, axis=1)
            mask = dists <= RANSAC_THRESH
            cnt = int(mask.sum())
            if cnt > best_count:
                best_count = cnt
                best_inliers = votes_arr[mask]

        if best_inliers is None or best_count < 2:
            # fallback: best single refined vote
            py, px, _ = max(votes, key=lambda v: v[2])
            est_y, est_x = py, px
        else:
            mean_in = best_inliers.mean(axis=0)
            est_y = float(mean_in[0])
            est_x = float(mean_in[1])

    est_y = int(np.clip(int(round(est_y)), 0, H - PATCH_H))
    est_x = int(np.clip(int(round(est_x)), 0, W - PATCH_W))
    dist = float(np.sqrt((est_y - y0) ** 2 + (est_x - x0) ** 2))
    print(f"[LBP] GT=({y0},{x0}), EST=({est_y},{est_x}), Error={dist:.2f}px")

    return (est_y, est_x), (y0, x0), query, dist
def sift_localize(img, sift_db):
    H, W = img.shape
    y0 = np.random.randint(0, H - PATCH_H + 1)
    x0 = np.random.randint(0, W - PATCH_W + 1)
    query = img[y0:y0+PATCH_H, x0:x0+PATCH_W]

    kps_q, desc_q = sift_extract(query)
    if desc_q is None:
        print("[SIFT] No keypoints in query.")
        return None, (y0, x0), query, None

    # Limit number of query keypoints to reduce per-trial cost
    if len(kps_q) > MAX_QUERY_KPS:
        # pick top keypoints by response
        responses = np.array([kp.response for kp in kps_q], dtype=np.float32)
        top_idx = np.argsort(responses)[-MAX_QUERY_KPS:]
        top_idx = np.sort(top_idx)
        kps_q = [kps_q[i] for i in top_idx]
        desc_q = desc_q[top_idx]

    desc_q_pca = sift_db["pca"].transform(desc_q.astype(np.float32))
    votes = []

    # Diagnostic counters
    cnt_total_kp = len(kps_q)
    cnt_no_bin = 0
    cnt_no_matches = 0
    cnt_ratio_reject = 0
    cnt_scale_reject = 0
    cnt_oob = 0
    cnt_accepted = 0
    kp_reasons = []  # store small per-keypoint reason samples

    for i, kp in enumerate(kps_q):
        s = kp.size
        b = np.digitize([s], sift_db["bin_edges"]) - 1
        b = int(np.clip(b[0], 0, SCALE_BINS - 1))
        if b not in sift_db["nn_per_bin"]:
            cnt_no_bin += 1
            if len(kp_reasons) < 8:
                kp_reasons.append(("no_bin", i, float(s)))
            continue

        nninfo = sift_db["nn_per_bin"][b]

        # Number of descriptors stored in this bin
        n_fit = nninfo["desc_pca"].shape[0]
        if n_fit == 0:
            cnt_no_matches += 1
            if len(kp_reasons) < 8:
                kp_reasons.append(("empty_bin", i))
            continue

        # k neighbors to request
        k = min(3, n_fit)
        qvec = desc_q_pca[i:i+1].astype(np.float32)
        k = min(max(2, k), nninfo["desc_pca"].shape[0])
        matches = nninfo["flann"].knnMatch(qvec, k=k)

        x_q, y_q = kp.pt

        if len(matches) == 0 or len(matches[0]) == 0:
            cnt_no_matches += 1
            if len(kp_reasons) < 8:
                kp_reasons.append(("no_matches", i))
            continue

        best_matches = matches[0]
        if len(best_matches) >= 2:
            m0 = best_matches[0]
            m1 = best_matches[1]
            if m0.distance > SIFT_RATIO * m1.distance:
                cnt_ratio_reject += 1
                if len(kp_reasons) < 8:
                    kp_reasons.append(("ratio_reject", i, float(m0.distance), float(m1.distance)))
                continue
            chosen = m0
        else:
            chosen = best_matches[0]

        neigh_dist = float(chosen.distance)
        idx = int(chosen.trainIdx)
        x_db, y_db = nninfo["positions"][idx]
        s_db = float(nninfo["scales"][idx]) if "scales" in nninfo else 0.0

        # scale similarity check (avoid matching wildly different scales)
        scale_tol = 1.8
        if kp.size > 0 and s_db > 0:
            ratio = s_db / kp.size
            if ratio < 1.0/scale_tol or ratio > scale_tol:
                cnt_scale_reject += 1
                if len(kp_reasons) < 8:
                    kp_reasons.append(("scale_reject", i, float(ratio)))
                continue

        # Predicted top-left of the query patch in the map is db_kp - query_kp
        est_y_raw = float(y_db - y_q)
        est_x_raw = float(x_db - x_q)

        # Discard votes outside the valid top-left range
        if est_y_raw < 0 or est_x_raw < 0 or est_y_raw > (H - PATCH_H) or est_x_raw > (W - PATCH_W):
            cnt_oob += 1
            if len(kp_reasons) < 8:
                kp_reasons.append(("oob", i, est_y_raw, est_x_raw))
            continue

        est_y = float(est_y_raw)
        est_x = float(est_x_raw)

        w_dist = 1.0 / (neigh_dist + 1e-6)
        w_scale = 1.0 - abs(1.0 - (s_db / (kp.size + 1e-9)))
        weight = max(0.0, w_dist * w_scale)

        votes.append((est_y, est_x, weight, x_db, y_db, x_q, y_q, neigh_dist))
        cnt_accepted += 1

    # If no votes, try a global fallback search (concatenate all bin descriptors) with looser checks
    if not votes:
        print("[SIFT] No SIFT votes (all OOB or no matches). Diagnostics:")
        print(f"  total_kps={cnt_total_kp}, accepted={cnt_accepted}, no_bin={cnt_no_bin}, no_matches={cnt_no_matches}, ratio_reject={cnt_ratio_reject}, scale_reject={cnt_scale_reject}, oob={cnt_oob}")
        if kp_reasons:
            print("  sample reasons:", kp_reasons)
        # Optionally dump a small debug file for offline inspection
        if 'DUMP_DIR' in globals() and DUMP_DIR:
            try:
                os.makedirs(DUMP_DIR, exist_ok=True)
                fname = os.path.join(DUMP_DIR, f"sift_no_votes_gt_{y0}_{x0}.txt")
                with open(fname, "w") as f:
                    f.write(f"GT=({y0},{x0})\n")
                    f.write(f"total_kps={cnt_total_kp}\n")
                    f.write(f"accepted={cnt_accepted}\n")
                    f.write(f"no_bin={cnt_no_bin}\n")
                    f.write(f"no_matches={cnt_no_matches}\n")
                    f.write(f"ratio_reject={cnt_ratio_reject}\n")
                    f.write(f"scale_reject={cnt_scale_reject}\n")
                    f.write(f"oob={cnt_oob}\n")
                    f.write("sample_reasons:\n")
                    for r in kp_reasons:
                        f.write(str(r) + "\n")
            except Exception:
                pass
        return None, (y0, x0), query, None

    votes_arr = np.array([[v[0], v[1]] for v in votes], dtype=np.float32)
    weights = np.array([v[2] for v in votes], dtype=np.float32)
    n_votes = votes_arr.shape[0]

    if n_votes == 1:
        est = (int(round(votes_arr[0,0])), int(round(votes_arr[0,1])))
    else:
        best_inliers = None
        best_weight_sum = 0.0
        RANSAC_ITERS = 200
        RANSAC_THRESH = 12.0  # pixels (looser to account for discretization)

        for _ in range(RANSAC_ITERS):
            idx = np.random.randint(0, n_votes)
            hypothesis = votes_arr[idx]
            dists_v = np.linalg.norm(votes_arr - hypothesis, axis=1)
            inliers_mask = dists_v <= RANSAC_THRESH
            weight_sum = float(weights[inliers_mask].sum())
            if weight_sum > best_weight_sum:
                best_weight_sum = weight_sum
                best_inliers = (votes_arr[inliers_mask], weights[inliers_mask])

        if best_inliers is None or best_weight_sum <= 0.0:
            # fallback to weighted mode: pick vote with max weight
            best_idx = int(np.argmax(weights))
            est = (int(round(votes_arr[best_idx,0])), int(round(votes_arr[best_idx,1])))
        else:
            in_coords, in_w = best_inliers
            # weighted average of inlier coordinates
            w_sum = float(in_w.sum()) + 1e-9
            mean_y = float((in_coords[:,0] * in_w).sum() / w_sum)
            mean_x = float((in_coords[:,1] * in_w).sum() / w_sum)
            est_y = int(np.clip(int(round(mean_y)), 0, H - PATCH_H))
            est_x = int(np.clip(int(round(mean_x)), 0, W - PATCH_W))
            est = (est_y, est_x)

    est_y, est_x = est
    dist = np.sqrt((est_y - y0)**2 + (est_x - x0)**2)
    print(f"[SIFT] GT=({y0},{x0}), EST=({est_y},{est_x}), Error={dist:.2f}px")

    # Optionally dump debug visualization for large errors
    if 'DUMP_DIR' in globals() and DUMP_DIR:
        try:
            # save up to a few failing examples (based on error magnitude)
            dump_threshold = max(PATCH_H, PATCH_W) * 1.0
            if dist >= dump_threshold:
                os.makedirs(DUMP_DIR, exist_ok=True)
                # construct debug figure
                import matplotlib.pyplot as _plt
                fig = _plt.figure(figsize=(10, 6))
                ax = fig.add_subplot(1, 1, 1)
                ax.imshow(img, cmap='gray')
                ax.axis('off')
                # true and estimated rectangles
                ax.add_patch(_plt.Rectangle((x0, y0), PATCH_W, PATCH_H, edgecolor='lime', facecolor='none', linewidth=2))
                ax.add_patch(_plt.Rectangle((est_x, est_y), PATCH_W, PATCH_H, edgecolor='red', facecolor='none', linewidth=2))

                # plot a subset of vote connections
                # sort votes by weight and take top 20
                votes_sorted = sorted(votes, key=lambda v: v[2], reverse=True)
                for v in votes_sorted[:20]:
                    _, _, w, x_db, y_db, x_q, y_q, nd = v
                    # draw a line from db kp to the predicted top-left (db_kp - query_kp)
                    ax.plot([x_db - 0, x_db - x_q], [y_db - 0, y_db - y_q], color='yellow', alpha=min(1.0, 0.2 + 0.8*(w/ (w+1e-6))))

                fname = os.path.join(DUMP_DIR, f"sift_debug_gt_{y0}_{x0}_est_{est_y}_{est_x}_err_{int(dist)}.png")
                _plt.tight_layout()
                _plt.savefig(fname, dpi=150)
                _plt.close(fig)
        except Exception as _e:
            print("Failed to write SIFT debug dump:", _e)
    return est, (y0, x0), query, dist


def sift_localize_query(query_patch, offline_img, sift_db):
    """
    Localize a given query patch (PATCH_H x PATCH_W) inside offline_img using the
    precomputed SIFT database (sift_db).

    query_patch : 2D float32 array in [0,1] or uint8 (live patch)
    offline_img : 2D float32 array in [0,1] (offline map)
    sift_db     : database built by build_sift_database(offline_img)

    Returns:
        (row, col) of top-left corner in offline_img, or None on failure.
    """
    H, W = offline_img.shape

    # 1) Compute SIFT on query patch
    if query_patch.dtype != np.float32:
        qp = query_patch.astype(np.float32) / (255.0 if query_patch.max() > 1.5 else 1.0)
    else:
        qp = query_patch
    kps_q, desc_q = sift_extract(qp)
    if desc_q is None or len(kps_q) == 0:
        return None

    # 2) Limit query keypoints by MAX_QUERY_KPS (as in sift_localize)
    if len(kps_q) > MAX_QUERY_KPS:
        responses = np.array([kp.response for kp in kps_q], dtype=np.float32)
        top_idx = np.argsort(responses)[-MAX_QUERY_KPS:]
        top_idx = np.sort(top_idx)
        kps_q = [kps_q[i] for i in top_idx]
        desc_q = desc_q[top_idx]

    # 3) Project query descriptors via DB PCA
    desc_q_pca = sift_db["pca"].transform(desc_q.astype(np.float32))
    votes = []

    cnt_total_kp = len(kps_q)
    cnt_no_bin = cnt_no_matches = cnt_ratio_reject = 0
    cnt_scale_reject = cnt_oob = cnt_accepted = 0

    for i, kp in enumerate(kps_q):
        s = kp.size
        b = np.digitize([s], sift_db["bin_edges"]) - 1
        b = int(np.clip(b[0], 0, SCALE_BINS - 1))
        if b not in sift_db["nn_per_bin"]:
            cnt_no_bin += 1
            continue

        nninfo = sift_db["nn_per_bin"][b]
        n_fit = nninfo["desc_pca"].shape[0]
        if n_fit == 0 or nninfo.get("flann", None) is None:
            cnt_no_matches += 1
            continue

        # k nearest neighbors
        k = min(max(2, 3), n_fit)
        qvec = desc_q_pca[i:i+1].astype(np.float32)
        matches = nninfo["flann"].knnMatch(qvec, k=k)

        x_q, y_q = kp.pt

        if len(matches) == 0 or len(matches[0]) == 0:
            cnt_no_matches += 1
            continue

        best_matches = matches[0]
        if len(best_matches) >= 2:
            m0, m1 = best_matches[0], best_matches[1]
            if m0.distance > SIFT_RATIO * m1.distance:
                cnt_ratio_reject += 1
                continue
            chosen = m0
        else:
            chosen = best_matches[0]

        neigh_dist = float(chosen.distance)
        idx = int(chosen.trainIdx)
        x_db, y_db = nninfo["positions"][idx]
        s_db = float(nninfo["scales"][idx]) if "scales" in nninfo else 0.0

        # scale check
        scale_tol = 1.8
        if kp.size > 0 and s_db > 0:
            ratio = s_db / kp.size
            if ratio < 1.0/scale_tol or ratio > scale_tol:
                cnt_scale_reject += 1
                continue

        # estimate top-left of patch in map: db_kp - query_kp
        est_y_raw = float(y_db - y_q)
        est_x_raw = float(x_db - x_q)
        if est_y_raw < 0 or est_x_raw < 0 or est_y_raw > (H - PATCH_H) or est_x_raw > (W - PATCH_W):
            cnt_oob += 1
            continue

        w_dist = 1.0 / (neigh_dist + 1e-6)
        w_scale = 1.0 - abs(1.0 - (s_db / (kp.size + 1e-9)))
        weight = max(0.0, w_dist * w_scale)
        votes.append((est_y_raw, est_x_raw, weight))
        cnt_accepted += 1

    if not votes:
        return None

    votes_arr = np.array([[v[0], v[1]] for v in votes], dtype=np.float32)
    weights = np.array([v[2] for v in votes], dtype=np.float32)
    n_votes = votes_arr.shape[0]

    # 4) Robust aggregation (RANSAC on vote positions)
    if n_votes == 1:
        est_y, est_x = votes_arr[0]
    else:
        best_inliers = None
        best_weight_sum = 0.0
        RANSAC_ITERS = 200
        RANSAC_THRESH = 12.0

        for _ in range(RANSAC_ITERS):
            idx = np.random.randint(0, n_votes)
            hyp = votes_arr[idx]
            dists_v = np.linalg.norm(votes_arr - hyp, axis=1)
            inliers_mask = dists_v <= RANSAC_THRESH
            weight_sum = float(weights[inliers_mask].sum())
            if weight_sum > best_weight_sum:
                best_weight_sum = weight_sum
                best_inliers = (votes_arr[inliers_mask], weights[inliers_mask])

        if best_inliers is None or best_weight_sum <= 0.0:
            best_idx = int(np.argmax(weights))
            est_y, est_x = votes_arr[best_idx]
        else:
            in_coords, in_w = best_inliers
            w_sum = float(in_w.sum()) + 1e-9
            est_y = float((in_coords[:, 0] * in_w).sum() / w_sum)
            est_x = float((in_coords[:, 1] * in_w).sum() / w_sum)

    # clamp to valid map region
    est_y = int(np.clip(int(round(est_y)), 0, H - PATCH_H))
    est_x = int(np.clip(int(round(est_x)), 0, W - PATCH_W))
    return (est_y, est_x)


def lbp_localize_query(query_patch, offline_img, lbp_db):
    """
    Localize a given query patch (PATCH_H x PATCH_W) inside offline_img using LBP DB.
    Returns (est_row, est_col) or None.
    """
    descs, poses, nn = lbp_db
    H, W = offline_img.shape

    # Descriptor of query patch
    q_desc = lbphf_descriptor(query_patch.astype(np.float32)).reshape(1, -1)
    _, idxs = nn.kneighbors(q_desc, n_neighbors=COARSE_TOPK)
    candidates = [poses[i] for i in idxs[0]]

    best_score = -1.0
    best_pos = None
    for (py, px) in candidates:
        if py + PATCH_H > H or px + PATCH_W > W:
            continue
        patch = offline_img[py:py+PATCH_H, px:px+PATCH_W]
        score = ncc_score(patch, query_patch)
        if score > best_score:
            best_score = score
            best_pos = (py, px)

    return best_pos


### ======================================================
### Visualization
### ======================================================
def visualize_result(img, est, gt, query):
    y0, x0 = gt
    true_patch = img[y0:y0+PATCH_H, x0:x0+PATCH_W]

    if est is not None:
        ey, ex = est
        est_patch = img[ey:ey+PATCH_H, ex:ex+PATCH_W]
    else:
        est_patch = np.zeros_like(true_patch)

    fig, axs = plt.subplots(1, 3, figsize=(18, 4))

    # ---------------------------------------------------
    # 0: full texture map with rectangles
    # ---------------------------------------------------
    axs[0].set_title("Offline Map (Green=True, Red=Estimated)")
    axs[0].imshow(img, cmap='gray')
    axs[0].axis('off')

    # true
    axs[0].add_patch(
        plt.Rectangle((x0, y0), PATCH_W, PATCH_H,
                      edgecolor='lime', facecolor='none', linewidth=2)
    )

    # estimated
    if est is not None:
        axs[0].add_patch(
            plt.Rectangle((ex, ey), PATCH_W, PATCH_H,
                          edgecolor='red', facecolor='none', linewidth=2)
        )

    # ---------------------------------------------------
    # 1: query patch
    # ---------------------------------------------------
    axs[1].set_title("Query Patch (online)")
    axs[1].imshow(query, cmap='gray')
    axs[1].axis('off')

    # ---------------------------------------------------
    # 2: true location patch
    # ---------------------------------------------------
    axs[2].set_title("True Patch (ground truth)")
    axs[2].imshow(true_patch, cmap='gray')
    axs[2].axis('off')

    # ---------------------------------------------------
    # 3: estimated patch
    # ---------------------------------------------------
    #axs[3].set_title("Matched Patch (estimated)")
    #axs[3].imshow(est_patch, cmap='gray')
    #axs[3].axis('off')

    plt.tight_layout()
    plt.show()


# ----------------------- New: helper for mapping reconstruction -----------------------
def reconstruct_image_from_mapping(mapping):
    """
    mapping: dict with keys (row,col) -> (gray_value, point_indices_array)
    returns: float32 image in [0,1]
    """
    if not mapping:
        raise ValueError("Empty mapping provided.")
    # ensure keys are tuples of ints
    keys = list(mapping.keys())
    rows = [int(k[0]) for k in keys]
    cols = [int(k[1]) for k in keys]
    H = max(rows) + 1
    W = max(cols) + 1
    img8 = np.zeros((H, W), dtype=np.uint8)
    for (r, c), val in mapping.items():
        # val may be (gray, arr) or scalar
        gray = val[0] if (isinstance(val, (tuple, list)) or (hasattr(val, '__len__') and not np.isscalar(val))) else val
        img8[int(r), int(c)] = np.clip(int(gray), 0, 255)
    return img8.astype(np.float32) / 255.0
# ----------------------- End helper -----------------------------------

### ======================================================
### Parameter Sweep
### ======================================================
def run_param_sweep(method, img, args, out_csv=None):
    """
    Parameter sweep is disabled: parameters now come only from YAML.
    """
    raise RuntimeError("run_param_sweep is disabled. All parameters must be set in localization_params.yaml.")

### ======================================================
### MAIN
### ======================================================
def main():
    global PATCH_H, PATCH_W
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["lbp", "sift"], required=True)
    parser.add_argument("--new", action="store_true")
    parser.add_argument("--pixel_map", type=str, default="", help="Path to .npy pixel->points mapping (reconstructs grayscale image)")
    parser.add_argument("--image", type=str, default="")
    parser.add_argument("--trials", type=int, default=1, help="Number of random trials to run")
    parser.add_argument("--nodisplay", action="store_true", help="Disable plotting/interactive display")
    parser.add_argument("--debug", action="store_true", help="Print debug info for localization")
    parser.add_argument("--dump_dir", type=str, default="", help="Directory to write SIFT debug dumps for failing trials")
    parser.add_argument("--sweep", action="store_true", help="Run parameter sweep (DISABLED)")
    parser.add_argument("--sweep_out", type=str, default="", help="Output CSV path for sweep (unused)")
    args = parser.parse_args()

    # ------------------ load YAML params & apply ------------------
    if not PARAM_YAML_PATH.exists():
        raise RuntimeError(f"Parameter YAML not found at {PARAM_YAML_PATH}")
    with open(PARAM_YAML_PATH, "r") as f:
        cfg = yaml.safe_load(f) or {}
    loc2d_cfg = cfg.get("loc2d", {})
    set_loc2d_config(loc2d_cfg)
    print(f"[CONFIG] Loaded localization parameters from {PARAM_YAML_PATH}")
    print(f"[CONFIG] {loc2d_cfg}")
    print(f"[CONFIG] Using PATCH_H={PATCH_H}, PATCH_W={PATCH_W}, STRIDE={STRIDE}")

    global DEBUG, DUMP_DIR
    DEBUG = args.debug
    DUMP_DIR = args.dump_dir

    # -----------------------------------------------------
    # Load external image or pixel_map if provided
    # -----------------------------------------------------
    if args.pixel_map:
        pm_path = Path(args.pixel_map)
        if not pm_path.exists():
            raise RuntimeError(f"Failed to find pixel_map: {pm_path}")
        data = np.load(str(pm_path), allow_pickle=True).item()
        keys = list(data.keys())
        if len(keys) == 0:
            raise RuntimeError(f"pixel_map {pm_path} contains no entries")
        rows = [int(k[0]) for k in keys]
        cols = [int(k[1]) for k in keys]
        H_img = max(rows) + 1
        W_img = max(cols) + 1
        I_uint8 = np.zeros((H_img, W_img), dtype=np.uint8)
        for (r, c), val in data.items():
            gray_val = int(val[0]) if isinstance(val, (tuple, list)) else int(val)
            I_uint8[int(r), int(c)] = np.clip(gray_val, 0, 255)
        img = I_uint8.astype(np.float32) / 255.0
        print(f"Loaded pixel_map {pm_path} -> reconstructed image shape {img.shape}")
        # NOTE: PATCH_H/PATCH_W are NOT changed here; use YAML only.

    elif args.image:
        img = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"Failed to load image: {args.image}")
        img = img.astype(np.float32) / 255.0
        print(f"Loaded input image: {args.image}")
        # NOTE: PATCH_H/PATCH_W remain as set by YAML/defaults.

    else:
        # Attempt to load OFFLINE_MAP_PATH; it can be either a raw image array or a mapping dict
        if OFFLINE_MAP_PATH.exists() and not args.new:
            try:
                loaded = np.load(str(OFFLINE_MAP_PATH), allow_pickle=True)
                if isinstance(loaded, np.ndarray) and loaded.dtype != np.object_:
                    img = loaded.astype(np.float32)
                    if img.max() > 1.5:
                        img = img / 255.0
                    print(f"Loaded offline raw image from {OFFLINE_MAP_PATH} shape={img.shape}")
                else:
                    mapping = loaded.item() if hasattr(loaded, 'item') else dict(loaded)
                    img = reconstruct_image_from_mapping(mapping)
                    print(f"Loaded OFFLINE_MAP_PATH mapping and reconstructed image shape={img.shape}")
            except Exception as e:
                raise RuntimeError(f"Failed to load offline map from {OFFLINE_MAP_PATH}: {e}")
        else:
            raise RuntimeError(
                "No offline map found. Provide --image or --pixel_map, or create offline_map.npy beforehand."
            )

    if args.sweep:
        raise RuntimeError("Parameter sweep is disabled. Tune parameters in localization_params.yaml instead.")

    # -----------------------------------------------------
    # LBP-HF MODE
    # -----------------------------------------------------
    if args.method == "lbp":
        print(f"Using LBP: PATCH_H={PATCH_H}, PATCH_W={PATCH_W}, STRIDE={STRIDE}")
        # try loading DB
        if not args.image and OFFLINE_LBP_DB_PATH.exists() and not args.new:
            try:
                with open(OFFLINE_LBP_DB_PATH, "rb") as f:
                    lbp_db = pickle.load(f)
                # quick compatibility check
                descs, poses, nn = lbp_db
                H, W = img.shape
                compatible = True
                if descs.ndim != 2 or descs.shape[1] != LBPHF_KEEP:
                    compatible = False
                if compatible:
                    for (py, px) in poses:
                        if py + PATCH_H > H or px + PATCH_W > W:
                            compatible = False
                            break
                if not compatible:
                    print("Offline LBP DB incompatible with current image/patch size — rebuilding.")
                    lbp_db = None
                else:
                    print("Loaded LBP DB.")
            except Exception:
                print("Failed to load offline LBP DB; will rebuild from current image.")
                lbp_db = None
        else:
            lbp_db = None

        if lbp_db is None:
            descs, poses = compute_map_lbphf_descriptors(img)
            nn = NearestNeighbors(n_neighbors=COARSE_TOPK, metric="cosine")
            nn.fit(descs)
            lbp_db = (descs, poses, nn)
            if not args.image:
                with open(OFFLINE_LBP_DB_PATH, "wb") as f:
                    pickle.dump(lbp_db, f)
            print("Built LBP DB.")

        per_trial_errors = []
        failures = 0
        runtimes = []
        for t in range(args.trials):
            start_t = time.time()
            est, gt, query, dist = lbp_localize(img, lbp_db)
            elapsed = time.time() - start_t
            runtimes.append(elapsed)
            if dist is None:
                failures += 1
                per_trial_errors.append(np.nan)
            else:
                per_trial_errors.append(dist)
            if args.trials == 1 and not args.nodisplay:
                visualize_result(img, est, gt, query)

        if args.trials > 1:
            errors = np.array(per_trial_errors, dtype=float)
            successes = int(np.isfinite(errors).sum())
            mean_err = float(np.nanmean(errors)) if successes > 0 else float("nan")
            std_err = float(np.nanstd(errors)) if successes > 0 else float("nan")
            print(f"LBP Trials={args.trials}, Failures={failures}, Successful={successes}")
            if successes > 0:
                print(f"LBP Mean Error={mean_err:.2f}px, Std={std_err:.2f}px")
            # Combined plot: per-trial error (left y) and runtime (right y)
            try:
                fig, ax1 = plt.subplots(figsize=(10, 4))
                trials_idx = np.arange(1, len(errors) + 1)
                ax1.plot(trials_idx, errors, marker='o', color='C0', label='Error (px)')
                ax1.set_xlabel('Trial')
                ax1.set_ylabel('Error (px)', color='C0')
                ax1.tick_params(axis='y', labelcolor='C0')
                ax2 = ax1.twinx()
                ax2.plot(trials_idx, runtimes, marker='s', linestyle='--', color='C1', label='Runtime (s)')
                ax2.set_ylabel('Runtime (s)', color='C1')
                ax2.tick_params(axis='y', labelcolor='C1')
                mean_rt = float(np.mean(runtimes)) if runtimes else float("nan")
                ax1.set_title(f"LBP per-trial Error & Runtime (mean_err={mean_err if not np.isnan(mean_err) else 'nan'} px, mean_rt={mean_rt:.3f}s)")
                # combined legend
                lines_1, labels_1 = ax1.get_legend_handles_labels()
                lines_2, labels_2 = ax2.get_legend_handles_labels()
                ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='best')
                outname_comb = "results_lbp_combined.png"
                plt.tight_layout()
                plt.savefig(outname_comb)
                plt.close(fig)
                print(f"Saved combined LBP plot: {outname_comb}")
            except Exception as e:
                print("Failed to save combined LBP plot:", e)

    # -----------------------------------------------------
    # SIFT MODE
    # -----------------------------------------------------
    if args.method == "sift":
        print(f"Using SIFT: PATCH_H={PATCH_H}, PATCH_W={PATCH_W}, STRIDE={STRIDE}")
        if not args.image and OFFLINE_SIFT_DB_PATH.exists() and not args.new:
            try:
                with open(OFFLINE_SIFT_DB_PATH, "rb") as f:
                    loaded = pickle.load(f)
                sift_db = reconstruct_sift_db(loaded)
                print("Loaded SIFT DB.")
            except Exception:
                print("Failed to load offline SIFT DB; will rebuild from current image.")
                sift_db = None
        else:
            sift_db = None

        if sift_db is None:
            sift_db = build_sift_database(img)
            if not args.image:
                try:
                    with open(OFFLINE_SIFT_DB_PATH, "wb") as f:
                        pickle.dump(make_savable_sift_db(sift_db), f)
                except Exception as _e:
                    print("Warning: failed to save SIFT DB to disk:", _e)
            print("Built SIFT DB.")

        per_trial_errors = []
        failures = 0
        runtimes = []
        for t in range(args.trials):
            start_t = time.time()
            est, gt, query, dist = sift_localize(img, sift_db)
            elapsed = time.time() - start_t
            runtimes.append(elapsed)
            if dist is None:
                failures += 1
                per_trial_errors.append(np.nan)
            else:
                per_trial_errors.append(dist)
            if args.trials == 1 and not args.nodisplay:
                visualize_result(img, est, gt, query)

        if args.trials > 1:
            errors = np.array(per_trial_errors, dtype=float)
            successes = int(np.isfinite(errors).sum())
            mean_err = float(np.nanmean(errors)) if successes > 0 else float("nan")
            std_err = float(np.nanstd(errors)) if successes > 0 else float("nan")
            print(f"SIFT Trials={args.trials}, Failures={failures}, Successful={successes}")
            if successes > 0:
                print(f"SIFT Mean Error={mean_err:.2f}px, Std={std_err:.2f}px")
            # Combined plot: per-trial error (left y) and runtime (right y)
            try:
                fig, ax1 = plt.subplots(figsize=(10, 4))
                trials_idx = np.arange(1, len(errors) + 1)
                ax1.plot(trials_idx, errors, marker='o', color='C0', label='Error (px)')
                ax1.set_xlabel('Trial')
                ax1.set_ylabel('Error (px)', color='C0')
                ax1.tick_params(axis='y', labelcolor='C0')
                ax2 = ax1.twinx()
                ax2.plot(trials_idx, runtimes, marker='s', linestyle='--', color='C1', label='Runtime (s)')
                ax2.set_ylabel('Runtime (s)', color='C1')
                ax2.tick_params(axis='y', labelcolor='C1')
                mean_rt = float(np.mean(runtimes)) if runtimes else float("nan")
                ax1.set_title(f"SIFT per-trial Error & Runtime (mean_err={mean_err if not np.isnan(mean_err) else 'nan'} px, mean_rt={mean_rt:.3f}s)")
                lines_1, labels_1 = ax1.get_legend_handles_labels()
                lines_2, labels_2 = ax2.get_legend_handles_labels()
                ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='best')
                outname_comb = "results_sift_combined.png"
                plt.tight_layout()
                plt.savefig(outname_comb)
                plt.close(fig)
                print(f"Saved combined SIFT plot: {outname_comb}")
            except Exception as e:
                print("Failed to save combined SIFT plot:", e)


if __name__ == "__main__":
    main()

