import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import imageio

# ============================================================
# 0. CLI for output paths
# ============================================================
parser = argparse.ArgumentParser(
    description="Generate synthetic wall point cloud + mid-path, with sine wall in z and random blobs."
)
parser.add_argument("--out_img", type=str, default="", help="Optional output grayscale image path (png/jpg).")
parser.add_argument("--out_cloud_csv", type=str, default="../data/demo_points.csv",
                    help="Output CSV for 3D point cloud (x,y,z).")
parser.add_argument("--out_midpath_csv", type=str, default="../data/demo_midpath.csv",
                    help="Output CSV for mid-path waypoints (x,y,z).")
parser.add_argument("--out_obj", type=str, default="wall_mesh.obj",
                    help="Output OBJ mesh for the wall surface.")
parser.add_argument("--out_dae", type=str, default="",
                    help="Optional output DAE mesh for the wall surface.")
args = parser.parse_args()

waypoint = 1000 # number of mid-path waypoints
wall_length = 10.0  # meters (length in x direction)
wall_height = 4.0   # meters (height in z direction)
num_points = 500000 # number of 3D points to simulate
num_blobs = 400  # number of random blobs on wall surface
# Define bin sizes for s and z
Delta_s = 0.005
Delta_z = 0.005

# ============================================================
# 1. Straight mid-path on the xy-plane (z = 0)
# ============================================================

t = np.linspace(0.0, wall_length, waypoint)

mid_x = t
mid_y = np.zeros_like(t)
mid_z = np.zeros_like(t)

Q = np.vstack((mid_x, mid_y, mid_z)).T  # shape (M, 3)

segment_vecs = Q[1:, :2] - Q[:-1, :2]
segment_lengths = np.linalg.norm(segment_vecs, axis=1)
s_k = np.zeros(Q.shape[0])
s_k[1:] = np.cumsum(segment_lengths)
L = s_k[-1]

# ============================================================
# 2. Generate 3D wall surface around the mid-path
# ============================================================

seg_indices = np.random.randint(0, Q.shape[0] - 1, size=num_points)
lambda_samples = np.random.rand(num_points)

q_start = Q[seg_indices]
q_end   = Q[seg_indices + 1]
mid_points = q_start + lambda_samples[:, None] * (q_end - q_start)

mx = mid_points[:, 0]
my = mid_points[:, 1]
mz = mid_points[:, 2]

seg_vec_xy = (q_end - q_start)[:, :2]
seg_len = np.linalg.norm(seg_vec_xy, axis=1)
t_hat_xy = seg_vec_xy / seg_len[:, None]

n_hat_xy = np.empty_like(t_hat_xy)
n_hat_xy[:, 0] = -t_hat_xy[:, 1]
n_hat_xy[:, 1] =  t_hat_xy[:, 0]

radial_offset = 0.1 + 0.02 * np.random.randn(num_points)

z_nominal = np.random.uniform(0.0, wall_height, num_points)

y_sine = 0.02 * np.sin(2.0 * np.pi * 2.0 * z_nominal / wall_height)

blob_centers_x = np.random.uniform(0.0, wall_length, size=num_blobs)
blob_centers_z = np.random.uniform(0.0, wall_height, size=num_blobs)

blob_amplitudes = np.random.uniform(-0.02, 0.02, size=num_blobs)
blob_sigma_x = np.random.uniform(0.03, 0.1, size=num_blobs)
blob_sigma_z = np.random.uniform(0.03, 0.1, size=num_blobs)

blob_delta_y = np.zeros(num_points, dtype=np.float32)
for cx, cz, A, sx, sz in zip(
    blob_centers_x, blob_centers_z,
    blob_amplitudes, blob_sigma_x, blob_sigma_z
):
    dx = mx - cx
    dz = z_nominal - cz
    r2 = (dx*dx) / (sx * sx) + (dz*dz) / (sz * sz)
    blob_delta_y += A * np.exp(-0.5 * r2)

y_wall_center = y_sine + blob_delta_y

z_offset_noise = 0.01 * np.random.randn(num_points)
z_wall = z_nominal + z_offset_noise

x_pts = mx + radial_offset * n_hat_xy[:, 0]
y_pts = my + radial_offset * n_hat_xy[:, 1] + y_wall_center
z_pts = z_wall

points = np.vstack((x_pts, y_pts, z_pts)).T  # (N, 3)

# ============================================================
# 3. Depth d_i and (s,z) binning
# ============================================================

t_hat = np.hstack((t_hat_xy, np.zeros((num_points, 1))))
n_hat = np.hstack((n_hat_xy, np.zeros((num_points, 1))))
gamma_si = mid_points

r_vec = points - gamma_si
d_i = np.einsum('ij,ij->i', r_vec, n_hat)

s_start = s_k[seg_indices]
s_i_star = s_start + lambda_samples * seg_len

z_i = z_pts

s_min, s_max = 0.0, L
z_min, z_max = np.min(z_i), np.max(z_i)

Ns = int((s_max - s_min) / Delta_s)
Nz = int((z_max - z_min) / Delta_z)

# ============================================================
# 5. Aggregate depths into pixel intensities I(u,v)
# ============================================================

I = np.zeros((Nz, Ns))
count = np.zeros((Nz, Ns), dtype=int)

u_idx = ((s_i_star - s_min) / Delta_s).astype(int)
v_idx = ((z_i       - z_min) / Delta_z).astype(int)

valid = (u_idx >= 0) & (u_idx < Ns) & (v_idx >= 0) & (v_idx < Nz)
orig_indices = np.nonzero(valid)[0]

u_idx = u_idx[valid]
v_idx = v_idx[valid]
d_valid = d_i[valid]

from collections import defaultdict
bin_depths = defaultdict(list)
bin_points = defaultdict(list)   # store original point indices per bin

for u, v, d, oi in zip(u_idx, v_idx, d_valid, orig_indices):
    bin_depths[(u, v)].append(d)
    bin_points[(u, v)].append(int(oi))

for (u, v), d_list in bin_depths.items():
    I[Nz - 1 - v, u] = np.median(d_list)
    count[Nz - 1 - v, u] = len(d_list)

# ============================================================
# 6. Normalize intensities to grayscale [0, 255]
# ============================================================

non_empty_mask = count > 0
I_non_empty = I[non_empty_mask]

I_min = I_non_empty.min()
I_max = I_non_empty.max()

I_gray = np.zeros_like(I)

if I_max > I_min:
    I_gray[non_empty_mask] = 255.0 * (I[non_empty_mask] - I_min) / (I_max - I_min)

# ============================================================
# 7–9. Plots (unchanged; omitted for brevity in this snippet)
# (You can keep your existing plotting code if you want)
# ============================================================

# ============================================================
# 10. Optional outputs (existing)
# ============================================================

if args.out_img:
    out_path = Path(args.out_img)
    ext = out_path.suffix.lower()
    if ext not in (".png", ".jpg", ".jpeg"):
        out_path = out_path.with_suffix(".png")
    try:
        I_uint8 = np.clip(I_gray, 0, 255).astype(np.uint8)
        imageio.imwrite(str(out_path), I_uint8)
        print(f"Saved final texture image (raw) to: {out_path}")
    except Exception as e:
        print(f"Failed to save final image to {out_path}: {e}")

if args.out_npy:
    out_npy = Path(args.out_npy)
    try:
        mapping = {}
        for (u, v), pts_idx in bin_points.items():
            row = Nz - 1 - v
            col = u
            gray_val = int(np.clip(I_gray[row, col], 0, 255))
            mapping[(int(row), int(col))] = (int(gray_val), np.array(pts_idx, dtype=int))
        np.save(str(out_npy), mapping, allow_pickle=True)
        print(f"Saved pixel->points mapping to: {out_npy}")
    except Exception as e:
        print(f"Failed to save mapping .npy to {out_npy}: {e}")

try:
    np.savetxt(args.out_cloud_csv, points, delimiter=",")
    print(f"Saved point cloud to {args.out_cloud_csv}")
except Exception as e:
    print(f"Failed to save point cloud CSV: {e}")

try:
    np.savetxt(args.out_midpath_csv, Q, delimiter=",")
    print(f"Saved mid-path CSV to {args.out_midpath_csv}")
except Exception as e:
    print(f"Failed to save mid-path CSV: {e}")

# ============================================================
# 11. Mesh export (OBJ + optional DAE)
# ============================================================

def export_wall_mesh(points, bin_points, Nz, Ns, out_obj_path=None, out_dae_path=None):
    """
    Build a structured wall mesh from binned points and export to OBJ/DAE.
    - vertices: average of points in each non-empty bin
    - faces: two triangles per quad in the (s,z) grid
    """
    # vertex grid, NaN for empty
    verts_grid = np.full((Nz, Ns, 3), np.nan, dtype=np.float32)

    for (u, v), idx_list in bin_points.items():
        idx_arr = np.array(idx_list, dtype=int)
        pts_bin = points[idx_arr]
        mean_pt = pts_bin.mean(axis=0)
        row = Nz - 1 - v  # consistent with I_gray row indexing
        col = u
        if 0 <= row < Nz and 0 <= col < Ns:
            verts_grid[row, col, :] = mean_pt

    vertices = []
    index_grid = -np.ones((Nz, Ns), dtype=int)

    # Assign vertex indices
    for r in range(Nz):
        for c in range(Ns):
            if not np.isnan(verts_grid[r, c, 0]):
                index_grid[r, c] = len(vertices) + 1  # OBJ is 1-based
                vertices.append(verts_grid[r, c, :])

    faces = []
    for r in range(Nz - 1):
        for c in range(Ns - 1):
            v00 = index_grid[r, c]
            v10 = index_grid[r, c + 1]
            v01 = index_grid[r + 1, c]
            v11 = index_grid[r + 1, c + 1]
            if v00 > 0 and v10 > 0 and v01 > 0 and v11 > 0:
                # two triangles
                faces.append((v00, v10, v11))
                faces.append((v00, v11, v01))

    if not vertices or not faces:
        print("Mesh export: no valid vertices/faces, skipping.")
        return

    # Write OBJ
    if out_obj_path:
        try:
            with open(out_obj_path, "w") as f:
                f.write("# Wall mesh generated by demo_image.py\n")
                for v in vertices:
                    f.write(f"v {v[0]} {v[1]} {v[2]}\n")
                for (i1, i2, i3) in faces:
                    f.write(f"f {i1} {i2} {i3}\n")
            print(f"Saved wall mesh OBJ to {out_obj_path}")
        except Exception as e:
            print(f"Failed to save OBJ to {out_obj_path}: {e}")

    # Write DAE (COLLADA)
    if out_dae_path:
        try:
            # DAE uses 0-based indices
            tri_indices = []
            for (i1, i2, i3) in faces:
                tri_indices.extend([i1 - 1, i2 - 1, i3 - 1])

            dae = []
            dae.append('<?xml version="1.0" encoding="utf-8"?>')
            dae.append('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">')
            dae.append('  <asset>')
            dae.append('    <contributor><author>demo_image.py</author></contributor>')
            dae.append('    <unit name="meter" meter="1"/>')
            dae.append('    <up_axis>Z_UP</up_axis>')
            dae.append('  </asset>')
            dae.append('  <library_geometries>')
            dae.append('    <geometry id="wallMesh" name="wallMesh">')
            dae.append('      <mesh>')
            # positions
            dae.append('        <source id="wallMesh-positions">')
            dae.append(f'          <float_array id="wallMesh-positions-array" count="{3*len(vertices)}">')
            dae.append('            ' + ' '.join(f"{v[0]} {v[1]} {v[2]}" for v in vertices))
            dae.append('          </float_array>')
            dae.append('          <technique_common>')
            dae.append(f'            <accessor source="#wallMesh-positions-array" count="{len(vertices)}" stride="3">')
            dae.append('              <param name="X" type="float"/>')
            dae.append('              <param name="Y" type="float"/>')
            dae.append('              <param name="Z" type="float"/>')
            dae.append('            </accessor>')
            dae.append('          </technique_common>')
            dae.append('        </source>')
            dae.append('        <vertices id="wallMesh-vertices">')
            dae.append('          <input semantic="POSITION" source="#wallMesh-positions"/>')
            dae.append('        </vertices>')
            dae.append(f'        <triangles count="{len(faces)}">')
            dae.append('          <input semantic="VERTEX" source="#wallMesh-vertices" offset="0"/>')
            dae.append('          <p>')
            dae.append('            ' + ' '.join(str(i) for i in tri_indices))
            dae.append('          </p>')
            dae.append('        </triangles>')
            dae.append('      </mesh>')
            dae.append('    </geometry>')
            dae.append('  </library_geometries>')
            dae.append('  <library_visual_scenes>')
            dae.append('    <visual_scene id="Scene" name="Scene">')
            dae.append('      <node id="wallNode" name="wallNode">')
            dae.append('        <instance_geometry url="#wallMesh"/>')
            dae.append('      </node>')
            dae.append('    </visual_scene>')
            dae.append('  </library_visual_scenes>')
            dae.append('  <scene>')
            dae.append('    <instance_visual_scene url="#Scene"/>')
            dae.append('  </scene>')
            dae.append('</COLLADA>')

            with open(out_dae_path, "w") as f:
                f.write("\n".join(dae))

            print(f"Saved wall mesh DAE to {out_dae_path}")
        except Exception as e:
            print(f"Failed to save DAE to {out_dae_path}: {e}")


# Call mesh export if requested
out_obj_path = args.out_obj if args.out_obj else None
out_dae_path = args.out_dae if args.out_dae else None

if out_obj_path or out_dae_path:
    export_wall_mesh(points, bin_points, Nz, Ns, out_obj_path, out_dae_path)
# ============================================================
# 12. Show plots if you kept them above
# ============================================================
plt.show()
