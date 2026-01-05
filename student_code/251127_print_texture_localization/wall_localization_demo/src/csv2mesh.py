import numpy as np
import open3d as o3d

# ================================
# 1. Load CSV as point cloud
# ================================
csv_path = "../data/demo_points.csv"

print("Loading CSV...")
pts = np.loadtxt(csv_path, delimiter=",")

# Remove NaN or infinite rows
pts = pts[~np.isnan(pts).any(axis=1)]
pts = pts[np.isfinite(pts).all(axis=1)]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)

print("Loaded", len(pts), "points")


# ================================
# 2. Estimate normals
# ================================
print("Estimating normals...")

pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.010,       # Tune: depends on point spacing
        max_nn=50
    )
)

# Make normals consistent (important for Poisson)
pcd.orient_normals_consistent_tangent_plane(100)
# Alternative:
# pcd.orient_normals_towards_camera_location(camera_location=np.array([0,0,0]))

print("Normals estimated.")


# ================================
# 3. Poisson Reconstruction
# ================================
print("Running Poisson reconstruction...")

mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd,
    depth=10,        # main Poisson resolution parameter
    width=0,         
    scale=1.1,       # enlarge Poisson bbox slightly
    linear_fit=False
)

print("Poisson reconstruction complete.")
print("Mesh has", len(mesh.vertices), "vertices and", len(mesh.triangles), "triangles")


# ================================
# 4. Remove floating artifacts
# ================================
print("Removing low-density artifacts...")

densities = np.asarray(densities)
density_threshold = np.quantile(densities, 0.02)   # Remove bottom 2%

mask = densities < density_threshold
mesh.remove_vertices_by_mask(mask)

print("Cleaned mesh now has", len(mesh.vertices), "vertices.")


# ================================
# 5. Final cleaning
# ================================
mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()

mesh.compute_vertex_normals()


# ================================
# 6. Save as OBJ
# ================================
o3d.io.write_triangle_mesh("wall_poisson.obj", mesh)
print("Saved wall_poisson.obj")

# If you want DAE, convert with meshlabserver or trimesh
# Example:
#   meshlabserver -i wall_poisson.obj -o wall_poisson.dae

# ================================
# 7. Visualize
# ================================
o3d.visualization.draw_geometries([mesh])
