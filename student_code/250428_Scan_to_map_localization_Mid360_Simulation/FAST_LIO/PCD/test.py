import open3d as o3d

# 读取 .pcd 文件
pcd = o3d.io.read_point_cloud("scans.pcd")

# 可视化点云
o3d.visualization.draw_geometries([pcd])

