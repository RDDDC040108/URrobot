import open3d as o3d
import numpy as np

# 创建一些点坐标 (N x 3)
points = np.random.rand(1000, 3)

# 创建 open3d 点云对象
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# 可选：添加颜色（与点数一致）
colors = np.random.rand(1000, 3)
pcd.colors = o3d.utility.Vector3dVector(colors)

# 显示点云
o3d.visualization.draw_geometries([pcd])
