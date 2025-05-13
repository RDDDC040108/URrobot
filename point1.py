# 导入Open3D库，主要用于处理点云数据
import open3d as o3d
# 读取点云数据
# ".txt" 是点云文件名
# format='xyz' 表示每行是三个浮点数，分别是X, Y, Z坐标
pcd = o3d.io.read_point_cloud("Elephant.txt", format='xyz')  

# 打印点云对象的信息（例如点的数量）
print(pcd)  # 显示点云包含多少个点，以及是否有颜色、法线等信息
pcd.paint_uniform_color([0.0, 0.0, 0.0])  # RGB值0~1
# 将读取到的点云保存为一个新的PLY格式文件
# PLY是一种常见的3D数据存储格式，可以保存坐标、颜色等信息
o3d.io.write_point_cloud("copy2.ply", pcd,write_ascii=True)
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
# 将点云可视化显示在一个弹出的窗口中
# 参数width和height设置了窗口大小（800x800像素）
# o3d.visualization.draw_geometries([pcd], width=800, height=800)
o3d.visualization.draw_geometries(
    [pcd, coordinate_frame],
    window_name='Point Cloud with XYZ Axis',
    width=800,
    height=800,
    point_show_normal=False,
)
