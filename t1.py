import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(pose):
    """
    pose = [x, y, z, rx, ry, rz] -> 4x4变换矩阵
    使用罗德里格斯公式将旋转向量转为旋转矩阵
    """
    r = R.from_rotvec(pose[3:6])
    R_mat = r.as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = pose[0:3]
    return T

def get_base_point_from_laser(distance, tcp_pose, sensor_offset):
    """
    只进行传感器坐标系 -> TCP -> 基座坐标系的变换
    :param distance: 激光测量值，单位：米
    :param tcp_pose: UR末端位姿 [x, y, z, rx, ry, rz]
    :param sensor_offset: 激光相对于TCP的偏移 [x, y, z, rx, ry, rz]
    :return: 激光点在基座坐标系下的坐标 [x, y, z]
    """
    p_sensor = np.array([0, 0, distance, 1]).reshape(4, 1)
    T_tcp_sensor = pose_to_matrix(sensor_offset)
    T_base_tcp = pose_to_matrix(tcp_pose)
    T_base_sensor = T_base_tcp @ T_tcp_sensor
    p_base = T_base_sensor @ p_sensor
    return p_base[:3].flatten()

def scan_along_line(start_pose, end_pose, num_points, sensor_offset, distance_func=None):
    """
    沿直线轨迹扫描并记录激光测点
    :param start_pose: 起点位姿 [x, y, z, rx, ry, rz]
    :param end_pose: 终点位姿 [x, y, z, rx, ry, rz]
    :param num_points: 扫描点数
    :param sensor_offset: 激光相对于TCP的偏移
    :param distance_func: 激光距离生成函数（可选），默认返回固定值0.102
    :return: 基座坐标系下的一系列激光点坐标列表
    """
    if distance_func is None:
        distance_func = lambda: 0.102  # 默认激光距离为0.102米

    # 将位姿转为numpy数组
    start_pose = np.array(start_pose)
    end_pose = np.array(end_pose)

    # 生成线性插值点（只插值位置，姿态保持不变）
    t = np.linspace(0, 1, num_points)  # 参数化变量t，从0到1
    tcp_poses = [start_pose + (end_pose - start_pose) * ti for ti in t]

    # 存储扫描结果
    scan_points = []

    # 对每个TCP位姿进行扫描
    for tcp_pose in tcp_poses:
        distance = distance_func()  # 获取激光测量值
        p_base = get_base_point_from_laser(distance, tcp_pose, sensor_offset)
        scan_points.append(p_base)

    return scan_points

# 示例使用
if __name__ == "__main__":
    # 定义扫描参数
    start_pose = [0.3, 0.2, 0.1, 0, 0, 0]  # 起点位姿
    end_pose = [0.5, 0.2, 0.1, 0, 0, 0]    # 终点位姿（沿X轴移动0.2米）
    num_points = 5                         # 扫描5个点
    sensor_offset = [0, 0, 0.05, 0, 0, 0]  # 传感器偏移

    # 模拟激光距离（这里用固定值，实际应用中可替换为传感器数据）
    def mock_distance():
        return 0.102  # 固定距离0.102米

    # 执行扫描
    scan_results = scan_along_line(start_pose, end_pose, num_points, sensor_offset, mock_distance)

    # 打印结果
    print("扫描得到的激光点坐标（基座坐标系）：")
    for i, point in enumerate(scan_results):
        print(f"点 {i+1}: {point}")