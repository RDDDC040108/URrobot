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
    # 激光测点（在激光传感器坐标系下，默认激光方向为 Z轴正方向）
    p_sensor = np.array([0, 0, distance, 1]).reshape(4, 1)

    # 构造变换矩阵
    T_tcp_sensor = pose_to_matrix(sensor_offset)
    T_base_tcp = pose_to_matrix(tcp_pose)

    # 坐标变换：sensor → TCP → base
    T_base_sensor = T_base_tcp @ T_tcp_sensor
    p_base = T_base_sensor @ p_sensor

    return p_base[:3].flatten()
# 示例数据（单位：米和弧度）
distance = 0.102  # 激光测量值，单位：米
tcp_pose = [0.3, 0.2, 0.1, 0, 0, 0]  # 当前末端在基座的位姿
sensor_offset = [0, 0, 0.05, 0, 0, 0]  # 激光在TCP坐标系下前方5cm，无旋转

p_base = get_base_point_from_laser(distance, tcp_pose, sensor_offset)
print("激光点在UR基座坐标系下的坐标：", p_base)