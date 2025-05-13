# -*- coding: utf-8 -*-
"""
Laser‑to‑Flange 外参标定 + 目标点坐标推算 + 快速自查
Created on Mon Apr 21 23:32:48 2025
@author: Sakur
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

# ------------------------------------------------------------
# 1. 基础工具函数
# ------------------------------------------------------------
def pose_to_T(x, y, z, rx, ry, rz):
    """旋转向量 + 平移  →  4×4 齐次矩阵"""
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec([rx, ry, rz]).as_matrix()
    T[:3,  3] = [x, y, z]
    return T

def residual(param, T_BF_list, D_list, P_ref_B, k=np.array([0, 0, 1])):
    """最小二乘残差：同一点残差拼接"""
    t = param[:3]                                # 平移
    q = param[3:] / np.linalg.norm(param[3:])    # 四元数归一化
    R_FL = R.from_quat(q).as_matrix()
    res = []
    for T_BF, D in zip(T_BF_list, D_list):
        R_BF, t_BF = T_BF[:3, :3], T_BF[:3, 3]
        pred = R_BF @ t + t_BF + D * R_BF @ (R_FL @ k)
        res.append(pred - P_ref_B)
    return np.concatenate(res)

def point_from_distance(T_BF_now, d, T_FL, k=np.array([0, 0, 1])):
    """给定当前法兰姿态 + 距离 d，求目标点在基座系坐标"""
    R_BF, t_BF = T_BF_now[:3, :3], T_BF_now[:3, 3]
    p_L  = R_BF @ T_FL[:3, 3] + t_BF
    v_L  = R_BF @ (T_FL[:3, :3] @ k)
    return p_L + d * v_L

# ------------------------------------------------------------
# 2. 观测数据区  ⇣⇣⇣  —— 请替换成你的实测
# ------------------------------------------------------------
P_ref_B = np.array([-100.0, -300.0, 0.0])          # 参考点 (m)

obs = np.array([                                   # 法兰在基座下的 6DoF
    [-210.73,  -342.35,  120.77,  0.3280,  0.0620,  0.0150],
    [-222.20,  -344.41,  136.32,  0.3273, -0.0228,  0.0403],
    [-210.31,  -313.24,  160.30,  0.1493,  0.0629,  0.0090],
    # … 可继续追加 …
])

D_list = np.array([                                # 激光‑参考点距离 (m)
    108.7,
    135.0,
    146.0
])
# ------------------------------------------------------------

# 若上面 xyz 用的是 mm，请统一除以 1000.0；这里假设单位一致

T_BF_list = [pose_to_T(*p) for p in obs]

# ------------------------------------------------------------
# 3. Levenberg–Marquardt 最小二乘求 T_F→L
# ------------------------------------------------------------
print("\n===  标定阶段  ===")
x0 = np.hstack([0, 0, 0,   0, 0, 0, 1])          # 初始猜测
opt = least_squares(residual, x0,
                    args=(T_BF_list, D_list, P_ref_B))

t_FL = opt.x[:3]
R_FL = R.from_quat(opt.x[3:] / np.linalg.norm(opt.x[3:])).as_matrix()
T_F_L = np.eye(4)
T_F_L[:3, :3] = R_FL
T_F_L[:3,  3] = t_FL
print("T_F→L =\n", np.round(T_F_L, 6))

# ------------------------------------------------------------
# 4. 快速自查：单位 / 光束方向 / Jacobian 条件数
# ------------------------------------------------------------
print("\n===  快速自查  ===")
# 4‑1 单位粗检
print("max|xyz| in poses  :", np.abs(obs[:,:3]).max())
print("max D_i           :", np.abs(D_list).max())

# 4‑2 三条光束在基座系下的方向向量
k = np.array([0, 0, 1])
print("\nBeam directions (normalised):")
for T in T_BF_list:
    v = T[:3,:3] @ (R_FL @ k)
    print(np.round(v / np.linalg.norm(v), 4))

# 4‑3 Jacobian 条件数（用最小二乘优化返回的雅可比）
try:
    J = opt.jac                            # (3N × 7)
    cond = np.linalg.cond(J)
    print("\nJacobian cond :", "%.2e" % cond)
except AttributeError:
    print("Scipy <1.8 没有 opt.jac，可用有限差分或升级版本查看")

# ------------------------------------------------------------
# 5. 目标点推算示例
# ------------------------------------------------------------
print("\n===  目标点求解示例  ===")
T_BF_now = pose_to_T(
    -219.70,  -289.97,  138.82,
      0.1452,  -0.013, -0.1970
)
d_now = 129.1                                # 激光‑目标点实测距离 (m)

P_target_B = point_from_distance(T_BF_now, d_now, T_F_L)
print("P_target_B =", np.round(P_target_B, 3))