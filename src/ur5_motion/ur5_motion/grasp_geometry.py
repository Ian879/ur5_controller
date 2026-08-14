"""纯 numpy 抓取几何：倾斜接近帧 + grasp/pre 位姿。无 rclpy 依赖，可单测。

设计（Eye-in-Hand 侧抓，解决「TCP=物体中心 Z → 夹爪扎桌面」）：
  - reach 方向 a 带下倾角 tilt（0=水平，>0 朝下）。tool0+Z = a。
  - 倾斜后夹爪本体沿 -a 抬高 offset·sin(tilt)，离桌面；指尖仍落物体中心。
  - 手指夹合轴 tool0+X = up×a 恒水平（任意 tilt），夹垂直圆柱腰部。
夹爪几何（URDF 实测）：GRIPPER_TIP_OFFSET=0.17127（tool0→tcp 沿 +Z），手指沿 tool0+X。
"""
from __future__ import annotations
import math
from typing import List, Sequence, Tuple
import numpy as np

GRIPPER_TIP_OFFSET = 0.17127
APPROACH_MARGIN = 0.06


def tilted_reach(azim: float, tilt: float) -> np.ndarray:
    """方位 azim（绕 world +z）、下倾角 tilt（0=水平）→ 单位 reach 向量 a。"""
    ct = math.cos(tilt)
    return np.array([math.cos(azim) * ct, math.sin(azim) * ct, -math.sin(tilt)])


def _rotmat_to_quat(R: np.ndarray) -> List[float]:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return [(R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s, 0.25*s]
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return [0.25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s, (R[2, 1]-R[1, 2])/s]
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return [(R[0, 1]+R[1, 0])/s, 0.25*s, (R[1, 2]+R[2, 1])/s, (R[0, 2]-R[2, 0])/s]
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return [(R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, 0.25*s, (R[1, 0]-R[0, 1])/s]


def quat_from_reach(a: Sequence[float]) -> List[float]:
    """reach a → tool0 朝向 [x,y,z,w]。tool0+Z=a, tool0+X=up×a（水平手指）, tool0+Y=a×(up×a)。"""
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, a); xn = np.linalg.norm(x)
    if xn < 1e-6:
        return [0.0, 1.0, 0.0, 0.0] if a[2] < 0 else None   # 纯竖直退化 → 顶部朝向
    x = x / xn; y = np.cross(a, x)
    return _rotmat_to_quat(np.column_stack([x, y, a]))


def grasp_poses(obj_pos: Sequence[float], reach: Sequence[float],
                quat: Sequence[float]) -> Tuple[List[float], List[float]]:
    """物体中心 + reach + quat → (tool0_grasp, tool0_pre)。TCP 落物体中心。"""
    a = np.asarray(reach, float); obj = np.asarray(obj_pos, float)
    grasp = (obj - GRIPPER_TIP_OFFSET * a).tolist()
    pre = (obj - (GRIPPER_TIP_OFFSET + APPROACH_MARGIN) * a).tolist()
    return grasp, pre


def grasp_cost(sol: Sequence[float], seed: Sequence[float], tilt: float, *,
               w_dist: float = 1.0, w_wrist: float = 0.5, w_tilt: float = 2.0) -> float:
    """多因子 cost：关节运动量 + wrist_3 旋转 + 倾角。小 = 好。wrist_3 单独罚（防乱转）。"""
    sol = np.asarray(sol, float); seed = np.asarray(seed, float)
    dist = float(np.sum((sol[:5] - seed[:5]) ** 2))
    wrist = float(abs(sol[5] - seed[5]))
    wrist = abs((wrist + math.pi) % (2 * math.pi) - math.pi)   # 包到 [-π,π] 距离
    tilt_pen = abs(tilt) / (math.pi / 2)
    return w_dist * dist + w_wrist * wrist + w_tilt * tilt_pen
