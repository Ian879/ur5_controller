"""纯 numpy 手眼标定求解器（AX=XB 线性解，Andreff/Fleury 风格）。无 rclpy/cv2，可单测。

问题（eye-in-hand）：相机固定在腕上（tool0/flange），标定板固定在世界。N 个臂位姿下
各拍一次标定板。记
    Hg_i = T_base←gripper(i)   （TF base_link→tool0，i.e. gripper 在 base 系）
    Hc_i = T_cam←target(i)     （标定板在相机光学系）
相机与手的相对位姿 X = T_gripper←camera 恒定，且 T_base←target 恒定，故

    Hg_i · X · Hc_i = T_base←target   （对所有 i 相等）

取 i≠j 消去常数项得经典 AX=XB：
    A_ij · X = X · B_ij，  A_ij = Hg_j⁻¹·Hg_i（手运动），B_ij = Hc_j·Hc_i⁻¹（相机运动）

线性化（Kronecker 直积，旋转+平移解耦）：
    旋转：Ra Rx = Rx Rb  →  (I⊗Ra − Rbᵀ⊗I)·vec(Rx) = 0，堆叠所有对 → SVD 取零空间，
          再投影回 SO(3)（极分解）。
    平移：(Ra − I)·tx = Rx·tb，堆叠所有对 → 最小二乘。

要求：N ≥ 5 对、旋转轴多样（同轴转再多也欠定）。返回 (R_x, t_x)：T_gripper←camera_optical。
用法（节点侧）：handeye_calib.py 采到 (Hg, Hc) 列表后调 solve_hand_eye()。
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np


def _homog(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _inv_homog(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _project_to_so3(M: np.ndarray) -> np.ndarray:
    """最近的旋转矩阵（极分解：M = U S Vᵀ → R = U Vᵀ，det 修正）。"""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:            # 反射 → 翻正
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt
    return R


def quat_to_rotmat(q: Sequence[float]) -> np.ndarray:
    """四元数 [x,y,z,w] → 3x3 旋转矩阵。

    自带而不 import grasp_geometry 的同名函数——那边的 v2 版本尚未提交，
    Jetson 从远端 clone 时拿不到（2026-10-09 实测 HEAD 里没有），标定代码
    不该依赖抓取侧的 WIP。"""
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_from_rpy(rpy: Sequence[float]) -> np.ndarray:
    """URDF 固定轴 rpy (x,y,z) → R = Rz(yaw)·Ry(pitch)·Rx(roll)。"""
    rx, ry, rz = (float(v) for v in rpy)
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rpy_from_rot(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → URDF 固定轴 rpy（Z·Y·X 欧拉分解）。"""
    R = np.asarray(R, float)
    sy = -R[2, 0]
    sy = float(np.clip(sy, -1.0, 1.0))
    if abs(sy) < 0.999999:
        pitch = np.arcsin(sy)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:                                  # 万向锁：取 yaw=0
        pitch = np.pi / 2 * np.sign(sy)
        roll = np.arctan2(-R[1, 2] * np.sign(sy), R[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def solve_hand_eye(Hg_list: List[np.ndarray], Hc_list: List[np.ndarray],
                   min_pair_angle_deg: float = 10.0,
                   ) -> Tuple[np.ndarray, np.ndarray]:
    """eye-in-hand 手眼标定。Hg_i/Hc_i = 4x4 齐次（见模块 docstring 的方向约定）。

    返回 (R_x, t_x)：T_gripper←camera。N<3 抛错；建议 N≥5 且位姿旋转轴多样。
    min_pair_angle_deg：丢弃相对转角小于它的位姿对——小转角对的 (Ra−I) 行近乎零，
    平移最小二乘被其放大噪声（0.3mm 检测噪声实测放大到 9cm 的教训）。
    """
    if len(Hg_list) != len(Hc_list):
        raise ValueError("Hg/Hc 数量不一致")
    n = len(Hg_list)
    if n < 3:
        raise ValueError(f"至少 3 对位姿（建议 ≥5，旋转轴多样），当前 {n}")

    # 组对（i<j）并过滤小转角对
    kron_rows = []
    A_list, B_list = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A = _inv_homog(Hg_list[j]) @ Hg_list[i]      # 手运动
            B = Hc_list[j] @ _inv_homog(Hc_list[i])      # 相机运动
            Ra, Rb = A[:3, :3], B[:3, :3]
            cos_a = (np.trace(Ra) - 1.0) / 2.0
            if math.degrees(math.acos(float(np.clip(cos_a, -1.0, 1.0)))) < min_pair_angle_deg:
                continue
            A_list.append(A)
            B_list.append(B)
            # numpy reshape 是行主序 vec；列主序恒等式 (I⊗Ra − Rbᵀ⊗I) 的因子要互换：
            # Ra Rx = Rx Rb ⇔ (Ra⊗I − I⊗Rbᵀ)·vec_row(Rx) = 0（实证合成数据 ~1e-15 零化）
            kron_rows.append(np.kron(Ra, np.eye(3)) - np.kron(np.eye(3), Rb.T))
    if not kron_rows:
        raise ValueError(f"过滤 <{min_pair_angle_deg}° 的对后一无所有——位姿太相近，重采")

    M = np.vstack(kron_rows)
    _, s, Vt = np.linalg.svd(M)
    v = Vt[-1]                                 # 零空间（最小奇异值方向）
    M0 = v.reshape(3, 3)
    # 齐次解符号/尺度歧义：v ∝ ±vec(R_true) 且范数任意 → 归一到 Frobenius √3，
    # det 定符号（+R 是 det=+1 的真解；−R 是 det=−1 伪解，极分解出 180° 乱值——
    # 用迹定符号会在真旋转 >104°（迹<0）时选错边，已实证），最后投影 SO(3)。
    M0 = M0 / np.linalg.norm(M0) * np.sqrt(3.0)
    if np.linalg.det(M0) < 0.0:
        M0 = -M0
    Rx = _project_to_so3(M0)

    # 平移：绝对位姿式（比配对式 (Ra−I)tx 好得多——后者小转角行近乎零，0.3mm 检测
    # 噪声实测放大到 18cm；且换式后每个位姿噪声只进一次）。
    # 由 const = Hg·X·Hc：t_const = Rg·(Rx·tc + tx) + tg
    #   ⇒ [Rg_i | −I]·[tx; t_const] = −Rg_i·Rx·tc_i − tg_i（对 tx、t_const 线性）
    t_rows, t_rhs = [], []
    for Hg, Hc in zip(Hg_list, Hc_list):
        Rg, tg = Hg[:3, :3], Hg[:3, 3]
        tc = Hc[:3, 3]
        t_rows.append(np.hstack([Rg, -np.eye(3)]))
        t_rhs.append(-Rg @ (Rx @ tc) - tg)
    sol, *_ = np.linalg.lstsq(np.vstack(t_rows), np.concatenate(t_rhs), rcond=None)
    return Rx, sol[:3]


def compose_mount(R_cam2gripper: np.ndarray, t_cam2gripper: np.ndarray,
                  T_camera_link_from_optical: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """标定结果 → URDF flange_to_camera_link 的 mount (R, t)。

    solve 给的是 T_flange←camera_optical；URDF joint 要 T_flange←camera_link。
    T_flange←link = T_flange←optical · T_optical←link = X · inv(T_link←optical)。
    T_camera_link_from_optical = 相机 URDF 里 link→optical 的静态 TF（运行时从 TF 树查）。
    """
    T_x = _homog(R_cam2gripper, t_cam2gripper)
    T_mount = T_x @ _inv_homog(np.asarray(T_camera_link_from_optical, float))
    return T_mount[:3, :3], T_mount[:3, 3]
