"""handeye_solver 合成数据单测（纯 numpy，无需 cv2/rclpy/硬件）。

造真值 X（flange←optical）与静态标定板，随机撒 N 个臂位姿，正推 Hg/Hc 喂求解器，
断言恢复精度——噪声-free 应到机器精度；小噪声应误差有界（旋转 <0.5°、平移 <1mm@1m 工况）。
"""
import math

import numpy as np
import pytest

from ur5_motion.handeye_solver import (
    _homog, _inv_homog, compose_mount, rot_from_rpy, rpy_from_rot, solve_hand_eye,
)

RNG = np.random.default_rng(20260928)


def _rand_rotation(rng, scale=1.0):
    """随机轴 × [-scale·π, scale·π] 角 → 旋转矩阵（Rodrigues）。"""
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    ang = rng.uniform(-math.pi, math.pi) * scale
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def _make_problem(n_poses, rng, noise_rot=0.0, noise_t=0.0):
    """造真值 → 正推 (Hg_list, Hc_list, X_true, T_b2t_true)。"""
    R_x = _rand_rotation(rng)
    t_x = rng.uniform(-0.1, 0.1, size=3) + np.array([0.0, 0.0, 0.06])
    X = _homog(R_x, t_x)                                    # flange←optical
    T_b2t = _homog(_rand_rotation(rng), np.array([0.6, 0.1, 0.4]))  # 静态板

    Hg_list, Hc_list = [], []
    for _ in range(n_poses):
        Hg = _homog(_rand_rotation(rng), rng.uniform([0.2, -0.3, 0.1], [0.7, 0.3, 0.6]))
        # 由 const = Hg·X·Hc 反解：Hc = X⁻¹ · Hg⁻¹ · T_base←target
        #（⚠ 不可换序——第一版把顺序写反，求解器全错，实证后修正）
        Hc = _inv_homog(X) @ _inv_homog(Hg) @ T_b2t
        if noise_rot or noise_t:
            # 小扰动 Δ 左乘（相机系检测噪声）。⚠ Δ 的平移就是小噪声本身——
            # 第一版写成 tc+noise（把 ~1m 原平移叠加了一遍），噪声量级完全失真。
            Hc = _homog(_rand_rotation(rng, noise_rot),
                        rng.normal(scale=noise_t, size=3)) @ Hc
        Hg_list.append(Hg)
        Hc_list.append(Hc)
    return Hg_list, Hc_list, X, T_b2t


def _rot_angle_deg(R_err):
    c = (np.trace(R_err) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def test_recovers_exact_no_noise():
    for n in (5, 8, 12):
        Hg, Hc, X, _ = _make_problem(n, RNG)
        R_x, t_x = solve_hand_eye(Hg, Hc)
        assert _rot_angle_deg(R_x @ X[:3, :3].T) < 1e-6, f"n={n} 旋转没恢复"
        assert np.allclose(t_x, X[:3, 3], atol=1e-6), f"n={n} 平移没恢复"


def test_robust_to_small_noise():
    """标定板检测 ~0.2mm/0.1° 量级噪声 → 误差应有界（旋转 <1°，平移 <2mm）。"""
    Hg, Hc, X, _ = _make_problem(12, RNG, noise_rot=0.002, noise_t=0.0003)
    R_x, t_x = solve_hand_eye(Hg, Hc)
    assert _rot_angle_deg(R_x @ X[:3, :3].T) < 1.0
    assert np.linalg.norm(t_x - X[:3, 3]) < 0.002


def test_too_few_poses_raises():
    Hg, Hc, _, _ = _make_problem(2, RNG)
    with pytest.raises(ValueError):
        solve_hand_eye(Hg, Hc)


def test_rpy_roundtrip():
    """URDF rpy ↔ R 往返（含一般位形 + 大角度）。"""
    for rpy in ([0.1, -0.2, 0.3], [-1.2, 0.4, 2.9], [0.0, 0.0, 0.0], [0.3, 1.5, -0.7]):
        R = rot_from_rpy(rpy)
        back = rpy_from_rot(R)
        assert np.allclose(back, rpy, atol=1e-9), f"{rpy} → {back}"
        assert np.allclose(rot_from_rpy(back), R, atol=1e-9)


def test_compose_mount():
    """X(optical) − link→optical 静态 TF → URDF mount：链式正推自洽。"""
    rng = np.random.default_rng(7)
    R_x, t_x = _rand_rotation(rng), np.array([0.01, -0.02, 0.058])
    T_link2opt = _homog(_rand_rotation(rng), np.array([0.001, 0.002, 0.015]))
    R_m, t_m = compose_mount(R_x, t_x, T_link2opt)
    # T_flange←link · T_link←optical 应等于 X
    T_check = _homog(R_m, t_m) @ T_link2opt
    assert np.allclose(T_check[:3, :3], R_x, atol=1e-9)
    assert np.allclose(T_check[:3, 3], t_x, atol=1e-9)
