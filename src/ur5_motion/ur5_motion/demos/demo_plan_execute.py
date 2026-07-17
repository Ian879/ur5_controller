"""端到端验证 demo：连 move_group → 规划+执行几个位姿。

前提：已启动运动栈
    ros2 launch ur5_bringup full_stack.launch.py backend:=mock

运行：
    ros2 run ur5_motion demo_plan_execute

成功表现：每个目标点打印 OK，无 ABORTED；RViz 中可见机器人按规划运动。
失败排查：见 docs/phase1_verification.md。
"""

import math
import time

import rclpy

from ur5_motion import UR5MoveIt

# 安全的已知可达位姿（UR5，base_link 系）
HOME_JOINTS = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]   # 经典 home（tool 朝上）
APPROACH_JOINTS = [0.0, -1.2, 1.2, -1.9, 0.0, 0.0]               # 一个不同的关节构型
# 笛卡尔点：工作空间内、保守距离（若 IK 失败请改这里）
CART_TARGET_POS = [0.35, 0.2, 0.45]
CART_TARGET_QUAT = [0.0, 0.0, 0.0, 1.0]  # 单位四元数


def _timed(label, fn, arm):
    log = arm.node.get_logger()
    log.info(f"==== {label} ====")
    t0 = time.monotonic()
    try:
        fn()
    except Exception as e:  # noqa: BLE001
        log.error(f"  [FAIL] {label}: {e}")
        return
    dt = time.monotonic() - t0
    log.info(f"  [OK] {label} 完成，用时 {dt:.2f}s")


def main():
    rclpy.init()
    with UR5MoveIt() as arm:
        log = arm.node.get_logger()
        log.info(f"当前关节: {[round(j, 3) for j in arm.get_current_joints()]}")

        # 先回 home（关节空间，必在限值内）
        _timed("回 home", lambda: arm.move_to_joint(HOME_JOINTS), arm)

        # 换一个构型
        _timed("到 approach 构型", lambda: arm.move_to_joint(APPROACH_JOINTS), arm)

        # 笛卡尔目标
        _timed(
            "到笛卡尔点",
            lambda: arm.move_to_pose(CART_TARGET_POS, CART_TARGET_QUAT),
            arm,
        )

        # 回 home 收尾
        _timed("回 home(收尾)", lambda: arm.move_to_joint(HOME_JOINTS), arm)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
