"""UR5 MoveIt2 规划客户端（pymoveit2 封装）。

提供同步式的「规划+执行」接口：调用即阻塞到该次运动完成。底层 pymoveit2 是异步的
（通过 move_group 的 /move_action 与 /execute_trajectory_action），本类在内部用一个
MultiThreadedExecutor 线程持续 spin。

典型用法::

    from ur5_motion import UR5MoveIt
    with UR5MoveIt() as arm:
        arm.move_to_joint([0, -1.5708, 0, -1.5708, 0, 0])     # 关节目标
        arm.move_to_pose([0.4, 0.0, 0.4], [0, 0, 0, 1])         # 笛卡尔目标(base_link)

注意：这些常量来自 ur_moveit_config 的 SRDF（已核实）：
  规划组 = ur_manipulator，chain base_link → tool0，命名位姿 home/up/test_configuration。

注意：rclpy 的 logger 只接受单条字符串（不支持 printf %s 多参数），故全程用 f-string。
"""

from __future__ import annotations

import threading
from typing import Optional, Sequence

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# ur_description 默认关节名（顺序对应 [base→tool0]）
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
BASE_LINK = "base_link"
END_EFFECTOR = "tool0"
GROUP_NAME = "ur_manipulator"


class UR5MoveIt:
    """对 pymoveit2.MoveIt2 的同步封装。"""

    def __init__(self, node: Optional[Node] = None, *, node_name: str = "ur5_moveit_client"):
        self._owns_node = node is None
        self.node = node if node is not None else rclpy.create_node(node_name)
        self._log = self.node.get_logger()

        # 延迟导入：pymoveit2 需 apt 安装（见 README）；未装时报清晰错误
        try:
            from pymoveit2 import MoveIt2
        except ImportError as e:  # pragma: no cover - 依赖缺失路径
            raise ImportError(
                "未找到 pymoveit2。请安装: sudo apt install ros-humble-pymoveit2"
            ) from e

        self._moveit = MoveIt2(
            node=self.node,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP_NAME,
            # 由 move_group 负责执行（下发到激活的轨迹控制器）。
            # use_move_group_action 是新参数名（旧的 execute_via_moveit 已弃用）。
            use_move_group_action=True,
        )

        # 后台 spin：pymoveit2 的 action client 需要持续 spin 才能收结果
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._log.info(f"UR5MoveIt 就绪: group={GROUP_NAME}, ee={END_EFFECTOR}")

    # ── 运动 ──────────────────────────────────────────────────────────
    def move_to_joint(self, joint_positions: Sequence[float]) -> None:
        """规划并执行到 6 关节目标（rad）。阻塞到执行完成。"""
        if len(joint_positions) != 6:
            raise ValueError(f"UR5 需要 6 个关节值，收到 {len(joint_positions)}")
        rounded = [round(j, 3) for j in joint_positions]
        self._log.info(f"move_to_joint: {rounded}")
        self._moveit.move_to_configuration(list(joint_positions))
        self._wait_executed("move_to_joint")

    def move_to_pose(
        self,
        position: Sequence[float],
        quat_xyzw: Sequence[float],
        frame_id: str = BASE_LINK,
    ) -> None:
        """规划并执行到 base_link（默认）下的笛卡尔位姿。阻塞到执行完成。

        position: [x, y, z]（米）
        quat_xyzw: [qx, qy, qz, qw]（单位四元数）
        """
        self._log.info(
            f"move_to_pose: pos={[round(p, 3) for p in position]} "
            f"quat_xyzw={[round(q, 3) for q in quat_xyzw]} frame={frame_id}"
        )
        self._moveit.move_to_pose(
            position=list(position), quat_xyzw=list(quat_xyzw), frame_id=frame_id
        )
        self._wait_executed("move_to_pose")

    # ── PlanningScene：加/删碰撞物体（委托 pymoveit2，复用同一 executor，无冲突）──
    def add_box(
        self,
        name: str,
        position: Sequence[float],
        size: Sequence[float] = (0.1, 0.1, 0.1),
        quat_xyzw: Optional[Sequence[float]] = None,
        frame_id: str = BASE_LINK,
    ) -> None:
        """往规划场景加一个盒子 CollisionObject。异步下发（后台 executor 处理）。

        quat_xyzw 缺省为单位四元数（pymoveit2 要求 position 与 quat_xyzw 成对给出）。
        """
        if quat_xyzw is None:
            quat_xyzw = (0.0, 0.0, 0.0, 1.0)
        self._moveit.add_collision_box(
            id=name,
            size=tuple(float(s) for s in size),
            position=tuple(float(p) for p in position),
            quat_xyzw=tuple(float(q) for q in quat_xyzw),
            frame_id=frame_id,
        )

    def remove_collision(self, name: str) -> None:
        """从规划场景删除指定 id 的碰撞物体。"""
        self._moveit.remove_collision_object(id=name)

    def cancel(self) -> None:
        """取消当前运动（pymoveit2 4.x 的方法是 cancel_execution）。"""
        try:
            self._moveit.cancel_execution()
        except Exception as e:  # noqa: BLE001
            self._log.warn(f"cancel 失败: {e}")

    def _wait_executed(self, label: str) -> None:
        """阻塞到执行完成；失败则抛错（让上层 try/except 可见）。"""
        ok = self._moveit.wait_until_executed()
        if not ok:
            try:
                err = self._moveit.get_last_execution_error_code()
            except Exception:
                err = "unknown"
            raise RuntimeError(f"{label} 执行失败 (error_code={err})")

    # ── 当前状态（便利读取，来自 joint_states）────────────────────────
    def get_current_joints(self) -> list[float]:
        """读当前关节角（来自 joint_states）。"""
        import time

        from sensor_msgs.msg import JointState

        holder: dict = {}

        def _cb(msg):
            if not holder:
                holder["msg"] = msg

        sub = self.node.create_subscription(JointState, "/joint_states", _cb, 10)
        try:
            end = time.monotonic() + 3.0
            while not holder and time.monotonic() < end:
                time.sleep(0.02)
        finally:
            self.node.destroy_subscription(sub)
        if not holder:
            return []
        msg = holder["msg"]
        name_to_pos = dict(zip(msg.name, msg.position))
        return [name_to_pos.get(n, 0.0) for n in JOINT_NAMES]

    # ── 生命周期 ──────────────────────────────────────────────────────
    def shutdown(self) -> None:
        # 先把节点从 executor 摘除、停 spin 线程，再销毁节点——避免
        # "terminate called without an active exception"（销毁时 executor 仍在 spin）。
        try:
            self._executor.remove_node(self.node)
        except Exception:
            pass
        self._executor.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        if self._owns_node and self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass

    def __enter__(self) -> "UR5MoveIt":
        return self

    def __exit__(self, *exc) -> None:
        self.shutdown()
