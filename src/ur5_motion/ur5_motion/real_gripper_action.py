#!/usr/bin/env python3
"""真机后端 gripper 中转节点：serial_gripper services → GripperCommand action。

与 isaac_gripper_action.py 同名同契（action ``lebai_gripper_controller/gripper_cmd``，
control_msgs/GripperCommand，position 0..π/3 rad 对应 gripper_r_joint1）——demo 的
GripperClient 不分后端，零改动。区别只在下层：isaac 版写 topic，本版调
lebai_driver/serial_gripper 的 services 并订 ``gripper/state``：

  goal(command.position rad, command.max_effort)
    → gripper/set_force + gripper/set_position（amplitude 0..100，lebai 惯例，
      rad↔amp 线性映射 π/3↔100，同 lebai_driver/gripper_action.py 的 _position_to_amplitude）
    → 轮询 gripper/state.position（amp）→ feedback（rad）/到位判定/stall 检测
  result: .position=实际开口(rad) .reached_goal .stalled .effort
    ——demo 的抓取验证（空夹≈0 / 夹住≈0.6 rad / 阈值 0.25）就吃这个。

同时把 gripper_r_joint1 转发到 /joint_states（20Hz）——MoveIt PlanningScene 要它，
mimic 关节由 RSP 按 URDF 派生（与 isaac 版同款，见其注释）。

⚠ 单位防线：若现场实测 gripper/state.position 不是 0..100 行程幅值（如 0..1 或 mm），
   改 amplitude_max 参数即可（ros2 run ... --ros-args -p amplitude_max:=1.0）。
⚠ 串口固定：serial_gripper 的 port 由 hardware.launch.py 的 gripper_port 传
   （udev 别名 /dev/lebai_gripper 最稳，见 docs/jetson_deployment.md §4.4）。

用法（真机栈里由 hardware.launch.py backend:=real 自动起）：
  ros2 run ur5_motion real_gripper_action
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import GripperCommand
from lebai_interfaces.msg import GripperState
from lebai_interfaces.srv import SetGripperForce, SetGripperPosition
from sensor_msgs.msg import JointState

# 与 isaac_gripper_action.py 完全一致的动作契约
ACTION_NAME = "lebai_gripper_controller/gripper_cmd"
DRIVEN_JOINT = "gripper_r_joint1"          # URDF 驱动关节（/joint_states 转发用）
MAX_POS_RAD = math.pi / 3.0                # 全开 = URDF gripper_r_joint1 上限

# serial_gripper 侧的 topic/service 名（相对名，随节点命名空间）
SET_POSITION_SRV = "gripper/set_position"
SET_FORCE_SRV = "gripper/set_force"
STATE_TOPIC = "gripper/state"


def rad_to_amp(rad: float, amp_max: float = 100.0) -> float:
    """gripper_r_joint1 弧度 → 串口夹爪行程幅值（0..amp_max）。"""
    rad_c = max(0.0, min(MAX_POS_RAD, float(rad)))
    return rad_c / MAX_POS_RAD * amp_max


def amp_to_rad(amp: float, amp_max: float = 100.0) -> float:
    """行程幅值 → 弧度（反馈方向）。"""
    return max(0.0, min(amp_max, float(amp))) / amp_max * MAX_POS_RAD


class RealGripperActionBridge:
    """把 serial_gripper（services+state topic）包成 GripperCommand action。

    类化（非 main 顶层）是为了单测：测试里用伪驱动节点提供同名 services/topic，
    同一 executor 上跑本桥，直接发 action 验证（无需真夹爪/串口）。
    """

    def __init__(self, node: rclpy.node.Node, *,
                 action_name: str = ACTION_NAME,
                 amplitude_max: float = 100.0,
                 tol_rad: float = 0.05,
                 stall_eps_amp: float = 0.5,
                 stall_cycles: int = 5,
                 poll_period: float = 0.1,
                 timeout_sec: float = 8.0,
                 publish_joint_states: bool = True):
        self.node = node
        self.log = node.get_logger()
        self.amp_max = float(amplitude_max)
        self.tol_rad = float(tol_rad)         # 到位容差（rad）——同 isaac 版
        self.stall_eps_amp = float(stall_eps_amp)  # 每周期幅值位移阈值（amp 单位）
        self.stall_cycles = int(stall_cycles)
        self.poll_period = float(poll_period)
        self.timeout_sec = float(timeout_sec)

        self._group = ReentrantCallbackGroup()
        self._state = {"amp": None, "connected": False, "force": 0.0}
        node.create_subscription(
            GripperState, STATE_TOPIC, self._on_state, 10, callback_group=self._group)

        self._pos_cli = node.create_client(
            SetGripperPosition, SET_POSITION_SRV, callback_group=self._group)
        self._force_cli = node.create_client(
            SetGripperForce, SET_FORCE_SRV, callback_group=self._group)

        self._js_pub = None
        if publish_joint_states:
            self._js_pub = node.create_publisher(JointState, "/joint_states", 10)
            node.create_timer(0.05, self._pub_joint_state, callback_group=self._group)

        self._server = ActionServer(
            node, GripperCommand, action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self.log.info(
            f"real_gripper_action: up — action '{action_name}' "
            f"(rad 0..{MAX_POS_RAD:.4f} ↔ amplitude 0..{self.amp_max:g}, "
            f"tol={self.tol_rad} rad, stall_eps={self.stall_eps_amp} amp × {self.stall_cycles})")

    # ── 下层状态 ────────────────────────────────────────────
    def _on_state(self, msg: GripperState):
        self._state["amp"] = float(msg.position)
        self._state["connected"] = bool(msg.connected)
        self._state["force"] = float(msg.force)

    def _pub_joint_state(self):
        """gripper_r_joint1(rad) → /joint_states（MoveIt 场景要；mimic 由 RSP 派生）。"""
        if self._js_pub is None or self._state["amp"] is None:
            return
        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = [DRIVEN_JOINT]
        js.position = [amp_to_rad(self._state["amp"], self.amp_max)]
        self._js_pub.publish(js)

    def _call_srv(self, cli, request, timeout: float = 2.0) -> bool:
        """call_async + 轮询（rclpy 禁在回调里同步 call()——会死锁；仓库同款模式）。"""
        fut = cli.call_async(request)
        deadline = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        return fut.done() and fut.result() is not None

    # ── action 回调 ─────────────────────────────────────────
    def _goal(self, goal_request: GripperCommand.Goal):
        self.log.info(f"goal accepted: position={goal_request.command.position:.4f} "
                      f"max_effort={goal_request.command.max_effort:.1f}")
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle: ServerGoalHandle):
        self.log.info("cancel requested")
        return rclpy.action.CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle):
        target_rad = max(0.0, min(MAX_POS_RAD, float(goal_handle.request.command.position)))
        effort = float(goal_handle.request.command.max_effort)

        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        # services 不在（serial_gripper 没起）→ 等 2s 后 abort
        if not (self._pos_cli.wait_for_service(timeout_sec=2.0)
                and self._force_cli.wait_for_service(timeout_sec=1.0)):
            self.log.error(f"{SET_POSITION_SRV}/{SET_FORCE_SRV} 不可用——serial_gripper 起了吗？")
            goal_handle.abort()
            result.stalled = False
            result.reached_goal = False
            return result

        # 力度（lebai 惯例 0..100；demo 默认传 50）
        if effort > 0.0:
            fr = SetGripperForce.Request()
            fr.force = int(max(0.0, min(self.amp_max, effort)))
            self._call_srv(self._force_cli, fr)

        pr = SetGripperPosition.Request()
        pr.position = int(round(rad_to_amp(target_rad, self.amp_max)))
        if not self._call_srv(self._pos_cli, pr):
            self.log.error("gripper/set_position 调用无响应（串口不通？）")
            goal_handle.abort()
            result.stalled = False
            result.reached_goal = False
            return result
        self.log.info(f"execute: target={target_rad:.4f} rad → amplitude={pr.position}")

        deadline = time.monotonic() + self.timeout_sec
        reached = False
        stalled = False
        prev_amp = None
        stall_count = 0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                break

            amp = self._state["amp"]
            cur = amp_to_rad(amp, self.amp_max) if amp is not None else None
            if cur is not None:
                feedback.position = cur
                goal_handle.publish_feedback(feedback)
                if abs(cur - target_rad) < self.tol_rad:
                    reached = True
                    break
                # stall：手指没动（夹住物体了）——真实夹爪持续压命令无益，判 stalled 收工
                if prev_amp is not None and abs(amp - prev_amp) < self.stall_eps_amp:
                    stall_count += 1
                    if stall_count >= self.stall_cycles:
                        stalled = True
                        self.log.info(
                            f"stall detected（夹住物体）: cur={cur:.4f} target={target_rad:.4f} rad")
                        break
                else:
                    stall_count = 0
                prev_amp = amp
            if time.monotonic() > deadline:
                self.log.warn(f"timeout: amp={self._state['amp']} target={target_rad:.4f} rad "
                              f"connected={self._state['connected']}")
                break
            time.sleep(self.poll_period)

        final_amp = self._state["amp"]
        result.position = amp_to_rad(final_amp, self.amp_max) if final_amp is not None else 0.0
        result.reached_goal = reached
        result.stalled = stalled
        result.effort = 0.0
        if not goal_handle.is_cancel_requested:
            goal_handle.succeed()
        self.log.info(f"execute done: reached={reached} stalled={stalled} "
                      f"final={result.position:.4f} rad")
        return result


def main():
    rclpy.init()
    node = rclpy.create_node("real_gripper_action")
    # amplitude_max 参数化 = 单位防线（现场行程幅值不是 0..100 时改它）
    node.declare_parameter("amplitude_max", 100.0)
    RealGripperActionBridge(
        node, amplitude_max=node.get_parameter("amplitude_max").value)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # SIGTERM 下 rclpy 可能已被信号处理器关过——destroy/shutdown 各自兜底，
        # 别让退出路径刷 Traceback（同 grasp_planner.shutdown 的防御样式）。
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
