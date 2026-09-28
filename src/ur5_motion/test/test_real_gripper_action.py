"""real_gripper_action 桥的集成测试（伪驱动，无需真夹爪/串口）。

FakeGripperNode 提供 serial_gripper 同名接口（gripper/set_position、set_force
services + gripper/state 10Hz），内部模拟手指以有限速度走向目标；blocked 模式下
停在 hold_amp 模拟夹住物体。桥（RealGripperActionBridge）与其同 executor 跑，
测试发 GripperCommand action 断言 result 契约——正是 demo 抓取验证吃到的字段。

跑法（系统 python + humble + 本仓库 overlay）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m pytest src/ur5_motion/test/test_real_gripper_action.py -q -p no:anyio
"""
import math
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionClient  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from control_msgs.action import GripperCommand  # noqa: E402
from lebai_interfaces.msg import GripperState  # noqa: E402
from lebai_interfaces.srv import SetGripperForce, SetGripperPosition  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from ur5_motion.real_gripper_action import (  # noqa: E402
    ACTION_NAME, RealGripperActionBridge, amp_to_rad, rad_to_amp,
)

MAX_RAD = math.pi / 3


# ── 纯函数：rad↔amplitude 映射 ───────────────────────────────
def test_rad_amp_mapping_roundtrip():
    assert rad_to_amp(0.0) == 0.0
    assert rad_to_amp(MAX_RAD) == pytest.approx(100.0)
    assert amp_to_rad(100.0) == pytest.approx(MAX_RAD)
    # 抓取验证的关键点位：夹住 6.5cm 罐 ≈0.59 rad ↔ amp 56.5
    assert rad_to_amp(0.59) == pytest.approx(0.59 / MAX_RAD * 100.0, abs=0.01)
    assert amp_to_rad(rad_to_amp(0.59)) == pytest.approx(0.59, abs=1e-9)
    # 越界 clamp
    assert rad_to_amp(-1.0) == 0.0
    assert rad_to_amp(9.9) == 100.0
    assert amp_to_rad(1e9) == pytest.approx(MAX_RAD)


# ── 伪驱动：serial_gripper 的行为替身 ────────────────────────
class FakeGripper:
    """模拟手指运动学：amp 以 speed_amp/s 走向 target；blocked 时停在 hold_amp。

    services/topic 与 lebai_driver.serial_gripper_node 同名同型（相对名），
    在独立 node 上注册——桥不知道对面是真驱动还是它。"""

    def __init__(self, node, *, speed_amp=400.0, start_amp=100.0):
        self.node = node
        self.speed = speed_amp
        self.amp = start_amp
        self.target = start_amp
        self.force_cmds = []
        self.pos_cmds = []
        self.blocked = False      # True：amp 冻结（夹住物体）
        grp = ReentrantCallbackGroup()
        node.create_service(SetGripperPosition, "gripper/set_position",
                            self._on_pos, callback_group=grp)
        node.create_service(SetGripperForce, "gripper/set_force",
                            self._on_force, callback_group=grp)
        self._pub = node.create_publisher(GripperState, "gripper/state", 10)
        node.create_timer(0.02, self._tick, callback_group=grp)

    def _on_pos(self, req, resp):
        self.pos_cmds.append(int(req.position))
        self.target = float(req.position)
        return resp

    def _on_force(self, req, resp):
        self.force_cmds.append(int(req.force))
        return resp

    def _tick(self):
        if not self.blocked:
            step = self.speed * 0.02
            self.amp += max(-step, min(step, self.target - self.amp))
        msg = GripperState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.connected = True
        msg.position = float(self.amp)
        msg.force = 50.0
        self._pub.publish(msg)


class _Stack:
    """一个进程内拉起：伪驱动 node + 桥 node + executor 线程。"""

    def __init__(self, *, fake_kwargs=None):
        rclpy.init()
        self.exec = MultiThreadedExecutor()
        self.fake_node = rclpy.create_node("fake_serial_gripper")
        self.fake = FakeGripper(self.fake_node, **(fake_kwargs or {}))
        self.bridge_node = rclpy.create_node("real_gripper_action_test")
        # 短超时让 stall/timeout 用例快；tol 同默认
        self.bridge = RealGripperActionBridge(
            self.bridge_node, poll_period=0.02, timeout_sec=2.0,
            stall_eps_amp=0.5, stall_cycles=3)
        self.exec.add_node(self.fake_node)
        self.exec.add_node(self.bridge_node)
        self._th = threading.Thread(target=self.exec.spin, daemon=True)
        self._th.start()
        self.client = ActionClient(self.bridge_node, GripperCommand, ACTION_NAME)
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("action server 没起来")

    def send(self, position, effort=50.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        fut = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        handle = fut.result()
        assert handle is not None and handle.accepted, "goal 被拒"
        rf = handle.get_result_async()
        deadline = time.monotonic() + 6.0
        while not rf.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return handle, rf.result().result

    def teardown(self):
        self.exec.shutdown()
        self.bridge_node.destroy_node()
        self.fake_node.destroy_node()
        rclpy.shutdown()


@pytest.fixture(scope="module")
def stack():
    s = _Stack()
    yield s
    s.teardown()


def test_close_reaches(stack):
    """开→合（target 0 rad）：应 reached，result.position≈0。"""
    _, res = stack.send(0.0)
    assert res.reached_goal is True
    assert res.stalled is False
    assert res.position == pytest.approx(0.0, abs=0.05)
    assert stack.fake.pos_cmds[-1] == 0          # set_position 收到 amplitude 0
    assert stack.fake.force_cmds[-1] == 50       # max_effort 透传 set_force


def test_open_partial_reaches(stack):
    """合→半开（0.59 rad ≈ 夹罐口径）：reached 且 rad 往返映射正确。"""
    stack.fake.amp = 0.0
    stack.fake.target = 0.0
    time.sleep(0.1)
    _, res = stack.send(0.59)
    assert res.reached_goal is True
    assert res.position == pytest.approx(0.59, abs=0.05)
    # set_position 收到的 amplitude ≈ 0.59/(π/3)*100 ≈ 56.5
    assert stack.fake.pos_cmds[-1] == pytest.approx(56.5, abs=1.0)


def test_blocked_reports_stall_and_opening(stack):
    """核心契约：合向被夹住的物体（amp 冻在 ≈56.5）→ stalled=True，
    result.position=实际开口 rad —— demo 的抓取验证（≥0.25 rad 判夹住）吃这个。"""
    stack.fake.blocked = True                     # 冻结在当前 amp
    stack.fake.amp = 56.5
    stack.fake.target = 56.5
    time.sleep(0.1)
    try:
        _, res = stack.send(0.0)                  # 命令全合
        assert res.stalled is True
        assert res.reached_goal is False
        opening = res.position
        assert opening == pytest.approx(amp_to_rad(56.5), abs=0.03)
        assert opening >= 0.25                    # 抓取验证阈值语境
    finally:
        stack.fake.blocked = False
        stack.fake.target = 100.0                 # 还原到全开，供后续用例
        stack.fake.amp = 100.0


def test_joint_states_forwarded(stack):
    """gripper_r_joint1 应被转发到 /joint_states（MoveIt 场景契约）。"""
    from rclpy.qos import QoSProfile
    got = {}

    def on_js(msg):
        if "gripper_r_joint1" in msg.name:
            got["pos"] = msg.position[msg.name.index("gripper_r_joint1")]

    sub = stack.bridge_node.create_subscription(
        JointState, "/joint_states", on_js, 10)
    deadline = time.monotonic() + 2.0
    while "pos" not in got and time.monotonic() < deadline:
        time.sleep(0.02)
    stack.bridge_node.destroy_subscription(sub)
    assert "pos" in got, "2s 内没收到 gripper_r_joint1 的 /joint_states"
    assert 0.0 <= got["pos"] <= MAX_RAD + 1e-6
