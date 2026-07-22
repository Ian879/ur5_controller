#!/usr/bin/env python3
"""Isaac 后端 gripper 中转节点（Task 5）。

暴露 control_msgs/GripperCommand action（名 ``lebai_gripper_controller/gripper_cmd``，
与 real 的 lebai_driver / MoveIt 的 GripperCommand 句柄同名同契），goal→写
``/isaac_gripper_joint_commands``（gripper_r_joint1 目标），到位（读
``/isaac_gripper_joint_states``）后 result。让 MoveIt 的 GripperCommand 句柄能控
Isaac Sim 里的 Lebai 夹爪 articulation。

设计：
  - MultiThreadedExecutor + ReentrantCallbackGroup —— action 的 execute_callback
    长轮询期间，executor 仍能在另一线程上 dispatch ``/isaac_gripper_joint_states``
    订阅回调更新当前位姿（单线程执行器会死锁：execute 占着线程，state 永不更新）。
  - 命令 topic 用 JointState（与桥的 ROS2SubscribeJointState → ArticulationController
    对齐）；执行期间周期重发目标，保证桥在 tick 时收到最新命令。
  - GripperCommand 的 position (0..π/3) 与 gripper_r_joint1 的弧度范围一一对应
    （URDF/SRDF 两者同契），故直接 clamp 后透传，无需额外映射。
  - 到位判据 abs(current - target) < TOL；支持 preempt（取消）提前退出。

关节名说明：~/Desktop/scene.usd 里 gripper articulation 的驱动关节 prim leaf 名即
``gripper_r_joint1``（physics:jointName 为 None，Isaac 用 prim leaf 名），与 URDF
一致，故 DRIVEN_JOINT 无需映射。若场景换名，改本常量即可。
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

# MoveIt/URDF 驱动关节名（也是 ~/Desktop/scene.usd 里 gripper articulation 的关节名）。
DRIVEN_JOINT = "gripper_r_joint1"
# 全开 = URDF gripper_r_joint1 上限 π/3。
MAX_POS = math.pi / 3.0  # 1.0471975511965976
TOL = 0.05  # 到位容差（弧度）

CMD_TOPIC = "/isaac_gripper_joint_commands"
STATE_TOPIC = "/isaac_gripper_joint_states"
ACTION_NAME = "lebai_gripper_controller/gripper_cmd"

# 执行循环里目标重发的周期与最大等待时间。
CMD_REPUBLISH_PERIOD = 0.1  # s
# 到位前的最长等待（避免桥不发状态时永久挂起）。
MAX_WAIT_SEC = 10.0


def main():
    rclpy.init()
    node = rclpy.create_node("isaac_gripper_action")
    node.get_logger().info(
        f"isaac_gripper_action: up — exposing GripperCommand action '{ACTION_NAME}' "
        f"(driven joint={DRIVEN_JOINT}, max={MAX_POS:.4f} rad, tol={TOL})"
    )

    # 单个 reentrant 组让订阅回调与 action execute 在不同线程上并发跑。
    cb_group = ReentrantCallbackGroup()

    pub = node.create_publisher(JointState, CMD_TOPIC, 10)
    state = {"pos": 0.0}

    def on_state(msg: JointState):
        try:
            i = msg.name.index(DRIVEN_JOINT)
            state["pos"] = float(msg.position[i])
        except (ValueError, IndexError):
            # 桥发的状态里没有我们要的关节名，或 position 数组长度不足——忽略
            # 这一帧（桥可能还没 attach 夹爪，或状态帧残缺）。
            pass

    node.create_subscription(
        JointState, STATE_TOPIC, on_state, 10, callback_group=cb_group
    )

    def goal_callback(goal_request: GripperCommand.Goal):
        # 总是接受（gripper 只有开/合两个方向，无复杂规划拒绝逻辑）。
        # 注意 rclpy 的 goal_callback 直接收到 Goal 消息（非 ServerGoalHandle），
        # 故访问路径是 goal_request.command.position，不是 .request.command.*。
        node.get_logger().info(
            f"goal accepted: position={goal_request.command.position:.4f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(goal_handle: ServerGoalHandle):
        node.get_logger().info("cancel requested — aborting current goal")
        return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(goal_handle: ServerGoalHandle):
        target = max(0.0, min(MAX_POS, float(goal_handle.request.command.position)))
        node.get_logger().info(
            f"execute: target={target:.4f} (clamped from {goal_handle.request.command.position:.4f})"
        )

        js = JointState()
        js.name = [DRIVEN_JOINT]
        js.position = [target]

        feedback = GripperCommand.Feedback()
        result = GripperCommand.Result()

        # 周期重发命令 + 轮询当前位姿。executor 在另一线程上 dispatch on_state 更新 state。
        deadline = node.get_clock().now().nanoseconds + int(MAX_WAIT_SEC * 1e9)
        reached = False
        while rclpy.ok():
            # 抢占/取消处理。
            if goal_handle.is_cancel_requested:
                node.get_logger().info("preempted/cancelled — exiting execute loop")
                goal_handle.canceled()
                break

            js.header.stamp = node.get_clock().now().to_msg()
            pub.publish(js)

            feedback.position = state["pos"]
            goal_handle.publish_feedback(feedback)

            if abs(state["pos"] - target) < TOL:
                reached = True
                break
            if node.get_clock().now().nanoseconds > deadline:
                node.get_logger().warn(
                    f"timeout waiting for gripper to reach target "
                    f"(current={state['pos']:.4f} target={target:.4f})"
                )
                break
            # 让出 CPU；MultiThreadedExecutor 在另一线程上 dispatch on_state。
            # NOTE: 此重发节拍用 wall-clock time.sleep，而上面的 deadline / stamp 用
            # node.get_clock().now()（use_sim_time:=true 下随仿真时钟）——两者口径不同，
            # 但 0.1s 节拍下无实质影响，勿误以为两者都是仿真时间。
            time.sleep(CMD_REPUBLISH_PERIOD)

        result.position = state["pos"]
        result.reached_goal = reached
        result.stalled = False
        result.effort = 0.0
        if not goal_handle.is_cancel_requested:
            goal_handle.succeed()
        node.get_logger().info(
            f"execute done: reached={reached} final={state['pos']:.4f}"
        )
        return result

    ActionServer(
        node,
        GripperCommand,
        ACTION_NAME,
        execute_callback=execute_callback,
        goal_callback=goal_callback,
        cancel_callback=cancel_callback,
        callback_group=cb_group,
    )

    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
