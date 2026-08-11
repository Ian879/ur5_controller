#!/usr/bin/env python3
"""IK 探针：对若干高度的工具位姿查 IK 可达性（不动机器人）。
用 /compute_ik service。frame base_link, group ur_manipulator, tcp 默认。
"""
import rclpy, math, sys
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (PositionIKRequest, RobotState, Constraints,
                             OrientationConstraint)
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

GRASP_QUAT = [0.0, 1.0, 0.0, 0.0]
XY = [0.45, 0.05]
HEIGHTS = [0.20, 0.12, 0.06, 0.0, -0.05, -0.10, -0.15]

rclpy.init()
n = Node("ik_probe")
cli = n.create_client(GetPositionIK, "/compute_ik")
print("等 compute_ik 服务...", flush=True)
cli.wait_for_service(timeout_sec=10.0)

# 拿当前 joint state 作为 IK 起始
js = None
from rclpy.qos import QoSProfile
def _js(msg): global js; js = msg
n.create_subscription(JointState, "/joint_states", _js, 10)
import time; t0=time.time()
while js is None and time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)

for z in HEIGHTS:
    req = GetPositionIK.Request()
    r = PositionIKRequest()
    r.group_name = "ur_manipulator"
    r.ik_link_name = "tool0"
    r.pose_stamped = PoseStamped()
    r.pose_stamped.header.frame_id = "base_link"
    r.pose_stamped.pose = Pose()
    r.pose_stamped.pose.position.x = XY[0]
    r.pose_stamped.pose.position.y = XY[1]
    r.pose_stamped.pose.position.z = z
    r.pose_stamped.pose.orientation.x = GRASP_QUAT[0]
    r.pose_stamped.pose.orientation.y = GRASP_QUAT[1]
    r.pose_stamped.pose.orientation.z = GRASP_QUAT[2]
    r.pose_stamped.pose.orientation.w = GRASP_QUAT[3]
    r.robot_state = RobotState()
    if js is not None:
        r.robot_state.joint_state = js
    # humble PositionIKRequest 无 timeout_sec；用 fut 轮询超时控制
    req.ik_request = r
    fut = cli.call_async(req)
    t0=time.time()
    while not fut.done() and time.time()-t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
    if fut.done():
        res = fut.result()
        err = res.error_code.val
        ok = (err == 1)
        print(f"z={z:+.2f}: {'✅ IK 可达' if ok else f'❌ 不可达 (err={err})'}", flush=True)
    else:
        print(f"z={z:+.2f}: ⏰ 超时", flush=True)
n.destroy_node(); rclpy.shutdown()
