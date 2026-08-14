"""对象感知抓取规划：候选生成 + /compute_ik 过滤。

输入：物体位姿(base_link) + 包围盒尺寸 + 形状类（cylinder/box）。
输出：最优 GraspCandidate（tool0 抓取位姿 + 接近方向 + pre-grasp 位姿），经 /compute_ik
      过滤（pick_ik 自动遵循 joint_limits_pilz.yaml 的 shoulder_lift 等限位）。

为什么圆柱体优先【侧面抓】而非顶部抓：
  - 侧面抓 = 水平接近，手指夹罐身中段。接近平面是水平的 → 只依赖 FP 的 xy 精度（够用），
    对 FP 的 z 误差（实测 ~10cm）不敏感——水平直进在罐身任意高度都能夹到，不会因 z 偏而
    撞桌/扑空。
  - 顶部抓 = 垂直下探，依赖精确知道桌面/罐顶 z；FP z 偏 10cm → 直接撞桌散架（已踩坑）。
  故圆柱体先生成 8 方位侧面候选，顶部候选仅作兜底（所有侧抓 IK 不可达时才用）。

夹爪几何（来自 ur5_with_gripper.urdf.xacro + gripper_macro.xacro，实测无转角）：
  - tool0→gripper_base 纯 Z 平移 0.01267，无旋转 → 夹爪轴系与 tool0 对齐。
  - tool0→gripper_tip_tcp 纯 Z 平移 GRIPPER_TIP_OFFSET=0.17127（手指夹角净 0）。
  - 两指在 tool0 ±x（base x=±0.025）→ 手指夹合方向 = tool0 X 轴。
  - 夹爪「伸够」方向 = tool0 +Z（TCP 沿 +Z 在 tool0 前方）。
由这些约定推侧面抓朝向（见 _quat_from_reach）：reach 方向 a（水平）= tool0+Z 在 world 的指向；
  tool0-X 取 world-up × a（水平、⊥ a）→ 手指水平夹合，正好夹垂直圆柱的腰部。

用法（demo 内嵌）::

    from ur5_motion.grasp_planner import GraspPlanner
    planner = GraspPlanner()
    cand = planner.plan(obj_pos, OBJECT_SIZE, shape="cylinder")
    arm.move_to_pose(cand.tool0_pre, cand.quat)        # pre-grasp
    arm.move_to_pose(cand.tool0_grasp, cand.quat, cartesian=True)  # 水平直进

standalone 诊断（不动机器人，只查 IK 可达性）::

    ros2 run ur5_motion grasp_plan            # 默认读 grasp_target TF
    POSE_SOURCE=fp ros2 run ur5_motion grasp_plan   # 读 fp_object TF
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .grasp_geometry import (tilted_reach, quat_from_reach, grasp_poses, grasp_cost,
                             GRIPPER_TIP_OFFSET, APPROACH_MARGIN)

# ── 夹爪/抓取几何参数（与 demo_grasp.py 一致；改这里调）──────────────
TOP_QUAT = [0.0, 1.0, 0.0, 0.0]     # 顶部抓：绕 y π，tool0+Z 朝下，手指水平
GROUP_NAME = "ur_manipulator"
IK_LINK = "tool0"
ARM_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
# 顶部兜底候选的排序惩罚（>0 才会在所有侧抓失败后被选中）
TOP_FALLBACK_PRIOR = 5.0
# 侧面抓候选数（每 360/NZ 一个方位；圆柱径向对称，方位只影响 IK 可达性）
N_SIDE_AZIMUTH = 8
# 侧抓下倾角候选（0=水平；>0 朝下，避夹爪扎桌面）。碰撞感知 IK 会剔掉仍碰桌面的。
SIDE_TILTS = [math.radians(t) for t in (0.0, 30.0, 45.0, 60.0)]
IK_TIMEOUT = 3.0                    # 单次 /compute_ik 的 wall 轮询超时（秒）


@dataclass
class GraspCandidate:
    """单个抓取候选（全部位姿在 base_link，对 tool0）。"""
    label: str
    kind: str                       # "side" | "top"
    reach: np.ndarray               # 伸够方向 a（tool0+Z 在 world，单位）—— 接近/退避轴
    quat: List[float]               # tool0 朝向 [x,y,z,w]
    tool0_grasp: List[float]        # 抓取位 tool0 xyz
    tool0_pre: List[float]          # pre-grasp 位 tool0 xyz（沿 -a 退 APPROACH_MARGIN）
    prior: float = 0.0              # 排序先验（小优先；顶部兜底加大惩罚）
    tilt: float = 0.0
    ik_joints: Optional[List[float]] = field(default=None, repr=False)


def _side_candidates(obj_pos: Sequence[float], n_azim: int) -> List[GraspCandidate]:
    """圆柱侧抓候选：方位 × 下倾角。TCP 落物体中心，tool0 退 OFFSET·a。倾角让夹爪本体离桌面。"""
    cands: List[GraspCandidate] = []
    for i in range(n_azim):
        phi = 2.0 * math.pi * i / n_azim
        for tilt in SIDE_TILTS:
            a = tilted_reach(phi, tilt)
            quat = quat_from_reach(a)
            if quat is None:
                continue
            tool0_grasp, tool0_pre = grasp_poses(obj_pos, a, quat)
            cands.append(GraspCandidate(
                label=f"side@{math.degrees(phi):.0f}°/tilt{math.degrees(tilt):.0f}°",
                kind="side", reach=a, quat=quat,
                tool0_grasp=tool0_grasp, tool0_pre=tool0_pre, tilt=tilt))
    return cands


def _top_candidate(obj_pos: Sequence[float], obj_size: Sequence[float]) -> GraspCandidate:
    """顶部抓兜底候选（复刻 demo_grasp 原算术）：tool0+Z 朝下，TCP 在物体中心。"""
    half_h = obj_size[2] / 2.0
    grasp_z = obj_pos[2] + GRIPPER_TIP_OFFSET
    pre_z = obj_pos[2] + half_h + APPROACH_MARGIN + GRIPPER_TIP_OFFSET
    a = np.array([0.0, 0.0, -1.0])
    return GraspCandidate(
        label="top(fallback)",
        kind="top", reach=a, quat=list(TOP_QUAT),
        tool0_grasp=[obj_pos[0], obj_pos[1], grasp_z],
        tool0_pre=[obj_pos[0], obj_pos[1], pre_z],
        prior=TOP_FALLBACK_PRIOR,
    )


def generate_candidates(
    obj_pos: Sequence[float],
    obj_size: Sequence[float],
    shape: str = "cylinder",
) -> List[GraspCandidate]:
    """物体位姿+尺寸+形状 → 排序好的候选列表（先优先型，兜底在后）。

    cylinder：侧面抓 ×N（优先）+ 顶部兜底。
    box：顶部抓（优先）+ 侧面抓 ×N（兜底）—— 盒状物从顶抓更稳。
    """
    if shape == "box":
        return [_top_candidate(obj_pos, obj_size)] + _side_candidates(obj_pos, N_SIDE_AZIMUTH)
    # 默认 cylinder
    return _side_candidates(obj_pos, N_SIDE_AZIMUTH) + [_top_candidate(obj_pos, obj_size)]


class GraspPlanner:
    """持有 ROS 节点 + /compute_ik 客户端，对候选逐个查 IK，选最优。

    IK 用 pick_ik（global_approximation=false）+ 当前关节做 seed → 解贴近当前臂态、
    遵循 joint_limits_pilz.yaml（shoulder_lift[-120°,30°] 等）。cost = ||IK解 - seed||²
    （关节运动量）+ prior；选 cost 最小的。
    """

    def __init__(self, node: Optional[Node] = None, log=None):
        self._owns_node = node is None
        self.node = node or rclpy.create_node("grasp_planner")
        self._log = log or self.node.get_logger()
        self._ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        # 缓存最新 /joint_states 做 IK seed（MultiThread/SingleThread 都能收）
        self._js: Optional[JointState] = None
        self._js_sub = self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        if self._owns_node:
            self._exec = SingleThreadedExecutor()
            self._exec.add_node(self.node)
            self._spin_th = threading.Thread(target=self._exec.spin, daemon=True)
            self._spin_th.start()

    def _js_cb(self, msg: JointState):
        self._js = msg

    def wait_ready(self, timeout: float = 10.0) -> bool:
        if not self._ik_cli.wait_for_service(timeout_sec=timeout):
            self._log.error("/compute_ik 服务不可用——move_group 起了吗？")
            return False
        # 等一帧 joint_states 做 seed
        t0 = time.time()
        while self._js is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        return True

    def _seed_state(self) -> RobotState:
        rs = RobotState()
        if self._js is not None:
            rs.joint_state = self._js
        return rs

    def _query_ik(self, pos: Sequence[float], quat: Sequence[float]) -> tuple:
        """查 /compute_ik。返回 (ik_joints_list|None, ok_bool)。"""
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = GROUP_NAME
        ik.ik_link_name = IK_LINK
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose = Pose()
        ik.pose_stamped.pose.position.x = float(pos[0])
        ik.pose_stamped.pose.position.y = float(pos[1])
        ik.pose_stamped.pose.position.z = float(pos[2])
        ik.pose_stamped.pose.orientation.x = float(quat[0])
        ik.pose_stamped.pose.orientation.y = float(quat[1])
        ik.pose_stamped.pose.orientation.z = float(quat[2])
        ik.pose_stamped.pose.orientation.w = float(quat[3])
        ik.robot_state = self._seed_state()
        try:
            ik.timeout = 0.2  # humble: float64 秒
        except Exception:  # 老接口无此字段也无所谓
            pass
        req.ik_request = ik
        fut = self._ik_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and rclpy.ok() and time.time() - t0 < IK_TIMEOUT:
            time.sleep(0.02)
        if not fut.done():
            return None, False
        res = fut.result()
        if res is None or res.error_code.val != 1:  # 1 = SUCCESS
            return None, False
        # 解里的关节（只取 6 个臂关节）
        js = res.solution.joint_state
        nm = dict(zip(js.name, js.position))
        joints = [nm.get(n, 0.0) for n in ARM_JOINT_NAMES]
        return joints, True

    def plan(
        self,
        obj_pos: Sequence[float],
        obj_size: Sequence[float],
        shape: str = "cylinder",
        verbose: bool = True,
    ) -> GraspCandidate:
        """生成候选 → /compute_ik 过滤 → 返回 cost 最小的可行候选。无可行则抛错。"""
        seed_joints = self._current_arm_joints()
        cands = generate_candidates(obj_pos, obj_size, shape)
        self._log.info(f"grasp plan: {len(cands)} 候选 (shape={shape})，逐个查 IK ...")
        scored = []
        for c in cands:
            joints, ok = self._query_ik(c.tool0_grasp, c.quat)
            if ok:
                cost = grasp_cost(joints, seed_joints, c.tilt) + c.prior
                c.ik_joints = joints
                scored.append((cost, c))
                if verbose:
                    self._log.info(
                        f"  ✓ {c.label}: IK 可达  cost={cost:.3f}  "
                        f"joints={[round(j, 2) for j in joints]}"
                    )
            elif verbose:
                self._log.info(f"  ✗ {c.label}: IK 不可达")
        if not scored:
            raise RuntimeError(
                f"所有 {len(cands)} 个抓取候选均 IK 不可达——检查物体位姿/关节限位/"
                "shoulder_lift 范围。可能物体太远或太低。"
            )
        scored.sort(key=lambda r: r[0])
        best = scored[0][1]
        self._log.info(
            f"  → 选 {best.label}（cost={scored[0][0]:.3f}, kind={best.kind}）\n"
            f"    grasp tool0={[round(v, 3) for v in best.tool0_grasp]} "
            f"quat={[round(v, 3) for v in best.quat]}"
        )
        return best

    def _current_arm_joints(self) -> List[float]:
        """读当前 6 臂关节（/joint_states 缓存）。"""
        if self._js is None:
            return [0.0] * 6
        nm = dict(zip(self._js.name, self._js.position))
        return [nm.get(n, 0.0) for n in ARM_JOINT_NAMES]

    def shutdown(self):
        if self._owns_node:
            try:
                self._exec.remove_node(self.node)
            except Exception:
                pass
            self._exec.shutdown()
            # 先停 executor 再 join spin 线程，最后销毁节点——否则后台 spin 线程
            # 在 destroy_node 后仍可能跑 callback → "terminate called without an active exception"
            if self._spin_th.is_alive():
                self._spin_th.join(timeout=1.0)
            try:
                self.node.destroy_node()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()


# ── standalone 诊断入口：读物体 TF → 打印 IK 过滤结果，不动机器人 ──────
def main():
    import tf2_ros

    rclpy.init()
    GRASP_FRAME = "fp_object" if os.environ.get("POSE_SOURCE", "gt") == "fp" else "grasp_target"
    OBJECT_SIZE = [float(v) for v in os.environ.get(
        "OBJECT_SIZE", "0.065,0.065,0.115").split(",")]
    SHAPE = os.environ.get("GRASP_SHAPE", "cylinder")

    tf_node = rclpy.create_node("grasp_plan_tf")
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, tf_node)
    tf_exec = SingleThreadedExecutor()
    tf_exec.add_node(tf_node)
    threading.Thread(target=tf_exec.spin, daemon=True).start()

    print(f"等 {GRASP_FRAME} TF ...", flush=True)
    pos = None
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 20.0:
        try:
            tf = tf_buf.lookup_transform("base_link", GRASP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            pos = [t.x, t.y, t.z]
            break
        except tf2_ros.TransformException:
            time.sleep(0.2)
    if pos is None:
        print(f"✗ 20s 未收到 {GRASP_FRAME} TF", flush=True)
        return

    with GraspPlanner() as planner:
        if not planner.wait_ready():
            return
        print(f"\n物体 pos={pos} size={OBJECT_SIZE} shape={SHAPE}", flush=True)
        try:
            best = planner.plan(pos, OBJECT_SIZE, SHAPE, verbose=True)
            print(f"\n★ 最优: {best.label} kind={best.kind}", flush=True)
            print(f"  reach(a) = {[round(v, 3) for v in best.reach]}", flush=True)
            print(f"  tool0_grasp = {[round(v, 3) for v in best.tool0_grasp]}", flush=True)
            print(f"  tool0_pre   = {[round(v, 3) for v in best.tool0_pre]}", flush=True)
            print(f"  quat        = {[round(v, 3) for v in best.quat]}", flush=True)
            print(f"  ik_joints   = {[round(v, 2) for v in (best.ik_joints or [])]}", flush=True)
        except RuntimeError as e:
            print(f"✗ {e}", flush=True)
    tf_exec.remove_node(tf_node)
    tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
