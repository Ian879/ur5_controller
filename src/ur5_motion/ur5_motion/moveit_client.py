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

import math
import os
import threading
from typing import Optional, Sequence

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
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
# SRDF 命名位姿 home（ur_manipulator/home）：pan/lift/elbow/wrist1/2/3
# 现场定版姿态（2026-10-24 用户指定）：[90,-45,35,-70,-90,0]°——示教器抄录
HOME_JOINTS = [1.5708, -0.7854, 0.6109, -1.2217, -1.5708, 0.0]
# 自定义 home：UR5_HOME_JOINTS="pan,lift,elbow,w1,w2,w3"（【度】，从示教器直接抄）
# 覆盖默认值，go_home/LOOK 类复位都跟着走。注意窄限位 shoulder_lift ∈ [-120°,30°]。
# 例：UR5_HOME_JOINTS="0,-60,30,0,90,0" ros2 run ur5_motion go_home
_home_env = os.environ.get("UR5_HOME_JOINTS", "")
if _home_env:
    _degs = [float(v) for v in _home_env.split(",")]
    if len(_degs) != 6:
        raise ValueError(f"UR5_HOME_JOINTS 需要 6 个逗号分隔的度数，收到 {len(_degs)} 个: {_home_env!r}")
    HOME_JOINTS = [math.radians(d) for d in _degs]
BASE_LINK = "base_link"
END_EFFECTOR = "tool0"
GROUP_NAME = "ur_manipulator"


class UR5MoveIt:
    """对 pymoveit2.MoveIt2 的同步封装。"""

    def __init__(self, node: Optional[Node] = None, *, node_name: str = "ur5_moveit_client",
                 use_sim_time: bool = False):
        self._owns_node = node is None
        if node is None:
            # use_sim_time 必须在节点创建时生效（影响 clock）。Isaac 后端要 True，否则 TF
            # lookup（sim 时戳）与节点（wall 时钟）时域不匹配 → base_link→fp_object 永远超时。
            from rclpy.parameter import Parameter
            node = rclpy.create_node(
                node_name,
                parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
            )
        self.node = node
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
            # ⚠️ ReentrantCallbackGroup（治本）：pymoveit2 默认 callback_group=None → 用 node
            # default（MutuallyExclusive）。MultiThreadedExecutor 对互斥组只单线程串行跑 callback；
            # 若该组唯一执行槽被某 callback 占着，get/apply_planning_scene 的 service response 排不上
            # → allow_collisions 内部同步 .call() 的 event.wait() 永久阻塞（demo 第4步卡死根因）。
            # Reentrant 解除串行，service response 可被 executor 任一线程即时派发。
            callback_group=ReentrantCallbackGroup(),
            # 由 move_group 负责执行（下发到激活的轨迹控制器）。
            # use_move_group_action 是新参数名（旧的 execute_via_moveit 已弃用）。
            use_move_group_action=True,
        )

        # 默认规划走 OMPL（moveit.launch.py 的 default_planning_pipeline=ompl）。
        # (1) planner_id 留空 → move_group 用 ompl pipeline 的默认 planner（RRTConnect）。
        #     ⚠️ 不能填 pilz 的 "PTP"——OMPL pipeline 没有 PTP planner，move_group 会报
        #     "No ContextLoader for planner_id 'PTP' found"。末端直线接近需切 pilz（见下）。
        # (2) max_velocity/accel（pymoveit2 属性，写 request.velocity/acceleration_scaling_factor）：
        #     OMPL pipeline 的 AddTimeOptimalParameterization adapter 用它给轨迹加时间参数，
        #     pymoveit2 默认 0.0 会让时间参数化失败。设 0.5（安全；要更快调到 1.0）。
        #     pilz 同样依赖它（默认 0.0 时 pilz 报 "not in range [0.0001, 1]"）。
        # 末端直线接近（抓取 approach）首期仍用 OMPL 关节空间；要强制 LIN 直线需切 pilz
        # pipeline——pymoveit2 不直接暴露 pipeline 选择，作为后续优化。
        self._moveit.planner_id = ""
        self._moveit.max_velocity = 0.2   # 真夹爪夹瓶子后 retreat 物理阻力大，降速避跟踪失败(-4)
        self._moveit.max_acceleration = 0.2
        self._moveit.allowed_planning_time = 5.0  # OMPL 默认太短(~0.5s)→"Unable to solve"

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

    def go_home(self) -> None:
        """移到 SRDF home 位姿。快速复位用。

        关节目标做【就近 2π 等价】处理（同 demo_grasp._junwrap）：真机 UR 的编码器
        会报累积角（wrist 报 355° 而 home 是 0°）——直接规划会得到"腕整圈回转"
        的疯狂轨迹（2026-10-13 真机实测：目标 0° vs 当前 305° → 规划 -305° 回转）。
        等价后 wrist 目标变 360°，只转零头。"""
        import math as _m
        try:
            cur = self.get_current_joints()
        except Exception as exc:  # noqa: BLE001
            self._log.warn(f"go_home 读当前关节失败（{exc}）——用未解缠 home 目标")
            cur = [0.0] * 6
        goal = [h + round((c - h) / (2 * _m.pi)) * 2 * _m.pi
                for c, h in zip(cur, HOME_JOINTS)]
        self.move_to_joint(goal)

    def move_to_pose(
        self,
        position: Sequence[float],
        quat_xyzw: Sequence[float],
        frame_id: str = BASE_LINK,
        cartesian: bool = False,
    ) -> None:
        """规划并执行到 base_link（默认）下的笛卡尔位姿。阻塞到执行完成。

        position: [x, y, z]（米）
        quat_xyzw: [qx, qy, qz, qw]（单位四元数）
        cartesian: True → 末端走直线（pre→grasp 直下 / retreat 直上，不绕圈撞物体）；
                   False → OMPL 关节空间（home→pre 等自由空间绕障用，默认）。
        """
        self._log.info(
            f"move_to_pose: pos={[round(p, 3) for p in position]} "
            f"quat_xyzw={[round(q, 3) for q in quat_xyzw]} frame={frame_id}"
            f"{' [CARTESIAN]' if cartesian else ''}"
        )
        self._moveit.move_to_pose(
            position=list(position), quat_xyzw=list(quat_xyzw), frame_id=frame_id,
            cartesian=cartesian,
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

    # ── PlanningScene：attach/detach（抓取后物体跟随末端；pymoveit2 已封装）──
    def attach_object(
        self,
        name: str,
        link_name: str = "gripper_tip_tcp",
        touch_links: Optional[Sequence[str]] = None,
    ) -> None:
        """把碰撞体 name attach 到 link_name（默认 gripper_tip_tcp），之后随该 link 移动。
        touch_links=允许物体重叠的 link（夹爪手指等，避免 attach 后误判碰撞）。
        用于抓取：闭合夹爪后 attach 物体跟随，放下前 detach。"""
        self._moveit.attach_collision_object(
            id=name, link_name=link_name, touch_links=list(touch_links or [])
        )

    def detach_object(self, name: str) -> None:
        """解除 attach（放下物体）。"""
        self._moveit.detach_collision_object(id=name)

    def allow_collisions(self, name: str, allow: bool, timeout: float = 8.0) -> bool:
        """允许/禁止 name 物体和所有 link 碰撞（ACM）。grasp 前 allow=True 让夹爪能碰物体 box；
        retreat 后 allow=False 恢复。ACM 逻辑照搬 pymoveit2 4.2 (moveit2.py:1864-1886)。

        ⚠️ 用【独立 node + 独立 executor】调 get/apply_planning_scene，不挂 arm.node。实测挂在
        arm.node 上的 pymoveit2 _get_planning_scene_service，call_async 的 response 不被 arm 的
        MultiThreadedExecutor 派发（同一 executor 下 pre-grasp 的 move_action 却能派发——疑 pymoveit2
        内部在 reentrant 组留了阻塞 callback，饿死 service response），get_planning_scene 5s 超时 →
        ACM 没设 → grasp goal 撞 box → OMPL "Unable to sample any valid states for goal tree"。
        独立 SingleThreadedExecutor（helper node 干净、无竞争 callback）确定性可派发，一刀切绕开。"""
        import threading
        from rclpy.executors import SingleThreadedExecutor
        from moveit_msgs.msg import AllowedCollisionEntry
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

        helper = rclpy.create_node("ur5_acm_helper")
        exe = SingleThreadedExecutor()
        exe.add_node(helper)
        spin_th = threading.Thread(target=exe.spin, daemon=True)
        spin_th.start()
        try:
            get_srv = helper.create_client(GetPlanningScene, "get_planning_scene")
            app_srv = helper.create_client(ApplyPlanningScene, "apply_planning_scene")
            if not get_srv.wait_for_service(timeout_sec=3.0) or not app_srv.wait_for_service(timeout_sec=3.0):
                self._log.warn(f"allow_collisions({name},{allow}): planning_scene service 未就绪")
                return False
            # 1. GET 当前 scene（独立 executor spin 派发 response；主线程轮询不阻塞它）
            get_fut = get_srv.call_async(GetPlanningScene.Request())
            if not self._poll_future(get_fut, timeout, f"get_planning_scene({name})"):
                return False
            scene = get_fut.result().scene
            # 2. 改 ACM：name ↔ 所有 entry (dis)allow（照搬 pymoveit2 4.2 allow_collisions:1864-1886）
            acm = scene.allowed_collision_matrix
            j = None
            if name not in acm.entry_names:
                acm.entry_names.append(name)
            else:
                j = acm.entry_names.index(name)
            for i in range(len(acm.entry_values)):
                if j is None:
                    acm.entry_values[i].enabled.append(allow)
                elif i != j:
                    acm.entry_values[i].enabled[j] = allow
            entry = AllowedCollisionEntry(enabled=[allow for _ in range(len(acm.entry_names))])
            if j is None:
                acm.entry_values.append(entry)
            else:
                acm.entry_values[j] = entry
            # 3. APPLY
            app_fut = app_srv.call_async(ApplyPlanningScene.Request(scene=scene))
            if not self._poll_future(app_fut, timeout, f"apply_planning_scene({name})"):
                return False
            if app_fut.result() is not None and not app_fut.result().success:
                self._log.warn(f"allow_collisions({name},{allow}): apply 返回 success=False")
                return False
            self._log.info(f"ACM 设好: {name} allow={allow}")
            return True
        finally:
            exe.remove_node(helper)
            helper.destroy_node()
            exe.shutdown()

    def allow_collisions_pair(self, name_a: str, name_b: str, allow: bool,
                              timeout: float = 8.0) -> bool:
        """只放行 name_a ↔ name_b 这一对（不动整行）。供「attached 物体 ↔ 桌子」——
        allow_collisions(name) 只放行 name↔机器人 link，name↔世界物体（table 等）仍判
        碰撞，带物体搬运的 OMPL 路径会被 -2 invalid states 拒掉。"""
        import threading
        import time as _time

        from moveit_msgs.msg import AllowedCollisionEntry
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        from rclpy.executors import SingleThreadedExecutor

        helper = rclpy.create_node("ur5_acm_pair_helper")
        exe = SingleThreadedExecutor()
        exe.add_node(helper)
        threading.Thread(target=exe.spin, daemon=True).start()
        try:
            get_srv = helper.create_client(GetPlanningScene, "get_planning_scene")
            app_srv = helper.create_client(ApplyPlanningScene, "apply_planning_scene")
            if not get_srv.wait_for_service(timeout_sec=3.0) or \
               not app_srv.wait_for_service(timeout_sec=3.0):
                self._log.warn(f"allow_collisions_pair({name_a},{name_b}): service 未就绪")
                return False
            get_fut = get_srv.call_async(GetPlanningScene.Request())
            if not self._poll_future(get_fut, timeout, "get_planning_scene(pair)"):
                return False
            scene = get_fut.result().scene
            acm = scene.allowed_collision_matrix
            for n in (name_a, name_b):          # 缺的 entry 补齐（默认全 False）
                if n not in acm.entry_names:
                    acm.entry_names.append(n)
                    for ev in acm.entry_values:
                        ev.enabled.append(False)
                    acm.entry_values.append(
                        AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
            ia, ib = acm.entry_names.index(name_a), acm.entry_names.index(name_b)
            acm.entry_values[ia].enabled[ib] = allow
            acm.entry_values[ib].enabled[ia] = allow
            app_fut = app_srv.call_async(ApplyPlanningScene.Request(scene=scene))
            if not self._poll_future(app_fut, timeout, "apply_planning_scene(pair)"):
                return False
            self._log.info(f"ACM pair 设好: {name_a}↔{name_b} allow={allow}")
            return True
        finally:
            exe.remove_node(helper)
            helper.destroy_node()
            exe.shutdown()

    def _poll_future(self, fut, timeout: float, label: str) -> bool:
        """轮询 rclpy future 到 done 或超时（主线程 sleep，不占 executor）。供 allow_collisions 的
        call_async service 调用用——避免 pymoveit2 同步 .call() 的无限 event.wait()。"""
        import time as _time
        t0 = _time.time()
        while not fut.done() and rclpy.ok():
            if _time.time() - t0 > timeout:
                self._log.warn(f"{label} {timeout}s 未完成（executor 未派发 service response？）")
                return False
            _time.sleep(0.05)
        return fut.done()

    def cancel(self) -> None:
        """取消当前运动（pymoveit2 4.x 的方法是 cancel_execution）。"""
        try:
            self._moveit.cancel_execution()
        except Exception as e:  # noqa: BLE001
            self._log.warn(f"cancel 失败: {e}")

    def _wait_executed(self, label: str) -> None:
        """阻塞到执行完成；失败则抛错（让上层 try/except 可见）。

        失败时打印执行中断位置的关节 + error_code 含义，辅助定位（规划/碰撞/IK/执行）。"""
        ok = self._moveit.wait_until_executed()
        if not ok:
            try:
                err = self._moveit.get_last_execution_error_code()
            except Exception:
                err = "unknown"
            try:
                cur_str = [round(j, 3) for j in self.get_current_joints()]
            except Exception:
                cur_str = "?"
            self._log.error(
                f"{label} 执行失败 (error_code={err})\n"
                f"  执行中断位置关节(rad): {cur_str}\n"
                f"  error_code 含义: -4=CONTROL_FAILED(执行层/isaac物理碰撞卡住), "
                f"-1=PLANNING_FAILED, -12=GOAL_IN_COLLISION, -31=NO_IK_SOLUTION, "
                f"99999=FAILURE。\n"
                f"  → 执行层(-4)失败：看 full_stack 终端 joint_trajectory_controller 日志"
                f"（abort reason / tracking error），多半是 isaac 物理层夹爪碰物体被卡。"
            )
            raise RuntimeError(f"{label} 执行失败 (error_code={err})")

    # ── 当前状态（便利读取，来自 joint_states）────────────────────────
    def get_current_joints(self) -> list[float]:
        """读当前 6 臂关节角（来自 /joint_states）。

        ⚠️ 用【独立 node + 独立 executor】订阅（同 allow_collisions 的 helper 模式）——
        实测挂在 self.node（pymoveit2 Reentrant 组 + 后台 executor）上的新订阅经常
        不触发回调（demo 的 TF listener 也是同样坑才独立成节点），第二次调用就收不到。
        /joint_states 还有夹爪帧（isaac_gripper_action 只发 gripper_r_joint1）——
        必须等【含臂关节名】的帧；超时抛错（不静默返回垃圾）。"""
        import threading
        import time

        import rclpy as _rclpy
        from rclpy.executors import SingleThreadedExecutor as _STE
        from sensor_msgs.msg import JointState

        helper = _rclpy.create_node("ur5_js_helper")
        exe = _STE()
        exe.add_node(helper)
        spin_th = threading.Thread(target=exe.spin, daemon=True)
        spin_th.start()
        holder: dict = {}

        def _cb(msg):
            if "shoulder_lift_joint" in msg.name and not holder:
                holder["msg"] = msg

        sub = helper.create_subscription(JointState, "/joint_states", _cb, 10)
        try:
            end = time.monotonic() + 5.0
            while not holder and time.monotonic() < end:
                time.sleep(0.02)
        finally:
            exe.remove_node(helper)
            helper.destroy_node()
            exe.shutdown()
        if not holder:
            raise RuntimeError("5s 未收到含臂关节的 /joint_states 帧（broadcaster 挂了？）")
        msg = holder["msg"]
        name_to_pos = dict(zip(msg.name, msg.position))
        return [name_to_pos.get(n, 0.0) for n in JOINT_NAMES]

    def get_tool0_pose(self):
        """读当前 tool0 在 base_link 的 (pos, quat_xyzw)。供 decomposed place 的相对移动用。

        ⚠️ 必须用【独立 node + 独立 executor】：arm.node 的 listener 实测收不到 /tf
        （MultiThreadedExecutor 被 pymoveit2 回调占住，TF 消息派发不到——demo_grasp 顶部
        注释同一个坑）。这里照 demo 的 tf_node 模式：临时节点+SingleThreadedExecutor
        后台线程，查完拆掉。"""
        import threading
        import time as _time

        import tf2_ros
        from rclpy.executors import SingleThreadedExecutor

        tmp_node = rclpy.create_node("ur5_tool0_tf_helper")
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, tmp_node)
        exe = SingleThreadedExecutor()
        exe.add_node(tmp_node)
        th = threading.Thread(target=exe.spin, daemon=True)
        th.start()
        try:
            for _ in range(250):  # 5s
                try:
                    t = buf.lookup_transform("base_link", "tool0", rclpy.time.Time())
                    tr = t.transform.translation; ro = t.transform.rotation
                    return [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w]
                except tf2_ros.TransformException:
                    _time.sleep(0.02)
            raise RuntimeError("tool0 TF 查不到（base_link→tool0 5s 内无数据）")
        finally:
            exe.remove_node(tmp_node)
            tmp_node.destroy_node()
            exe.shutdown()

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
