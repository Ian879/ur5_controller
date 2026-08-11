"""Isaac 抓取 demo：tf2 lookup grasp_target（桥 --object_prim 从 USD 读瓶子 ground-truth
位姿发的 base_link→grasp_target TF），规划 pre-grasp→grasp→闭合→attach→retreat→松开。

pose source 抽象：本 demo 只认 base_link→grasp_target TF。GT pose 由桥发（--object_prim）；
以后接 foundation pose，让同事节点发同名 TF（base_link→grasp_target），抓取逻辑不动。
⚠️ isaac python 是 3.12、humble rclpy 是 3.10（ABI 不兼容），isaac 侧不能 import rclpy，
故 GT pose 走桥的 ROS2PublishTransformTree（发 TF），demo 侧用 tf2 读。

前置：
  终端 A 桥（--object_prim 指瓶子）：
    ~/isaacsim/python.sh isaac_scripts/ur5_isaac_bridge.py --stream --robot_prim /World/ur5 --object_prim /World/bottle
  终端 B full stack：source tools/source_env.sh && ros2 launch ur5_bringup full_stack.launch.py backend:=isaac
运行：
  终端 C：ros2 run ur5_motion demo_grasp

抓取姿态说明（首期简化，参数化可调）：
  - 顶部抓：GRASP_QUAT 绕 y 轴 π → tool0 z 朝下（world -z）。
  - tool0→gripper_tip_tcp 偏移 GRIPPER_TIP_OFFSET（URDF 固定 TCP）。move_to_pose 移的是 tool0，
    故 grasp 的 tool0 在物体上方 GRIPPER_TIP_OFFSET（让 gripper_tip 落到物体）。
  - 物体近似 box（OBJECT_SIZE），圆柱瓶用包围盒。
  这是规划层抓取（PlanningScene attach 跟随），验证 motion+夹爪+attach 流程；物理层真夹
  需 isaac 夹爪与物体物理碰撞（后期）。"""
import math
import os
import time

import rclpy
import tf2_ros
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from ur5_motion import UR5MoveIt

# ── 抓取参数（按实际场景/物体调）────────────────────────────────────
GRIPPER_TIP_OFFSET = 0.17127          # tool0→gripper_tip_tcp（URDF 固定 TCP）
APPROACH_MARGIN = 0.06                # pre-grasp 时 gripper_tip 距物体「顶面」的安全余量（米）
GRASP_QUAT = [0.0, 1.0, 0.0, 0.0]     # 顶部抓：绕 y π，tool0 z 朝下（竖直物 + 桌面 → 顶部最稳）
OBJECT_NAME = "target_object"
OBJECT_SIZE = [0.065, 0.065, 0.115]   # cola 罐包围盒 x,y,z（米）；0.0646×0.1152×0.0646
# ── FP 观测位（眼在手，相机沿 tool0+z；GRASP_QUAT 时相机垂直朝下）──
# 触发 FP 前先把臂移到物体上方俯视，否则 HOME 位相机朝侧方(104°)看不到物体。
# LOOK_XY=物体大致位置先验（桌面该区域）；LOOK_TOOL0_Z=tool0 高度(base 系)，相机距物体 ~0.4m。
LOOK_XY = [0.45, 0.05]
LOOK_TOOL0_Z = 0.10
LOOK_SAM_DWELL = 3.0   # 到位后等 SAM 出一帧新 mask（1Hz，等 3s 拿到 look-pose mask）
# ── pose source：gt = 桥 USD 静态 / fp = FoundationPose 动态感知 ──
POSE_SOURCE = os.environ.get("POSE_SOURCE", "gt")  # 环境变量切换：POSE_SOURCE=fp ros2 run ...
GRASP_FRAME = "fp_object" if POSE_SOURCE == "fp" else "grasp_target"
# ── use_sim_time：实测 wall 时钟(wall)下 demo 能正常读 fp_object TF（sim 时戳也能 latest 查）；
#    反之 use_sim_time=True 时 tf listener 收不到 fp_object（疑似 executor/时钟启动顺序问题）。
#    故默认 wall(0)。motion 在 wall 下也正常。如需 sim 时钟：USE_SIM_TIME=1。
USE_SIM_TIME = os.environ.get("USE_SIM_TIME", "0") == "1"
# ── 每次跑前先复位到 home（避免上次停在观测位/抓取位）。RESET_HOME=0 跳过。
RESET_HOME = os.environ.get("RESET_HOME", "1") != "0"
# attach 时允许物体重叠的 link（夹爪手指等，避免 attach 后误判自碰撞）
TOUCH_LINKS = [
    "gripper_base_link",
    "gripper_l_link1", "gripper_l_link2", "gripper_l_link_finger",
    "gripper_r_link1", "gripper_r_link2", "gripper_r_link_finger",
    "gripper_tip", "gripper_tip_midpoint_arm", "gripper_tip_midpoint_counterarm",
]


class GripperClient:
    """GripperCommand action 客户端（isaac 的 isaac_gripper_action / real 夹爪桥同名）。
    复用 arm.node（其后台 executor spin 推进 future），轮询 done() 不再 spin 以免 executor 冲突。"""
    ACTION = "lebai_gripper_controller/gripper_cmd"
    OPEN = math.pi / 3   # URDF gripper_r_joint1 上限 = 全开
    CLOSE = 0.0          # 全合

    def __init__(self, node, log):
        self.log = log
        self.client = ActionClient(node, GripperCommand, self.ACTION)
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"gripper action '{self.ACTION}' 不可用——"
                "isaac 后端起 isaac_gripper_action 了吗？real 后端的夹爪桥呢？"
            )

    def move(self, position, effort=50.0, timeout=3.0):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        fut = self.client.send_goal_async(goal)
        if not self._await(fut, timeout, "gripper goal"):
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.log.warn("gripper goal 被拒")
            return False
        res_fut = gh.get_result_async()
        if not self._await(res_fut, timeout, "gripper result"):
            return False
        res = res_fut.result().result
        return res.reached_goal

    def _await(self, fut, timeout, label):
        t0 = time.time()
        while not fut.done() and rclpy.ok():
            if time.time() - t0 > timeout:
                self.log.warn(f"{label} 超时")
                return False
            time.sleep(0.05)
        return fut.done()

    def open(self):
        self.log.info("gripper OPEN")
        self.move(self.OPEN)

    def close(self):
        self.log.info("gripper CLOSE")
        self.move(self.CLOSE)


def main():
    rclpy.init()
    with UR5MoveIt(use_sim_time=USE_SIM_TIME) as arm:
        log = arm.node.get_logger()
        gripper = GripperClient(arm.node, log)

        if RESET_HOME:
            log.info("复位到 home")
            try:
                arm.go_home()
            except Exception as e:  # noqa: BLE001
                log.warn(f"go_home 失败（继续）: {e}")

        # ── TF listener：独立节点+executor（不挂 arm.node——arm.node 的 listener 实测
        #    收不到 /tf，base_link 都 not exist；独立节点像 tf2_echo 那样能收）。──
        import threading
        tf_node = rclpy.create_node("demo_grasp_tf")
        tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buf, tf_node)
        _tf_exec = rclpy.executors.SingleThreadedExecutor()
        _tf_exec.add_node(tf_node)
        threading.Thread(target=_tf_exec.spin, daemon=True).start()

        def _read_pose(timeout_s):
            """轮询 base_link→GRASP_FRAME，返回 (pos,quat) 或 None。"""
            t0 = time.time(); last = 0.0
            while rclpy.ok():
                try:
                    tf = tf_buf.lookup_transform("base_link", GRASP_FRAME, rclpy.time.Time())
                    t = tf.transform.translation; q = tf.transform.rotation
                    return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
                except tf2_ros.TransformException as e:
                    if time.time() - t0 > timeout_s:
                        return None
                    if time.time() - last > 5.0:
                        log.warn(f"TF {GRASP_FRAME} 未就绪: {e}"); last = time.time()
                    time.sleep(0.2)

        def _pose_sane(pos):
            """FP 位姿合理性：物体应在 look pose 视野下方（LOOK_XY 附近、桌面高度）。
            FP 对俯视对称物收敛不稳——有时 x 漂到 base 原点(0)或 z 漂到桌下(-0.9)。
            用 look pose 的 xy 做先验框（相机正对着物体，物体必在视野下方附近）。"""
            if pos is None:
                return False
            return (abs(pos[0] - LOOK_XY[0]) < 0.20 and   # 物体在 look xy ±0.2m
                    abs(pos[1] - LOOK_XY[1]) < 0.20 and
                    -0.58 < pos[2] < -0.15)               # 桌面高度（base z ~-0.4）

        obj_pos = obj_quat = None
        if POSE_SOURCE == "fp":
            # 0. 移到观测位（眼在手，HOME 相机朝侧方看不到物体）
            log.info(f"0/9 移到观测位 tool0={LOOK_XY}+z{LOOK_TOOL0_Z}（相机俯视物体）")
            arm.move_to_pose([LOOK_XY[0], LOOK_XY[1], LOOK_TOOL0_Z], GRASP_QUAT)
            log.info(f"   到位，等 {LOOK_SAM_DWELL}s 让 mask 出 look-pose ...")
            time.sleep(LOOK_SAM_DWELL)
            fp_cli = arm.node.create_client(Trigger, "/foundationpose/track")
            if not fp_cli.wait_for_service(timeout_sec=10.0):
                log.error("/foundationpose/track 服务不可用——fp_node 起了吗？")
                return
            # ⚠️ FP 对【无纹理 mesh + 俯视对称物（cola 罐从上看=圆）】配准不稳，~1/N 次才收敛对。
            #    重试：触发→读 TF→合理性检查，不合理就重触发（FP 每次随机假设）。
            #    根治：给 FP 带纹理的 mesh（见 isaac_scripts/export_textured_cola.py）。
            MAX_TRIES = int(os.environ.get("FP_MAX_TRIES", "6"))
            for attempt in range(1, MAX_TRIES + 1):
                log.info(f"触发 FP register+track（第 {attempt}/{MAX_TRIES} 次，register 需 10-30s）...")
                fut = fp_cli.call_async(Trigger.Request())
                t0 = time.time()
                while not fut.done() and rclpy.ok() and time.time() - t0 < 10.0:
                    time.sleep(0.1)
                res = _read_pose(35.0)
                obj_pos, obj_quat = res if res else (None, None)
                if _pose_sane(obj_pos):
                    log.info(f"FP 收敛 ✓（第 {attempt} 次）")
                    break
                log.warn(f"FP 位姿异常 {[round(v,3) for v in obj_pos] if obj_pos else None}"
                         f"（预期 z≈-0.4），{'重试...' if attempt < MAX_TRIES else '已达上限'}")
            if not _pose_sane(obj_pos):
                log.error(f"FP {MAX_TRIES} 次均未收敛到合理位姿（最后 "
                          f"{[round(v,3) for v in obj_pos] if obj_pos else None}）。"
                          "多半 mesh 无纹理 → 见 isaac_scripts/export_textured_cola.py 导出带纹理 mesh。")
                # 诊断链路（确认是 FP 精度，不是 TF 链断）
                for fa, fb in [("base_link", "camera_color_optical_frame"),
                               ("camera_color_optical_frame", GRASP_FRAME)]:
                    try:
                        tf_buf.lookup_transform(fa, fb, rclpy.time.Time())
                        log.error(f"  诊断 {fa}→{fb}: ✓（TF 链正常，问题是 FP 精度）")
                    except tf2_ros.TransformException as e2:
                        log.error(f"  诊断 {fa}→{fb}: ✗ {e2}")
                return
        else:
            # GT：单次读 grasp_target（桥 --object_prim 发的静态 TF）
            obj_pos, obj_quat = _read_pose(20.0)
            if obj_pos is None:
                log.error(f"20s 未收到 {GRASP_FRAME} TF——桥 --object_prim <物体prim> 传了吗？")
                return
        log.info(f"物体位姿(base): pos={[round(v,3) for v in obj_pos]} quat={[round(v,3) for v in obj_quat]}")

        # 抓取位姿算术（顶部抓，沿 world +z）：
        #   grasp：gripper_tip 落到物体中心（手指夹瓶身中段）→ tool0 在中心上方 GRIPPER_TIP_OFFSET。
        #   pre  ：gripper_tip 退到物体「顶面」(= 中心 + 半高) 上方 APPROACH_MARGIN。
        # ⚠️ pre 必须高过顶面：旧 APPROACH_DIST=0.10 < 半高 0.11，pre tip 反插进刚加的物体 box
        #   顶 1cm → goal_in_collision → move_group ABORT(99999)，step3 直接挂。改用「顶面+余量」。
        half_h = OBJECT_SIZE[2] / 2.0
        # 顶部接近（竖直物 + 桌面 → 最稳）：tool0 z 朝下（绕 y π），finger 水平 ±x 夹物体侧面。
        # arm 在物体上方（不经过物体侧面）→ 不碰 box（侧面 arm 必碰物体，已验证）。
        # GRASP_DEPTH=0.04：finger 到物体身上部，本体(base)在瓶顶上方不碰。之前顶部已验证夹到+close 成功。
        # retreat 靠柔顺 drive（joint1 stiffness 2000/maxForce 15，fix_gripper_drive 修过）避崩。
        GRASP_DEPTH = 0.0   # cola 罐矮(0.115m)，finger 在中心=稳夹；之前 bottle 0.22m 用 0.04
        grasp = [obj_pos[0], obj_pos[1], obj_pos[2] + GRASP_DEPTH + GRIPPER_TIP_OFFSET]
        pre = [obj_pos[0], obj_pos[1], obj_pos[2] + half_h + APPROACH_MARGIN + GRIPPER_TIP_OFFSET]
        # place（放置点，顶部）：夹住后移到这里松开。改 PLACE_XY 换放置位置（base 系 xy）。
        PLACE_XY = [0.40, -0.30]
        place_pre = [PLACE_XY[0], PLACE_XY[1], obj_pos[2] + half_h + APPROACH_MARGIN + GRIPPER_TIP_OFFSET]
        place = [PLACE_XY[0], PLACE_XY[1], obj_pos[2] + GRASP_DEPTH + GRIPPER_TIP_OFFSET]

        try:
            log.info("1/9 张开夹爪")
            gripper.open()
            log.info(f"2/9 加物体 box（避 home→pre 路径撞物体）")
            arm.add_box(OBJECT_NAME, position=obj_pos, size=OBJECT_SIZE, quat_xyzw=obj_quat)
            time.sleep(1.0)
            log.info(f"3/9 pre-grasp -> {[round(v,3) for v in pre]}")
            arm.move_to_pose(pre, GRASP_QUAT)
            log.info("4/9 allow_collisions(物体,True)——grasp 时夹爪碰物体不判碰撞")
            arm.allow_collisions(OBJECT_NAME, True)
            time.sleep(0.3)
            log.info(f"5/9 grasp -> {[round(v,3) for v in grasp]} [CARTESIAN 直下]")
            arm.move_to_pose(grasp, GRASP_QUAT, cartesian=True)
            log.info("6/9 attach + 闭合夹爪（MoveIt 规划层 attach；真夹爪物理夹持）")
            arm.attach_object(OBJECT_NAME, link_name="gripper_tip_tcp", touch_links=TOUCH_LINKS)
            gripper.close()  # 真夹爪物理夹持：手指合拢夹瓶侧（碰阻力→超时正常，move 返回 False 不抛）
            time.sleep(2.0)  # 等 close 物理收敛（接触/摩擦稳定）再 retreat，避 retreat 跟踪失败(-4)
            log.info(f"7/9 retreat [CARTESIAN 直上，带物体]")
            arm.move_to_pose(pre, GRASP_QUAT, cartesian=True)
            log.info(f"8/9 移到 place 上方 -> {PLACE_XY}（带物体，OMPL 绕障）")
            arm.move_to_pose(place_pre, GRASP_QUAT)
            log.info(f"9/9 下探 place -> {[round(v,3) for v in place]} [CARTESIAN 直下]")
            arm.move_to_pose(place, GRASP_QUAT, cartesian=True)
            gripper.open()  # place 松开，物体放下
            time.sleep(1.0)
            log.info("retreat 离开 place [CARTESIAN 直上]")
            arm.move_to_pose(place_pre, GRASP_QUAT, cartesian=True)
            log.info("收尾: detach + disallow + 清理")
            arm.detach_object(OBJECT_NAME)
            arm.allow_collisions(OBJECT_NAME, False)
            time.sleep(0.3)
            arm.remove_collision(OBJECT_NAME)
            log.info("pick-and-place 完成 ✓")
        except Exception as e:  # noqa: BLE001
            log.error(f"抓取流程失败: {e}")
            try:
                arm.detach_object(OBJECT_NAME)
                arm.remove_collision(OBJECT_NAME)
            except Exception:
                pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
