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

抓取姿态说明（对象感知，见 grasp_planner.py）：
  - grasp 朝向不再硬编码：grasp_planner 按物体形状（GRASP_SHAPE）生成候选 → /compute_ik
    过滤 → 选最优。圆柱体优先【侧面抓】（水平接近、对 FP z 误差不敏感），顶部抓仅兜底。
  - tool0→gripper_tip_tcp 偏移由 grasp_geometry.GRIPPER_TIP_OFFSET（URDF 固定 TCP）给出。move_to_pose
    移的是 tool0；侧抓时 tool0 在物体侧面、TCP 沿水平 reach 方向落到物体中心。
  - 物体近似 box（OBJECT_SIZE），圆柱罐用包围盒；GRASP_PLAN_ONLY=1 只规划不执行（诊断）。
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
from ur5_motion.grasp_geometry import grasp_poses as _geo_grasp_poses, APPROACH_MARGIN
from ur5_motion.grasp_planner import GraspPlanner

# ── 抓取参数（按实际场景/物体调）────────────────────────────────────
# GRIPPER_TIP_OFFSET / APPROACH_MARGIN 从 grasp_geometry 导入（单一来源，与 grasp_poses 的
# pre 计算同值 0.06；TCP 偏移经 _geo_grasp_poses 在 grasp_geometry 内部使用）。
# 抓取朝向不再硬编码：由 grasp_planner 按【物体形状】生成候选（圆柱→侧面抓，盒→顶部抓），
# 经 /compute_ik 过滤选 IK 可达且关节运动量最小的。详见 grasp_planner.py。
GRASP_SHAPE = os.environ.get("GRASP_SHAPE", "cylinder")  # cylinder|box
GRASP_PLAN_ONLY = os.environ.get("GRASP_PLAN_ONLY", "0") == "1"  # 只规划不执行（诊断）
GRASP_INSPECT = os.environ.get("GRASP_INSPECT", "0") == "1"  # 到 grasp 位张开夹爪停住，看手指是否跨住物体
# 观测朝向：tool0 z 斜指下方 ~45°（绕 y 135°）→ 相机斜看桌面，FP 看到圆柱侧面（非对称）
LOOK_QUAT = [0.0, 0.9239, 0.0, 0.3827]  # cos(67.5°), sin(67.5°)
OBJECT_NAME = "target_object"
OBJECT_SIZE = [0.065, 0.065, 0.115]   # cola 罐包围盒 x,y,z（米）；0.0646×0.1152×0.0646
# ── FP 观测位（眼在手，相机沿 tool0+z；grasp_quat 时相机垂直朝下）──
# 触发 FP 前先把臂移到物体上方俯视，否则 HOME 位相机朝侧方(104°)看不到物体。
# LOOK_XY=物体大致位置先验（桌面该区域）；LOOK_TOOL0_Z=tool0 高度(base 系)，相机距物体 ~0.4m。
LOOK_XY = [0.30, 0.0]
LOOK_TOOL0_Z = 0.25
LOOK_SAM_DWELL = 3.0   # 到位后等 SAM 出一帧新 mask（1Hz，等 3s 拿到 look-pose mask）
# look pose 用关节目标（用户实测位姿 [0,-50,36,-107,-90,0]°，避开 OMPL 选扭曲笛卡尔解）
LOOK_JOINTS = [0.0, -0.8727, 0.6283, -1.3963, -1.5708, 0.0]  # deg: [0,-50,36,-80,-90,0]
# ── pose source：gt = 桥 USD 静态 / fp = FoundationPose 动态感知 ──
POSE_SOURCE = os.environ.get("POSE_SOURCE", "gt")  # 环境变量切换：POSE_SOURCE=fp ros2 run ...
GRASP_FRAME = "fp_object" if POSE_SOURCE == "fp" else "grasp_target"
# ── use_sim_time：isaac 后端【必须】sim 时钟(1)。wall 时钟(0)下 motion 会卡死——move_group
#    (sim 时戳) 的 action goal 响应发不回 wall 客户端（fastdds "Failed to send goal response
#    (timeout)"），pymoveit2 wait_until_executed 永久挂。sim 时钟下 motion + FP TF 读取都正常
#    （FP 收敛第 1 次，fp_object latest 查 OK）。真机后端用 wall：USE_SIM_TIME=0。
USE_SIM_TIME = os.environ.get("USE_SIM_TIME", "1") == "1"
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
            """FP 位姿合理性：物体在桌面上（不是 base 原点(0,0)或桌下(-0.9)）。
            用桌面工作空间范围做合理性框（不依赖相机位置——斜视时相机≠物体正上方）。"""
            if pos is None:
                return False
            return (0.25 < pos[0] < 0.95 and   # 桌面 x 范围（可乐可能挪到远处）
                    abs(pos[1]) < 0.40 and      # 桌面 y 范围
                    -0.62 < pos[2] < -0.15)     # 桌面高度（base z ~-0.4）

        obj_pos = obj_quat = None
        if POSE_SOURCE == "fp":
            # 0. 移到观测位（眼在手，HOME 相机朝侧方看不到物体）
            log.info(f"移到观测位 LOOK_JOINTS（关节目标，确定性到达）")
            arm.move_to_joint(LOOK_JOINTS)
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

        # ── 抓取规划（对象感知）：圆柱体优先【侧面抓】（水平接近，对 FP z 误差不敏感）──
        # grasp_planner 生成多方位候选 → /compute_ik 过滤（pick_ik 自动遵循 shoulder_lift 限位）
        # → 选 IK 可达且关节运动量最小的。返回的候选含 tool0 grasp/pre 位姿 + 接近方向 reach。
        planner = GraspPlanner(log=log)
        if not planner.wait_ready():
            return
        cands_ranked = planner.plan_ranked(obj_pos, OBJECT_SIZE, shape=GRASP_SHAPE)
        cand = cands_ranked[0]

        def _poses_for(obj_pos_):
            """物体位姿 + 当前候选 reach/quat → (tool0_grasp, tool0_pre)。可对再观测的新位姿重算。"""
            grasp, pre = _geo_grasp_poses(obj_pos_, cand.reach, cand.quat)
            if cand.kind == "top":
                # 顶部兜底的 pre 须在物体顶面上方（grasp_poses 不含 half_h，补上）
                pre = [pre[0], pre[1], pre[2] + OBJECT_SIZE[2] / 2.0]
            return grasp, pre

        grasp_quat = cand.quat                # 选中的 tool0 朝向（侧抓=水平 / 兜底=朝下）
        grasp, pre = _poses_for(obj_pos)
        PLACE_XY = [0.40, -0.30]              # 放置点 xy（base 系）；decomposed place 段用
        planner.shutdown()  # 规划完释放（/compute_ik 客户端不再需要；后续纯运动）
        if GRASP_PLAN_ONLY:
            log.info("GRASP_PLAN_ONLY=1：只规划不执行，退出。")
            return

        try:
            log.info("张开夹爪")
            gripper.open()
            log.info(f"加物体 box + allow_collisions（侧抓 pre 在盒侧同高，须先 allow 否则 OMPL 末段碰盒）")
            arm.add_box(OBJECT_NAME, position=obj_pos, size=OBJECT_SIZE, quat_xyzw=obj_quat)
            arm.allow_collisions(OBJECT_NAME, True)  # 侧抓：pre-grasp 紧贴盒侧，先 allow 才规划得过去
            time.sleep(1.0)
            # ── pre-grasp：候选重试。IK 可达 ≠ OMPL 规划得过去（99999 goal-sampling），
            #    失败就换 plan_ranked 的下一个候选（换方位/倾角常能解开）。──
            reached_pre = False
            for gi, gc in enumerate(cands_ranked):
                cand = gc                      # _poses_for 闭包引用 cand，重绑即生效
                grasp_quat = cand.quat
                grasp, pre = _poses_for(obj_pos)
                log.info(f"pre-grasp [{gi + 1}/{len(cands_ranked)}] {gc.label} "
                         f"-> {[round(v, 3) for v in pre]}")
                try:
                    arm.move_to_pose(pre, grasp_quat)
                    reached_pre = True
                    break
                except Exception as ge:  # noqa: BLE001
                    log.warn(f"  {gc.label} pre-grasp 失败（{ge}），换下一候选 ...")
            if not reached_pre:
                raise RuntimeError(f"全部 {len(cands_ranked)} 个候选的 pre-grasp 均失败")
            # ── 再观测（Eye-in-Hand 闭环）：pre-grasp 处近距离再跑一次 FP，修正最终抓取位 ──
            if POSE_SOURCE == "fp":
                log.info("pre-grasp 到位，重新触发 FP 修正最终抓取位 ...")
                fp_cli.call_async(Trigger.Request())          # 复用已建的 /foundationpose/track client
                res = _read_pose(35.0)                         # 复用现有 TF 轮询
                obj_pos2 = res[0] if res else None
                if obj_pos2 is not None and _pose_sane(obj_pos2):
                    log.info(f"再观测修正：{[round(v,3) for v in obj_pos2]}"
                             f"（漂移 {[round(obj_pos2[i]-obj_pos[i],3) for i in range(3)]}）")
                    grasp, pre = _poses_for(obj_pos2)          # 用新位姿重算 grasp/pre
                else:
                    log.warn("再观测未收敛，沿用首次位姿")
                    obj_pos2 = obj_pos                          # fallback：保证 obj_pos2 始终有定义
            else:
                obj_pos2 = obj_pos                              # GT 路径无再观测
            log.info(f"grasp -> {[round(v,3) for v in grasp]} [CARTESIAN 直进]")
            arm.move_to_pose(grasp, grasp_quat, cartesian=True)
            if GRASP_INSPECT:
                log.info("GRASP_INSPECT: 停在 grasp 位（夹爪张开）— 看手指是否跨住可乐 vs 跨偏了。"
                         "看完记下方向，Ctrl-C 退出。")
                while rclpy.ok():
                    time.sleep(1.0)
                return
            log.info("attach + 闭合夹爪（MoveIt 规划层 attach；真夹爪物理夹持）")
            arm.attach_object(OBJECT_NAME, link_name="gripper_tip_tcp", touch_links=TOUCH_LINKS)
            # attached 可乐 ↔ 桌子 放行：allow_collisions(可乐) 只放可乐↔机器人 link，
            # 可乐↔桌子仍判碰撞 → 带物体的 OMPL 搬运路径被 -2 invalid states 拒。
            arm.allow_collisions_pair(OBJECT_NAME, "table", True)
            gripper.close()  # 真夹爪物理夹持：手指合拢夹瓶侧（碰阻力→超时正常，move 返回 False 不抛）
            time.sleep(2.0)  # 等 close 物理收敛（接触/摩擦稳定）再 retreat，避 retreat 跟踪失败(-4)
            # ── decomposed place：短段拆开，OMPL 只管自由空间搬运 ──
            LIFT_Z = 0.20          # 抬离桌面高度
            TRANSPORT_Z = 0.45     # 安全搬运高度（带 attached 物体的 transit 路径要离桌面足够远，
                                    # 0.30 实测 OMPL 判 -2 invalid states；0.45 留余量）
            # 1) 沿接近轴退 + 上抬（笛卡尔，带物体）
            pos, _q = arm.get_tool0_pose()
            a = cand.reach
            retreat = [pos[0] - APPROACH_MARGIN * a[0],
                       pos[1] - APPROACH_MARGIN * a[1],
                       pos[2] + LIFT_Z]
            log.info("退避+抬离 [CARTESIAN]")
            arm.move_to_pose(retreat, grasp_quat, cartesian=True)
            # 2) 安全搬运到 place 上方（OMPL 关节空间，自由空间，带物体）
            transport = [PLACE_XY[0], PLACE_XY[1], TRANSPORT_Z]
            log.info(f"搬运到 place 上方 {PLACE_XY} [OMPL]")
            arm.move_to_pose(transport, grasp_quat, cartesian=False)
            # 3) 笛卡尔下探到 place（TCP 落 place 点，同高）
            place, _ = _poses_for([PLACE_XY[0], PLACE_XY[1], obj_pos2[2]])
            log.info("下探 place [CARTESIAN]")
            arm.move_to_pose(place, grasp_quat, cartesian=True)
            gripper.open()  # 松开
            # 4) 笛卡尔退离
            pos, _ = arm.get_tool0_pose()
            log.info("退离 place [CARTESIAN]")
            arm.move_to_pose([pos[0], pos[1], pos[2] + LIFT_Z], grasp_quat, cartesian=True)
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
