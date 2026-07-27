"""Isaac 抓取 demo：订阅 /grasp_target_pose（publish_object_pose.py 从 USD 读的瓶子
ground-truth 位姿），规划 pre-grasp→grasp→闭合→attach→retreat→松开。

pose source 抽象：本 demo 只认 /grasp_target_pose（geometry_msgs/PoseStamped,
frame_id=base_link）。以后接 foundation pose，让同事节点发到同名/同型 topic（或改下面
OBJECT_TOPIC 一处），抓取逻辑不动。

前置：
  终端 A 桥：       ~/isaacsim/python.sh isaac_scripts/ur5_isaac_bridge.py --stream --robot_prim /World/ur5
  终端 B 物体 pose：~/isaacsim/python.sh isaac_scripts/publish_object_pose.py --object_prim /World/bottle
  终端 C full stack：source tools/source_env.sh && ros2 launch ur5_bringup full_stack.launch.py backend:=isaac
运行：
  终端 D：ros2 run ur5_motion demo_grasp

抓取姿态说明（首期简化，参数化可调）：
  - 顶部抓：GRASP_QUAT 绕 y 轴 π → tool0 z 朝下（world -z）。
  - tool0→gripper_tip_tcp 偏移 GRIPPER_TIP_OFFSET（URDF 固定 TCP）。move_to_pose 移的是 tool0，
    故 grasp 的 tool0 在物体上方 GRIPPER_TIP_OFFSET（让 gripper_tip 落到物体）。
  - 物体近似 box（OBJECT_SIZE），圆柱瓶用包围盒。
  这是规划层抓取（PlanningScene attach 跟随），验证 motion+夹爪+attach 流程；物理层真夹
  需 isaac 夹爪与物体物理碰撞（后期）。"""
import math
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from ur5_motion import UR5MoveIt

# ── 抓取参数（按实际场景/物体调）────────────────────────────────────
GRIPPER_TIP_OFFSET = 0.17127          # tool0→gripper_tip_tcp（URDF 固定 TCP）
APPROACH_DIST = 0.10                  # pre-grasp 沿工具轴退的距离（米）
GRASP_QUAT = [0.0, 1.0, 0.0, 0.0]     # 顶部抓：绕 y π，tool0 z 朝下
OBJECT_NAME = "target_object"
OBJECT_SIZE = [0.06, 0.06, 0.22]      # 瓶子近似包围盒 x,y,z（米）；按实际调
OBJECT_TOPIC = "/grasp_target_pose"   # pose source（publish_object_pose 发）
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

    def move(self, position, effort=50.0, timeout=10.0):
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
    with UR5MoveIt() as arm:
        log = arm.node.get_logger()
        gripper = GripperClient(arm.node, log)

        # 订阅 pose source
        target = {}

        def on_pose(msg: PoseStamped):
            target["pos"] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            target["quat"] = [
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w,
            ]

        arm.node.create_subscription(PoseStamped, OBJECT_TOPIC, on_pose, 10)

        log.info(f"等待物体 pose ({OBJECT_TOPIC})...")
        t0 = time.time()
        while "pos" not in target and rclpy.ok():
            if time.time() - t0 > 20.0:
                log.error("20s 未收到物体 pose——publish_object_pose.py 在跑吗？")
                return
            time.sleep(0.1)
        obj_pos = target["pos"]
        obj_quat = target["quat"]
        log.info(f"物体位姿(base): pos={[round(v,3) for v in obj_pos]}")

        # 抓取位姿算术（顶部抓，沿 world +z 偏移）
        pre = [obj_pos[0], obj_pos[1], obj_pos[2] + GRIPPER_TIP_OFFSET + APPROACH_DIST]
        grasp = [obj_pos[0], obj_pos[1], obj_pos[2] + GRIPPER_TIP_OFFSET]

        try:
            log.info("1/7 张开夹爪")
            gripper.open()
            log.info(f"2/7 加物体碰撞体 {OBJECT_NAME}（近似 box）")
            arm.add_box(OBJECT_NAME, position=obj_pos, size=OBJECT_SIZE, quat_xyzw=obj_quat)
            time.sleep(1.0)
            log.info(f"3/7 pre-grasp -> {[round(v,3) for v in pre]}")
            arm.move_to_pose(pre, GRASP_QUAT)
            log.info(f"4/7 grasp -> {[round(v,3) for v in grasp]}")
            arm.move_to_pose(grasp, GRASP_QUAT)
            log.info("5/7 attach + 闭合夹爪")
            arm.attach_object(OBJECT_NAME, link_name="gripper_tip_tcp", touch_links=TOUCH_LINKS)
            time.sleep(0.5)
            gripper.close()
            time.sleep(1.0)
            log.info(f"6/7 retreat -> {[round(v,3) for v in pre]}")
            arm.move_to_pose(pre, GRASP_QUAT)
            log.info("7/7 detach + 松开 + 清理")
            arm.detach_object(OBJECT_NAME)
            gripper.open()
            arm.remove_collision(OBJECT_NAME)
            log.info("抓取流程完成 ✓")
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
