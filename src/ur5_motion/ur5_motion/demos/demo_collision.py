"""碰撞检测验证 demo：往 PlanningScene 加障碍盒，确认 MoveIt 规划会绕开。

前提：已启动运动栈
    ros2 launch ur5_bringup full_stack.launch.py backend:=mock

运行：
    ros2 run ur5_motion demo_collision

流程：
  1. 回 home；
  2. 在工作空间内加一个障碍盒（同时在 RViz 里能看到它）；
  3. 规划到「障碍另一侧」的目标 —— 成功=MoveIt 找到了无碰撞路径（绕开）；
  4. 删掉障碍，回 home 收尾。

观察：RViz 的 MotionPlanning 视图里能看到规划出的绿色轨迹绕开红色障碍盒。
"""

import math
import time

import rclpy

from ur5_motion import UR5MoveIt

HOME_JOINTS = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

# 障碍盒：放在 base 前方、工作高度，体积明显
OBSTACLE_NAME = "demo_obstacle"
OBSTACLE_POS = [0.45, 0.0, 0.30]
OBSTACLE_SIZE = [0.12, 0.40, 0.12]  # x,y,z（米）

# 目标笛卡尔点：故意放在障碍「另一侧」，逼规划器绕行
TARGET_POS = [0.60, 0.0, 0.40]
TARGET_QUAT = [0.0, 0.0, 0.0, 1.0]


def main():
    rclpy.init()
    with UR5MoveIt() as arm:
        log = arm.node.get_logger()
        arm.move_to_joint(HOME_JOINTS)

        log.info(f"加入障碍盒 {OBSTACLE_NAME} ...")
        arm.add_box(OBSTACLE_NAME, position=OBSTACLE_POS, size=OBSTACLE_SIZE)
        time.sleep(1.0)  # 给 planning scene 一点时间同步（异步下发）

        log.info("规划到障碍另一侧的目标（应绕开障碍）...")
        try:
            arm.move_to_pose(TARGET_POS, TARGET_QUAT)
            log.info("[OK] 找到无碰撞路径并执行——碰撞检测生效")
        except Exception as e:  # noqa: BLE001
            log.warn(
                f"[注意] 规划/执行未成功: {e}\n"
                "  可能原因：目标不可达/被障碍完全包围，或规划超时。"
                "请到 RViz 手动试，确认障碍盒已出现。"
            )

        # 收尾：清理障碍 + 回 home
        arm.remove_collision(OBSTACLE_NAME)
        time.sleep(0.5)
        arm.move_to_joint(HOME_JOINTS)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
