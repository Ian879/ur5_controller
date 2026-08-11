#!/usr/bin/env python3
"""快速复位到 home + 张开夹爪。任何时候跑：ros2 run ur5_motion go_home
（Isaac 后端默认 use_sim_time=True；真机 USE_SIM_TIME=0）。
"""
import os
import rclpy
from ur5_motion import UR5MoveIt
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

USE_SIM_TIME = os.environ.get("USE_SIM_TIME", "1") != "0"


def main():
    rclpy.init()
    with UR5MoveIt(use_sim_time=USE_SIM_TIME) as arm:
        log = arm.node.get_logger()
        # 张开夹爪（安全复位；夹爪 action 不可用时跳过）
        try:
            import math
            cli = ActionClient(arm.node, GripperCommand, "lebai_gripper_controller/gripper_cmd")
            if cli.wait_for_server(timeout_sec=5.0):
                g = GripperCommand.Goal()
                g.command.position = math.pi / 3   # 全开
                g.command.max_effort = 50.0
                fut = cli.send_goal_async(g)
                import time
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 4.0:
                    time.sleep(0.05)
                log.info("夹爪张开")
        except Exception as e:  # noqa: BLE001
            log.warn(f"夹爪复位跳过: {e}")
        log.info("复位到 home ...")
        arm.go_home()
        log.info("复位完成 ✓")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
