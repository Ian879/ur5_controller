"""把桌面 + 地面作为【世界】collision object 加进 MoveIt 规划场景。

为什么需要这个：RViz Scene Objects 里手加的 box 容易被默认 attach 到机器人 link
（type 变成 "Robot attached"），机器人一动就自碰（如 upper_arm_link ↔ Box_0）、
规划全 invalid。本脚本用 pymoveit2 的 add_collision_box，加的是 **WORLD object**
（固定在世界），不会 attach。

位姿对齐 Isaac 桥（isaac_scripts/ur5_isaac_bridge.py）里的桌面/地面，base_link 系：
  table: center (0.6, 0, 0.785), size (0.8, 0.8, 0.03)  → 顶面 z≈0.80
  floor: center (0.0, 0.0, -0.005), size (4.0, 4.0, 0.01)

用法：
  1) 先清掉 RViz 里之前手加的 box（Scene Objects 删掉，或直接重启 RViz 清场景），
     否则旧的 attached Box_0 还在、还会自碰。
  2) 栈起着（backend:=isaac + move_group）时，另开终端：
       ros2 run ur5_motion demo_table_scene
  3) 回 RViz 规划 —— 应绕桌，不再 invalid。
"""

import rclpy

from ur5_motion import UR5MoveIt

# (name, center_position[m], size[m]) in base_link frame — 对齐 Isaac 桥
OBJECTS = [
    ("table", [0.6, 0.0, 0.785], [0.8, 0.8, 0.03]),
    ("floor", [0.0, 0.0, -0.005], [4.0, 4.0, 0.01]),
]


def main():
    rclpy.init()
    with UR5MoveIt() as arm:
        log = arm.node.get_logger()
        for name, pos, size in OBJECTS:
            arm.add_box(name, position=pos, size=size)
            log.info(f"added WORLD collision '{name}' at {pos} size {size}")
        log.info("done. 回 RViz 规划，应绕桌。若 box 与 Isaac 桌子视觉不对齐，"
                 "说明 base_link↔Isaac 世界有偏移，告诉我。")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
