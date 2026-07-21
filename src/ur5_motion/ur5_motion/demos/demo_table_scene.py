"""把桌面 + 地面作为【世界】collision object 加进 MoveIt 规划场景（base_link 系）。

坐标来自用户的 ~/Desktop/scene.usd（机器人被圆柱底座抬到 base_link 世界
(-0.55,0,1.211)，桌面世界 (0.6,0,0.735) 尺寸 2.0×0.8×0.03）。
换算到 base_link 系 = 世界 - base_link_world（假设 base_link 无旋转）：
  桌面: center (1.15, 0, -0.476), size (2.0, 0.8, 0.03)
  地面: center (0.55, 0, -1.211), size (4.0, 4.0, 0.02)

MoveIt 规划是 base_link 相对的，所以机器人抬高不影响规划正确性（关节角和 Isaac 一致）。
RViz 视觉对齐需另发 world→base_link TF（见 sim_isaac.launch.py 的 static_transform_publisher）。

⚠️ 这些坐标是手算的、对应当前 scene.usd。你若在 GUI 里移动了机器人/桌子，需重算
（或以后做自动同步：读 scene.usd 的碰撞体 + base_link 世界变换 → 自动生成）。

用法（栈起着时）：
  ros2 run ur5_motion demo_table_scene
"""

import rclpy

from ur5_motion import UR5MoveIt

# (name, center_position[m] in base_link, size[m]) —— 对应 ~/Desktop/scene.usd
OBJECTS = [
    ("table", [1.15, 0.0, -0.476], [2.0, 0.8, 0.03]),
    ("floor", [0.55, 0.0, -1.211], [4.0, 4.0, 0.02]),
]


def main():
    rclpy.init()
    with UR5MoveIt() as arm:
        log = arm.node.get_logger()
        for name, pos, size in OBJECTS:
            arm.add_box(name, position=pos, size=size)
            log.info(f"added WORLD collision '{name}' at (base_link) {pos} size {size}")
        log.info("done. 回 RViz 规划应绕桌。若视觉错位，确认 RViz Fixed Frame=world "
                 "且 world->base_link TF 已发。")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
