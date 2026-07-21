"""读 ~/Desktop/scene_collisions.json，把里面所有碰撞体加进 MoveIt PlanningScene。

配合 isaac_scripts/export_scene_collisions.py 用：
  1) Isaac 侧导出: ~/isaacsim/python.sh isaac_scripts/export_scene_collisions.py
  2) ROS 侧同步（栈起着时）: ros2 run ur5_motion sync_scene

所有碰撞体（桌面、桌腿、底座、地面…）都作为 WORLD collision object（box 近似）加进
base_link 系。改了 scene.usd 后重跑步骤 1 即可。
"""

import json
import os

import rclpy

from ur5_motion import UR5MoveIt

JSON_PATH = os.path.expanduser("~/Desktop/scene_collisions.json")


def main():
    if not os.path.isfile(JSON_PATH):
        print(f"ERROR: {JSON_PATH} 不存在。先跑 export_scene_collisions.py 导出。")
        return

    with open(JSON_PATH) as f:
        data = json.load(f)

    rclpy.init()
    with UR5MoveIt() as arm:
        log = arm.node.get_logger()
        log.info(f"frame={data.get('frame','base_link')}, {len(data['objects'])} 个碰撞体")
        for obj in data["objects"]:
            arm.add_box(obj["name"], position=obj["position"], size=obj["size"])
            log.info(
                f"  + {obj['name']:20s} pos={obj['position']} size={obj['size']}"
            )
        log.info(f"done. {len(data['objects'])} 个碰撞体已加进 MoveIt PlanningScene。")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
