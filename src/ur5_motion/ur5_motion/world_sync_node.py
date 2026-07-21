"""ROS2 节点：读 ~/Desktop/scene_collisions.json，把里面所有 compound 对象
（按父节点分组的碰撞体）发布到 MoveIt PlanningScene。

每个 group → 一个 moveit_msgs/CollisionObject（含多个 sub-boxes）。
通过 /move_group/apply_planning_scene 服务一次性发布。
发布后退出（one-shot）。等 move_group 就绪才执行。

通常由 sim_isaac.launch.py 自动启动（backend:=isaac 时随栈起），不需手动跑。
"""

import json
import os

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

JSON_PATH = os.path.expanduser("~/Desktop/scene_collisions.json")
APPLY_PLANNING_SCENE = "/move_group/apply_planning_scene"


def main():
    rclpy.init()
    node = rclpy.create_node("world_sync_node")
    log = node.get_logger()

    client = node.create_client(ApplyPlanningScene, APPLY_PLANNING_SCENE)
    log.info("Waiting for move_group apply_planning_scene...")
    if not client.wait_for_service(timeout_sec=20.0):
        log.error(
            "apply_planning_scene 不可用——move_group 是否在跑？world_sync 放弃。"
        )
        node.destroy_node(); rclpy.shutdown(); return

    if not os.path.isfile(JSON_PATH):
        log.warn(
            f"{JSON_PATH} 不存在——跳过场景同步。先跑 "
            "isaac_scripts/export_scene_collisions.py 导出。"
        )
        node.destroy_node(); rclpy.shutdown(); return

    with open(JSON_PATH) as f:
        data = json.load(f)

    frame = data.get("frame", "base_link")
    groups = data.get("groups", [])

    # 构建 ApplyPlanningScene 请求（所有 compound 对象一次性发）
    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True

    for group in groups:
        co = CollisionObject()
        co.header.frame_id = frame
        co.id = group["name"]
        co.operation = CollisionObject.ADD
        for shape in group["shapes"]:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(s) for s in shape["size"]]
            co.primitives.append(prim)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                float(p) for p in shape["position"]
            )
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)
        req.scene.world.collision_objects.append(co)
        log.info(
            f"  + compound '{group['name']}' ({len(group['shapes'])} shapes)"
        )

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() and future.result().success:
        log.info(
            f"Done. {len(groups)} compound objects → PlanningScene "
            f"(frame={frame})."
        )
    else:
        log.error("Failed to apply planning scene.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
