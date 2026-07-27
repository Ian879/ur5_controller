#!/usr/bin/env python3
"""读 USD 里抓取物体（饮料瓶）的 ground-truth 世界位姿，换算到 base_link 系，
发 geometry_msgs/PoseStamped 到 /grasp_target_pose。

★ pose source 抽象层：demo_grasp 订阅这个 topic。以后接 foundation pose 时，把同事
的节点发到同名/同型 topic（/grasp_target_pose, geometry_msgs/PoseStamped,
frame_id=base_link），或改 demo_grasp 的订阅点一处，不改抓取逻辑。

复用：world pose 读法照搬 introspect_mount_geometry.py:33-54（ComputeLocalToWorldTransform
+ ExtractRotationQuat）；世界→base 换算照搬 export_scene_collisions.py（base_world.GetInverse()
* obj_world）。

跑法（isaac python，独立于桥进程；先 source setup_ros_env.sh）：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/publish_object_pose.py \
      --object_prim /World/bottle

物体是 USD 里摆好的静态 reference（瓶子）。读 default-time pose。若仿真中物体被物理推动，
本脚本周期重读会跟随（--rate）。"""
import argparse
import os
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", default=os.path.expanduser("~/Desktop/scene.usd"))
parser.add_argument(
    "--robot_prim", default="/World/ur5",
    help="机器人 prim（base_link 取 <robot_prim>/base_link）",
)
parser.add_argument(
    "--object_prim", default=None,
    help="抓取物体 prim 路径。不给则按 --object_name 子串遍历找",
)
parser.add_argument(
    "--object_name", default="bottle",
    help="物体 prim 名/路径子串（--object_prim 为空时用）",
)
parser.add_argument("--topic", default="/grasp_target_pose")
parser.add_argument("--rate", type=float, default=2.0, help="发布频率 Hz")
args, _ = parser.parse_known_args()

if not os.path.isfile(args.usd):
    print(f"ERROR: scene.usd 不存在: {args.usd}", flush=True)
    raise SystemExit(1)

# SimulationApp 仅用于初始化 isaac 运行时（让 pxr 可用）；本脚本不跑仿真、不发 isaac graph。
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

stage = Usd.Stage.Open(args.usd)

# base_link 世界 transform → 世界→base 换算矩阵
base_prim = stage.GetPrimAtPath(args.robot_prim + "/base_link")
if not base_prim.IsValid():
    print(
        f"ERROR: base_link prim 无效: {args.robot_prim}/base_link"
        "（用 --robot_prim 指你的机器人 prim）",
        flush=True,
    )
    raise SystemExit(1)
base_inv = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()
).GetInverse()


def find_object_by_name(stg, name_substr):
    """按名字/路径子串遍历找物体 prim（Xform/Scope/Mesh/几何体）。"""
    low = name_substr.lower()
    for p in stg.Traverse():
        if p.GetTypeName() in ("Xform", "Scope", "Mesh", "Cube", "Sphere", "Cylinder"):
            ps = str(p.GetPath()).lower()
            if low in ps or low in str(p.GetName()).lower():
                return str(p.GetPath())
    return None


obj_path = args.object_prim or find_object_by_name(stage, args.object_name)
obj_prim = stage.GetPrimAtPath(obj_path) if obj_path else None
if obj_prim is None or not obj_prim.IsValid():
    print(
        f"ERROR: 物体 prim 无效: {obj_path}"
        "（用 --object_prim <路径> 或 --object_name <名字子串>）",
        flush=True,
    )
    raise SystemExit(1)
print(
    f"[publish_object_pose] object={obj_path}  base={args.robot_prim}/base_link  "
    f"topic={args.topic}  rate={args.rate}Hz",
    flush=True,
)


def world_tf(prim):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


# ROS 发布（独立 rclpy node，不和桥进程冲突——这是独立进程）
rclpy.init()
node = rclpy.create_node("publish_object_pose")
pub = node.create_publisher(PoseStamped, args.topic, 10)
period = 1.0 / max(args.rate, 0.1)

try:
    while rclpy.ok():
        # 物体在 base_link 系的 4x4 = (world→base) * (object→world)
        m = base_inv * world_tf(obj_prim)
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()  # Gf.Quatd：实部 GetReal()=w，虚部 GetImaginary()=Vec3(x,y,z)
        imag = q.GetImaginary()
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(t[0])
        msg.pose.position.y = float(t[1])
        msg.pose.position.z = float(t[2])
        msg.pose.orientation.x = float(imag[0])
        msg.pose.orientation.y = float(imag[1])
        msg.pose.orientation.z = float(imag[2])
        msg.pose.orientation.w = float(q.GetReal())
        pub.publish(msg)
        node.get_logger().info(
            f"obj in base: t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})",
            throttle_duration_sec=1.0,
        )
        time.sleep(period)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
    app.close()
