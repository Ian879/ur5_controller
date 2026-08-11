#!/usr/bin/env python3
"""读 ~/Desktop/scene.usd 里所有带 collider 的非机器人刚体，
按父节点分组（如 /World/table/* → 一个 "table" 组），换算到 base_link 系，
输出 JSON 供 ROS 侧 world_sync_node 发布到 MoveIt PlanningScene。

跑法（Isaac python）:
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/export_scene_collisions.py

输出: ~/Desktop/scene_collisions.json（groups 格式，每个 group = 一个 compound 对象）
"""

import argparse
import json
import os
import sys
from collections import defaultdict

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scene", default=os.path.expanduser("~/Desktop/scene.usd"),
                    help="USD 场景路径（默认 ~/Desktop/scene.usd）")
parser.add_argument("--robot_prim", default="/World/ur5",
                    help="机器人 prim 前缀；其下挂载件不当静态障碍（与桥 --robot_prim 默认一致）")
parser.add_argument("--exclude_object", default="/World/bottle",
                    help="抓取物体 prim 前缀（被抓，SurfaceGripper attach，不作静态障碍）。"
                         "逗号分隔多个。默认 /World/bottle。底座/桌面/桌腿等静态障碍保留。")
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf

# 机器人挂载 prim 叶名排除名单（腕上 Gemini2 相机机身各件；gripper 已被
# art_roots 排除，"gripper" 子串兜底）。叶名 = path.split("/")[-1]，
# 源自 ~/Desktop/scene.usd 实测（brief 的名单漏了 foot_*/camera_left/right，此处补全）。
EXCLUDE_NAMES = (
    "gemini2_body", "back_cover", "solid",
    "camera_rgb", "camera_ldm", "camera_left", "camera_right",
    "foot_left", "foot_right", "gripper_abs_stl",
)

SCENE = args.scene
OUT = os.path.expanduser("~/Desktop/scene_collisions.json")

if not os.path.isfile(SCENE):
    print(f"ERROR: {SCENE} not found"); app.close(); sys.exit(1)

stage = Usd.Stage.Open(SCENE)

# 找机器人 articulation root（排除其下的碰撞体）
art_roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"Articulation roots (robot, excluded): {art_roots}")

# base_link 世界变换（collider 世界坐标 → base_link 系）
base_link = None
for root in art_roots:
    parent = "/".join(root.split("/")[:-1]) + "/base_link"
    p = stage.GetPrimAtPath(parent)
    if p.IsValid():
        base_link = p; break
if base_link is None:
    print("ERROR: base_link not found"); app.close(); sys.exit(1)

base_world = UsdGeom.Xformable(base_link).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
base_inv = base_world.GetInverse()
base_trans = base_world.ExtractTranslation()
print(f"base_link world = ({base_trans[0]:.3f}, {base_trans[1]:.3f}, {base_trans[2]:.3f})")

# 遍历 collider，提取 AABB → base_link 系，按父节点分组
groups = defaultdict(list)  # group_name → [{position, size}]

for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    path = str(prim.GetPath())
    if any(path.startswith(a) for a in art_roots):
        continue

    # 排除机器人挂载 prim（腕上相机、gripper 等机器人挂载件，不当静态障碍）。
    # ROBOT_PRIM 与桥 --robot_prim 默认一致；夹爪碰撞来自 URDF，相机碰撞暂不进 URDF。
    path_name = path.split("/")[-1]
    if (path.startswith(args.robot_prim + "/")
            or path_name in EXCLUDE_NAMES
            or "gripper" in path.lower()):
        continue

    # 排除抓取物体（被抓，SurfaceGripper attach，不作静态障碍；否则 grasp 夹爪碰 box）
    _excl = [e.strip() for e in args.exclude_object.split(",") if e.strip()]
    if any(path == e or path.startswith(e.rstrip("/") + "/") for e in _excl):
        continue

    # 世界 AABB（world axis-aligned）。⚠️ 必须用 ComputeAlignedRange，绝不能用 GetRange()：
    # GetRange() 返回【局部】 range——center 恒在原点（不含 world transform），cylinder 会被
    # 算到 world 原点（RViz 里架子"跑到桌底"）；且尺寸是 unscaled 的（elevate_base 是 scaled
    # Cylinder：局部 0.5×0.5×1 经 scale 0.2,0.2,1.21 → world 0.1×0.1×1.21，GetRange 给 0.5
    # = "半径粗细不对"）。ComputeAlignedRange 正确反映 world 尺寸 + world 位置。
    try:
        bbox = UsdGeom.Imageable(prim).ComputeWorldBound(
            Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        wrange = bbox.ComputeAlignedRange()
        if wrange.IsEmpty():
            continue
        wmin, wmax = wrange.GetMin(), wrange.GetMax()
    except Exception:
        continue

    size = wmax - wmin
    if any(s > 100 for s in (size[0], size[1], size[2])):
        print(f"  skip (too big): {path}")
        continue

    # 世界中心 → base_link 系
    wcenter = (wmin + wmax) / 2.0
    bcenter = base_inv.Transform(wcenter)

    # 按父节点分组：/World/table/top → "table"；/World/ur5_elevate_base → "ur5_elevate_base"
    parent_path = "/".join(path.split("/")[:-1])
    if parent_path in ("/World", "/"):
        group_name = path.split("/")[-1]
    else:
        group_name = parent_path.split("/")[-1]

    groups[group_name].append({
        "position": [round(bcenter[0], 4), round(bcenter[1], 4), round(bcenter[2], 4)],
        "size": [round(size[0], 4), round(size[1], 4), round(size[2], 4)],
    })
    print(f"  {group_name:20s} ← {path.split('/')[-1]:20s} base_link=({bcenter[0]:.3f},{bcenter[1]:.3f},{bcenter[2]:.3f}) size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f})")

# 输出 groups JSON
out = {
    "frame": "base_link",
    "base_link_world": [round(base_trans[0], 4), round(base_trans[1], 4), round(base_trans[2], 4)],
    "groups": [{"name": k, "shapes": v} for k, v in sorted(groups.items())],
}
with open(OUT, "w") as f:
    json.dump(out, f, indent=2)

total_shapes = sum(len(v) for v in groups.values())
print(f"\n导出 {len(groups)} 个 compound 组（共 {total_shapes} 个 shape）→ {OUT}")

app.close()
