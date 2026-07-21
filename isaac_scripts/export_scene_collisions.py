#!/usr/bin/env python3
"""读 ~/Desktop/scene.usd 里所有带 collider 的非机器人刚体，
按父节点分组（如 /World/table/* → 一个 "table" 组），换算到 base_link 系，
输出 JSON 供 ROS 侧 world_sync_node 发布到 MoveIt PlanningScene。

跑法（Isaac python）:
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/export_scene_collisions.py

输出: ~/Desktop/scene_collisions.json（groups 格式，每个 group = 一个 compound 对象）
"""

import json
import os
import sys
from collections import defaultdict

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf

SCENE = os.path.expanduser("~/Desktop/scene.usd")
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

    # 世界 AABB
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
