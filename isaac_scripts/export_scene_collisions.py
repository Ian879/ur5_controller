#!/usr/bin/env python3
"""读 ~/Desktop/scene.usd 里所有带 collider 的非机器人刚体，
换算到 base_link 系，输出 JSON 供 ROS 侧同步到 MoveIt PlanningScene。

跑法（Isaac python）:
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/export_scene_collisions.py

输出: ~/Desktop/scene_collisions.json
然后 ROS 侧: ros2 run ur5_motion sync_scene
"""

import json
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf

SCENE = os.path.expanduser("~/Desktop/scene.usd")
OUT = os.path.expanduser("~/Desktop/scene_collisions.json")

if not os.path.isfile(SCENE):
    print(f"ERROR: {SCENE} not found"); app.close(); sys.exit(1)

stage = Usd.Stage.Open(SCENE)

# 找机器人 articulation root（排除其下的碰撞体——那是机器人自身）
art_roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"Articulation roots (robot, excluded): {art_roots}")

# 找 base_link 世界变换（把 collider 世界坐标换算到 base_link 系）
base_link = None
for root in art_roots:
    # base_link 通常在 articulation root 的父级（如 /World/ur5/base_link）
    parent = "/".join(root.split("/")[:-1]) + "/base_link"
    p = stage.GetPrimAtPath(parent)
    if p.IsValid():
        base_link = p
        break
if base_link is None:
    print("ERROR: base_link not found under articulation root"); app.close(); sys.exit(1)

base_world = UsdGeom.Xformable(base_link).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
base_inv = base_world.GetInverse()
base_trans = base_world.ExtractTranslation()
print(f"base_link world = ({base_trans[0]:.3f}, {base_trans[1]:.3f}, {base_trans[2]:.3f})")

# 遍历所有带 CollisionAPI 的 prim，提取世界 AABB → base_link 系 box
objects = []
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    path = str(prim.GetPath())
    # 排除机器人自身碰撞（articulation root 下）
    if any(path.startswith(a) for a in art_roots):
        continue

    # 计算世界 AABB
    try:
        bbox = UsdGeom.Imageable(prim).ComputeWorldBound(
            Usd.TimeCode.Default(), UsdGeom.Tokens.default_
        )
        wrange = bbox.ComputeAlignedRange()
        if wrange.IsEmpty():
            continue
        wmin, wmax = wrange.GetMin(), wrange.GetMax()
    except Exception:
        continue

    # 跳过过大的（如无限地面 Plane → AABB 巨大）
    size = wmax - wmin
    if any(s > 100 for s in (size[0], size[1], size[2])):
        print(f"  skip (too big, likely infinite plane): {path} size={size}")
        continue

    # 世界中心 → base_link 系
    wcenter = (wmin + wmax) / 2.0
    bcenter = base_inv.Transform(wcenter)

    # 用 prim 路径最后一段做名字
    name = path.split("/")[-1] or "obj"
    # 避免重名
    base_name = name
    idx = 1
    existing = [o["name"] for o in objects]
    while name in existing:
        name = f"{base_name}_{idx}"; idx += 1

    objects.append({
        "name": name,
        "prim": path,
        "position": [round(bcenter[0], 4), round(bcenter[1], 4), round(bcenter[2], 4)],
        "size": [round(size[0], 4), round(size[1], 4), round(size[2], 4)],
    })
    print(f"  {name:20s} prim={path}  base_link pos=({bcenter[0]:.3f},{bcenter[1]:.3f},{bcenter[2]:.3f})  size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f})")

# 写 JSON
out = {"frame": "base_link", "base_link_world": [round(base_trans[0],4), round(base_trans[1],4), round(base_trans[2],4)], "objects": objects}
with open(OUT, "w") as f:
    json.dump(out, f, indent=2)
print(f"\n导出 {len(objects)} 个碰撞体 → {OUT}")

app.close()
