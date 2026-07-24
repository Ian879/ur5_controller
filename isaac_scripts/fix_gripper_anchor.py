#!/usr/bin/env python3
"""Robust fix: 松开夹爪地锚（Isaac Sim 6.0 Newton physics 版）。

问题：夹爪 /World/gripper_abs_stl/Geometry 上是 NewtonArticulationRootAPI（不是
UsdPhysics.ArticulationRootAPI），且夹爪可能是 referenced 进来的 → 之前用
RemoveAPI(UsdPhysics.ArticulationRootAPI) / RemovePrim 不生效。

本脚本用 Sdf 在根层 author override：
  1. 给 /World/gripper_abs_stl/Geometry 的 apiSchemas 加 deletedItems，删掉
     NewtonArticulationRootAPI + PhysxArticulationAPI（+ PhysicsArticulationRootAPI）
  2. 把地锚 root_joint SetActive(False)（referenced prim 用这个才生效）
然后存盘 → 重开 → 验证（assert 真的没了）。

跑法：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh …/isaac_scripts/fix_gripper_anchor.py
"""
import os
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, Sdf

SCENE = os.path.expanduser("~/Desktop/scene.usd")
GEOM = "/World/gripper_abs_stl/Geometry"
GROUND_JOINT = "/World/gripper_abs_stl/Physics/root_joint"
DELETE_TOKENS = ["NewtonArticulationRootAPI", "PhysxArticulationAPI",
                 "PhysicsArticulationRootAPI"]


def compose_apiSchemas(stage, path):
    p = stage.GetPrimAtPath(path)
    return list(p.GetAppliedSchemas()) if p.IsValid() else None


stage = Usd.Stage.Open(SCENE)
layer = stage.GetRootLayer()
print(f"SCENE: {SCENE}")
print(f"root layer: {layer.realPath}")

# --- 1) delete articulation API tokens via root-layer apiSchemas override ---
print(f"\nBEFORE: {GEOM} appliedSchemas = {compose_apiSchemas(stage, GEOM)}")
spec = layer.GetPrimAtPath(GEOM)
if spec is None:
    spec = Sdf.CreatePrimInLayer(layer, GEOM)  # create 'over' for override
lop = spec.GetInfo("apiSchemas")
if lop is None:
    lop = Sdf.TokenListOp()
del_items = list(lop.deletedItems) if lop.deletedItems else []
for tok in DELETE_TOKENS:
    if tok not in del_items:
        del_items.append(tok)
lop.deletedItems = del_items
spec.SetInfo("apiSchemas", lop)
print(f"authored apiSchemas.deletedItems = {del_items}  (in root layer)")

# --- 2) deactivate the ground-anchor root_joint ---
jp = stage.GetPrimAtPath(GROUND_JOINT)
print(f"\nBEFORE: {GROUND_JOINT} valid={jp.IsValid()} active={jp.IsActive() if jp.IsValid() else None}")
if jp.IsValid():
    jp.SetActive(False)  # authors active=false in root layer (works for referenced prims)

# --- save ---
stage.Save()
print("\nsaved.")

# --- reopen + verify ---
s2 = Usd.Stage.Open(SCENE)
arts = [str(p.GetPath()) for p in s2.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"\nAFTER reopen:")
print(f"  {GEOM} appliedSchemas = {compose_apiSchemas(s2, GEOM)}")
jp2 = s2.GetPrimAtPath(GROUND_JOINT)
print(f"  {GROUND_JOINT} active={jp2.IsActive() if jp2.IsValid() else 'GONE'}")
print(f"  ALL ArticulationRootAPI prims = {arts}")

geom_has = bool(s2.GetPrimAtPath(GEOM).HasAPI(UsdPhysics.ArticulationRootAPI))
if len(arts) == 1 and arts[0] == "/World/ur5/root_joint":
    print("\n✅ 成功：只剩 UR5 一个 articulation root，夹爪已并入。")
else:
    print(f"\n⚠️  还有 {len(arts)} 个 articulation root（预期 1）。Newton schema 可能要用别的类删。")

app.close()
