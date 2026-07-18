"""轻量探查：打开 Isaac 自带 ur5.usd，dump 关节名 + 链接路径 + 根结构。

确认关节名是否与 ur_description（shoulder_pan_joint…wrist_3_joint）一致，
以及腕部链接的 prim 路径（build_scene.py 要把相机挂到它下面）。

运行：~/isaacsim/python.sh isaac_scripts/introspect_ur5.py
"""
import sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, UsdGeom

UR5_USD = "/home/gds/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5/ur5.usd"

stage = Usd.Stage.Open(UR5_USD)
if not stage:
    print("ERROR: 打不开", UR5_USD)
    app.close(); sys.exit(1)

print("\n===== 默认 prim / 根 =====")
dp = stage.GetDefaultPrim()
print("  default prim:", dp.GetPath())

print("\n===== 关节（UsdPhysics.Joint）=====")
joints = []
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Joint):
        joints.append((str(p.GetPath()), p.GetTypeName()))
for path, typ in joints:
    print(f"  {typ:20s} {path}")
print(f"  关节总数: {len(joints)}")

print("\n===== 含 'wrist' 或 'tool' 的 prim（找相机挂载点）=====")
for p in stage.Traverse():
    s = str(p.GetPath()).lower()
    if "wrist" in s or "tool" in s:
        print(f"  {p.GetTypeName():16s} {p.GetPath()}")

print("\n===== 顶层 prim（看 ur5.usd 根结构）=====")
for p in stage.GetPseudoRoot().GetChildren():
    print(f"  {p.GetTypeName():16s} {p.GetPath()}")

app.close()
