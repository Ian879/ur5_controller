#!/usr/bin/env python3
"""探查 ~/Desktop/scene.usd + scenes/scene.usd 里的 gripper articulation +
其驱动关节名（供 task-5 桥 OmniGraph 的 robotPath/prim + isaac_gripper_action
的 DRIVEN_JOINT 映射用）。

跑法：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh isaac_scripts/introspect_gripper.py
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics

SCENES = [
    os.path.expanduser("~/Desktop/scene.usd"),
    "/home/gds/projects/shengzhi/ur5_controller/scenes/scene.usd",
]

for SCENE in SCENES:
    print(f"\n===== SCENE: {SCENE} =====")
    if not os.path.isfile(SCENE):
        print("  NOT FOUND")
        continue
    s = Usd.Stage.Open(SCENE)
    if not s:
        print("  CANNOT OPEN")
        continue
    print("  default prim:", s.GetDefaultPrim().GetPath())

    print("  --- ALL ArticulationRootAPI prims ---")
    art_paths = []
    for p in s.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_paths.append(str(p.GetPath()))
            print(f"    ART: {p.GetPath()} | typeName={p.GetTypeName()}")
    if not art_paths:
        print("    (none)")

    print("  --- prims with gripper/lebai/LMG in path ---")
    hits = 0
    for p in s.Traverse():
        ps = str(p.GetPath()).lower()
        if "gripper" in ps or "lebai" in ps or "lmg" in ps:
            hits += 1
            print(f"    {p.GetTypeName():24s} {p.GetPath()}")
    if hits == 0:
        print("    (none — gripper not in this scene)")

    print("  --- ALL physics joints + their jointName ---")
    joints = 0
    for p in s.Traverse():
        if p.IsA(UsdPhysics.Joint):
            joints += 1
            jn_attr = p.GetAttribute("physics:jointName")
            jn = jn_attr.Get() if jn_attr else None
            # only print gripper-ish to limit noise
            ps = str(p.GetPath()).lower()
            if "gripper" in ps or "lebai" in ps or "lmg" in ps:
                print(f"    JOINT: {p.GetPath()} | type={p.GetTypeName()} | jointName={jn}")
    print(f"  (total physics joints in scene: {joints})")

app.close()
