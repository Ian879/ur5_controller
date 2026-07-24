#!/usr/bin/env python3
"""Dump the flange-region prim tree + ALL physics joints in ~/Desktop/scene.usd,
to understand the current assembly: how camera adapter / gripper adapter / gripper
/ robot flange are (or aren't) connected.

跑法：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/introspect_assembly.py
"""
import os
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, UsdGeom

SCENE = os.path.expanduser("~/Desktop/scene.usd")
print(f"===== SCENE: {SCENE} =====")
s = Usd.Stage.Open(SCENE)
print("default prim:", s.GetDefaultPrim().GetPath())

print("\n--- FULL /World prim tree (typeName + path) ---")
for p in s.Traverse():
    ps = str(p.GetPath())
    if ps.startswith("/World"):
        tn = p.GetTypeName()
        # only show Xform/Scope/Joint/mesh-ish + anything with flange/camera/gripper/adapter/support/tn_j0
        low = ps.lower()
        if (tn in ("Xform", "Scope", "PhysicsJoint", "PhysicsFixedJoint",
                   "PhysicsRevoluteJoint", "PhysicsPrismaticJoint")
                or any(k in low for k in
                       ("camera", "flange", "adapter", "support", "gripper",
                        "tn_j0", "gemini", "lmg", "wrist", "tool", "finger"))):
            depth = ps.count("/") - 1
            print(f"{'  '*depth}{tn:24s} {ps}")

print("\n--- ALL physics joints (body0 -> body1, type, jointName) ---")
nj = 0
for p in s.Traverse():
    if p.IsA(UsdPhysics.Joint) or "Joint" in p.GetTypeName():
        nj += 1
        b0 = p.GetAttribute("physics:body0")
        b1 = p.GetAttribute("physics:body1")
        jn = p.GetAttribute("physics:jointName")
        print(f"  [{p.GetTypeName():22s}] {p.GetPath()}")
        print(f"      body0={b0.Get() if b0 else None}  body1={b1.Get() if b1 else None}  jointName={jn.Get() if jn else None}")

print(f"\nTOTAL joints: {nj}")

print("\n--- ArticulationRootAPI prims ---")
for p in s.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"  ART: {p.GetPath()} | {p.GetTypeName()}")

app.close()
