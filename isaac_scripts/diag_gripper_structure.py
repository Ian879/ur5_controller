#!/usr/bin/env python3
"""打印夹爪 USD 完整结构（prim 树 + joint body0/body1 + mimic + exclude）。

诊断夹爪连接关系：哪些 link、joint 怎么连、mimic 配没配、闭环 joint 在不在。
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

stage = Usd.Stage.Open(os.path.expanduser("~/Desktop/scene.usd"))
ROOT = "/World/gripper_abs_stl"

print(f"\n########## 夹爪结构 {ROOT} ##########")

print("\n=== 所有 prim（路径 [类型] name=）===")
prims = []
for p in stage.Traverse():
    path = str(p.GetPath())
    if path.startswith(ROOT):
        prims.append((path, p.GetTypeName(), p.GetName()))
for path, t, name in sorted(prims):
    print(f"  {path} [{t}] name={name}")

print("\n=== joint 详细（body0/body1 + mimic gearing/referenceJoint + exclude）===")
for p in stage.Traverse():
    t = p.GetTypeName()
    if "Joint" not in t:
        continue
    r0 = p.GetRelationship("physics:body0")
    r1 = p.GetRelationship("physics:body1")
    b0 = r0.GetTargets() if r0.IsValid() else None
    b1 = r1.GetTargets() if r1.IsValid() else None
    g = p.GetAttribute("physxMimicJoint:mimic:gearing")
    gearing = g.Get() if (g.IsValid() and g.Get() is not None) else None
    rj = p.GetRelationship("physxMimicJoint:mimic:referenceJoint")
    refj = rj.GetTargets() if (rj.IsValid() and rj.GetTargets()) else None
    excl = p.GetAttribute("physics:excludeFromArticulation")
    excl_v = excl.Get() if excl.IsValid() else None
    print(f"  {p.GetName()} [{t}] body0={b0} body1={b1} | gearing={gearing} refJoint={refj} excl={excl_v}")

app.close()
