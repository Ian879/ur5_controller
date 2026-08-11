#!/usr/bin/env python3
"""诊断夹爪 USD 的关节机构：查 joint1/joint2/joint_finger 是否 mimic 耦合、闭环 joint 在不在。

Lebai 平行夹爪是欠驱动四连杆：joint1 驱动，joint2/joint_finger/end_connection 应 mimic joint1；
link1_end ↔ finger_end 在真实机构是一根连杆（闭环）。URDF 是树不能表达闭环，转 USD 时 mimic
可能丢失 / 闭环 joint 可能缺失 → 夹的时候 link1 自己转、手指不跟 → 散架。

跑：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh ~/projects/shengzhi/ur5_controller/isaac_scripts/diag_gripper_joints.py
"""
import os
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics  # noqa: E402

stage = Usd.Stage.Open(os.path.expanduser("~/Desktop/scene.usd"))

print("\n=== 所有 Joint（type + body0/body1 + mimic/gear）===")
for p in stage.Traverse():
    t = p.GetTypeName()
    if "Joint" not in t:
        continue
    b0 = p.GetAttribute("physics:body0").Get() if p.HasAttribute("physics:body0") else None
    b1 = p.GetAttribute("physics:body1").Get() if p.HasAttribute("physics:body1") else None
    mimics = []
    for a in p.GetAttributes():
        n = a.GetName().lower()
        if any(k in n for k in ("mimic", "gear", "coupl", "master", "reference")):
            mimics.append(f"{a.GetName()}={a.Get()}")
    # excludeFromArticulation（闭环 joint 通常 True）
    excl = p.GetAttribute("physics:excludeFromArticulation").Get() \
        if p.HasAttribute("physics:excludeFromArticulation") else None
    print(f"{p.GetPath()} [{t}]\n    body0={b0}\n    body1={b1}\n    "
          f"mimic={' | '.join(mimics) if mimics else '❌无'}  excludeFromArtic={excl}")

print("\n=== MimicJoint prims（USD 物理 mimic schema）===")
found = False
for p in stage.Traverse():
    if "Mimic" in p.GetTypeName():
        found = True
        print(f" {p.GetPath()} [{p.GetTypeName()}]")
        for a in p.GetAttributes():
            v = a.Get()
            if v is not None and a.GetName() not in ("primOrder",):
                print(f"    {a.GetName()}={v}")
if not found:
    print(" ❌ 没有任何 MimicJoint prim（mimic 没转成 USD schema）")

print("\n=== ArticulationRoot ===")
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f" root: {p.GetPath()}")

print("\n=== PhysicsScene（物理后端：看 physx: / newton: 哪套在用）===")
for p in stage.Traverse():
    if p.GetTypeName() == "PhysicsScene":
        print(f" {p.GetPath()}")
        for a in p.GetAttributes():
            v = a.Get()
            if v is not None and a.GetName() not in ("primOrder",):
                print(f"    {a.GetName()}={v}")

print("\n=== gripper_r_joint2 全部属性（确认 referenceJoint master 在不在）===")
j2 = stage.GetPrimAtPath("/World/gripper_abs_stl/Physics/gripper_r_joint2")
if j2.IsValid():
    for a in j2.GetAttributes():
        v = a.Get()
        if v is not None:
            print(f"    {a.GetName()} = {v}")
    # 显式查 referenceJoint（rel，可能存在但 None）
    rj = j2.GetAttribute("physxMimicJoint:mimic:referenceJoint")
    print(f"    [显式] physxMimicJoint:mimic:referenceJoint 存在={rj.IsValid()} 值={rj.Get() if rj.IsValid() else 'N/A'}")
else:
    print(" gripper_r_joint2 prim 不存在")

app.close()
