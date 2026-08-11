#!/usr/bin/env python3
"""修夹爪 mimic：给每个 mimic joint 补 physxMimicJoint:mimic:referenceJoint → gripper_r_joint1。

根因：夹爪 USD 的 mimic joint（joint2/joint_finger/end_connection/...）有 gearing（系数对），
但【缺 referenceJoint】（没告诉 PhysX master 是谁）+ 没 MimicJoint prim → PhysX mimic 不生效
→ joint2/joint_finger 不跟 joint1 → 夹的时候散架。Newton 那套（newton:mimicEnabled）在 PhysX
场景下也不认。本脚本扫所有有 physxMimicJoint:mimic:gearing 的 joint，补 referenceJoint 指向
gripper_r_joint1（master，URDF 里所有 mimic 都跟它），存盘 ~/Desktop/scene.usd。

跑：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh ~/projects/shengzhi/ur5_controller/isaac_scripts/fix_gripper_mimic.py
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

SCENE = os.path.expanduser("~/Desktop/scene.usd")
stage = Usd.Stage.Open(SCENE)

MASTER = "/World/gripper_abs_stl/Physics/gripper_r_joint1"
assert stage.GetPrimAtPath(MASTER).IsValid(), f"master {MASTER} 不存在——检查 prim 路径"

fixed = 0
for p in stage.Traverse():
    gearing = p.GetAttribute("physxMimicJoint:mimic:gearing")
    if not gearing.IsValid() or gearing.Get() is None:
        continue  # 不是 mimic joint
    # 补 referenceJoint（relationship → master）。gearing/offset/axis 已对（URDF 转的），只差这个。
    rel = p.CreateRelationship("physxMimicJoint:mimic:referenceJoint", custom=False)
    rel.SetTargets([MASTER])
    print(f"RESULT + {p.GetPath()}  referenceJoint → {MASTER}  (gearing={gearing.Get()})")
    fixed += 1

stage.Save()
print(f"\nRESULT DONE: 补了 {fixed} 个 mimic joint 的 referenceJoint（master={MASTER}），存盘 {SCENE}")
print("RESULT 重启桥后，夹爪驱动 gripper_r_joint1 时 link1/link2/finger 应同步开合（不再散架）。")
app.close()
