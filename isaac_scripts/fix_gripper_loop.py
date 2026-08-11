#!/usr/bin/env python3
"""重建 link1_end_connection + 闭环 joint，把夹爪补成完整平行四边形（与 URDF 一致）。

根因：URDF 里 r/l_link1_end_connection（link1 末端铰链耳）+ 它到 finger_end 的闭环铰链，在
转 USD 时整个丢了 → link1 末端悬空、没拉 finger → 耷拉/崩断。本脚本：
  1. 重建 gripper_r/l_link1_end_connection prim（Xform + RigidBodyAPI）。
  2. 建 fixed joint（link1 → link1_end，localPos0 用 URDF origin (0, 0.053, 0)）。
  3. 建闭环 joint（link1_end ↔ finger_end，revolute，excludeFromArticulation=True）。
打印 link1_end 与 finger_end 的世界距离确认铰链点对齐，存盘 ~/Desktop/scene.usd。

前置：先跑 fix_gripper_mimic.py（mimic referenceJoint）。

跑：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh ~/projects/shengzhi/ur5_controller/isaac_scripts/fix_gripper_loop.py
"""
import math
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, UsdGeom, Gf  # noqa: E402

SCENE = os.path.expanduser("~/Desktop/scene.usd")
stage = Usd.Stage.Open(SCENE)


def find_first(name):
    for p in stage.Traverse():
        if p.GetName() == name:
            return str(p.GetPath())
    return None


def world_pos(path):
    p = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
    return xf.ExtractTranslation()


# (side, link1, link1_end(待建), finger_end(已存在), fixed_joint名, loop_joint名, URDF origin)
sides = [
    ("r", "gripper_r_link1", "gripper_r_link1_end_connection",
     "gripper_r_link_finger_end_connection",
     "gripper_r_link1_end_connection_joint", "gripper_r_loop_joint", (0.0, 0.053, 0.0)),
    ("l", "gripper_l_link1", "gripper_l_link1_end_connection",
     "gripper_l_link_finger_end_connection",
     "gripper_l_link1_end_connection_joint", "gripper_l_loop_joint", (0.0, 0.053, 0.0)),
]

for side, link1_name, end_name, finger_end_name, j_fixed, j_loop, urdf_origin in sides:
    link1 = find_first(link1_name)
    finger_end = find_first(finger_end_name)
    if not link1 or not finger_end:
        print(f"RESULT [{side}] ⚠️ 找不到 {link1_name}({link1}) 或 {finger_end_name}({finger_end})，跳过")
        continue

    # 1. 重建 link1_end_connection prim（在 link1 下；Xform + RigidBodyAPI）
    end_path = f"{link1}/{end_name}"
    if stage.GetPrimAtPath(end_path).IsValid():
        stage.RemovePrim(end_path)
    end_prim = UsdGeom.Xform.Define(stage, end_path)
    UsdPhysics.RigidBodyAPI.Apply(end_prim.GetPrim())
    print(f"RESULT [{side}] + prim {end_path}")

    # 2. fixed joint: link1 → link1_end（URDF r_link1_end_connection_joint origin xyz）
    jp_fixed = f"/World/gripper_abs_stl/Physics/{j_fixed}"
    if stage.GetPrimAtPath(jp_fixed).IsValid():
        stage.RemovePrim(jp_fixed)
    fixed = UsdPhysics.FixedJoint.Define(stage, jp_fixed)
    fixed.CreateBody0Rel().SetTargets([link1])
    fixed.CreateBody1Rel().SetTargets([end_path])
    fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(*urdf_origin))  # joint 点在 link1 frame（URDF origin）
    fixed.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))  # end 原点 = joint 点
    fixed.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    fixed.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    print(f"RESULT [{side}] + fixed joint {jp_fixed}: {link1} → {end_path} (localPos0={urdf_origin})")

    # 3. 闭环 joint: link1_end ↔ finger_end（revolute，闭环，排除出 articulation）
    try:
        p_end = world_pos(end_path)
        p_fend = world_pos(finger_end)
        dist = math.dist(p_end, p_fend)
        print(f"RESULT [{side}] link1_end @ {[round(v, 4) for v in p_end]}")
        print(f"RESULT [{side}] finger_end @ {[round(v, 4) for v in p_fend]}")
        print(f"RESULT [{side}] 两端世界距离 = {dist * 1000:.2f} mm "
              f"(<5mm 铰链点对齐✓；>5mm URDF 几何有偏差，joint 会硬拉)")
    except Exception as e:  # noqa: BLE001
        print(f"RESULT [{side}] world pos 读失败({e})")

    jp_loop = f"/World/gripper_abs_stl/Physics/{j_loop}"
    if stage.GetPrimAtPath(jp_loop).IsValid():
        stage.RemovePrim(jp_loop)
    loop = UsdPhysics.RevoluteJoint.Define(stage, jp_loop)
    loop.CreateBody0Rel().SetTargets([end_path])
    loop.CreateBody1Rel().SetTargets([finger_end])
    loop.CreateExcludeFromArticulationAttr().Set(True)  # 闭环 joint 必须排除出 articulation tree
    loop.CreateAxisAttr("Z")
    loop.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    loop.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    loop.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    loop.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    print(f"RESULT [{side}] + loop joint {jp_loop}: {end_path} ↔ {finger_end}")

stage.Save()
print(f"\nRESULT DONE: link1_end_connection 重建 + 闭环 joint 加好，存盘 {SCENE}")
app.close()
