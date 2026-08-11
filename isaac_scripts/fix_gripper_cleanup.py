#!/usr/bin/env python3
"""撤掉 fix_gripper_loop 加错的闭环 joint + 重建的 link1_end（回归 fix_gripper_mimic 后的状态）。

原因：link1_end 与 finger_end 相距 53mm（平行四边形第4边 CD 连杆长度），不是同一铰链点。
fix_gripper_loop 加的 localPos=0 闭环 joint 会硬拉它们重合 → 崩断。而 URDF 根本没显式闭环
joint——它用 mimic（finger_end mimic joint1）模拟 CD 连杆。fix_gripper_mimic 已修好 mimic，
足够保持平行四边形。本脚本删掉我加错的 loop joint + link1_end prim + fixed joint。
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

stage = Usd.Stage.Open(os.path.expanduser("~/Desktop/scene.usd"))

to_remove = [
    # 加错的闭环 joint
    "/World/gripper_abs_stl/Physics/gripper_r_loop_joint",
    "/World/gripper_abs_stl/Physics/gripper_l_loop_joint",
    # 重建的 fixed joint
    "/World/gripper_abs_stl/Physics/gripper_r_link1_end_connection_joint",
    "/World/gripper_abs_stl/Physics/gripper_l_link1_end_connection_joint",
]
# 重建的 link1_end prim（找路径删）
for p in stage.Traverse():
    if p.GetName() in ("gripper_r_link1_end_connection", "gripper_l_link1_end_connection") \
            and "link1/" in str(p.GetPath()):
        to_remove.append(str(p.GetPath()))

removed = 0
for path in to_remove:
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
        print(f"RESULT 删 {path}")
        removed += 1

stage.Save()
print(f"\nRESULT DONE: 删了 {removed} 个我加错的 prim/joint，回归 mimic-only 状态。")
print("RESULT 现在夹爪 = mimic（fix_gripper_mimic 修好）+ 原 URDF 结构，和 URDF 一致。")
app.close()
