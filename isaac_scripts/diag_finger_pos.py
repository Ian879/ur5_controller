#!/usr/bin/env python3
"""确认 finger 相对 gripper_tip_tcp 的方向（水平夹侧 vs 垂直夹顶）。"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf  # noqa: E402

stage = Usd.Stage.Open(os.path.expanduser("~/Desktop/scene.usd"))


def world(name):
    for p in stage.Traverse():
        if p.GetName() == name:
            xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
            return xf.ExtractTranslation(), xf
    return None, None


gt_pos, gt_xf = world("gripper_tip_tcp")
f_pos, _ = world("gripper_r_link_finger")
fl_pos, _ = world("gripper_l_link_finger")

print(f"gripper_tip_tcp world = {[round(v, 4) for v in gt_pos]}")
print(f"r_finger world = {[round(v, 4) for v in f_pos]}")
print(f"l_finger world = {[round(v, 4) for v in fl_pos]}")

gt_inv = gt_xf.GetInverse()
fr = gt_inv.Transform(Gf.Vec3d(*f_pos))
fl = gt_inv.Transform(Gf.Vec3d(*fl_pos))
print(f"r_finger 相对 gripper_tip (tip frame xyz) = {[round(v, 4) for v in fr]}")
print(f"l_finger 相对 gripper_tip (tip frame xyz) = {[round(v, 4) for v in fl]}")
print("-> x=水平左右, z=上下。x大且z≈0 = 手指水平(夹侧面)；z大 = 手指在上下(夹顶/底)")

app.close()
