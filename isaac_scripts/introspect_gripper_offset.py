#!/usr/bin/env python3
"""量 ~/Desktop/scene.usd 里 gripper_base_link 相对 UR5 tool0/flange 的实际 transform
   （转接环 offset 修正用）。跑法：
   cd ~/isaacsim && source setup_ros_env.sh
   ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/introspect_gripper_offset.py
"""
import os
import math
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf

SCENE = os.path.expanduser("~/Desktop/scene.usd")
stage = Usd.Stage.Open(SCENE)
assert stage, f"cannot open {SCENE}"

print(f"scene: {SCENE}")
print(f"default prim: {stage.GetDefaultPrim().GetPath()}\n")


def world_xform(prim):
    xform = UsdGeom.Xformable(prim)
    M = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return M  # Gf.Matrix4d


def decompose(M):
    """4x4 -> (translation[3], rpy[3]) ROS rpy from rotation."""
    t = (M[3][0], M[3][1], M[3][2])
    R = Gf.Matrix3d()
    for i in range(3):
        for j in range(3):
            R[i][j] = M[i][j]
    # extract euler ZYX (rpy) — ROS convention r=pitch(y), p=... use Gf rotation
    rot = R.ExtractRotation()  # Gf.Quatd? -> use Rotation() helper
    # Gf.Matrix3d -> Rigid3? Use euler decomposition manually
    # roll(x), pitch(y), yaw(z) from rotation matrix
    r = R  # 3x3
    # standard rpy extraction (ROS = fixed-xyz = Rz(yaw)*Ry(pitch)*Rx(roll))
    sy = math.sqrt(r[0][0] ** 2 + r[1][0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(r[2][1], r[2][2])
        pitch = math.atan2(-r[2][0], sy)
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = math.atan2(-r[1][2], r[1][1])
        pitch = math.atan2(-r[2][0], sy)
        yaw = 0.0
    return t, (roll, pitch, yaw)


# collect candidate link prims by name
links = {}
for p in stage.Traverse():
    name = p.GetName()
    ps = str(p.GetPath()).lower()
    if name in ("base_link", "shoulder_link", "upper_arm_link", "forearm_link",
               "wrist_1_link", "wrist_2_link", "wrist_3_link", "flange", "tool0",
               "ft_frame", "gripper_base_link", "gripper_tip",
               "world") or "adapter" in ps or "tool" in name.lower() or "gripper_base" in name.lower():
        links[name] = p

print("--- candidate link prims + world transforms ---")
for name, p in links.items():
    M = world_xform(p)
    t, rpy = decompose(M)
    print(f"  {name:22s} path={p.GetPath()}  xyz=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f}) "
          f"rpy=({rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f})")

print("\n--- relative transforms (gripper_base wrt tool0 / flange) ---")


def rel(child_name, parent_name):
    if child_name not in links or parent_name not in links:
        print(f"  {child_name} wrt {parent_name}: MISSING prim")
        return None
    Mc = world_xform(links[child_name])
    Mp = world_xform(links[parent_name])
    # Gf Matrix4d uses ROW-vector convention (p' = p * M), so
    # child-relative-to-parent = Mc * Mp.GetInverse()
    Mrel = Mc * Mp.GetInverse()
    t, rpy = decompose(Mrel)
    print(f"  {child_name} wrt {parent_name}:  xyz=({t[0]:.5f},{t[1]:.5f},{t[2]:.5f})  "
          f"rpy=({rpy[0]:.5f},{rpy[1]:.5f},{rpy[2]:.5f})")
    return t, rpy


rel("gripper_base_link", "tool0")
rel("gripper_base_link", "flange")
rel("gripper_base_link", "wrist_3_link")

# also try adapter prim if present
adapter_prims = [p for p in stage.Traverse() if "adapter" in str(p.GetPath()).lower()]
print(f"\n--- adapter prims found: {len(adapter_prims)} ---")
for p in adapter_prims[:5]:
    M = world_xform(p)
    t, rpy = decompose(M)
    print(f"  {p.GetPath()}  type={p.GetTypeName()}  "
          f"xyz=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f}) rpy=({rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f})")

# print raw rotation matrix of flange + gripper_base to double-check rpy extraction
print("\n--- raw flange / gripper_base rotation matrices (sanity for rpy) ---")
for nm in ("flange", "wrist_3_link", "gripper_base_link"):
    if nm in links:
        M = world_xform(links[nm])
        print(f"  {nm} R = [{M[0][0]:.4f} {M[0][1]:.4f} {M[0][2]:.4f}] "
              f"[{M[1][0]:.4f} {M[1][1]:.4f} {M[1][2]:.4f}] "
              f"[{M[2][0]:.4f} {M[2][1]:.4f} {M[2][2]:.4f}]  t=({M[3][0]:.4f},{M[3][1]:.4f},{M[3][2]:.4f})")

app.close()
