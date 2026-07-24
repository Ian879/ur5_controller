#!/usr/bin/env python3
"""Dump world poses (for relative-offset computation) + bboxes of the flange,
camera adapter, gripper adapter, gripper_base, camera body — to get REAL numbers
for welding the gripper into the UR5 articulation + URDF mount offset.

跑法：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/introspect_mount_geometry.py
"""
import os
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf, UsdPhysics

SCENE = os.path.expanduser("~/Desktop/scene.usd")
s = Usd.Stage.Open(SCENE)

PRIMS = [
    "/World/ur5/wrist_3_link/flange",
    "/World/ur5/wrist_3_link",
    "/World/rgbd_camera/cameraURflangeSupport",
    "/World/rgbd_camera/cameraURflangeSupport/node_",
    "/World/LMG_90_adapter/tn__j0l6Rg0k0f35W04y7p1",
    "/World/gripper_abs_stl/Geometry/gripper_base_link",
    "/World/gripper_abs_stl/Geometry/gripper_base_link/visual_gripper_base",
    "/World/rgbd_camera/orbbec_gemini_2/Orbbec_Gemini2/gemini2_body",
    "/World/rgbd_camera/orbbec_gemini_2/Orbbec_Gemini2/camera_rgb",
]


def world_pose(stage, path):
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        print(f"  {path}: NOT FOUND")
        return None
    xf = UsdGeom.Xformable(p)
    try:
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    except Exception as e:
        print(f"  {path}: pose err {e}")
        return None
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()  # Gf.Quatd (real, imag)
    real = q.GetReal()
    try:
        imag = q.GetImaginary()
    except Exception:
        imag = (q.__imag if hasattr(q, "_Quatd__imag") else 0.0, 0.0, 0.0)
    print(f"  {path}")
    print(f"      world t = [{t[0]:.5f}, {t[1]:.5f}, {t[2]:.5f}]")
    print(f"      quat (xyzw) = [{imag[0]:.5f}, {imag[1]:.5f}, {imag[2]:.5f}, {real:.5f}]")
    return (t, m)


def bbox(stage, path):
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return
    img = UsdGeom.Imageable(p)
    try:
        b = img.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        r = b.ComputeAlignedRange()
        if r.IsEmpty():
            print(f"  bbox {path}: empty")
            return
        mn, mx = r.GetMin(), r.GetMax()
        size = mx - mn
        ctr = (mn + mx) / 2.0
        print(f"  bbox {path}: size=[{size[0]:.5f},{size[1]:.5f},{size[2]:.5f}] "
              f"worldCenter=[{ctr[0]:.5f},{ctr[1]:.5f},{ctr[2]:.5f}]")
    except Exception as e:
        print(f"  bbox {path}: err {e}")


print("=== WORLD POSES ===")
poses = {}
for path in PRIMS:
    res = world_pose(s, path)
    poses[path] = res

print("\n=== BBOXES (adapter thickness / extents) ===")
for path in PRIMS:
    bbox(s, path)

# relative: cameraURflangeSupport in flange frame
print("\n=== cameraURflangeSupport POSE IN FLANGE FRAME ===")
fl = poses.get("/World/ur5/wrist_3_link/flange")
ca = poses.get("/World/rgbd_camera/cameraURflangeSupport")
if fl and ca:
    fl_t, fl_m = fl
    ca_t, _ = ca
    fl_inv = fl_m.GetInverse()
    rel = fl_inv.Transform(ca_t)
    print(f"  cameraURflangeSupport in flange frame: t=[{rel[0]:.5f}, {rel[1]:.5f}, {rel[2]:.5f}]")

print("\n=== gripper_base POSE IN FLANGE FRAME (current/floating — may be arbitrary) ===")
if fl:
    fl_t, fl_m = fl
    fl_inv = fl_m.GetInverse()
    for path in ["/World/gripper_abs_stl/Geometry/gripper_base_link",
                 "/World/LMG_90_adapter/tn__j0l6Rg0k0f35W04y7p1"]:
        gb = poses.get(path)
        if gb:
            gb_t, _ = gb
            rel = fl_inv.Transform(gb_t)
            print(f"  {path} in flange frame: t=[{rel[0]:.5f}, {rel[1]:.5f}, {rel[2]:.5f}]")

# Also dump the 2 FixedJoints under cameraURflangeSupport (rel targets)
print("\n=== cameraURflangeSupport FixedJoints (body0/body1 rels) ===")
for jn in ["FixedJoint", "FixedJoint0"]:
    jp = f"/World/rgbd_camera/cameraURflangeSupport/{jn}"
    p = s.GetPrimAtPath(jp)
    if p and p.IsValid():
        r0 = p.GetRelationship("physics:body0")
        r1 = p.GetRelationship("physics:body1")
        t0 = r0.GetTargets() if r0 else []
        t1 = r1.GetTargets() if r1 else []
        # local pos
        lp = p.GetAttribute("physics:localPos0")
        lp1 = p.GetAttribute("physics:localPos1")
        print(f"  {jn}: body0={t0} body1={t1}")
        print(f"      localPos0={lp.Get() if lp else None} localPos1={lp1.Get() if lp1 else None}")

app.close()
