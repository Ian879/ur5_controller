#!/usr/bin/env python3
"""诊断 elevate_base 等 Cylinder collider 的 AABB（GetRange vs ComputeAlignedRange）
+ 挪 /World/bottle 到 base 系 x≈0.45（y/z 不变）。

跑：
  source ~/isaacsim/setup_ros_env.sh
  ~/isaacsim/python.sh isaac_scripts/diag_move_bottle.py 2>&1 | grep ">>>"
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf  # noqa: E402

SCENE = os.path.expanduser("~/Desktop/scene.usd")
stage = Usd.Stage.Open(SCENE)
tc = Usd.TimeCode.Default()


def P(*a):
    print(">>>", *a)


# base_link world transform（算 base 系 + bottle target）
P("articulation roots:", [str(p.GetPath()) for p in stage.Traverse()
                          if p.HasAPI(UsdPhysics.ArticulationRootAPI)])
base_prim = None
for root in [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]:
    cand = "/".join(root.split("/")[:-1]) + "/base_link"
    if stage.GetPrimAtPath(cand).IsValid():
        base_prim = cand
        break
P("base_link prim:", base_prim)
base_w = UsdGeom.Xformable(stage.GetPrimAtPath(base_prim)).ComputeLocalToWorldTransform(tc)
base_inv = base_w.GetInverse()
P("base_link world trans:", base_w.ExtractTranslation())

# 1. 诊断所有 Cylinder/Sphere/Cone/Capsule collider 的 AABB 两种方法
P("=== Cylinder/Sphere AABB 诊断（GetRange=局部 / ComputeAlignedRange=世界）===")
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    tn = prim.GetTypeName()
    if tn not in ("Cylinder", "Sphere", "Cone", "Capsule"):
        continue
    path = str(prim.GetPath())
    wb = UsdGeom.Imageable(prim).ComputeWorldBound(tc, "default")
    gr = wb.GetRange()
    ar = wb.ComputeAlignedRange()
    if gr.IsEmpty() and ar.IsEmpty():
        continue
    wt = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tc).ExtractTranslation()
    gr_c = (gr.GetMin() + gr.GetMax()) / 2
    ar_c = (ar.GetMin() + ar.GetMax()) / 2
    gr_sz = gr.GetMax() - gr.GetMin()
    ar_sz = ar.GetMax() - ar.GetMin()
    P(f"[{tn}] {path}")
    P(f"   prim worldTrans = {wt}")
    P(f"   GetRange        center={gr_c} size={gr_sz}")
    P(f"   ComputeAligned  center={ar_c} size={ar_sz}  (centerInBase={base_inv.Transform(ar_c)})")

# 2. 挪 /World/bottle 到 base x=0.45（y/z 不变）
bottle = stage.GetPrimAtPath("/World/bottle")
P("=== bottle ===")
if not bottle.IsValid():
    P("/World/bottle 不存在！确认物体 prim 路径（demo --object_prim）")
else:
    bw = UsdGeom.Xformable(bottle).ComputeLocalToWorldTransform(tc)
    b_in_base = (base_inv * bw).ExtractTranslation()
    P(f"bottle in base (before) = {b_in_base}")
    target_base = Gf.Vec3d(0.45, b_in_base[1], b_in_base[2])
    target_world = base_w.Transform(target_base)
    P(f"target world = {target_world}  (base x 0.45, y/z 不变)")
    xf = UsdGeom.Xformable(bottle)
    trans_op = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            trans_op = op
            break
    if trans_op is not None:
        cur = trans_op.Get(tc)
        trans_op.Set(target_world)  # bottle 挂在 /World 下，op 值即 world translation
        P(f"translate op {cur} -> {target_world}")
    else:
        op = xf.AddTranslateOp()
        op.Set(target_world)
        P(f"+translate op = {target_world}")
    bw2 = UsdGeom.Xformable(bottle).ComputeLocalToWorldTransform(tc)
    P(f"bottle in base (after) = {(base_inv * bw2).ExtractTranslation()}")
    stage.Save()
    P(f"存盘 {SCENE}")

app.close()
