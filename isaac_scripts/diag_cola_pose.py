"""诊断：直接从 USD 读 cola + base_link 世界 pose，算 cola-in-base（独立于 TF）。
用法: /home/gds/isaacsim/python.sh isaac_scripts/diag_cola_pose.py [--scene PATH] [--cola PATH]
默认 scene=~/Desktop/scene.usd  cola=/World/cola
"""
import argparse
from pxr import Usd, UsdGeom, Gf

p = argparse.ArgumentParser()
p.add_argument("--scene", default="/home/gds/Desktop/scene.usd")
p.add_argument("--cola", default="/World/cola")
p.add_argument("--robot", default="/World/ur5")
a = p.parse_args()

stage = Usd.Stage.Open(a.scene)


def world_xform(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, f"prim {prim_path} NOT FOUND"
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return m, None


# base_link
bw, err = world_xform(a.robot + "/base_link")
if err:
    print(f"[base_link] {err}")
else:
    bt = bw.ExtractTranslation()
    print(f"[base_link] world trans = ({bt[0]:.4f}, {bt[1]:.4f}, {bt[2]:.4f})")

# cola
ow, err = world_xform(a.cola)
if err:
    print(f"[cola] {err}")
    # 试一下 cola 下的 mesh/collision 子 prim
    for child in stage.Traverse():
        if "cola" in str(child.GetPath()).lower():
            print(f"   found prim: {child.GetPath()}")
else:
    ot = ow.ExtractTranslation()
    print(f"[cola] world trans = ({ot[0]:.4f}, {ot[1]:.4f}, {ot[2]:.4f})")
    if bw is not None:
        ob = bw.GetInverse() * ow  # cola 在 base 系
        obt = ob.ExtractTranslation()
        print(f"[cola] IN BASE trans = ({obt[0]:.4f}, {obt[1]:.4f}, {obt[2]:.4f})")
        # 也打印旋转
        oq = ob.ExtractRotationQuat()
        oi = oq.GetImaginary()
        print(f"[cola] IN BASE quat(xyzw) = ({oi[0]:.4f}, {oi[1]:.4f}, {oi[2]:.4f}, {oq.GetReal():.4f})")
        # RPY
        import math
        # 旋转矩阵的欧拉角 (ZYX)
        r = Gf.Rotation(Gf.Quatd(oq.GetReal(), oi[0], oi[1], oi[2])).Decompose(
            Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()
        )
        print(f"[cola] IN BASE RPY(deg) = ({r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f})")

# 也列出所有含 cola 的 prim
print("--- all cola-related prims ---")
seen = set()
for prim in stage.Traverse():
    pth = str(prim.GetPath())
    if "cola" in pth.lower():
        key = "/".join(pth.split("/")[:5])
        if key not in seen:
            seen.add(key)
            print(f"  {pth}")
