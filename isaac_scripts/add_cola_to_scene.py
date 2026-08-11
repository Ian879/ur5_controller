#!/usr/bin/env python3
"""把 cola 罐加到 ~/Desktop/scene.usd 的桌上（替换旧物体），带完整材质+物理属性。

直接引用 GenieSimAssets 原始 Aligned.usd（含材质+贴图），不走 assets/meshes 的拷贝。

跑：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh ~/projects/shengzhi/ur5_controller/isaac_scripts/add_cola_to_scene.py
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

SCENE = os.path.expanduser("~/Desktop/scene.usd")
# 直接用原始资产（含材质 + textures/T_body_Alb.jpg）
COLA_USD = "/home/gds/GenieSimAssets/objects/genie/cola/Aligned.usd"

stage = Usd.Stage.Open(SCENE)

# 桌面位置（桌面 ~0.75；cola 高 0.115m，中心 = 0.75 + 0.058 = 0.808）
COLA_POS = Gf.Vec3d(-0.1, 0.05, 0.808)

# 清掉旧物体（bottle 或 cola）
for old in ("/World/bottle", "/World/cola"):
    if stage.GetPrimAtPath(old).IsValid():
        stage.RemovePrim(old)
        print(f"RESULT 删旧 {old}")

# 加 cola（引用原始 Aligned.usd，含材质+贴图）
cola_path = "/World/cola"
cola = stage.DefinePrim(cola_path, "Xform")
cola.GetReferences().AddReference(COLA_USD)

# 设位置（Aligned.usd 自带 xformOp:translate/orient/scale，覆盖 translate）
trans_attr = cola.GetAttribute("xformOp:translate")
if trans_attr and trans_attr.IsValid():
    trans_attr.Set(COLA_POS)
else:
    UsdGeom.Xformable(cola).AddTranslateOp().Set(COLA_POS)
print(f"RESULT cola 位置: {COLA_POS}")

# 确保可见
UsdGeom.Imageable(cola).MakeVisible()

# 加 Collision + RigidBody（可被抓取）
for p in Usd.PrimRange(cola):
    if p.IsA(UsdGeom.Mesh) or p.GetTypeName() == "Mesh":
        UsdPhysics.CollisionAPI.Apply(p)
        print(f"RESULT + collision: {p.GetPath()}")

UsdPhysics.RigidBodyAPI.Apply(cola)
mass_api = UsdPhysics.MassAPI.Apply(cola)
mass_api.CreateMassAttr().Set(0.25)

stage.Save()
print(f"RESULT DONE: cola 加到 {cola_path} @ {COLA_POS}（含材质），存盘 {SCENE}")
app.close()
