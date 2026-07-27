"""构建最小 Isaac 场景：UR5 + 桌面 + 腕部相机，存成 scenes/ur5_table.usd。

用 ~/isaacsim_assets 的现成 ur5.usd（关节名已确认与 ur_description 一致），
加一个桌面 box + 一个挂在 wrist_3_link/flange 下的 RTX 相机 prim。

T7 的桥脚本会加载这个 .usd、注入 ROS2 bridge（关节/clock/相机发布）。

运行：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/build_scene.py

注：app.close() 在 Isaac 6.0.1 上有个已知 TaskGroup 断言崩溃；本脚本先 stage.Save()
再 close，崩溃不影响已存的 .usd（退出码非 0 可忽略）。
"""

import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf
import isaacsim.core.experimental.utils.stage as stage_utils

UR5_USD = "/home/gds/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5/ur5.usd"
REPO = "/home/gds/projects/shengzhi/ur5_controller"
OUT = os.path.join(REPO, "scenes", "ur5_table.usd")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
if os.path.exists(OUT):
    os.remove(OUT)

stage = Usd.Stage.CreateNew(OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# ---- 机器人 UR5（reference 现成 USD）----
stage_utils.add_reference_to_stage(UR5_USD, "/ur5")

# ---- 桌面（一个扁 box，放机器人前方）----
# 0.8m x 0.8m x 0.03m，顶面约 z=0.8
table = UsdGeom.Cube.Define(stage, "/table")
table.CreateSizeAttr(1.0)
table.AddScaleOp().Set(Gf.Vec3f(0.8, 0.8, 0.03))
table.AddTranslateOp().Set(Gf.Vec3f(0.6, 0.0, 0.785))
# 给个颜色，便于辨认
table.GetDisplayColorAttr().Set([(0.5, 0.35, 0.2)])

# ---- 腕部 RTX 相机（挂 wrist_3_link/flange 下）----
cam_path = "/ur5/wrist_3_link/flange/wrist_camera"
cam = UsdGeom.Camera.Define(stage, cam_path)
xf = UsdGeom.Xformable(cam)
# 沿工具方向偏移一点，朝下看（绕 X 转 180°，使相机 -z 朝向工具前方/桌面）
xf.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.04))
xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
cam.GetFocalLengthAttr().Set(24.0)

# ---- 地面（大扁 box，仅视觉参照）----
floor = UsdGeom.Cube.Define(stage, "/floor")
floor.CreateSizeAttr(1.0)
floor.AddScaleOp().Set(Gf.Vec3f(4.0, 4.0, 0.01))
floor.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, -0.005))
floor.GetDisplayColorAttr().Set([(0.7, 0.7, 0.7)])

# ---- 默认 prim = 机器人，存盘 ----
stage.SetDefaultPrim(stage.GetPrimAtPath("/ur5"))
stage.Save()
print("SAVED:", OUT)

# ---- 校验：关节 + 相机 + 桌面 ----
revolute = [
    str(p.GetPath())
    for p in stage.Traverse()
    if p.IsA(UsdPhysics.Joint) and p.GetTypeName() == "PhysicsRevoluteJoint"
]
print("Revolute joints (%d):" % len(revolute), [p.split("/")[-1] for p in revolute])
print("Camera prim exists:", stage.GetPrimAtPath(cam_path).IsValid(), cam_path)
print("Table prim exists:", stage.GetPrimAtPath("/table").IsValid())

try:
    app.close()
except Exception as e:
    print("app.close warning (ignored, scene already saved):", e)
