#!/usr/bin/env python3
"""[已弃用] 给 r finger 建单个 SurfaceGripper prim（吸附式物理抓取）。

⚠️ 已弃用：SurfaceGripper 是「吸附 attach」（类吸盘/磁力，刚性焊到 finger），不是真夹爪的物理
夹持（手指对夹+摩擦）；且 isaac 特有、real 后端没有 → isaac/real 抓取机制不一致。已改回真夹爪
物理夹持（见 demo_grasp）。本脚本保留仅作存档，不要再跑——scene 里若已有 SurfaceGripper prim
建议删掉以免干扰真夹爪物理夹持。

（以下为原说明，存档）给 r finger 建单个 SurfaceGripper prim（物理抓取），设吸附属性，存盘 ~/Desktop/scene.usd。

单 finger attach（非双 finger）：双 gripper 同时刚性焊同一物体 → 过约束 → solver 内力互斗
→ 机械臂抽搐（isaac 官方测试从不双 gripper 接同一 body）。故只 r finger 建 SurfaceGripper
attach 物体，l finger 仅物理 contact 夹（平衡）。

force_limit 是脱附阈值（非吸力）；25/40N 是 0.5kg 物体合理量级（mg 5N + retreat 惯性，
留余量防脱，不用 500000 掩盖问题）。max_grip_distance=0.04：r finger 距瓶侧面常 >1cm，0.01
太严扫不到物体（attach 命令发了但没真吸附→瓶子夹不起→retreat 推倒→第二次撞移位瓶）；0.04 容忍对位偏差。

跑：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/add_surface_gripper.py
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics  # noqa: E402
from isaacsim.robot.surface_gripper import create_surface_gripper  # noqa: E402
from usd.schema.isaac import robot_schema  # noqa: E402

SCENE = os.path.expanduser("~/Desktop/scene.usd")
stage = Usd.Stage.Open(SCENE)

# 找 r finger（RigidBody，SurfaceGripper 合法 parent）
FINGER_LEAF = "gripper_r_link_finger"
finger = None
for p in stage.Traverse():
    if p.GetName() == FINGER_LEAF and p.HasAPI(UsdPhysics.RigidBodyAPI):
        finger = str(p.GetPath())
        break
print(f"RESULT r finger: {finger}", flush=True)
if finger is None:
    print(f"RESULT ⚠️ 没找到 {FINGER_LEAF}（RigidBody），放弃", flush=True)
    app.close()
    raise SystemExit(1)

# 清掉所有 finger（r+l）下残留的旧 SurfaceGripper（之前双 gripper 建过 l 的）
for p in list(stage.Traverse()):
    if p.GetName() == "SurfaceGripper" and "link_finger" in str(p.GetPath()):
        stage.RemovePrim(p.GetPath())
        print(f"RESULT   清残留 {p.GetPath()}", flush=True)

# 建 r finger 单 SurfaceGripper
sg = create_surface_gripper(stage, finger)
sg.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(0.04)
sg.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(25.0)
sg.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(40.0)
sg.GetAttribute(robot_schema.Attributes.RETRY_INTERVAL.name).Set(1.0)
print(f"RESULT   + {sg.GetPath()}", flush=True)

stage.Save()
print(f"RESULT DONE: 单 finger SurfaceGripper 建好，存盘 {SCENE}", flush=True)
print("RESULT 桥 --surface_gripper_regex: .*/gripper_r_link_finger/SurfaceGripper", flush=True)
app.close()
