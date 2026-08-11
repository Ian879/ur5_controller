#!/usr/bin/env python3
"""降 gripper_r_joint1 drive（柔顺 close，避夹硬物体崩断）。

根因：joint1 drive stiffness=100000（极硬），close 到 target=0 时硬推；夹到瓶子后 finger 被挡
不能转，但 mimic 强制 joint2/joint_finger 跟 joint1 → 约束冲突 → 应力集中在外侧 link1 → 崩断。
真夹爪应柔顺 close：夹到阻力就停（maxForce 限夹持力），不硬推到位。降 stiffness + maxForce。
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

stage = Usd.Stage.Open(os.path.expanduser("~/Desktop/scene.usd"))

J1 = "/World/gripper_abs_stl/Physics/gripper_r_joint1"  # 唯一驱动；l_joint1/joint2/finger 都 mimic 它
j = stage.GetPrimAtPath(J1)
assert j.IsValid(), f"{J1} 不存在"

# 柔顺 close：低 stiffness（不硬推）+ 够 maxForce（夹住 0.5kg 瓶不掉）。
# 旧 stiffness=100000/damping=1000/maxForce=50 → 硬推崩。
j.GetAttribute("drive:angular:physics:stiffness").Set(2000.0)   # 100000 → 2000（柔顺，夹到阻力让步）
j.GetAttribute("drive:angular:physics:damping").Set(200.0)      # 1000 → 200
j.GetAttribute("drive:angular:physics:maxForce").Set(15.0)      # 50 → 15（限夹持力，夹住即停不硬推）
print(f"RESULT {J1}: stiffness=2000 damping=200 maxForce=15 (旧 100000/1000/50)")

stage.Save()
print("RESULT DONE: joint1 drive 降为柔顺，存盘 ~/Desktop/scene.usd")
print("RESULT 重启桥后 close 夹到瓶子应停（保持夹持力），不再硬推崩断。")
app.close()
