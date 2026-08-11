#!/usr/bin/env python3
"""测 finger 张开(open)与合拢(close)的间距，判断能不能跨过瓶径(6cm)。

如果 open 间距 > 瓶径：顶部接近时 finger 在瓶子两侧（包瓶）→ 合理侧面夹。
如果 open 间距 < 瓶径：finger 从顶上下来直接戳进瓶子 → 夹顶/撞（你看到的现象）。
"""
import math
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from pxr import UsdGeom  # noqa: E402

omni.usd.get_context().open_stage(os.path.expanduser("~/Desktop/scene.usd"))
app.update()
app_utils.play()
app.update()

art = Articulation("/World/ur5/root_joint")
for _ in range(30):
    if art.is_physics_tensor_entity_valid():
        break
    app.update()

stage = omni.usd.get_context().get_stage()


def fpos(name):
    for p in stage.Traverse():
        if p.GetName() == name:
            return UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0).ExtractTranslation()
    return None


def report(tag):
    r = fpos("gripper_r_link_finger")
    l = fpos("gripper_l_link_finger")
    if r and l:
        d = math.dist(r, l) * 1000
        print(f"RESULT [{tag}] r={[round(v,4) for v in r]} l={[round(v,4) for v in l]} 间距={d:.1f}mm")
        return d
    print(f"RESULT [{tag}] 找不到 finger")
    return None


report("current")
names = list(art.dof_names)
print("RESULT dof_names:", names)
if "gripper_r_joint1" in names:
    idx = int(art.get_dof_indices("gripper_r_joint1").numpy()[0])
    pos = art.get_dof_positions().numpy()[0].copy()
    # open = π/3
    pos[idx] = math.pi / 3
    art.set_dof_positions(pos)
    for _ in range(8):
        app.update()
    report("OPEN(π/3)")
    # close = 0
    pos2 = art.get_dof_positions().numpy()[0].copy()
    pos2[idx] = 0.0
    art.set_dof_positions(pos2)
    for _ in range(8):
        app.update()
    report("CLOSE(0)")
print("RESULT 瓶径=60mm。open>60 = 能包瓶侧面；open<60 = 戳进瓶子（夹顶/撞）")
app.close()
