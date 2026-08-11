#!/usr/bin/env python3
"""从 USD 文件导出 mesh 到 OBJ（供 FoundationPose register 用）。

跑：
  cd ~/isaacsim && source setup_ros_env.sh
  ./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/export_mesh_to_obj.py
"""
import os
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, Vt

USD_PATH = "/home/gds/projects/shengzhi/ur5_controller/assets/meshes/cola.usd"
OBJ_PATH = "/home/gds/projects/shengzhi/ur5_controller/assets/meshes/cola.obj"

stage = Usd.Stage.Open(USD_PATH)

# 遍历找所有 Mesh prim，合并 vertices + faces
all_verts = []
all_faces = []
vert_offset = 0

for prim in stage.Traverse():
    if prim.GetTypeName() == "Mesh":
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        if not points or not indices or not counts:
            continue
        all_verts.extend(points)
        idx = 0
        for count in counts:
            face = [indices[idx + i] + vert_offset for i in range(count)]
            all_faces.append(face)
            idx += count
        vert_offset += len(points)
        print(f"RESULT + {prim.GetPath()}: {len(points)} verts, {sum(counts) if counts else 0} indices")

if not all_verts:
    print("RESULT ❌ 没找到 Mesh prim")
    app.close()
    raise SystemExit(1)

verts = np.array(all_verts)
print(f"RESULT 总计 {len(verts)} verts, {len(all_faces)} faces")
print(f"RESULT bbox min={verts.min(axis=0).tolist()} max={verts.max(axis=0).tolist()}")
print(f"RESULT 尺寸 (x,y,z) = {(verts.max(axis=0) - verts.min(axis=0)).tolist()}")

# 居中原点（FoundationPose 要求 mesh 原点在 bbox 中心）
center = (verts.max(axis=0) + verts.min(axis=0)) / 2.0
verts_centered = verts - center

# 写 OBJ
with open(OBJ_PATH, "w") as f:
    for v in verts_centered:
        f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
    for face in all_faces:
        # OBJ face index 从 1 开始
        f.write("f " + " ".join(str(i + 1) for i in face) + "\n")

print(f"RESULT DONE: 导出 {OBJ_PATH}（{len(verts_centered)} verts, {len(all_faces)} faces, 居中）")
print(f"RESULT 居中后 bbox min={verts_centered.min(axis=0).tolist()} max={verts_centered.max(axis=0).tolist()}")
app.close()
