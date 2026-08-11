"""导出带纹理的 cola mesh 给 FoundationPose（pxr 手写 OBJ，正确处理 faceVarying UV）。

cola_centered.obj 无 UV 无纹理 → FP 配不上 → 位姿不稳。本脚本：开 cola USD → 合并所有
Mesh prim → 用 ComputeFlattened 取 faceVarying UV → 写 .obj(v/vt/f) + .mtl(map_Kd) + 贴图。

用法：~/isaacsim/python.sh isaac_scripts/export_textured_cola.py
"""
import os, shutil
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom
import numpy as np

USD = "/home/gds/GenieSimAssets/objects/genie/cola/Aligned.usd"
TEX_SRC = "/home/gds/GenieSimAssets/objects/genie/cola/textures/T_body_Alb.jpg"
OUT = "/home/gds/projects/jiayin/mesh"
OBJ = os.path.join(OUT, "cola_textured.obj")
MTL = os.path.join(OUT, "cola_textured.mtl")
TEX = os.path.join(OUT, "cola_textured.jpg")

stage = Usd.Stage.Open(USD)
all_v, all_vt, all_f = [], [], []
v_off = vt_off = 0
n_mesh = 0
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    g = UsdGeom.Mesh(prim)
    pts = g.GetPointsAttr().Get()
    fvi = g.GetFaceVertexIndicesAttr().Get()
    fvc = g.GetFaceVertexCountsAttr().Get()
    if not pts or not fvi or not fvc:
        continue
    pts = np.asarray(pts, dtype=float)
    # UV primvar（st）faceVarying → ComputeFlattened 展开成 per-face-vertex
    uv_pv = None
    for pv in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        n = str(pv.GetPrimvarName()).lower()
        if n in ("st", "uv", "st0", "texcoord0"):
            uv_pv = pv; break
    uvs = None
    if uv_pv is not None:
        uvs = np.asarray(uv_pv.ComputeFlattened(), dtype=float)  # len = sum(fvc)
    print(f"[Mesh] {prim.GetPath()} verts={len(pts)} faces={len(fvc)} UVs={len(uvs) if uvs is not None else 0}")
    n_mesh += 1
    # 累积顶点
    all_v.append(pts)
    if uvs is not None:
        all_vt.append(uvs)
    # 累积面（faceVarying：UV 按 face-vertex 顺序，索引 = 全局 face-vertex 计数）
    k = 0  # 本 mesh 的 face-vertex 计数
    for c in fvc:
        face = []
        for _ in range(c):
            gv = v_off + int(fvi[k])      # 全局顶点索引
            gvt = (vt_off + k) if uvs is not None else None  # 全局 UV 索引（face-vertex 顺序）
            face.append((gv, gvt))
            k += 1
        all_f.append(face)
    v_off += len(pts)
    if uvs is not None:
        vt_off += len(uvs)

if n_mesh == 0:
    print("❌ 没找到 Mesh"); app.close(); raise SystemExit(1)

V = np.concatenate(all_v)
V = V - V.mean(axis=0)  # 居中（FP 要求）
has_uv = len(all_vt) > 0
VT = np.concatenate(all_vt) if has_uv else None
print(f"\n合并 {n_mesh} mesh: verts={len(V)} faces={len(all_f)} UVs={len(VT) if has_uv else 0}")

# 写 OBJ
with open(OBJ, "w") as f:
    f.write("mtllib cola_textured.mtl\n")
    for p in V:
        f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
    if has_uv:
        for uv in VT:
            f.write(f"vt {uv[0]:.6f} {1.0 - uv[1]:.6f}\n")  # obj v 轴翻转
    f.write("usemtl cola\n")  # ⚠️ 必须在 faces 之前，否则 trimesh 按默认 material_0(2x2 贴图) 解析
    for face in all_f:
        if has_uv:
            f.write("f " + " ".join(f"{gv+1}/{gvt+1}" for gv, gvt in face) + "\n")
        else:
            f.write("f " + " ".join(f"{gv+1}" for gv, _ in face) + "\n")

# MTL + 贴图
with open(MTL, "w") as f:
    f.write("newmtl cola\nKd 0.8 0.8 0.8\n")
    if os.path.exists(TEX_SRC):
        shutil.copy(TEX_SRC, TEX)
        f.write(f"map_Kd cola_textured.jpg\n")
        print(f"复制贴图 -> cola_textured.jpg")
    else:
        print(f"⚠ 贴图 {TEX_SRC} 不存在")

print(f"\n✓ 导出 {OBJ} (+ {MTL}" + (", cola_textured.jpg" if os.path.exists(TEX_SRC) else "") + ")")
print("下一步：configs/default.yaml fp.mesh -> mesh/cola_textured.obj，重启 fp_server")
app.close()
