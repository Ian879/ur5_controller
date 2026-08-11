#!/usr/bin/env python3
"""诊断：HOME 位相机朝向 vs cola 方向，确认 cola 是否在视野外；并建议 look pose。
cola-in-base 真值 = base世系(-0.55,0,1.211) + cola世系(-0.1,0.05,0.808) 推得 (0.45,0.05,-0.403)。
"""
import numpy as np, rclpy, math
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

COLA_IN_BASE = np.array([0.45, 0.05, -0.403])  # 推得真值（见上）
FOV_DEG = 65  # Orbbec/Gemini2 水平 FOV 估计（半角 ~32°）

rclpy.init()
n = Node("diag_cam_fov")
buf = Buffer()
TransformListener(buf, n)

cam = None
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        t = buf.lookup_transform("base_link", "camera_color_optical_frame", rclpy.time.Time())
        cam = t; break
    except Exception:
        pass

if cam is None:
    print("拿不到 base_link->camera_color_optical_frame（TF 时间戳0问题？）"); raise SystemExit

tr = cam.transform.translation
q = cam.transform.translation
qx, qy, qz, qw = cam.transform.rotation.x, cam.transform.rotation.y, cam.transform.rotation.z, cam.transform.rotation.w
cam_pos = np.array([tr.x, tr.y, tr.z])
print(f"相机(base) pos = {cam_pos.round(3)}  quat(xyzw)=[{qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}]")

# 旋转矩阵
xx, yy, zz = qx*qx, qy*qy, qz*qz
xy, xz, yz = qx*qy, qx*qz, qy*qz
wx, wy, wz = qw*qx, qw*qy, qw*qz
R = np.array([
    [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
    [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
    [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)],
])
# 光学系：z=前, x=右, y=下
fwd  = R[:, 2]   # 光轴 forward
right = R[:, 0]
down  = R[:, 1]

dir_to_cola = COLA_IN_BASE - cam_pos
dist = np.linalg.norm(dir_to_cola)
dirn = dir_to_cola / dist
print(f"\ncola(base) = {COLA_IN_BASE.round(3)}")
print(f"相机→cola 向量 = {dir_to_cola.round(3)}  距离 {dist:.3f} m")

cos_a = float(np.dot(fwd, dirn))
cos_a = max(-1, min(1, cos_a))
ang = math.degrees(math.acos(cos_a))
print(f"\n相机光轴 forward(base) = {fwd.round(3)}")
print(f"光轴 vs 相机→cola 夹角 = {ang:.1f}°  (FOV 半角 ~{FOV_DEG/2:.0f}°)")
print(f"  → {'在视野内' if ang < FOV_DEG/2 else '❌ 在视野外（看不到 cola）'}")

# 投影到相机像平面（光轴=+z, 右=+x, 下=+y），单位：相对焦距
cam_fwd_to_cola = dir_to_cola / np.dot(dir_to_cola, fwd)  # 沿光轴归一
u = np.dot(cam_fwd_to_cola, right)  # 像平面 x（右正）
v = np.dot(cam_fwd_to_cola, down)   # 像平面 y（下正）
print(f"  若能看见，cola 在像平面 (u,v)=({u:.2f},{v:.2f})  (单位=相对焦距；|·|>~0.6 即出水平FOV)")

# 建议 look pose：相机放在 cola 上方斜看，距 cola ~0.30m
print("\n=== 建议 look pose（相机看向 cola）===")
for name, offset_base in [
    ("正上方斜前", np.array([0.0, 0.0, 0.30])),     # cola 正上 0.30m
    ("前方斜上",   np.array([0.28, 0.10, 0.12])),   # +x +y 略上
    ("侧前方",     np.array([0.22, 0.20, 0.10])),
]:
    cam_target = COLA_IN_BASE + offset_base
    # 光轴 = cam_target - cam_pos 的反方向... 这里 cam_pos=cam_target, 光轴指向 cola
    look_dir = COLA_IN_BASE - cam_target
    ld = look_dir / np.linalg.norm(look_dir)
    print(f"  [{name}] 相机pos(base)={cam_target.round(3)}  光轴指向={ld.round(3)}  距cola={np.linalg.norm(look_dir):.2f}m")

n.destroy_node(); rclpy.shutdown()
