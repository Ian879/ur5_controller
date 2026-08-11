#!/usr/bin/env python3
"""诊断：tool0→camera_color_optical_frame 静态 mount 变换，推 look pose。
得 tool0→camera 后：camera 光轴(z)在 tool0 系的方向 → 求让 camera 朝 -base_z(下) 的 tool0 朝向。
"""
import numpy as np, rclpy, math
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

rclpy.init()
n = Node("diag_mount")
buf = Buffer(); TransformListener(buf, n)

def get(f, t):
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.1)
        try: return buf.lookup_transform(f, t, rclpy.time.Time())
        except Exception: pass
    return None

def rotm(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    xx,yy,zz=x*x,y*y,z*z; xy,xz,yz=x*y,x*z,y*z; wx,wy,wz=w*x,w*y,w*z
    return np.array([[1-2*(yy+zz),2*(xy-wz),2*(xz+wy)],
                     [2*(xy+wz),1-2*(xx+zz),2*(yz-wx)],
                     [2*(xz-wy),2*(yz+wx),1-2*(xx+yy)]])

# tool0→camera 静态 mount
mt = get("tool0", "camera_color_optical_frame")
if mt is None:
    print("拿不到 tool0->camera_color_optical_frame"); raise SystemExit
mt_q = mt.transform.rotation
R_m = rotm(mt_q)
cam_fwd_in_tool0 = R_m[:, 2]   # 相机光轴在 tool0 系
print(f"tool0->camera mount: pos=({mt.transform.translation.x:.3f},{mt.transform.translation.y:.3f},{mt.transform.translation.z:.3f})")
print(f"相机光轴(z)在 tool0 系 = {cam_fwd_in_tool0.round(3)}")
print(f"  → 相机大致沿 tool0 的 {'+x' if cam_fwd_in_tool0[0]>0.5 else '+y' if cam_fwd_in_tool0[1]>0.5 else '+z' if cam_fwd_in_tool0[2]>0.5 else '混合'} 轴看")

# 我想让相机光轴在 base 系朝下 = (0,0,-1)。即 R_tool0 @ cam_fwd_in_tool0 = (0,0,-1)
# 求一个 tool0 朝向：让 cam_fwd_in_tool0（tool0系） 映射到 base系 (0,0,-1)
target_axis_base = np.array([0.0, 0.0, -1.0])  # 朝下
src = cam_fwd_in_tool0 / np.linalg.norm(cam_fwd_in_tool0)
# 最小旋转把 src(在tool0系表达的相机轴) 当成 tool0 的某个轴... 这里换个思路：
# tool0 朝向 R_t 使得 R_t @ src = target_axis_base。用 axis-angle 求把 src->target 的旋转。
def quat_from_two_vectors(a, b):
    a=a/np.linalg.norm(a); b=b/np.linalg.norm(b)
    d=np.dot(a,b)
    if d>0.99999: return np.array([0,0,0,1.0])  # xyzw
    if d<-0.99999:
        # 任意垂直轴 π
        ax=np.array([1,0,0]) if abs(a[0])<0.9 else np.array([0,1,0])
        k=np.cross(a,ax); k/=np.linalg.norm(k)
        return np.array([k[0],k[1],k[2],0.0])
    axis=np.cross(a,b); axis/=np.linalg.norm(axis)
    ang=math.acos(d); s=math.sin(ang/2)
    return np.array([axis[0]*s,axis[1]*s,axis[2]*s,math.cos(ang/2)])

# 但 tool0 朝向还要考虑相机光轴是 tool0系里固定的 src；R_t @ src 应= base 的下方向。
# 注意：src 是相机光轴在 tool0 系的坐标。R_t（base<-tool0）@ src = 相机光轴在 base 系。
q_t = quat_from_two_vectors(src, target_axis_base)
print(f"\n→ 让相机垂直朝下，tool0 需要的朝向 quat(xyzw)≈[{q_t[0]:.3f},{q_t[1]:.3f},{q_t[2]:.3f},{q_t[3]:.3f}]")
print(f"  (对比 GRASP_QUAT=[0,1,0,0]：tool0-z 朝下)")

# 也看 base->tool0 home 朝向，确认 home 时 tool0 朝哪
bt = get("base_link", "tool0")
if bt:
    print(f"\nhome: base->tool0 pos=({bt.transform.translation.x:.3f},{bt.transform.translation.y:.3f},{bt.transform.translation.z:.3f}) quat=[{bt.transform.rotation.x:.3f},{bt.transform.rotation.y:.3f},{bt.transform.rotation.z:.3f},{bt.transform.rotation.w:.3f}]")
    R_h = rotm(bt.transform.rotation)
    print(f"home tool0 各轴在 base: x={R_h[:,0].round(2)} y={R_h[:,1].round(2)} z={R_h[:,2].round(2)}")

n.destroy_node(); rclpy.shutdown()
