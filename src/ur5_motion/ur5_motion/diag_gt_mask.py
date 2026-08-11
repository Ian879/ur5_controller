#!/usr/bin/env python3
"""一次性验证：抓一帧深度，算 GT mask，看是否是 cola 形状。用 robot conda python。"""
import time, numpy as np, rclpy, cv2, sys
from rclpy.node import Node
from sensor_msgs.msg import Image
sys.path.insert(0, "/home/gds/projects/shengzhi/ur5_controller/src/ur5_motion/ur5_motion")
from gt_mask_from_depth import depth_to_mask

class G(Node):
    def __init__(self):
        super().__init__("diag_gtmask")
        self.depth = None
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._d, 10)
    def _d(self, msg):
        if self.depth is None:
            self.depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()

rclpy.init(); n=G()
t0=time.time()
while n.depth is None and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.2)
if n.depth is None: print("NO DEPTH"); sys.exit(1)

d = n.depth
valid = d[np.isfinite(d)&(d>0.05)&(d<5.0)]
print(f"depth shape={d.shape} 有效像素={valid.size}/{d.size} ({100*valid.size/d.size:.0f}%)")
print(f"depth 统计: min={valid.min():.3f} median={np.median(valid):.3f} max={valid.max():.3f} m")

res = depth_to_mask(d, 0.05, 200)
if res is None: print("mask 为空（没检测到凸起物）"); sys.exit(0)
mask, table_d = res
h,w = mask.shape; ys,xs = np.where(mask>127); cov=100*len(xs)/(h*w)
print(f"\nGT mask: cov={cov:.1f}% bbox=({int(xs.min())},{int(xs.max())},{int(ys.min())},{int(ys.max())}) cent=({int(xs.mean())},{int(ys.mean())})")
print(f"判断: {'✅ 像 cola(小、居中)' if cov<8 and abs(int(xs.mean())-640)<120 else '❌ 太大/偏（cola 可能没在视野）'}")
cv2.imwrite("/tmp/gtmask.png", mask)
print("已存 /tmp/gtmask.png")
n.destroy_node();rclpy.shutdown()
