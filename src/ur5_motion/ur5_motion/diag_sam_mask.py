#!/usr/bin/env python3
"""诊断：抓一帧 /camera/color/image_raw + /segmentation，分析 mask 覆盖 + 存 PNG。
用法: source ROS + install/setup.bash; python3 diag_sam_mask.py
"""
import sys, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Grab(Node):
    def __init__(self):
        super().__init__("diag_sam_mask")
        self.color = None
        self.mask = None
        self.create_subscription(Image, "/camera/color/image_raw", self._c, 10)
        self.create_subscription(Image, "/segmentation", self._m, 10)
    def _c(self, msg):
        if self.color is None:
            a = np.frombuffer(msg.data, np.uint8)
            self.color = (a.reshape(msg.height, msg.width, 3), msg.encoding)
    def _m(self, msg):
        if self.mask is None:
            a = np.frombuffer(msg.data, np.uint8)
            self.mask = a.reshape(msg.height, msg.width)

rclpy.init()
n = Grab()
import time
t0 = time.time()
while (n.color is None or n.mask is None) and time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.2)

if n.color is None:
    print("NO COLOR frame received (检查 /camera/color/image_raw)"); sys.exit(1)
if n.mask is None:
    print("NO MASK received (检查 /segmentation，SAM 是否在跑)"); sys.exit(1)

img, enc = n.color
mask = n.mask
h, w = mask.shape
nonzero = int((mask > 127).sum())
total = h * w
cy, cx = h // 2, w // 2
center_val = int(mask[cy, cx])
# 找 mask 质心
ys, xs = np.where(mask > 127)
if len(xs):
    cent_x, cent_y = int(xs.mean()), int(ys.mean())
    bbox = (int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max()))
else:
    cent_x = cent_y = -1; bbox = None

print(f"image: {img.shape[1]}x{img.shape[0]} enc={enc}")
print(f"mask:  {w}x{h}")
print(f"mask 覆盖像素: {nonzero} / {total} = {100*nonzero/total:.1f}%")
print(f"图像中心 ({cx},{cy}) 的 mask 值: {center_val} ({'在 mask 内' if center_val>127 else '不在 mask'})")
print(f"mask 质心: ({cent_x},{cent_y})  距图像中心: ({cent_x-cx},{cent_y-cy})")
print(f"mask bbox (xmin,xmax,ymin,ymax): {bbox}")

import cv2
# 存图（color 转 BGR 存 jpg）
if enc == "rgb8":
    img = img[:, :, ::-1]
cv2.imwrite("/tmp/diag_color.png", img)
cv2.imwrite("/tmp/diag_mask.png", mask)
ov = img.copy()
ov[mask > 127] = (0, 0, 255)
cv2.imwrite("/tmp/diag_overlay.png", ov)
print("已存: /tmp/diag_color.png /tmp/diag_mask.png /tmp/diag_overlay.png")
n.destroy_node(); rclpy.shutdown()
