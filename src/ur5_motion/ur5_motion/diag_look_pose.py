#!/usr/bin/env python3
"""验证 look pose：移到 cola 上方俯视，抓一帧 color+mask，确认相机看见 cola 且 mask 合理。
用 robot conda python 跑（cv2 兼容）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ~/.conda/envs/robot/bin/python src/ur5_motion/ur5_motion/diag_look_pose.py
"""
import time, numpy as np, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from ur5_motion import UR5MoveIt

GRASP_QUAT = [0.0, 1.0, 0.0, 0.0]   # tool0-z 朝下 = 相机朝下
LOOK_POS = [0.45, 0.05, 0.10]        # cola(0.45,0.05,-0.403) 上方，相机距 cola ~0.47m

class Grab(Node):
    def __init__(self):
        super().__init__("diag_look_grab")
        self.color = None; self.mask = None
        self.create_subscription(Image, "/camera/color/image_raw", self._c, 10)
        self.create_subscription(Image, "/segmentation", self._m, 10)
    def _c(self, msg):
        if self.color is None:
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8": a = a[:, :, ::-1]
            self.color = a
    def _m(self, msg):
        if self.mask is None:
            self.mask = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)

def main():
    rclpy.init()
    arm = UR5MoveIt()
    log = arm.node.get_logger()
    grab = Grab()

    log.info(f"移到 look pose tool0={LOOK_POS} GRASP_QUAT（相机俯视 cola）...")
    arm.move_to_pose(LOOK_POS, GRASP_QUAT)
    log.info("到位，等 3s 让相机/SAM 更新...")
    time.sleep(3.0)

    t0 = time.time()
    while (grab.color is None or grab.mask is None) and time.time() - t0 < 10:
        rclpy.spin_once(grab, timeout_sec=0.2)

    if grab.color is None:
        print("NO COLOR"); return
    if grab.mask is None:
        print("NO MASK（SAM 没发？）"); return

    img = grab.color; mask = grab.mask
    h, w = mask.shape
    fg = mask > 127
    cov = 100 * fg.sum() / (h * w)
    n, lbl, stats, cent = cv2.connectedComponentsWithStats(fg.astype(np.uint8), 8)
    comps = sorted([stats[i] for i in range(1, n)], key=lambda s: -s[4])[:3]
    print(f"\n=== look pose 图像分析 ===")
    print(f"mask 覆盖: {cov:.1f}%  连通分量数: {n-1}")
    for i, s in enumerate(comps):
        x, y, ww, hh, area = s
        print(f"  最大分量{i+1}: area={area} ({100*area/(h*w):.1f}%) bbox=({x},{y},{ww}x{hh}) cent=({cent[i+1][0]:.0f},{cent[i+1][1]:.0f})")

    cv2.imwrite("/tmp/look_color.png", img)
    cv2.imwrite("/tmp/look_mask.png", mask)
    ov = img.copy(); ov[fg] = (0, 0, 255)
    cv2.imwrite("/tmp/look_overlay.png", ov)
    print("已存 /tmp/look_color.png /tmp/look_mask.png /tmp/look_overlay.png")
    print(f"\n判断: {'✅ mask 像物体(<30% 且单连通)' if cov < 30 and n-1 <= 5 else '❌ mask 仍太大/太碎（cola 可能仍没看到，或 SAM prompt 没对准）'}")

    grab.destroy_node(); arm.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
