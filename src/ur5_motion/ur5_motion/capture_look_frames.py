#!/usr/bin/env python3
"""采集 look-pose 的 RGBD+mask+K+GT 帧，存盘供离线测 FP 精度（脱离 demo/sim/TF）。

移到观测位 → 抓 color/depth/mask/K → 用深度算 GT（物体在相机系的真实位置）→ 存 npz。
GT 用深度（不是授权位姿）：mask 质心 + 中位深度 → 相机系 3D，即物体真实中心。

跑（系统 python，UR5MoveIt + rclpy）：
  source tools/source_env.sh && python3 -m ur5_motion.capture_look_frames [--n 3]
"""
import os, argparse, numpy as np, rclpy, time, tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from ur5_motion import UR5MoveIt

LOOK_POS = [0.45, 0.05, 0.10]; LOOK_QUAT = [0.0, 1.0, 0.0, 0.0]
OUT = os.path.expanduser("~/projects/jiayin/fp_frames")


class Grab(Node):
    def __init__(self):
        super().__init__("cap")
        self.c = self.d = self.m = self.K = None
        self.create_subscription(Image, "/camera/color/image_raw", self._c, 10)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._d, 10)
        self.create_subscription(Image, "/segmentation", self._m, 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._k, 10)

    def _c(self, msg):
        if self.c is None:
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            self.c = a.copy() if msg.encoding == "rgb8" else a[:, :, ::-1].copy()
    def _d(self, msg):
        if self.d is None:
            self.d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    def _m(self, msg):
        if self.m is None:
            self.m = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width).copy()
    def _k(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, float).reshape(3, 3).copy()


def depth2xyz_at(depth, K, mask):
    """mask 区域的深度 → 相机系 xyz；返回质心 + 中位深度（GT 物体中心，相机系）。"""
    vs, us = np.where(mask > 127)
    if len(us) == 0:
        return None
    fx, fy = K[0, 0], K[1, 1]; cx, cy = K[0, 2], K[1, 2]
    d = depth[vs, us]; d = d[np.isfinite(d) & (d > 0.05)]
    if len(d) == 0:
        return None
    zc = float(np.median(d))
    uc = float((us.min() + us.max()) / 2.0); vc = float((vs.min() + vs.max()) / 2.0)
    xc = (uc - cx) * zc / fx; yc = (vc - cy) * zc / fy
    return np.array([xc, yc, zc])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--n", type=int, default=3)
    args = p.parse_args()
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    arm = UR5MoveIt(use_sim_time=True)   # isaac 后端：match move_group sim 时钟（wall 会 error_code=None）
    log = arm.node.get_logger()
    grab = Grab()

    log.info("复位到 home ...")
    try:
        arm.go_home()
    except Exception as e:
        log.warn(f"go_home 失败（继续）: {e}")
    log.info(f"移到 look pose {LOOK_POS} ...")
    arm.move_to_pose(LOOK_POS, LOOK_QUAT)
    time.sleep(3.0)

    for i in range(args.n):
        grab.c = grab.d = grab.m = grab.K = None
        t0 = time.time()
        while (grab.c is None or grab.d is None or grab.m is None or grab.K is None) and time.time() - t0 < 8:
            rclpy.spin_once(grab, timeout_sec=0.1)
        if grab.c is None or grab.d is None or grab.m is None or grab.K is None:
            log.error(f"帧 {i} 抓不全"); continue
        gt = depth2xyz_at(grab.d, grab.K, grab.m)
        path = os.path.join(OUT, f"frame_{i:03d}.npz")
        np.savez_compressed(path, color=grab.c, depth=grab.d, mask=grab.m, K=grab.K, gt_cam_xyz=gt)
        log.info(f"帧 {i} 存 {path}  GT_cam(trans)={np.round(gt,3) if gt is not None else None}")
        time.sleep(0.5)
    log.info(f" DONE: {args.n} 帧在 {OUT}/")
    grab.destroy_node(); arm.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
