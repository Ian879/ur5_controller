#!/usr/bin/env python3
"""GT mask（仿真，深度阈值）：cola 是桌上唯一凸起物 → 俯视时深度明显小于桌面。
订阅 /camera/depth/image_rect_raw → 阈值出凸起物 mask → 发 /segmentation（mono8）。

替代 SAM（SAM 单点 prompt 无法从桌面剥离小物体）。仿真深度干净，此法可靠。
⚠️ 不用 cv_bridge（NumPy 2.x 不兼容）；手动 imgmsg↔numpy。

跑（robot conda env）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ~/.conda/envs/robot/bin/python src/ur5_motion/ur5_motion/gt_mask_from_depth.py
  # 参数: --ros-args -p table_margin:=0.05 -p min_obj_area:=200
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def depth_to_mask(depth: np.ndarray, table_margin: float, min_area: int,
                  max_above_table: float = 0.15) -> np.ndarray:
    """深度图 → 凸起物二值 mask。
    桌面深度 ≈ 有效像素中位数；mask = depth 在带 [table_d-max_above_table, table_d-table_margin]
    内的连通块（桌面上方 table_margin~max_above_table 的凸起物）。
    ⚠️ 必须有下界 max_above_table：眼在手相机视野里有手腕/夹爪（离相机很近，~0.1-0.3m），
    比桌面近得多；若只阈值 depth<(桌面-margin) 会把夹爪当最大凸起物 → mask=夹爪，FP 把
    cola mesh 配到夹爪上 → 位姿落在夹爪深度（实测把物体放近 3×）。加下界排除近端机器人。
    """
    valid = depth[np.isfinite(depth) & (depth > 0.05) & (depth < 5.0)]
    if valid.size < 100:
        return None
    table_d = float(np.median(valid))   # 桌面占多数 → 中位数≈桌面深度
    lo = table_d - max_above_table      # 近端：比这还近的是机器人手腕/夹爪，排除
    hi = table_d - table_margin         # 远端：桌面本身
    binmask = ((depth > lo) & (depth < hi)).astype(np.uint8) * 255
    # 形态学去噪 + 取最大连通块（cola）
    k = np.ones((5, 5), np.uint8)
    binmask = cv2.morphologyEx(binmask, cv2.MORPH_OPEN, k)
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(binmask, 8)
    if n <= 1:
        return None
    # 过滤太小的，取最大
    areas = [(i, stats[i][4]) for i in range(1, n) if stats[i][4] >= min_area]
    if not areas:
        return None
    biggest = max(areas, key=lambda t: t[1])[0]
    out = np.zeros_like(binmask)
    out[lbl == biggest] = 255
    return out, table_d


class GTMaskNode(Node):
    def __init__(self):
        super().__init__("gt_mask_from_depth")
        self.declare_parameter("table_margin", 0.05)   # 凸起 >5cm 才算物体
        self.declare_parameter("max_above_table", 0.15)  # 物体最高 15cm；排除更近的夹爪/手腕
        self.declare_parameter("min_obj_area", 200)    # 最小像素面积
        self.declare_parameter("rate_hz", 5.0)
        self._margin = self.get_parameter("table_margin").value
        self._max_above = self.get_parameter("max_above_table").value
        self._min_area = self.get_parameter("min_obj_area").value
        self._rate = self.get_parameter("rate_hz").value
        self._depth = None
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._on_depth, 10)
        self._pub = self.create_publisher(Image, "/segmentation", 10)
        self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"GT mask(深度带): 桌面上方[{self._margin},{self._max_above}]m min_area={self._min_area}px"
        )

    def _on_depth(self, msg):
        try:
            self._depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"depth 转换失败: {e}")

    def _tick(self):
        if self._depth is None:
            return
        res = depth_to_mask(self._depth, self._margin, self._min_area, self._max_above)
        if res is None:
            return
        mask, table_d = res
        msg = Image()
        msg.height, msg.width = mask.shape
        msg.encoding = "mono8"
        msg.step = mask.shape[1]
        msg.data = mask.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        self._pub.publish(msg)
        ys, xs = np.where(mask > 127)
        cov = 100 * len(xs) / (mask.shape[0] * mask.shape[1])
        self.get_logger().info(
            f"mask {mask.shape[1]}x{mask.shape[0]} cov={cov:.1f}% "
            f"bbox=({int(xs.min())},{int(xs.max())},{int(ys.min())},{int(ys.max())}) table_d={table_d:.3f}",
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    rclpy.spin(GTMaskNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
