#!/usr/bin/env python3
"""SAM 分割节点：订阅 /camera/color/image_raw → 调 SAM HTTP API → 发 /segmentation。

⚠️ 不用 cv_bridge（其 boost 编译模块与 NumPy 2.x 不兼容）。ROS Image↔numpy 手动转换。
cv2 本身（5.0.0）在 robot conda env 兼容 NumPy 2.x。

跑（用 robot env python，不用 ros2 run）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /home/gds/.conda/envs/robot/bin/python src/ur5_motion/ur5_motion/sam_segmentation.py
"""
import base64
import json
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """ROS Image → numpy（不用 cv_bridge）。支持 bgr8/rgb8/mono8。"""
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        return arr.reshape(msg.height, msg.width, 3)
    if msg.encoding == "mono8":
        return arr.reshape(msg.height, msg.width)
    # 兜底：按 step 对齐
    return arr.reshape(msg.height, msg.step)[:, :msg.width]


def numpy_to_imgmsg(arr: np.ndarray, encoding: str = "mono8") -> Image:
    """numpy → ROS Image（不用 cv_bridge）。"""
    msg = Image()
    msg.height = arr.shape[0]
    msg.width = arr.shape[1]
    msg.encoding = encoding
    msg.step = arr.shape[1] * (1 if encoding == "mono8" else 3)
    msg.data = arr.tobytes()
    msg.is_bigendian = False
    return msg


class SAMSegmentationNode(Node):
    def __init__(self):
        super().__init__("sam_segmentation")

        self.declare_parameter("sam_url", "http://10.70.76.200:7890/api/sam/segment")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("prompt_x", -1)
        self.declare_parameter("prompt_y", -1)

        self._sam_url = self.get_parameter("sam_url").value
        self._prompt_x = self.get_parameter("prompt_x").value
        self._prompt_y = self.get_parameter("prompt_y").value
        self._rate = self.get_parameter("rate_hz").value

        self._latest_img = None

        self.create_subscription(Image, "/camera/color/image_raw", self._on_image, 10)
        self._pub = self.create_publisher(Image, "/segmentation", 10)
        self.create_timer(1.0 / self._rate, self._segment)

        self.get_logger().info(
            f"SAM 分割: url={self._sam_url} rate={self._rate}Hz "
            f"prompt=({'中心' if self._prompt_x < 0 else self._prompt_x},"
            f"{'中心' if self._prompt_y < 0 else self._prompt_y})"
        )

    def _on_image(self, msg):
        try:
            img = imgmsg_to_numpy(msg)
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self._latest_img = img
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"图像转换失败: {e}")

    def _segment(self):
        if self._latest_img is None:
            return

        img = self._latest_img
        h, w = img.shape[:2]
        px = w // 2 if self._prompt_x < 0 else int(self._prompt_x)
        py = h // 2 if self._prompt_y < 0 else int(self._prompt_y)

        try:
            _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])

            boundary = "----SAMBoundary"
            body = (
                f"--{boundary}\r\n".encode()
                + b'Content-Disposition: form-data; name="image"; filename="frame.jpg"\r\n'
                + b"Content-Type: image/jpeg\r\n\r\n"
                + buf.tobytes()
                + f"\r\n--{boundary}\r\n".encode()
                + b'Content-Disposition: form-data; name="points"\r\n\r\n'
                + f"[[{px},{py}]]".encode()
                + f"\r\n--{boundary}--\r\n".encode()
            )

            req = urllib.request.Request(self._sam_url, data=body, method="POST")
            req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")

            resp = urllib.request.urlopen(req, timeout=10)
            result = json.loads(resp.read())

            mask_b64 = result.get("mask_png_b64")
            if not mask_b64:
                self.get_logger().warn("SAM 返回无 mask_png_b64")
                return

            mask = cv2.imdecode(
                np.frombuffer(base64.b64decode(mask_b64), np.uint8), cv2.IMREAD_GRAYSCALE
            )
            if mask is None:
                self.get_logger().warn("mask PNG 解码失败")
                return

            msg = numpy_to_imgmsg(mask, "mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"
            self._pub.publish(msg)

            score = result.get("score", 0)
            self.get_logger().info(f"mask 发布 {mask.shape[1]}x{mask.shape[0]} score={score:.3f}")

        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"SAM 调用失败: {e}")


def main():
    rclpy.init()
    rclpy.spin(SAMSegmentationNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
