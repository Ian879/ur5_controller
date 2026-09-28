#!/usr/bin/env python3
"""手眼标定节点（eye-in-hand）：ArUco 采点 → AX=XB 解 → 回写 URDF 相机 mount。

数学在 handeye_solver.py（纯 numpy，合成数据已单测：0.3mm/0.1° 检测噪声 →
旋转 ≤0.12°、平移 ≤0.5mm）。本节点只做 IO：图像/TF/服务/URDF。

现场流程（真机栈起来后，服务器或 Jetson 上跑均可——要能同时看到
/camera/color/image_raw 和 TF）：
  1. 打印一张 ArUco 标记（默认 6x6_100 字典 id=0），平整贴在固定物上，
     量好边长（默认 5cm，参数 marker_length 改准）。
  2. 起本节点（robot env 有 cv2）：
       $HOME/.conda/envs/robot/bin/python -m ur5_motion.handeye_calib --ros-args \
         -p marker_length:=0.05 -p marker_id:=0
     （或 ros2 run ur5_motion handeye_calib —— 前提跑它的 python 里有 cv2）
  3. 用 RViz/teach 把臂挪到 ≥8 个不同朝向的位姿（标记始终在画面里，旋转轴多样），
     每个位姿采一次：
       ros2 service call /handeye_calib/capture std_srvs/srv/Trigger {}
  4. 求解（打印建议的 URDF origin；reprojection 残差大会有提示）：
       ros2 service call /handeye_calib/solve std_srvs/srv/Trigger {}
  5. 确认量级合理后回写 URDF（-p write_urdf:=true 再 solve 一次；自动留 .bak）：
     flange_to_camera_link 的 origin 被替换为标定值。重启栈生效
     （服务器 moveit 与 Jetson RSP 都吃这份 URDF——单一事实源）。

⚠ 标定质量取决于位姿多样性和标记检测质量；本节点每次 capture 打印检测到的
  角点数与标记 id，solve 打印每对残差，异常值人工重采。
"""
from __future__ import annotations

import os
import re

import numpy as np

try:
    import cv2
    import cv2.aruco
except ImportError as exc:  # 运行环境须有 cv2（robot conda env 有）
    cv2 = None
    _cv2_err = exc

import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from .grasp_geometry import quat_to_rotmat
from .handeye_solver import compose_mount, rot_from_rpy, rpy_from_rot, solve_hand_eye

URDF_REL = "urdf/ur5_with_gripper.urdf.xacro"
JOINT_NAME = "flange_to_camera_link"


def _find_urdf() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ur5_bringup"), URDF_REL)


def _marker_object_points(length: float) -> np.ndarray:
    """ArUco 角点（TL,TR,BR,BL）在标记系的 3D 坐标。标记系朝向只需【每次一致】——
    AX=XB 里 T_base←target 是被消掉的常量，绝对朝向无所谓。"""
    h = length / 2.0
    return np.array([[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]], np.float32)


class HandEyeCalibNode(Node):
    def __init__(self):
        super().__init__("handeye_calib")
        if cv2 is None:
            raise RuntimeError(f"缺 cv2（{_cv2_err}）——用 robot conda env 跑本节点")
        self.declare_parameter("marker_dict", "DICT_6X6_100")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_length", 0.05)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gripper_frame", "tool0")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("info_topic", "/camera/color/camera_info")
        self.declare_parameter("min_captures", 8)
        self.declare_parameter("write_urdf", False)
        self.declare_parameter("urdf_path", "")          # 空=自动找 share 里的

        dname = self.get_parameter("marker_dict").value
        try:
            dic = getattr(cv2.aruco, dname)
        except AttributeError:
            raise RuntimeError(f"未知 marker_dict: {dname}")
        self._dict = cv2.aruco.getPredefinedDictionary(dic)
        try:                                             # cv2 ≥4.7 API
            self._detector = cv2.aruco.ArucoDetector(self._dict, cv2.aruco.DetectorParameters())
        except AttributeError:                            # 旧 API
            self._detector = None
        self._marker_id = int(self.get_parameter("marker_id").value)
        self._obj_pts = _marker_object_points(float(self.get_parameter("marker_length").value))

        self._img: Image | None = None
        self._info: CameraInfo | None = None
        self._pairs: list[tuple[np.ndarray, np.ndarray]] = []   # (Hg, Hc)

        self.create_subscription(Image, self.get_parameter("color_topic").value,
                                 self._on_img, 10)
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value,
                                 self._on_info, 10)
        self._tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf, self)

        self.create_service(Trigger, "~/capture", self._capture)
        self.create_service(Trigger, "~/solve", self._solve)
        self.get_logger().info(
            f"handeye_calib: up — 标记 {dname} id={self._marker_id} "
            f"L={self.get_parameter('marker_length').value}m；"
            f"≥{int(self.get_parameter('min_captures').value)} 个多样位姿各 capture 一次，然后 solve")

    # ── 订阅缓存 ────────────────────────────────────────────
    def _on_img(self, msg: Image):
        self._img = msg

    def _on_info(self, msg: CameraInfo):
        self._info = msg

    # ── 标记检测：图像 + 内参 → 标记在光学系的位姿 (R,t) ──────
    def _detect(self, stamp):
        if self._img is None or self._info is None:
            raise RuntimeError("还没收到图像/内参（相机在发吗？）")
        img = np.frombuffer(self._img.data, dtype=np.uint8)
        if self._img.encoding in ("rgb8", "bgr8"):
            frame = img.reshape(self._img.height, self._img.width, 3)
            if self._img.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif self._img.encoding == "mono8":
            frame = img.reshape(self._img.height, self._img.width)
        else:
            raise RuntimeError(f"不支持的编码 {self._img.encoding}")
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self._dict)
        if ids is None or len(ids) == 0:
            raise RuntimeError("画面里没有 ArUco 标记")
        idx = list(ids.flatten()).index(self._marker_id) \
            if self._marker_id in ids.flatten() else None
        if idx is None:
            raise RuntimeError(f"检测到 id={list(ids.flatten())}，没有目标 id={self._marker_id}")
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)

        K = np.array(self._info.k, np.float64).reshape(3, 3)
        D = np.array(self._info.d, np.float64)
        ok, rvec, tvec = cv2.solvePnP(self._obj_pts, img_pts, K, D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise RuntimeError("solvePnP 失败")
        R, _ = cv2.Rodrigues(rvec)
        self.get_logger().info(
            f"capture: marker id={self._marker_id} @ 光学系 t={np.round(tvec.flatten(), 4)} m")
        return R, tvec.flatten()

    # ── services ────────────────────────────────────────────
    def _capture(self, req, resp):
        try:
            stamp = self._img.header.stamp if self._img else None
            R_c, t_c = self._detect(stamp)                       # T_cam←target
            base = self.get_parameter("base_frame").value
            grip = self.get_parameter("gripper_frame").value
            tf = self._tf.lookup_transform(base, grip, rclpy.time.Time())  # 最新即可
            q = tf.transform.rotation
            R_g = quat_to_rotmat([q.x, q.y, q.z, q.w])
            t_g = np.array([tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z])
            Hg = np.eye(4); Hg[:3, :3] = R_g; Hg[:3, 3] = t_g
            Hc = np.eye(4); Hc[:3, :3] = R_c; Hc[:3, 3] = t_c
            self._pairs.append((Hg, Hc))
            resp.success = True
            resp.message = f"已采 {len(self._pairs)} 对（TF 时间戳 {stamp.sec if stamp else '?'}.{stamp.nanosec if stamp else 0}）"
        except Exception as exc:
            resp.success = False
            resp.message = f"capture 失败：{exc}"
        return resp

    def _solve(self, req, resp):
        n_min = int(self.get_parameter("min_captures").value)
        if len(self._pairs) < n_min:
            resp.success = False
            resp.message = f"只有 {len(self._pairs)} 对（要 ≥{n_min}）——继续 capture"
            return resp
        try:
            R_x, t_x = solve_hand_eye([p[0] for p in self._pairs],
                                      [p[1] for p in self._pairs])
            link = self.get_parameter("camera_link_frame").value
            opt = self.get_parameter("camera_frame").value
            tf = self._tf.lookup_transform(link, opt, rclpy.time.Time())
            q = tf.transform.rotation
            T_link2opt = np.eye(4)
            T_link2opt[:3, :3] = quat_to_rotmat([q.x, q.y, q.z, q.w])
            T_link2opt[:3, 3] = [tf.transform.translation.x,
                                 tf.transform.translation.y,
                                 tf.transform.translation.z]
            R_m, t_m = compose_mount(R_x, t_x, T_link2opt)
            rpy = rpy_from_rot(R_m)
            line = (f'xyz="{" ".join(f"{v:.6f}" for v in t_m)}" '
                    f'rpy="{" ".join(f"{v:.6f}" for v in rpy)}"')
            self.get_logger().info(f"标定结果 → flange_to_camera_link origin {line}")
            self.get_logger().info(
                f"光学系口径：R_gripper←optical t={np.round(t_x, 4)}；"
                f"旋转角={np.degrees(np.arccos(np.clip((np.trace(R_x)-1)/2, -1, 1))):.1f}°")
            msg = f"OK: <origin {line} />"
            if self.get_parameter("write_urdf").value:
                path = self.get_parameter("urdf_path").value or _find_urdf()
                backup = path + ".bak"
                if not os.path.exists(backup):
                    os.replace(path, backup)                # 首次留底
                with open(backup, "r", encoding="utf-8") as f:
                    src = f.read()
                pat = re.compile(
                    rf'(<joint name="{JOINT_NAME}"[^>]*>.*?<origin )xyz="[^"]*" rpy="[^"]*"(.*?</joint>)',
                    re.S)
                if not pat.search(src):
                    resp.success = False
                    resp.message = f"{path} 里没找到 {JOINT_NAME} 的 origin"
                    return resp
                with open(path, "w", encoding="utf-8") as f:
                    f.write(pat.sub(rf"\g<1>{line}\g<2>", src, count=1))
                msg += f"（已回写 {path}，备份 {backup}；重启栈生效）"
            resp.success = True
            resp.message = msg
        except Exception as exc:
            resp.success = False
            resp.message = f"solve 失败：{exc}"
        return resp


def main():
    if cv2 is None:
        raise SystemExit(f"缺 cv2（{_cv2_err}）——用 robot conda env 跑："
                         "$HOME/.conda/envs/robot/bin/python -m ur5_motion.handeye_calib")
    rclpy.init()
    node = HandEyeCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
