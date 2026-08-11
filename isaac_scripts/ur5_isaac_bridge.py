#!/usr/bin/env python3
"""Isaac Sim -> MoveIt2 bridge for the UR5 (SP1 Task 8).

A standalone Isaac Sim script that references the UR5 robot USD at runtime
(moveit.py style) and exposes it to MoveIt2:

  * publishes ``/clock`` (so the ROS2 side can run ``use_sim_time:=true``);
  * publishes ``/isaac_joint_states`` (read by the TopicBasedSystem hardware
    plugin wired in SP1 Task 5/6);
  * subscribes ``/isaac_joint_commands`` (written by TopicBasedSystem) and drives
    the UR5 articulation via ``IsaacArticulationController``;
  * publishes wrist-camera RGB + depth + camera_info（标准 ROS RGBD 契约，与真机 Orbbec 一致）:
    ``/camera/color/image_raw``, ``/camera/color/camera_info``,
    ``/camera/depth/image_rect_raw``, ``/camera/depth/camera_info``
    (frame: camera_color_optical_frame / camera_depth_optical_frame).

This is the UR5 equivalent of Isaac's bundled Franka ``moveit.py``, plus the RTX
camera graph from ``camera_periodic.py``. The robot USD is referenced at runtime
via ``stage_utils.add_reference_to_stage(--usd, --robot_prim)`` (NOT opened as a
pre-built scene): the build_scene.py pre-built scene did not compose the robot's
physics/articulation sublayer, leaving ``/ur5/root_joint`` invalid and producing
``Failed to find articulation``. The articulation is targeted at
``--articulation_prim`` (default ``/ur5/root_joint``, where ArticulationRootAPI
lives), NOT at ``/ur5``. The OmniGraph node wiring is copied *verbatim* from the
two references. See:

  * ``/home/gds/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py``
  * ``/home/gds/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py``

Run with Isaac's bundled interpreter::

    ~/isaacsim/python.sh isaac_scripts/ur5_isaac_bridge.py [--headless]

    # Defaults: --usd ~/Desktop/scene.usd  --robot_prim /ur5
    #           --articulation_prim /ur5/root_joint
    #           --camera_prim /ur5/wrist_3_link/flange/wrist_camera
    #           (flange 在 ROS/MoveIt 里即 tool0 —— 相机挂载在末端工具帧下)
    # 你在 Isaac GUI 里搭 ~/Desktop/scene.usd（UR5+桌子+相机+灯光），桥加载它。

The topic names ``isaac_joint_states`` / ``isaac_joint_commands`` are REQUIRED:
they are the names T5/T6's TopicBasedSystem reads/writes (matching the official
Franka ``isaac_moveit`` wiring). Do not change them.

The gripper articulation is wired onto the SAME ActionGraph with G-suffixed
nodes (ReadJointStateG/PublishJointStateG/SubscribeJointStateG/ArticulationControllerG),
publishing ``/isaac_gripper_joint_states`` and subscribing
``/isaac_gripper_joint_commands`` (JointState). The gripper prim defaults to
``/World/gripper_abs_stl/Geometry`` (verified ArticulationRootAPI location in
~/Desktop/scene.usd); override with ``--gripper_prim``. The ``isaac_gripper_action``
rclpy node (started by sim_isaac.launch.py) bridges MoveIt's GripperCommand handle
``lebai_gripper_controller/gripper_cmd`` onto these two topics.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

# Default UR5 robot USD (Isaac 6.0 asset). Its ArticulationRootAPI lives on the
# PhysicsFixedJoint ``/ur5/root_joint`` (NOT on ``/ur5``), and the 6 revolute
# joints are at ``/ur5/joints/<name>_joint`` with names that match
# ``ur_description`` exactly. Camera mount is ``/ur5/wrist_3_link/flange``.
UR5_USD_DEFAULT = (
    "/home/gds/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5/ur5.usd"
)

# argparse MUST be declared before constructing SimulationApp: the app installs
# its own argparse machinery on import. This mirrors moveit.py / camera_periodic.
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--usd",
    default=os.path.expanduser("~/Desktop/scene.usd"),
    help="Path to the scene .usd to load as the world stage. You build/edit this "
    "in the Isaac GUI (UR5 + table + camera + lights). Default: ~/Desktop/scene.usd. "
    "The scene should reference the UR5 at --robot_prim (articulation at "
    "--articulation_prim) and have a camera at --camera_prim.",
)
parser.add_argument(
    "--robot_prim",
    default="/ur5",
    help="Prim path to reference the robot USD under. Default: /ur5.",
)
parser.add_argument(
    "--articulation_prim",
    default=None,
    help="Prim with ArticulationRootAPI (PhysicsFixedJoint <robot_prim>/root_joint). "
    "Used for ArticulationController.robotPath AND ReadJointState.prim. "
    "If not given, defaults to <robot_prim>/root_joint (auto-derived).",
)
parser.add_argument(
    "--camera_prim",
    default=None,
    help="Wrist camera prim path. If not given, defaults to "
    "<robot_prim>/wrist_3_link/flange/wrist_camera (auto-derived).",
)
parser.add_argument(
    "--gripper_prim",
    default=None,
    help="Prim the gripper ArticulationControllerG/ReadJointStateG target. After "
    "the gripper is welded into the UR5 articulation (one merged root), this "
    "defaults to --articulation_prim (<robot_prim>/root_joint) — the SAME root "
    "the arm graph uses. Two IsaacArticulationControllers on one root with "
    "disjoint jointNames coexist (arm 6 joints via /isaac_joint_commands; "
    "gripper joints via /isaac_gripper_joint_commands). Override only if your "
    "gripper is still a SEPARATE articulation (pre-weld).",
)
parser.add_argument(
    "--object_prim",
    default=None,
    help="抓取物体 prim 路径（如 /World/bottle）。给了则读其世界 pose，发 base_link→grasp_target "
    "TF（ROS2PublishTransformTree）。⚠️ isaac python 是 3.12，humble rclpy 是 3.10，import rclpy "
    "会 ABI 不兼容失败，故 object pose 走 isaac ros2 bridge graph（和 world→base 同套），demo_grasp "
    "用 tf2 lookup grasp_target 读。不给则不发 object TF。",
)
# (SurfaceGripper 相关 CLI 已撤——真夹爪物理夹持不需要)
parser.add_argument(
    "--headless",
    default=False,
    action="store_true",
    help="Run Isaac Sim headless (no viewport window).",
)
parser.add_argument(
    "--stream",
    default=False,
    action="store_true",
    help="Enable WebRTC livestream (omni.kit.livestream.app) so you can view the "
    "sim in the Isaac Sim WebRTC Streaming Client (browser) -- same viewing as "
    "isaac-sim.streaming.sh, but with the ROS bridge logic running. "
    "Implies headless render (no local window); set hide_ui=False so the GUI "
    "streams. Use this on machines without a local display.",
)
args, _ = parser.parse_known_args()

# articulation_prim / camera_prim 默认从 robot_prim 自动推导（这样只需 --robot_prim，
# 适配用户改过的 prim 层级，不用单独传 articulation/camera 路径）。
if args.articulation_prim is None:
    args.articulation_prim = args.robot_prim + "/root_joint"
if args.camera_prim is None:
    args.camera_prim = args.robot_prim + "/wrist_3_link/flange/wrist_camera"
# Gripper is welded into the UR5 articulation (one merged root) → gripper graph
# targets the SAME articulation_prim as the arm (see A2 / weld_gripper.py).
if args.gripper_prim is None:
    args.gripper_prim = args.articulation_prim

# Fail fast on a missing scene: booting Isaac takes ~30s, so check now.
if not os.path.isfile(args.usd):
    print(
        f"ERROR: scene .usd not found: {args.usd}\n"
        "  在 Isaac GUI 里搭建场景（UR5 + 桌子 + 相机 + 灯光），存成这个路径再跑。",
        file=sys.stderr,
    )
    sys.exit(1)

ACTION_GRAPH_PATH = "/ActionGraph"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"

# SimulationApp config. --stream 用【完整】streaming kit 配置（isaacsim.exp.full.streaming.kit，
# 和 isaac-sim.streaming.sh 一样，全插件 + WebRTC）；否则用 python.sh 默认的精简 base.python.kit。
if args.stream:
    _launch_config = {
        "width": 1280,
        "height": 720,
        "headless": True,        # no local window; render offscreen for the stream
        "hide_ui": False,        # compose the GUI so the WebRTC client sees it
        "renderer": "RealTimePathTracing",
        "display_options": 3286,
    }
    # 关键：指定 experience=full.streaming.kit，否则 python.sh 默认加载精简的
    # base.python.kit（少很多模块/插件）。
    _isaac_root = os.path.dirname(os.environ.get("CARB_APP_PATH", "/home/gds/isaacsim/kit"))
    _experience = os.path.join(_isaac_root, "apps", "isaacsim.exp.full.streaming.kit")
    if not os.path.isfile(_experience):
        print(f"ERROR: streaming kit not found: {_experience}", file=sys.stderr)
        sys.exit(1)
else:
    _launch_config = {"headless": args.headless, "renderer": "RealTimePathTracing"}
    _experience = ""  # 默认 base.python.kit
simulation_app = SimulationApp(_launch_config, experience=_experience)

# --- Isaac imports (only valid after SimulationApp is constructed) ---
import carb  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni  # noqa: E402
import omni.usd  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
# SurfaceGripper 已撤——真夹爪靠物理夹持（手指对夹+摩擦），不靠吸附 attach。
# isaac/real 一致：real Lebai 夹爪也是物理夹持，无 SurfaceGripper。

# Enable the ROS2 bridge extension and let it settle for one frame.
app_utils.enable_extension("isaacsim.ros2.bridge")

# Optionally enable the WebRTC livestream server (same viewing as
# isaac-sim.streaming.sh). Connect from the Isaac Sim WebRTC Streaming Client.
if args.stream:
    simulation_app.set_setting("/app/window/drawMouse", True)
    app_utils.enable_extension("omni.kit.livestream.app")

simulation_app.update()

stage_utils.set_stage_units(meters_per_unit=1.0)

# 用 carb.log_warn 醒目打印（刷进 kit 日志，stdout 会被块缓冲看不到）
carb.log_warn(
    f"[ur5_isaac_bridge] >>> usd={args.usd}\n"
    f"    robot_prim={args.robot_prim}\n"
    f"    articulation_prim={args.articulation_prim}  (ArticulationController 用这个)\n"
    f"    camera_prim={args.camera_prim}\n"
    f"    gripper_prim={args.gripper_prim}  (gripper ArticulationController 用这个)\n"
    f"    headless={args.headless} stream={args.stream}"
)

# --- Load the user's scene .usd as the world stage ---
# You build/edit ~/Desktop/scene.usd in the Isaac GUI (UR5 + table + camera +
# lights). The bridge loads it and wires the ROS graph onto the robot's
# articulation + camera. Ensure the scene references the UR5 at --robot_prim
# (articulation at --articulation_prim) and has a camera at --camera_prim.
_open_ok = omni.usd.get_context().open_stage(args.usd)
if not _open_ok:
    carb.log_error(f"Failed to open scene '{args.usd}' as world stage")
    simulation_app.close()
simulation_app.update()

stage = omni.usd.get_context().get_stage()

# (机器人/桌子/相机/灯光都在你的 scene.usd 里，桥不再运行时创建。)

# --- diagnostic: 加载后核对关键 prim 是否存在/位置合理 ---
# 核对场景里的 robot 根、articulation（承重）、相机、base_link。若 NOT FOUND，
# 说明你的 scene.usd 里这些 prim 不在默认路径——用 --robot_prim /
# --articulation_prim / --camera_prim 指过去，或在场景里放到默认路径。
try:
    from pxr import Usd as _Usd  # noqa: E402
    for _path, _name in [
        (args.robot_prim, "robot"),
        (args.articulation_prim, "articulation"),
        (args.camera_prim, "camera"),
        (args.robot_prim + "/base_link", "base_link"),
        (args.gripper_prim, "gripper"),
    ]:
        _prim = stage.GetPrimAtPath(_path)
        if _prim.IsValid():
            _t = UsdGeom.Xformable(_prim).ComputeLocalToWorldTransform(
                _Usd.TimeCode.Default()
            ).ExtractTranslation()
            carb.log_warn(
                f"[scene-diag] {_name:10s} world = ({_t[0]:.3f}, {_t[1]:.3f}, {_t[2]:.3f})"
            )
        else:
            carb.log_warn(f"[scene-diag] {_name:10s} prim NOT FOUND at {_path}")
except Exception as _e:  # noqa: BLE001
    carb.log_warn(f"[scene-diag] failed: {_e}")

# ── world→base_link TF 数据（P2）：读 base_link 世界 pose，供下面 ActionGraph 的
#    ROS2PublishTransformTree 发 tf_static。替代 sim_isaac.launch.py 的硬编码 TF——
#    GUI 挪机器人后重启桥即自动跟随。兜底 identity（prim 无效时不阻塞 arm/clock/camera）。
_base_trans = [0.0, 0.0, 0.0]
_base_quat = [0.0, 0.0, 0.0, 1.0]  # xyzw
_base_inv = None  # world→base 逆变换，供 object-in-base 换算
try:
    _base_prim = stage.GetPrimAtPath(args.robot_prim + "/base_link")
    if _base_prim.IsValid():
        _bw = UsdGeom.Xformable(_base_prim).ComputeLocalToWorldTransform(_Usd.TimeCode.Default())
        _base_inv = _bw.GetInverse()
        _bt = _bw.ExtractTranslation()
        _bq = _bw.ExtractRotationQuat()  # Gf.Quatd：实部 GetReal()=w，虚部 GetImaginary()=Vec3(x,y,z)
        _bi = _bq.GetImaginary()
        _base_trans = [float(_bt[0]), float(_bt[1]), float(_bt[2])]
        _base_quat = [float(_bi[0]), float(_bi[1]), float(_bi[2]), float(_bq.GetReal())]
        carb.log_warn(f"[world-tf] world→base_link = trans={_base_trans} quat(xyzw)={_base_quat}")
    else:
        carb.log_warn(f"[world-tf] base_link prim 无效（{args.robot_prim}/base_link），用 identity 兜底")
except Exception as _e:  # noqa: BLE001
    carb.log_warn(f"[world-tf] 算 base pose 失败: {_e}（用 identity 兜底）")

# ── object→base TF 数据（P4 抓取目标 pose）：--object_prim 给出才读。发 base_link→grasp_target TF。
#    isaac python 3.12 不能 import rclpy（humble 3.10 ABI 不兼容），故走 isaac ros2 bridge graph。
_obj_trans = [0.0, 0.0, 0.0]
_obj_quat = [0.0, 0.0, 0.0, 1.0]
if args.object_prim and _base_inv is not None:
    try:
        _op = stage.GetPrimAtPath(args.object_prim)
        if _op.IsValid():
            _ow = UsdGeom.Xformable(_op).ComputeLocalToWorldTransform(_Usd.TimeCode.Default())
            _ob = _base_inv * _ow  # object 在 base 系 4x4
            _ot = _ob.ExtractTranslation()
            _oq = _ob.ExtractRotationQuat()
            _oi = _oq.GetImaginary()
            _obj_trans = [float(_ot[0]), float(_ot[1]), float(_ot[2])]
            _obj_quat = [float(_oi[0]), float(_oi[1]), float(_oi[2]), float(_oq.GetReal())]
            carb.log_warn(f"[object-tf] {args.object_prim} in base = trans={_obj_trans} quat(xyzw)={_obj_quat}")
        else:
            carb.log_warn(f"[object-tf] object_prim {args.object_prim} 无效，object TF 发 identity")
    except Exception as _e:  # noqa: BLE001
        carb.log_warn(f"[object-tf] 读 object pose 失败: {_e}")
elif args.object_prim:
    carb.log_warn(f"[object-tf] base_inv 不可用，object TF 发 identity")

# Sanity-check that the prims we wire into the OmniGraph actually exist in the
# loaded stage. The articulation prim is the load-bearing one: if it's invalid,
# the OG nodes emit "Failed to find articulation" / "no DOFs found".
_art_ok = bool(stage.GetPrimAtPath(args.articulation_prim).IsValid())
_camera_ok = bool(stage.GetPrimAtPath(args.camera_prim).IsValid())
# 默认 camera_prim（<robot_prim>/wrist_3_link/flange/wrist_camera）在眼在手场景里通常
# 不存在（相机是独立 rgbd_camera 装配、靠 FixedJoint 挂在手法上，不在 robot_prim 下）。
# 此时自动找场景里的 RGB Camera prim（路径含 'rgb'，排除 ir/depth）改用，省得每次手传 --camera_prim。
if not _camera_ok:
    _found_cam = None
    for _p in stage.Traverse():
        if _p.GetTypeName() == "Camera":
            _ps = str(_p.GetPath()).lower()
            if "rgb" in _ps and "ir" not in _ps and "depth" not in _ps:
                _found_cam = str(_p.GetPath())
                break
    if _found_cam:
        carb.log_warn(
            f"camera_prim '{args.camera_prim}' 无效；自动改用场景里的 RGB 相机 prim: {_found_cam}"
            f"\n    （要换相机，传 --camera_prim <Camera prim 路径>）"
        )
        args.camera_prim = _found_cam
        _camera_ok = True
_gripper_ok = bool(stage.GetPrimAtPath(args.gripper_prim).IsValid())
if not _art_ok:
    carb.log_error(
        f"\n*** ARTICULATION PRIM '{args.articulation_prim}' 在场景 '{args.usd}' 里找不到 ***\n"
        f"    你传的 --robot_prim='{args.robot_prim}' → 推导出 articulation='{args.articulation_prim}'。\n"
        f"    但你的场景里机器人不在这个 prim。用 --robot_prim <你的机器人 prim> 覆盖\n"
        f"    （例如 --robot_prim /World/ur5）。introspect 场景找 ArticulationRootAPI：\n"
        f"      ~/isaacsim/python.sh -c \"from isaacsim import SimulationApp; app=SimulationApp({'headless':True}); "
        f"from pxr import Usd,UsdPhysics; s=Usd.Stage.Open('{args.usd}'); "
        f"[print(p.GetPath()) for p in s.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]\""
    )
    simulation_app.close()
else:
    carb.log_warn(
        f"[ur5_isaac_bridge] articulation prim OK: {args.articulation_prim} "
        f"({stage.GetPrimAtPath(args.articulation_prim).GetTypeName()})"
    )
if not _camera_ok:
    carb.log_warn(
        f"Camera prim '{args.camera_prim}' is not valid; camera RGBD publishing "
        "will be skipped (joint/clock bridge still runs)."
    )
if not _gripper_ok:
    carb.log_warn(
        f"\n[ur5_isaac_bridge] GRIPPER prim '{args.gripper_prim}' 在场景里找不到 "
        f"(type={stage.GetPrimAtPath(args.gripper_prim).GetTypeName()}).\n"
        "    夹爪 articulation 读写图会建但运行时报 'Failed to find articulation'.\n"
        "    用 --gripper_prim <你的夹爪 prim> 覆盖（ArticulationRootAPI 所在 prim）.\n"
        "    若场景里没有夹爪 articulation，忽略本警告——arm/clock/camera 照常跑."
    )

# --- 清理残留 graph（之前的桥运行可能把图存进了 scene.usd，robotPath/cameraPrim 是老的；
#     场景自带的 /World/ROS_Camera 也会和桥的 /ROS_Camera 抢同一个相机 → 都不出图）---
for _stale_path in ["/World/ActionGraph", "/ActionGraph",
                    "/World/ROS_Camera", "/ROS_Camera"]:
    _stale = stage.GetPrimAtPath(_stale_path)
    if _stale.IsValid():
        stage.RemovePrim(_stale.GetPath())
        carb.log_warn(f"[ur5_isaac_bridge] removed stale graph {_stale_path}")

# =====================================================================
# Action graph: /clock + /isaac_joint_states pub + /isaac_joint_commands sub
#   (copied verbatim from moveit.py; retargeted robotPath / ReadJointState.prim
#    to args.articulation_prim = /ur5/root_joint, and pinned the two required
#    isaac topic names).
# =====================================================================
try:
    og.Controller.edit(
        {"graph_path": ACTION_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                # --- gripper articulation read/write (Task 5) ---
                # Mirror the UR5 joint graph (ReadJointState/PublishJointState/
                # SubscribeJointState/ArticulationController) with G-suffixed node
                # names, the gripper articulation prim, and the gripper topic names.
                # Node types are IDENTICAL to the UR5 ones above (verified against the
                # installed isaacsim.ros2.bridge). Only the prim/topic differ.
                ("ReadJointStateG", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PublishJointStateG", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointStateG", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerG", "isaacsim.core.nodes.IsaacArticulationController"),
                # world→base_link TF 发布（P2：从 USD 读机器人 base 世界 pose 发 tf_static，
                # 替代 sim_isaac.launch.py 的硬编码 static_transform_publisher，GUI 挪机器人自动跟随）。
                ("WorldTfPub", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                # object→base TF（P4 抓取目标）：发 base_link→grasp_target，值=_obj_trans/_obj_quat
                # （--object_prim 读的物体在 base 系 pose）。demo_grasp tf2 lookup grasp_target 读。
                ("ObjectTfPub", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                # --- gripper articulation graph wiring (Task 5) ---
                # Mirror the UR5 wiring above, node names G-suffixed. Same OnPlaybackTick
                # tick + same ROS2Context drive the gripper graph in parallel. The
                # ArticulationControllerG targets the gripper articulation (see SET_VALUES).
                ("OnPlaybackTick.outputs:tick", "ReadJointStateG.inputs:execIn"),
                ("ReadJointStateG.outputs:execOut", "PublishJointStateG.inputs:execIn"),
                ("ReadJointStateG.outputs:jointNames", "PublishJointStateG.inputs:jointNames"),
                ("ReadJointStateG.outputs:jointPositions", "PublishJointStateG.inputs:jointPositions"),
                ("ReadJointStateG.outputs:jointVelocities", "PublishJointStateG.inputs:jointVelocities"),
                ("ReadJointStateG.outputs:jointEfforts", "PublishJointStateG.inputs:jointEfforts"),
                ("ReadJointStateG.outputs:jointDofTypes", "PublishJointStateG.inputs:jointDofTypes"),
                ("ReadJointStateG.outputs:stageMetersPerUnit", "PublishJointStateG.inputs:stageMetersPerUnit"),
                ("ReadJointStateG.outputs:sensorTime", "PublishJointStateG.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointStateG.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationControllerG.inputs:execIn"),
                ("Context.outputs:context", "PublishJointStateG.inputs:context"),
                ("Context.outputs:context", "SubscribeJointStateG.inputs:context"),
                ("SubscribeJointStateG.outputs:jointNames", "ArticulationControllerG.inputs:jointNames"),
                (
                    "SubscribeJointStateG.outputs:positionCommand",
                    "ArticulationControllerG.inputs:positionCommand",
                ),
                (
                    "SubscribeJointStateG.outputs:velocityCommand",
                    "ArticulationControllerG.inputs:velocityCommand",
                ),
                ("SubscribeJointStateG.outputs:effortCommand", "ArticulationControllerG.inputs:effortCommand"),
                # WorldTfPub 由 OnPlaybackTick + ROS2Context 驱动（translation/orientation 在 SET_VALUES 里设常量）
                ("OnPlaybackTick.outputs:tick", "WorldTfPub.inputs:execIn"),
                ("Context.outputs:context", "WorldTfPub.inputs:context"),
                # ObjectTfPub 同样由 OnPlaybackTick + ROS2Context 驱动
                ("OnPlaybackTick.outputs:tick", "ObjectTfPub.inputs:execIn"),
                ("Context.outputs:context", "ObjectTfPub.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Point the articulation controller + joint reader at the UR5
                # articulation root (ArticulationRootAPI lives on the
                # PhysicsFixedJoint /ur5/root_joint, NOT on /ur5). Targeting the
                # root_joint is what makes Isaac find the 6-DOF articulation;
                # pointing these at /ur5 reproduces the T7 "Failed to find
                # articulation" / "no DOFs found" failure.
                ("ArticulationController.inputs:robotPath", args.articulation_prim),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(args.articulation_prim)]),
                # REQUIRED topic names — match T5/T6 TopicBasedSystem + official moveit.py.
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                # --- gripper articulation (Task 5) ---
                # Point the gripper ArticulationController + ReadJointState at the
                # gripper articulation root. Verified against ~/Desktop/scene.usd:
                # ArticulationRootAPI lives on the Scope /World/gripper_abs_stl/Geometry
                # (the joints live under the sibling /World/gripper_abs_stl/Physics/*
                # scope and are resolved via body-link refs). The driven joint name
                # is gripper_r_joint1 (prim leaf; physics:jointName is None so Isaac
                # uses the leaf) — matches the URDF, so isaac_gripper_action needs no
                # joint-name mapping.
                # NOTE: robotPath takes a string prim path; ReadJointState.prim takes
                # a LIST of usdrt.Sdf.Path — same asymmetry as the UR5 graph above.
                ("ArticulationControllerG.inputs:robotPath", args.gripper_prim),
                ("ReadJointStateG.inputs:prim", [usdrt.Sdf.Path(args.gripper_prim)]),
                # Gripper topics — distinct from the arm's isaac_joint_* topics.
                # isaac_gripper_action writes commands here & reads states here.
                ("PublishJointStateG.inputs:topicName", "isaac_gripper_joint_states"),
                ("SubscribeJointStateG.inputs:topicName", "isaac_gripper_joint_commands"),
                # WorldTfPub：world→base_link static TF。parent/child/translation/orientation 用
                # 预读的 base_link 世界 pose 常量（机器人 base 静态，staticPublisher=latch 发 tf_static）。
                ("WorldTfPub.inputs:topicName", "tf_static"),
                ("WorldTfPub.inputs:staticPublisher", True),
                ("WorldTfPub.inputs:parentFrames", ["world"]),
                ("WorldTfPub.inputs:childFrames", ["base_link"]),
                ("WorldTfPub.inputs:translations", [_base_trans]),
                ("WorldTfPub.inputs:orientations", [_base_quat]),
                # ObjectTfPub：base_link→grasp_target TF（物体在 base 系 pose；--object_prim 未给则 identity）
                ("ObjectTfPub.inputs:topicName", "tf_static"),
                ("ObjectTfPub.inputs:staticPublisher", True),
                ("ObjectTfPub.inputs:parentFrames", ["base_link"]),
                ("ObjectTfPub.inputs:childFrames", ["grasp_target"]),
                ("ObjectTfPub.inputs:translations", [_obj_trans]),
                ("ObjectTfPub.inputs:orientations", [_obj_quat]),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to build joint/clock action graph at {ACTION_GRAPH_PATH}: {e}")

simulation_app.update()

# =====================================================================
# Camera RGBD publish subgraph (copied verbatim from camera_periodic.py).
#   The wrist camera prim was created above (UsdGeom.Camera.Define at
#   args.camera_prim, mounted under /ur5/wrist_3_link/flange), so we skip
#   camera_periodic.py's prim-creation block and point IsaacCreateRenderProduct
#   straight at it. Topic/frame 名对齐 Orbbec/ROS 标准 RGBD 契约（SP2 相机集成）：
#   /camera/color/image_raw, /camera/color/camera_info,
#   /camera/depth/image_rect_raw, /camera/depth/camera_info;
#   frame camera_color_optical_frame / camera_depth_optical_frame.
# =====================================================================
if _camera_ok:
    try:
        keys = og.Controller.Keys
        ros_camera_graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": ROS_CAMERA_GRAPH_PATH,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperDepthInfo.inputs:execIn"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepthInfo.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(args.camera_prim)]),
                    # Resolution matches camera_periodic.py; adjust here to throttle GPU cost.
                    ("createRenderProduct.inputs:width", 1280),
                    ("createRenderProduct.inputs:height", 720),
                    # RGB → /camera/color/image_raw
                    ("cameraHelperRgb.inputs:frameId", "camera_color_optical_frame"),
                    ("cameraHelperRgb.inputs:topicName", "camera/color/image_raw"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    # color CameraInfo → /camera/color/camera_info
                    ("cameraHelperInfo.inputs:frameId", "camera_color_optical_frame"),
                    ("cameraHelperInfo.inputs:topicName", "camera/color/camera_info"),
                    # Depth → /camera/depth/image_rect_raw
                    ("cameraHelperDepth.inputs:frameId", "camera_depth_optical_frame"),
                    ("cameraHelperDepth.inputs:topicName", "camera/depth/image_rect_raw"),
                    ("cameraHelperDepth.inputs:type", "depth"),
                    # depth CameraInfo → /camera/depth/camera_info（★新增，原来只有 color 的）
                    ("cameraHelperDepthInfo.inputs:frameId", "camera_depth_optical_frame"),
                    ("cameraHelperDepthInfo.inputs:topicName", "camera/depth/camera_info"),
                ],
            },
        )
        # Run the camera graph once so the ROS image publishers are instantiated
        # inside the SDG pipeline (matches camera_periodic.py line 120).
        og.Controller.evaluate_sync(ros_camera_graph)
    except Exception as e:
        carb.log_warn(f"Failed to build camera RGBD graph at {ROS_CAMERA_GRAPH_PATH}: {e}")

    simulation_app.update()

# =====================================================================
# Initialize physics and run the simulation loop.
# =====================================================================
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")

app_utils.play()
simulation_app.update()

# 物理抓取：真夹爪靠手指对夹 + 摩擦力（close 驱动 gripper_r_joint1 → 手指合拢夹瓶侧），不用
# SurfaceGripper 吸附（吸盘机制，isaac 特有、real 没有）。桥只管 isaac↔ROS2 桥接（joint
# states/commands/camera/TF）；夹爪物理由 OG ArticulationControllerG 写关节 + isaac_gripper_action
# 节点驱动。抓取位姿/摩擦见 demo_grasp。

try:
    while simulation_app.is_running():
        # Fixed-step update; the OmniGraph action graph ticks each playback tick.
        simulation_app.update()
except KeyboardInterrupt:
    print("[ur5_isaac_bridge] interrupted by user (KeyboardInterrupt).")
finally:
    app_utils.stop()
    simulation_app.close()
