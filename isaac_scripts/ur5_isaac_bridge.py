#!/usr/bin/env python3
"""Isaac Sim -> MoveIt2 bridge for the UR5 (SP1 Task 8).

A standalone Isaac Sim script that references the UR5 robot USD at runtime
(moveit.py style) and exposes it to MoveIt2:

  * publishes ``/clock`` (so the ROS2 side can run ``use_sim_time:=true``);
  * publishes ``/isaac_joint_states`` (read by the TopicBasedSystem hardware
    plugin wired in SP1 Task 5/6);
  * subscribes ``/isaac_joint_commands`` (written by TopicBasedSystem) and drives
    the UR5 articulation via ``IsaacArticulationController``;
  * publishes wrist-camera RGB + depth + camera_info
    (``/camera/color``, ``/camera/depth``, ``/camera/camera_info``).

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

    # Defaults: --usd <Isaac ur5.usd asset> --robot_prim /ur5
    #           --articulation_prim /ur5/root_joint
    #           --camera_prim /ur5/wrist_3_link/flange/wrist_camera

The topic names ``isaac_joint_states`` / ``isaac_joint_commands`` are REQUIRED:
they are the names T5/T6's TopicBasedSystem reads/writes (matching the official
Franka ``isaac_moveit`` wiring). Do not change them.
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
    default=UR5_USD_DEFAULT,
    help="Path to the UR5 robot .usd to reference at runtime (moveit.py style). "
    "Default: the Isaac 6.0 ur5.usd asset.",
)
parser.add_argument(
    "--robot_prim",
    default="/ur5",
    help="Prim path to reference the robot USD under. Default: /ur5.",
)
parser.add_argument(
    "--articulation_prim",
    default="/ur5/root_joint",
    help="Prim with ArticulationRootAPI (the PhysicsFixedJoint /ur5/root_joint). "
    "Used for ArticulationController.robotPath AND ReadJointState.prim -- this is "
    "the path that actually resolves the 6-DOF articulation. Default: /ur5/root_joint.",
)
parser.add_argument(
    "--camera_prim",
    default="/ur5/wrist_3_link/flange/wrist_camera",
    help="Wrist camera prim path (created by the bridge if absent). "
    "Default: /ur5/wrist_3_link/flange/wrist_camera.",
)
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

# Fail fast on a bad --usd path: booting Isaac takes ~30s, so checking now saves
# the user from waiting just to see a load failure later.
if not os.path.isfile(args.usd):
    print(f"ERROR: --usd path not found or not a file: {args.usd}", file=sys.stderr)
    sys.exit(1)

ACTION_GRAPH_PATH = "/ActionGraph"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"

# SimulationApp config. --stream uses the official livestream config (headless
# render + hide_ui=False so the GUI is composed for the WebRTC stream); see
# ~/isaacsim/standalone_examples/api/isaacsim.simulation_app/livestream.py.
if args.stream:
    _launch_config = {
        "width": 1280,
        "height": 720,
        "headless": True,        # no local window; render offscreen for the stream
        "hide_ui": False,        # but compose the GUI so the client sees it
        "renderer": "RealTimePathTracing",
        "display_options": 3286,  # show default grid in the streamed view
    }
else:
    _launch_config = {"headless": args.headless, "renderer": "RealTimePathTracing"}
simulation_app = SimulationApp(_launch_config)

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

# Enable the ROS2 bridge extension and let it settle for one frame.
app_utils.enable_extension("isaacsim.ros2.bridge")

# Optionally enable the WebRTC livestream server (same viewing as
# isaac-sim.streaming.sh). Connect from the Isaac Sim WebRTC Streaming Client.
if args.stream:
    simulation_app.set_setting("/app/window/drawMouse", True)
    app_utils.enable_extension("omni.kit.livestream.app")

simulation_app.update()

stage_utils.set_stage_units(meters_per_unit=1.0)

print(
    f"[ur5_isaac_bridge] usd={args.usd} robot_prim={args.robot_prim} "
    f"articulation_prim={args.articulation_prim} camera_prim={args.camera_prim} "
    f"headless={args.headless}"
)

# --- Open the user's scene .usd (built in Isaac Sim GUI) as the world stage ---
# The scene should contain the UR5 at args.robot_prim (/ur5, with
# ArticulationRootAPI on /ur5/root_joint) plus whatever you added in the GUI
# (table / ground / lights / objects). We OPEN it as the root stage (the scene
# IS the world) and only inject the ROS2 bridge graph + a wrist camera.
# Default --usd = robot-only ur5.usd (for quick tests without a full scene).
_open_ok = omni.usd.get_context().open_stage(args.usd)
if not _open_ok:
    carb.log_error(f"Failed to open scene '{args.usd}' as world stage")
    simulation_app.close()
simulation_app.update()

stage = omni.usd.get_context().get_stage()

# --- Wrist camera (created only if the scene doesn't already have one) ---
# 你在 GUI 里摆场景（桌/地面/灯/物体）；相机是 ROS 桥的事，场景没有才自动建一个
# 挂到 wrist_3_link/flange（-Z 朝桌面）。若你在 GUI 里已加相机，传 --camera_prim 指向它。
if not stage.GetPrimAtPath(args.camera_prim).IsValid():
    camera_prim = UsdGeom.Camera.Define(stage, args.camera_prim)
    cam_xf = UsdGeom.Xformable(camera_prim)
    cam_xf.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.04))
    cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
    camera_prim.GetFocalLengthAttr().Set(24.0)

simulation_app.update()

# --- diagnostic: print world positions so we can align MoveIt scene coords ---
# base_link 应在 (0,0,0)；桌子 center 应在 (0.6,0,0.785)；地面 center 在 (0,0,-0.005)。
# 若实际值不符，说明 transform/units 有问题。
try:
    from pxr import Usd as _Usd  # noqa: E402
    for _path, _name in [
        ("/ur5/base_link", "base_link"),
        (args.articulation_prim, "articulation"),
        (args.camera_prim, "camera"),
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

# Sanity-check that the prims we wire into the OmniGraph actually exist in the
# loaded stage. The articulation prim is the load-bearing one: if it's invalid,
# the OG nodes emit "Failed to find articulation" / "no DOFs found".
_art_ok = bool(stage.GetPrimAtPath(args.articulation_prim).IsValid())
_camera_ok = bool(stage.GetPrimAtPath(args.camera_prim).IsValid())
if not _art_ok:
    carb.log_warn(
        f"Articulation prim '{args.articulation_prim}' is not valid after "
        f"referencing '{args.usd}'. The joint-state/command graph will be built "
        "but will NOT function (expect 'Failed to find articulation')."
    )
else:
    print(
        f"[ur5_isaac_bridge] articulation prim OK: {args.articulation_prim} "
        f"({stage.GetPrimAtPath(args.articulation_prim).GetTypeName()})"
    )
if not _camera_ok:
    carb.log_warn(
        f"Camera prim '{args.camera_prim}' is not valid; camera RGBD publishing "
        "will be skipped (joint/clock bridge still runs)."
    )

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
#   straight at it. Topic names are camera/{color,depth,camera_info} as required
#   by the task spec.
# =====================================================================
if _camera_ok:
    try:
        # frameId = trailing segment of the camera prim path (e.g. "wrist_camera").
        camera_frame_id = args.camera_prim.rstrip("/").split("/")[-1] or "wrist_camera"
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
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(args.camera_prim)]),
                    # Resolution matches camera_periodic.py; adjust here to throttle GPU cost.
                    ("createRenderProduct.inputs:width", 1280),
                    ("createRenderProduct.inputs:height", 720),
                    ("cameraHelperRgb.inputs:frameId", camera_frame_id),
                    ("cameraHelperRgb.inputs:topicName", "camera/color"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", camera_frame_id),
                    ("cameraHelperInfo.inputs:topicName", "camera/camera_info"),
                    ("cameraHelperDepth.inputs:frameId", camera_frame_id),
                    ("cameraHelperDepth.inputs:topicName", "camera/depth"),
                    ("cameraHelperDepth.inputs:type", "depth"),
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

try:
    while simulation_app.is_running():
        # Fixed-step update; the OmniGraph action graph ticks each playback tick.
        simulation_app.update()
except KeyboardInterrupt:
    print("[ur5_isaac_bridge] interrupted by user (KeyboardInterrupt).")
finally:
    app_utils.stop()
    simulation_app.close()
