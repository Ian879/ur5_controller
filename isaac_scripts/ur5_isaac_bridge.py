#!/usr/bin/env python3
"""Isaac Sim -> MoveIt2 bridge for the UR5 (SP1 Task 7).

A standalone Isaac Sim script that turns a user-provided UR5 ``.usd`` scene into
a MoveIt2-driveable simulation:

  * publishes ``/clock`` (so the ROS2 side can run ``use_sim_time:=true``);
  * publishes ``/isaac_joint_states`` (read by the TopicBasedSystem hardware
    plugin wired in SP1 Task 5/6);
  * subscribes ``/isaac_joint_commands`` (written by TopicBasedSystem) and drives
    the UR5 articulation via ``IsaacArticulationController``;
  * publishes wrist-camera RGB + depth + camera_info
    (``/camera/color``, ``/camera/depth``, ``/camera/camera_info``).

This is the UR5-scene equivalent of Isaac's bundled Franka ``moveit.py``, plus
the RTX camera graph from ``camera_periodic.py``. The OmniGraph node wiring is
copied *verbatim* from those two references and retargeted to ``--robot_prim`` /
``--camera_prim``. See:

  * ``/home/gds/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py``
  * ``/home/gds/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py``

Run with Isaac's bundled interpreter::

    ~/isaacsim/python.sh isaac_scripts/ur5_isaac_bridge.py \
        --usd /path/to/ur5_scene.usd \
        [--robot_prim /UR5] \
        [--camera_prim /UR5/wrist_camera] \
        [--headless]

The topic names ``isaac_joint_states`` / ``isaac_joint_commands`` are REQUIRED:
they are the names T5/T6's TopicBasedSystem reads/writes (matching the official
Franka ``isaac_moveit`` wiring). Do not change them.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

# argparse MUST be declared before constructing SimulationApp: the app installs
# its own argparse machinery on import. This mirrors moveit.py / camera_periodic.
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--usd",
    required=True,
    help="Path to the user's UR5 scene .usd to load (the robot + wrist camera).",
)
parser.add_argument(
    "--robot_prim",
    default="/UR5",
    help="Articulation root prim path the .usd is referenced under. Default: /UR5.",
)
parser.add_argument(
    "--camera_prim",
    default="/UR5/wrist_camera",
    help="Wrist camera prim path for RGBD publishing. Default: /UR5/wrist_camera.",
)
parser.add_argument(
    "--headless",
    default=False,
    action="store_true",
    help="Run Isaac Sim headless (no viewport window).",
)
args, _ = parser.parse_known_args()

# Fail fast on a bad --usd path: booting Isaac takes ~30s, so checking now saves
# the user from waiting just to see a load failure later.
if not os.path.isfile(args.usd):
    print(f"ERROR: --usd path not found or not a file: {args.usd}", file=sys.stderr)
    sys.exit(1)

ACTION_GRAPH_PATH = "/ActionGraph"
ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"

simulation_app = SimulationApp({"headless": args.headless, "renderer": "RealTimePathTracing"})

# --- Isaac imports (only valid after SimulationApp is constructed) ---
import carb  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni  # noqa: E402
import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

# Enable the ROS2 bridge extension and let it settle for one frame.
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

stage_utils.set_stage_units(meters_per_unit=1.0)

print(
    f"[ur5_isaac_bridge] usd={args.usd} robot_prim={args.robot_prim} "
    f"camera_prim={args.camera_prim} headless={args.headless}"
)

# --- Load the user's scene ---
# Unlike moveit.py (which loads Franka + simple_room from Isaac assets), we load
# ONLY the user-provided .usd, referenced under args.robot_prim. The wrist
# camera is expected to already live in that .usd at args.camera_prim.
stage_utils.add_reference_to_stage(args.usd, args.robot_prim)
simulation_app.update()

# Sanity-check that the prims we wire into the OmniGraph actually exist in the
# loaded stage. These are warnings (not fatal) so the user can still bring up a
# partial bridge while debugging their scene.
stage = omni.usd.get_context().get_stage()
_robot_ok = bool(stage.GetPrimAtPath(args.robot_prim).IsValid())
_camera_ok = bool(stage.GetPrimAtPath(args.camera_prim).IsValid())
if not _robot_ok:
    carb.log_warn(
        f"Robot prim '{args.robot_prim}' is not valid after loading '{args.usd}'. "
        "The joint-state/command graph will be built but will not function."
    )
if not _camera_ok:
    carb.log_warn(
        f"Camera prim '{args.camera_prim}' is not valid; camera RGBD publishing "
        "will be skipped (joint/clock bridge still runs)."
    )

# =====================================================================
# Action graph: /clock + /isaac_joint_states pub + /isaac_joint_commands sub
#   (copied verbatim from moveit.py; retargeted robotPath / ReadJointState.prim
#    to args.robot_prim, and pinned the two required isaac topic names).
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
                # Point the articulation controller + joint reader at our UR5 prim.
                ("ArticulationController.inputs:robotPath", args.robot_prim),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(args.robot_prim)]),
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
#   Deviation: camera_periodic.py *creates* a Camera prim via DefinePrim; here
#   the camera already lives in the user's USD at args.camera_prim, so we skip
#   creation and point IsaacCreateRenderProduct straight at it. Topic names are
#   set to camera/{color,depth,camera_info} as required by the task spec.
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
