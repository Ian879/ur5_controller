# SP1 Isaac Motion Bridge — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Nature of this work:** Integration against Isaac Sim — tasks use **observable verification** (run a command, check the expected output/topic/motion) instead of unit tests, because the behavior ("sim robot moves", "camera publishes") is not unit-testable without the simulator. Tasks marked **[USER GATE]** require you (GUI/scene/teach-pendant-equivalent) and cannot be done headless by a subagent.

**Goal:** Make MoveIt2 drive a UR5 (with wrist camera) inside Isaac Sim via the official `isaac_moveit` path, publishing RGBD — so the same `ur5_motion` code runs on mock/real/isaac.

**Architecture:** Isaac Sim runs a standalone bridge script that loads the user's `.usd`, publishes `/clock` + `/isaac_joint_states` + RGBD, and subscribes `/isaac_joint_commands` to drive the UR5 articulation. The PC runs `ros2_control_node` with the `TopicBasedSystem` hardware plugin (reads/writes those topics) + `ur_moveit_config` move_group. Official-first; custom passthrough bridge only if Humble proves unstable.

**Tech Stack:** ROS2 Humble (system `/opt/ros/humble`), Isaac Sim 6.0.1-rc.7 (`~/isaacsim`), `topic_based_ros2_control`, MoveIt2 + pymoveit2, official `isaac_moveit` (from `isaac-sim/IsaacSim-ros_workspaces`).

## Global Constraints

- **Every** ROS2 command preceded by `source tools/source_env.sh` (deactivates conda base → system `/usr/bin/python3`); conda active breaks the C++ nodes.
- Isaac Sim at `~/isaacsim`; run scripts via `~/isaacsim/python.sh` (after `source ~/isaacsim/setup_ros_env.sh`).
- UR5 planning group `ur_manipulator`, chain `base_link → tool0`, joints `shoulder_pan_joint … wrist_3_joint`.
- No gripper in SP1 (gripper is SP2).
- Isaac↔PC same machine, same RMW (`rmw_fastrtps_cpp`), same `ROS_DOMAIN_ID` (default 0).
- `backend:=isaac` uses `use_sim_time:=true` (Isaac publishes `/clock`).
- Commits require git — Task 1 inits it.

## File Structure

- **External (clone):** `~/isaac_ros_ws/IsaacSim-ros_workspaces/` — official `isaac_moveit` (template + Franka validation). Not part of our repo.
- **Create:** `isaac_scripts/ur5_isaac_bridge.py` — standalone Isaac script (adapted from `~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py`); loads user `.usd`, builds ROS2 bridge graph, publishes wrist-camera RGBD.
- **Create:** `src/ur5_moveit_config/` (ament_cmake) + `urdf/ur5_isaac.ros2_control.xacro` — `sim_isaac` branch using `topic_based_ros2_control/TopicBasedSystem`.
- **Modify:** `src/ur5_bringup/launch/full_stack.launch.py` — implement the `backend:=isaac` branch (currently a stub).
- **Create:** `src/ur5_bringup/launch/sim_isaac.launch.py` — wraps the generic `ros2_control_node` + isaac description + controllers.
- **Reference (read, don't modify):** `~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py`, `~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py`, `~/isaacsim/exts/isaacsim.ros2.examples/isaacsim/ros2/examples/ros_moveit_sample.py`.

---

### Task 1: Initialize git + capture baseline

**Files:**
- Create: `.gitignore`
- Modify: repo root (git init)

**Interfaces:** Produces a versioned baseline so subsequent tasks can commit.

- [ ] **Step 1: init git + gitignore**

```bash
cd /home/gds/projects/shengzhi/ur5_controller
git init -b main 2>/dev/null || git init
cat > .gitignore <<'EOF'
build/
install/
log/
__pycache__/
*.pyc
.tmp/
# external isaac workspace (cloned elsewhere)
isaac_ros_ws/
EOF
```

- [ ] **Step 2: baseline commit**

```bash
git add -A && git commit -m "chore: baseline before SP1 (motion stack mock+real verified)" | tail -1
```
Expected: `[main ...] chore: baseline ...`

- [ ] **Step 3: verify env (no Isaac run yet)**

```bash
source tools/source_env.sh
python3 --version            # expect 3.10.12 (system)
echo "$ROS_DISTRO"           # expect humble
~/isaacsim/python.sh --version 2>&1 | head -1   # expect isaacsim 6.0.1 line
nvidia-smi --query-gpu=name --format=csv,noheader | head -1   # expect RTX PRO 5000
```

- [ ] **Step 4: commit gitignore if changed**

```bash
git add .gitignore && git commit -m "chore: gitignore" 2>/dev/null || echo "nothing to commit"
```

---

### Task 2: Clone + build the official `isaac_moveit` workspace

**Files:**
- External clone: `~/isaac_ros_ws/IsaacSim-ros_workspaces/`

**Interfaces:** Produces the official `isaac_moveit` package (launch + Franka configs) on the PC's ROS path.

- [ ] **Step 1: clone the matching tag**

```bash
mkdir -p ~/isaac_ros_ws && cd ~/isaac_ros_ws
git clone -b IsaacSim-6.0.1 https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
ls IsaacSim-ros_workspaces      # expect humble_ws/ jazzy_ws/ build_ros.sh README.md
```

- [ ] **Step 2: read the build instructions for humble_ws**

```bash
cat ~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws/README.md 2>/dev/null || \
cat ~/isaac_ros_ws/IsaacSim-ros_workspaces/README.md
cat ~/isaac_ros_ws/IsaacSim-ros_workspaces/build_ros.sh
```
Note: the exact build invocation (likely `cd humble_ws && colcon build` after sourcing Isaac Sim's setup). Record it.

- [ ] **Step 3: build the workspace (get isaac_moveit)**

```bash
cd ~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws
source /home/gds/isaacsim/setup_ros_env.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install 2>&1 | tail -15
source install/setup.bash
```
Expected: `Summary: N packages finished` with no failed packages; `isaac_moveit` among them.

- [ ] **Step 4: verify isaac_moveit is on the ROS path**

```bash
source ~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
ros2 pkg list | grep -E "^isaac_moveit$"
ls ~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws/install/isaac_moveit/share/isaac_moveit/launch/ 2>/dev/null
```
Expected: prints `isaac_moveit` and lists `isaac_moveit.launch.py`.

- [ ] **Step 5: commit a pointer note (the clone is gitignored)**

```bash
cd /home/gds/projects/shengzhi/ur5_controller
cat > docs/isaac_moveit_setup.md <<'EOF'
# isaac_moveit (official) workspace
Cloned at ~/isaac_ros_ws/IsaacSim-ros_workspaces (tag IsaacSim-6.0.1), built in humble_ws/.
Source before use:
  source ~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
EOF
git add docs/isaac_moveit_setup.md && git commit -m "docs: how to get official isaac_moveit" | tail -1
```

---

### Task 3: Validate official Franka MoveIt — [USER GATE] + DECISION GATE

**Interfaces:** Produces a verdict: **STABLE** (→ Task 4) or **UNSTABLE** (→ Task 9 fallback). No code.

- [ ] **Step 1: start the official Franka scene in Isaac Sim GUI**

You (GUI): launch Isaac Sim (`~/isaacsim/isaac-sim.sh`), then `Window > Examples > Robotics Examples > Robotics Examples tab > ROS2 > MoveIt > Franka MoveIt` → **Play**.

- [ ] **Step 2: start MoveIt2 on the PC**

```bash
source ~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
ros2 launch isaac_moveit isaac_moveit.launch.py
```
Expected: RViz opens showing the Franka; no fatal errors.

- [ ] **Step 3: Plan & Execute in RViz**

In RViz: Planning Group `hand` → Goal State `open` → **Plan** → **Execute**; then group `panda_arm` → set a goal → **Plan** → **Execute**. The Franka should move in Isaac Sim.

- [ ] **Step 4: record stability verdict**

```bash
# In a third terminal, confirm the bridge topics are alive:
ros2 topic hz /joint_states      # expect steady rate
ros2 topic hz /clock             # expect steady rate
```
Record in `docs/sp1-phase0-verdict.md`: did Plan/Execute succeed consistently on Humble, or did it intermittently fail/timeout (the known Humble issue)?
- **STABLE** → proceed to Task 4.
- **UNSTABLE** → skip to Task 9 (fallback).

```bash
cd /home/gds/projects/shengzhi/ur5_controller
$EDITOR docs/sp1-phase0-verdict.md   # write STABLE or UNSTABLE + notes
git add docs/sp1-phase0-verdict.md && git commit -m "docs: SP1 phase0 verdict" | tail -1
```

---

### Task 4: Document the official `isaac_moveit.launch.py` wiring (reference)

**Files:**
- Create: `docs/sp1-isaac-moveit-wiring-notes.md`

**Interfaces:** Produces the exact facts (hardware plugin, topic names, controllers, moveit config) needed to mirror for UR5 in Tasks 5–6.

- [ ] **Step 1: read the official launch + its moveit/ros2_control config**

```bash
LAUNCH=~/isaac_ros_ws/IsaacSim-ros_workspaces/humble_ws/install/isaac_moveit/share/isaac_moveit/launch/isaac_moveit.launch.py
sed -n '1,200p' "$LAUNCH"
# find the ros2_control / robot_description / controllers it uses; read those files too
```

- [ ] **Step 2: capture the wiring facts into notes**

Write `docs/sp1-isaac-moveit-wiring-notes.md` answering precisely:
- Which `<plugin>` does the ros2_control hardware use? (expected `topic_based_ros2_control/TopicBasedSystem`)
- The topic params (`joint_states_topic`, `joint_commands_topic` — expected `isaac_joint_states` / `isaac_joint_commands`)
- Which controllers are active? (joint_state_broadcaster, joint_trajectory_controller?)
- How does it pass the robot_description + moveit config?
- `use_sim_time`?

```bash
cd /home/gds/projects/shengzhi/ur5_controller
git add docs/sp1-isaac-moveit-wiring-notes.md && git commit -m "docs: isaac_moveit.launch.py wiring notes" | tail -1
```

---

### Task 5: UR5 isaac ros2_control xacro (TopicBasedSystem)

**Files:**
- Create: `src/ur5_moveit_config/package.xml`, `src/ur5_moveit_config/CMakeLists.txt`
- Create: `src/ur5_moveit_config/urdf/ur5_isaac.ros2_control.xacro`
- Test (observable): xacro processes and emits the TopicBasedSystem plugin

**Interfaces:**
- Consumes: Task 4 wiring notes (topic names, plugin).
- Produces: a `sim_isaac`-flagged xacro that swaps the ros2_control hardware to `TopicBasedSystem`.

- [ ] **Step 1: scaffold the package**

```bash
cd /home/gds/projects/shengzhi/ur5_controller
mkdir -p src/ur5_moveit_config/urdf
```
`src/ur5_moveit_config/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>ur5_moveit_config</name>
  <version>0.1.0</version>
  <description>UR5 isaac (TopicBasedSystem) ros2_control xacro overlay.</description>
  <maintainer email="todo@todo.todo">shengzhi</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ur_description</exec_depend>
  <exec_depend>topic_based_ros2_control</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```
`src/ur5_moveit_config/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 2: install topic_based_ros2_control if missing**

```bash
dpkg -s ros-humble-topic-based-ros2-control >/dev/null 2>&1 || \
  echo "NEEDS SUDO: sudo apt install ros-humble-topic-based-ros2-control"
```

- [ ] **Step 3: write the isaac ros2_control xacro**

`src/ur5_moveit_config/urdf/ur5_isaac.ros2_control.xacro` — mirror the structure of `/opt/ros/humble/share/ur_description/urdf/ur.ros2_control.xacro` but replace the `<plugin>` branch. Skeleton (fill topic names from Task 4; defaults match moveit.py convention):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="ur5_isaac">
  <xacro:macro name="ur5_isaac_ros2_control" params="prefix">
    <ros2_control name="ur5_isaac_system" type="system">
      <hardware>
        <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
        <param name="joint_states_topic">isaac_joint_states</param>
        <param name="joint_commands_topic">isaac_joint_commands</param>
      </hardware>
      <xacro:macro name="ur_joint" params="name">
        <joint name="${name}">
          <command_interface>position</command_interface>
          <state_interface>position</state_interface>
          <state_interface>velocity</state_interface>
        </joint>
      </xacro:macro>
      <xacro:ur_joint name="${prefix}shoulder_pan_joint"/>
      <xacro:ur_joint name="${prefix}shoulder_lift_joint"/>
      <xacro:ur_joint name="${prefix}elbow_joint"/>
      <xacro:ur_joint name="${prefix}wrist_1_joint"/>
      <xacro:ur_joint name="${prefix}wrist_2_joint"/>
      <xacro:ur_joint name="${prefix}wrist_3_joint"/>
    </ros2_control>
  </xacro:macro>
</robot>
```

- [ ] **Step 4: build + verify xacro emits the TopicBasedSystem plugin**

```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_moveit_config 2>&1 | tail -3
source install/setup.bash
# Sanity: confirm the xacro is well-formed XML
python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/ur5_moveit_config/urdf/ur5_isaac.ros2_control.xacro'); print('xacro XML OK')"
```
Expected: build OK; `xacro XML OK`. (Full xacro→URDF validation happens in Task 7 once the description wiring exists.)

- [ ] **Step 5: commit**

```bash
git add src/ur5_moveit_config && git commit -m "feat(sp1): ur5 isaac ros2_control xacro (TopicBasedSystem)" | tail -1
```

---

### Task 6: `backend:=isaac` launch (full_stack branch)

**Files:**
- Modify: `src/ur5_bringup/launch/full_stack.launch.py` (the `backend == "isaac"` branch in `_setup`)
- Create: `src/ur5_bringup/launch/sim_isaac.launch.py`

**Interfaces:**
- Consumes: Task 5 xacro; Task 4 wiring notes (controllers, use_sim_time).
- Produces: `ros2 launch ur5_bringup full_stack.launch.py backend:=isaac` brings up `ros2_control_node` (generic, TopicBasedSystem) + `joint_trajectory_controller` + `joint_state_broadcaster` + move_group + RViz, `use_sim_time:=true`.

- [ ] **Step 1: write the isaac bringup launch**

`src/ur5_bringup/launch/sim_isaac.launch.py` — builds the isaac `robot_description` (ur5 urdf + the isaac ros2_control xacro) via xacro, starts `controller_manager`/`ros2_control_node` with `joint_trajectory_controller` + `joint_state_broadcaster`, spawns them. Model it on the structure of `sim_mock.launch.py` + `ur_control.launch.py`'s spawner block, but: `use_fake_hardware:=false` semantics via the TopicBasedSystem xacro (the robot_description must reference `ur5_isaac.ros2_control.xacro`). Generate robot_description with a Command(xacro ...) that composes `ur_macro.xacro` + the isaac ros2_control macro.

- [ ] **Step 2: wire backend:=isaac in full_stack**

In `full_stack.launch.py` `_setup`, the `backend == "isaac"` branch: `actions.append(_bringup_include("sim_isaac.launch.py"))` then `actions.append(_moveit_include("true", launch_rviz))`. (Currently the isaac branch only starts moveit — add the bringup include.)

- [ ] **Step 3: rebuild + verify launch parses**

```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup 2>&1 | tail -2
source install/setup.bash
ros2 launch ur5_bringup full_stack.launch.py --show-args 2>&1 | grep -A1 -iE "backend|launch_rviz" | head
```
Expected: prints args without error (no connection attempted; `--show-args` doesn't run `OpaqueFunction`).

- [ ] **Step 4: commit**

```bash
git add src/ur5_bringup && git commit -m "feat(sp1): backend:=isaac launch (ros2_control + moveit + rviz)" | tail -1
```

---

### Task 7: Isaac bridge script (`ur5_isaac_bridge.py`) — loads your `.usd` + publishes RGBD

**Files:**
- Create: `isaac_scripts/ur5_isaac_bridge.py`

**Reference (read first):**
- `~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py` (the joint-state/clock/commands graph — copy verbatim, retarget to UR5)
- `~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py` (the RTX camera RGBD publish graph — copy the camera nodes)

**Interfaces:**
- Consumes: user's `.usd` (UR5 articulation + wrist camera), prim paths as args.
- Produces: Isaac publishes `/clock`, `/isaac_joint_states`, RGBD `/camera/{color,depth,camera_info}`; subscribes `/isaac_joint_commands`.

- [ ] **Step 1: read the two reference scripts and note the exact OmniGraph nodes**

```bash
sed -n '1,200p' ~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/moveit.py
sed -n '1,220p' ~/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
```
Note: `IsaacReadJointState`/`ROS2PublishJointState`/`ROS2SubscribeJointState`/`IsaacArticulationController`/`ROS2PublishClock` (from moveit.py); the camera publish nodes (from camera_periodic.py — typically `IsaacCreateRenderProduct` + `ROS2CameraHelper` publishing color/depth/camera_info).

- [ ] **Step 2: write the bridge script**

`isaac_scripts/ur5_isaac_bridge.py` — structure (start from `moveit.py`, make these changes):
- `argparse`: `--usd` (user's scene path), `--robot_prim` (default `/UR5`), `--camera_prim` (default `/UR5/wrist_camera`), `--headless`.
- Replace `FRANKA_USD_PATH`/`BACKGROUND_USD_PATH` loading with `stage_utils.add_reference_to_stage(args.usd, args.robot_prim)`.
- Remove Franka variant-set lines.
- In the `og.Controller.edit` graph: keep the joint/clock nodes; set `ArticulationController.inputs:robotPath = args.robot_prim`, `ReadJointState.inputs:prim = [Path(args.robot_prim)]`, `PublishJointState topicName=isaac_joint_states`, `SubscribeJointState topicName=isaac_joint_commands`.
- **Add** the camera publish subgraph (from camera_periodic.py): create a render product on `args.camera_prim`, add `ROS2CameraHelper` nodes publishing `color`, `depth`, `camera_info` to `/camera/...`.
- `CONFIG = {"headless": args.headless, "renderer": "RealTimePathTracing"}`.

- [ ] **Step 3: smoke-test the script loads Isaac + parses args (no scene yet)**

```bash
cd /home/gds/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/ur5_isaac_bridge.py --help 2>&1 | grep -E "usd|robot_prim|headless" | head
```
Expected: argparse help shows the flags (Isaac boots, no crash before help). This validates imports + arg parsing.

- [ ] **Step 4: commit**

```bash
cd /home/gds/projects/shengzhi/ur5_controller
git add isaac_scripts/ur5_isaac_bridge.py && git commit -m "feat(sp1): isaac bridge script (usd + joint bridge + RGBD camera)" | tail -1
```

---

### Task 8: End-to-end headless verification — [needs your `.usd`]

**Interfaces:** The SP1 acceptance test.

- [ ] **Step 1: you provide a minimal `.usd`**

You: in Isaac Sim GUI, save a scene with a UR5 articulation (prim path you pass as `--robot_prim`) + a wrist RTX camera (`--camera_prim`). Note the file path.

- [ ] **Step 2: start the Isaac bridge (headless)**

Terminal A:
```bash
cd /home/gds/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/ur5_isaac_bridge.py \
  --usd <YOUR_SCENE.usd> --robot_prim /UR5 --camera_prim /UR5/wrist_camera --headless
```

- [ ] **Step 3: start the PC stack**

Terminal B:
```bash
cd /home/gds/projects/shengzhi/ur5_controller && source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=isaac launch_rviz:=false
```

- [ ] **Step 4: verify topics + controllers**

Terminal C:
```bash
source tools/source_env.sh
ros2 control list_controllers        # joint_trajectory_controller + joint_state_broadcaster active
ros2 topic hz /joint_states          # steady (from Isaac via TopicBasedSystem)
ros2 topic hz /clock                 # steady
ros2 topic hz /camera/color          # steady
```
Expected: all show a steady rate.

- [ ] **Step 5: drive it — demo_plan_execute**

Terminal C:
```bash
ros2 run ur5_motion demo_plan_execute
```
Expected: each move `[OK]` (or the known PREEMPTED-but-reached pattern); `/isaac_joint_commands` changes; in Isaac (run Task 8 step 2 without `--headless` to watch) the UR5 moves to the targets.

- [ ] **Step 6: commit verification record**

```bash
cd /home/gds/projects/shengzhi/ur5_controller
printf "# SP1 verified\nIsaac UR5 moves under MoveIt; RGBD publishes.\n" > docs/sp1-verified.md
git add docs/sp1-verified.md && git commit -m "docs: SP1 isaac motion bridge verified" | tail -1
```

**SP1 done.** → proceed to SP2 (gripper) when ready.

---

### Task 9: FALLBACK — custom passthrough bridge (ONLY if Task 3 = UNSTABLE)

Skip this task entirely if Task 3 was STABLE.

**Files:**
- Modify: `src/ur5_bringup/launch/sim_isaac.launch.py` (add `motion_iface:=passthrough` switch)
- Modify: `isaac_scripts/ur5_isaac_bridge.py` (subscribe to a full JointTrajectory topic instead of `/isaac_joint_commands`, play it with correct timing)

**Interfaces:** Avoids TopicBasedSystem timestamp jitter by having Isaac consume the whole trajectory.

- [ ] **Step 1: switch the active controller to passthrough_trajectory_controller** in `sim_isaac.launch.py` (when `motion_iface:=passthrough`), and make MoveIt's `controllers.yaml` default to it.
- [ ] **Step 2: in the bridge script**, replace `ROS2SubscribeJointState` + `ArticulationController` with a `ROS2Subscriber` of `sensor_msgs/JointTrajectory` on `/isaac_joint_trajectory`, and drive the articulation along its waypoints with Isaac sim time.
- [ ] **Step 3: re-run Task 8 steps 2–5; confirm smoother following** (no timestamp-induced jitter).
- [ ] **Step 4: commit**

```bash
git add -A && git commit -m "feat(sp1): passthrough bridge fallback (TopicBasedSystem unstable on Humble)" | tail -1
```

---

## Notes for the implementer

- Tasks 2–3 depend on a network clone + a GUI run; do them with the user present.
- The `.usd` scene (Task 8) is the user's deliverable; SP1 can't fully verify until it exists. Tasks 1–7 can proceed and be smoke-tested without it (Task 7 step 3 validates the script parses).
- If `topic_based_ros2_control` isn't installed (Task 5 step 2), it needs `sudo apt install ros-humble-topic-based-ros2-control` (user runs it via `!`).
