# `isaac_moveit` — vendored into this workspace (SP1 Task 2)

This documents why and how NVIDIA's official `isaac_moveit` package (and its
sibling `moveit_resources` packages) live inside **our** colcon workspace at
`src/isaac_moveit/` and `src/moveit_resources/`, instead of being built in an
external `~/isaac_ros_ws/IsaacSim-ros_workspaces`.

## Source / version

- Upstream repo: https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
- Branch / tag checked out: **`IsaacSim-6.0.1`**
- Commit SHA: `dd3eeede7912755996a18f4884285d9f50843f79` ("Added changes for
  Isaac Sim 6.0.1 (#24)")
- Sub-path copied: `humble_ws/src/moveit/` (the `isaac_moveit` package plus the
  `moveit_resources` git submodule that sits next to it).

Submodule (also vendored):

- URL: https://github.com/ayushgnv/moveit_resources.git
- Branch: `isaacsim_moveit_tutorial`
- Commit SHA: `b25658851d7a585da1d11a1103eb6ec7cb1ffc80`
- Provides three ROS packages: `moveit_resources` (metapkg),
  `moveit_resources_panda_description` (Franka URDF + meshes),
  `moveit_resources_panda_moveit_config` (MoveIt config + ros2_control xacros).

## Why vendor (not build in the external IsaacSim-ros_workspaces)

Per the user's hard constraint, we do **not** build or write code in an external
`~/isaac_ros_ws/IsaacSim-ros_workspaces`. Instead the official `isaac_moveit`
becomes part of *this* repo, so that:

1. SP1 Task 3 can do `ros2 launch isaac_moveit isaac_moveit.launch.py` to
   validate the official Franka link (with Isaac Sim hardware type), purely
   from our own workspace overlay.
2. SP1 Tasks 4–7 can use `src/isaac_moveit/` (launch file, xacro, gripper
   bridge) as a template when adapting the same approach to UR5.
3. The whole motion bridge is reproducible from a single `git clone` + one
   `colcon build` — no second workspace to maintain, no out-of-tree code.

## How the source was fetched

The official repo was cloned to a **temp** dir only to extract source; nothing
was built or modified there:

```bash
mkdir -p /tmp/isaac_ws_src && cd /tmp/isaac_ws_src
git clone -b IsaacSim-6.0.1 https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
cd IsaacSim-ros_workspaces
git submodule update --init humble_ws/src/moveit/moveit_resources

cp -r humble_ws/src/moveit/isaac_moveit      <repo>/src/
cp -r humble_ws/src/moveit/moveit_resources  <repo>/src/moveit_resources
rm -f <repo>/src/moveit_resources/.git   # drop submodule pointer
```

The `.github/` workflow dir from the submodule fork was removed (not relevant
to a build).

## How to build (in THIS repo)

Every new shell, before invoking `colcon` / `ros2`:

```bash
source /home/gds/isaacsim/setup_ros_env.sh   # Isaac Sim ROS env
source tools/source_env.sh                   # deactivate conda base + source /opt/ros/humble
# sanity: must print /usr/bin/python3 (NOT /opt/miniconda3/...)
which python3
```

Then:

```bash
colcon build --symlink-install --packages-up-to isaac_moveit
source install/setup.bash
ros2 pkg list | grep '^isaac_moveit$'        # -> isaac_moveit
ls install/isaac_moveit/share/isaac_moveit/launch/isaac_moveit.launch.py
```

`--packages-up-to isaac_moveit` pulls in the two `moveit_resources_panda_*`
siblings automatically (they are `package.xml` `<depend>`s of `isaac_moveit`
and are not in `/opt/ros/humble`).

## Dependency notes (what had to be vendored vs. what's already installed)

Checked against `/opt/ros/humble`:

| isaac_moveit `<depend>`              | In /opt/ros/humble? | Vendored?                     |
|--------------------------------------|---------------------|-------------------------------|
| `moveit`                             | yes                 | no                            |
| `robot_state_publisher`              | yes                 | no                            |
| `moveit_resources_panda_moveit_config` | **no**            | **yes** (from submodule)      |
| `topic_based_ros2_control`           | **no**             | **no** — see note below       |
| `ros2_control` / `ros2_controllers`  | metapkg not installed as such; underlying controller_manager / joint_trajectory_controller etc. ARE present | no |
| `moveit_simple_controller_manager`   | yes                 | no                            |
| `visualization_msgs` / `rclpy` / `control_msgs` / `sensor_msgs` | yes | no |
| `launch` / `launch_ros` / `rviz2`    | yes                 | no                            |

Note: `ament_python` does not fail the build when run-time `<depend>`s are
absent, so `isaac_moveit` builds clean even though
`topic_based_ros2_control` is not installed.

### `topic_based_ros2_control` — still missing (Task 3 prerequisite)

The launch file's `ros2_control_hardware_type` defaults to `"isaac"`, which is
the plugin provided by `topic_based_ros2_control` (PickNik Robotics). On this
machine it is **not** in `/opt/ros/humble`, and it is **not** a sibling under
`humble_ws/src/` of the official repo (it only enters as a git submodule in
`jazzy_ws/`). For SP1 Task 2 we only need the build to succeed, so it is not
vendored here. **SP1 Task 3** (official Franka launch validation) will need
this package — either install via apt (`ros-humble-topic-based-ros2-control`,
requires `sudo`), clone https://github.com/PickNikRobotics/topic_based_ros2_control.git
on a Humble-compatible branch into `src/`, or revisit at Task 3 time.

## Launch entry point (for Task 3)

`install/isaac_moveit/share/isaac_moveit/launch/isaac_moveit.launch.py` (symlink
to `src/isaac_moveit/launch/isaac_moveit.launch.py` under `--symlink-install`).

Key launch arguments:
- `ros2_control_hardware_type` (default `isaac`) — `isaac` ⇄
  `topic_based_ros2_control`; `mock_components` ⇄ fake hardware.
- `use_sim_time` (default `true`) — needed because Isaac Sim drives `/clock`.

The launch file uses `MoveItConfigsBuilder("moveit_resources_panda")` and an
absolute-path override `config/panda_isaac.urdf.xacro` (this xacro pulls in the
panda URDF + ros2_control xacro from the vendored `moveit_resources_panda_*`
packages). It also starts a `gripper_to_isaac` bridge node (entry point defined
in `src/isaac_moveit/setup.py`, source in
`src/isaac_moveit/isaac_moveit/gripper_to_isaac.py`) — this is the template
we'll mimic (or reuse) for the UR5 hand.
