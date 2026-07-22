# Lebai 夹爪集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 Lebai gripper 完整集成进 UR5 MoveIt 栈（real + isaac）：URDF+SRDF+TCP、real 走 lebai_driver 串口、isaac 走桥驱动 articulation + action 中转节点，并修正 collider 导出排除机器人挂载 prim。

**Architecture:** 一个 `ur5_with_gripper.urdf.xacro`（ur_macro + lebai gripper_macro + tool0→gripper_base fixed joint）三后端共用；自己的 SRDF xacro（include ur srdf + 加 gripper group/TCP/open-closed）；moveit.launch.py 注入 GripperCommand 句柄（action 名 `lebai_gripper_controller/gripper_cmd`，real+isaac 共用，各后端起对应 action 提供节点）；isaac 侧桥扩展读写 gripper articulation + `isaac_gripper_action` rclpy 节点中转 GripperCommand。

**Tech Stack:** ROS2 Humble, MoveIt2, lebai-ros-sdk（submodule in-tree）, ur_description/ur_moveit_config（system）, isaacsim 6.0.1 ROS2 bridge (OmniGraph), xacro.

**Spec:** `docs/superpowers/specs/2026-07-22-gripper-integration-design.md`

## Global Constraints

- **conda 陷阱**：所有 ROS2/colcon 命令前 `source tools/source_env.sh`。
- **lebai 已 in-tree**：`src/third_party/lebai-ros-sdk`（submodule, humble-dev），`$(find lebai_lm3_support)` / `$(find lebai_driver)` 直接可用，**不需 overlay**。
- **gripper 1-DOF**：驱动关节 `gripper_r_joint1`（其余 mimic），TCP link = `gripper_tip`，open=π/3(1.0472)、closed=0.0。
- **GripperCommand action 名固定** `lebai_gripper_controller/gripper_cmd`（real=lebai_driver 提供，isaac=我们的 `isaac_gripper_action` 节点提供，同名）。
- **gripper_macro** 在 `lebai_lm3_support/urdf/gripper_macro.xacro`，macro 名 `lebai_gripper`，参数 `prefix`。
- **mount**：`tool0 → gripper_base_link` fixed joint。LM3 用 identity origin；本实现先用 identity，verify 时对照 RViz 与 scene 调。
- **branch**：`main`（所有工作在 main）。
- 验证最轻用 **mock**（gripper 无驱动但 URDF 可见）或 **real**（serial_gripper）；isaac 验证需桥 + 场景。

---

## File Structure

| 文件 | 责任 | 动作 |
|---|---|---|
| `ur5_bringup/urdf/ur5_with_gripper.urdf.xacro` | ur_macro + gripper_macro + tool0→gripper_base fixed | 新建 |
| `ur5_bringup/srdf/ur5_with_gripper.srdf.xacro` | include ur srdf + gripper group/TCP/open-closed/disable_collisions | 新建 |
| `ur5_bringup/launch/moveit.launch.py` | description_file + moveit_config_file 指向我们的；注入 gripper GripperCommand 句柄 | 改 |
| `ur5_bringup/launch/real_robot.launch.py` | 给 ur_control 传 description_file（带 gripper） | 改 |
| `ur5_bringup/launch/full_stack.launch.py` | real 分支起 serial_gripper 节点 | 改 |
| `ur5_bringup/urdf/ur5_isaac.urdf.xacro` | include gripper_macro + tool0→gripper fixed | 改 |
| `ur5_bringup/launch/sim_isaac.launch.py` | 起 isaac_gripper_action 节点 | 改 |
| `ur5_bringup/config/ur5_isaac_controllers.yaml` | （moveit 句柄在 moveit.launch.py 注入，此文件可能不动；如需 isaac 专用则加） | 视情况 |
| `ur5_motion/ur5_motion/isaac_gripper_action.py` | GripperCommand action → /isaac_gripper_joint_commands 中转 | 新建 |
| `ur5_motion/setup.py` | 加 isaac_gripper entry_point | 改 |
| `isaac_scripts/ur5_isaac_bridge.py` | OmniGraph 加 gripper articulation 读写 | 改 |
| `isaac_scripts/export_scene_collisions.py` | 排除 robot_prim 子 prim + 显式名单 | 改 |
| `ur5_bringup/package.xml` | 加 lebai_lm3_support / lebai_driver exec_depend | 改 |
| `ur5_bringup/CMakeLists.txt` | install srdf 目录（若新增 srdf/） | 改 |

---

### Task 1: URDF xacro（gripper 进 robot_description）

**Files:**
- Create: `src/ur5_bringup/urdf/ur5_with_gripper.urdf.xacro`
- Modify: `src/ur5_bringup/package.xml`（加 exec_depend）
- Modify: `src/ur5_bringup/CMakeLists.txt`（确认 urdf 目录已 install——已有 `install(DIRECTORY urdf ...)`）

**Interfaces:**
- Produces: `ur5_with_gripper.urdf.xacro`，被 Task 3（moveit.launch.py description_file）+ Task 4（real_robot description_file）+ Task 5（isaac ur5_isaac.urdf.xacro include）使用。

- [ ] **Step 1: 写 `src/ur5_bringup/urdf/ur5_with_gripper.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="ur5_with_gripper">
  <!-- UR5 arm（同 ur_description 的 ur_macro，参数透传）-->
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
  <!-- Lebai gripper（lebai_lm3_support，submodule in-tree）-->
  <xacro:include filename="$(find lebai_lm3_support)/urdf/gripper_macro.xacro"/>

  <xacro:arg name="ur_type" default="ur5"/>
  <xacro:arg name="joint_limit_params" default="$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml"/>
  <xacro:arg name="kinematics_params" default="$(find ur_description)/config/$(arg ur_type)/default_kinematics.yaml"/>
  <xacro:arg name="physical_params" default="$(find ur_description)/config/$(arg ur_type)/physical_parameters.yaml"/>
  <xacro:arg name="visual_params" default="$(find ur_description)/config/$(arg ur_type)/visual_parameters.yaml"/>
  <xacro:arg name="safety_limits" default="true"/>
  <xacro:arg name="safety_pos_margin" default="0.15"/>
  <xacro:arg name="safety_k_position" default="20"/>
  <xacro:arg name="prefix" default=""/>

  <xacro:ur name="ur" prefix="$(arg prefix)" ur_type="$(arg ur_type)"
            joint_limit_params="$(arg joint_limit_params)"
            kinematics_params="$(arg kinematics_params)"
            physical_params="$(arg physical_params)"
            visual_params="$(arg visual_params)"
            safety_limits="$(arg safety_limits)"
            safety_pos_margin="$(arg safety_pos_margin)"
            safety_k_position="$(arg safety_k_position)"/>

  <xacro:lebai_gripper prefix="$(arg prefix)"/>

  <!-- gripper 挂到 tool0（LM3 范式：identity origin；若物理有转接件 offset，verify 时调）-->
  <joint name="tool0_to_gripper_base" type="fixed">
    <parent link="$(arg prefix)tool0"/>
    <child link="$(arg prefix)gripper_base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```

- [ ] **Step 2: `package.xml` 加 lebai exec_depend**

在 `src/ur5_bringup/package.xml` 现有 `<exec_depend>pilz_industrial_motion_planner</exec_depend>` 之后加：
```xml
  <exec_depend>lebai_lm3_support</exec_depend>
  <exec_depend>lebai_driver</exec_depend>
```

- [ ] **Step 3: xacro 生成 + check_urdf 验证链**

Run:
```bash
source tools/source_env.sh
xacro $(ros2 pkg prefix ur5_bringup)/share/ur5_bringup/urdf/ur5_with_gripper.urdf.xacro ur_type:=ur5 > /tmp/ur5_gripper.urdf 2>&1 && echo "xacro OK"
check_urdf /tmp/ur5_gripper.urdf 2>&1 | grep -E "gripper_base_link|gripper_tip|gripper_r_joint1|tool0|Multi-root" | head
```
Expected: `xacro OK`；check_urdf 显示 `tool0 → gripper_base_link → … → gripper_tip` 链（含 gripper_r_joint1），无 multi-root 错误。若 xacro 报 `find lebai_lm3_support` 失败 → 确认 `ros2 pkg list | grep lebai_lm3_support` 有（rebuild ur5_bringup）。

- [ ] **Step 4: build + 提交**

```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup
git add src/ur5_bringup/urdf/ur5_with_gripper.urdf.xacro src/ur5_bringup/package.xml
git commit -m "feat(gripper): ur5_with_gripper.urdf.xacro（ur_macro + lebai gripper_macro + tool0→gripper_base）+ lebai exec_depend"
```

---

### Task 2: SRDF xacro（gripper group + TCP）

**Files:**
- Create: `src/ur5_bringup/srdf/ur5_with_gripper.srdf.xacro`
- Modify: `src/ur5_bringup/CMakeLists.txt`（install srdf 目录）

**Interfaces:**
- Produces: `ur5_with_gripper.srdf.xacro`，被 Task 3（moveit.launch.py moveit_config_file）使用。gripper group（gripper_r_joint1）+ end_effector + open/closed。

- [ ] **Step 1: 探查 ur_moveit 的 ur.srdf.xacro 能否被 xacro-include**

Run:
```bash
source tools/source_env.sh
head -20 $(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/srdf/ur.srdf.xacro
```
看其是否是 `<robot>` 根 + 能被 include。若能 include → Step 2 用 include 法；若不能（非 xacro 或结构特殊）→ Step 2 用"复制内容"法（把 ur.srdf.xacro 全文复制进我们的再追加 gripper 段）。下文 Step 2 给 include 法，若失败改复制法。

- [ ] **Step 2: 写 `src/ur5_bringup/srdf/ur5_with_gripper.srdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="ur5">
  <!-- include 系统 ur_moveit 的 SRDF（ur_manipulator group、disable_collisions 等）-->
  <xacro:include filename="$(find ur_moveit_config)/srdf/ur.srdf.xacro"/>

  <!-- ===== Lebai gripper（照搬 lebai_lm3_moveit_config/config/lebai_lm3.srdf 的 gripper 段）===== -->
  <group name="gripper">
    <joint name="gripper_r_joint1"/>
  </group>
  <end_effector name="gripper" parent_link="tool0" group="gripper" parent_group="ur_manipulator"/>
  <group_state name="open" group="gripper">
    <joint name="gripper_r_joint1" value="1.0471975512"/>
  </group_state>
  <group_state name="closed" group="gripper">
    <joint name="gripper_r_joint1" value="0.0"/>
  </group_state>

  <!-- disable_collisions：照搬 lebai_lm3.srdf 里 gripper 那一组（gripper_base_link vs 各 finger/link）。
       实现时从 lebai_lm3_moveit_config/config/lebai_lm3.srdf 里 grep 'gripper' 的 disable_collisions 行复制过来。 -->
  <disable_collisions link1="tool0" link2="gripper_base_link" reason="Adjacent"/>
  <disable_collisions link1="gripper_base_link" link2="gripper_l_link1" reason="Never"/>
  <disable_collisions link1="gripper_base_link" link2="gripper_l_link2" reason="Never"/>
  <disable_collisions link1="gripper_base_link" link2="gripper_l_link_finger" reason="Never"/>
  <disable_collisions link1="gripper_base_link" link2="gripper_r_link1" reason="Never"/>
  <disable_collisions link1="gripper_base_link" link2="gripper_r_link2" reason="Never"/>
  <disable_collisions link1="gripper_base_link" link2="gripper_r_link_finger" reason="Never"/>
  <disable_collisions link1="gripper_l_link1" link2="gripper_l_link2" reason="Never"/>
  <disable_collisions link1="gripper_l_link1" link2="gripper_l_link_finger" reason="Never"/>
  <disable_collisions link1="gripper_l_link1" link2="gripper_r_link1" reason="Never"/>
  <disable_collisions link1="gripper_l_link1" link2="gripper_r_link2" reason="Never"/>
  <disable_collisions link1="gripper_l_link1" link2="gripper_r_link_finger" reason="Never"/>
  <disable_collisions link1="gripper_l_link2" link2="gripper_l_link_finger" reason="Never"/>
  <disable_collisions link1="gripper_l_link2" link2="gripper_r_link1" reason="Never"/>
  <disable_collisions link1="gripper_l_link2" link2="gripper_r_link2" reason="Never"/>
</robot>
```
> 实现时把 lebai_lm3.srdf 里所有含 `gripper` 的 `<disable_collisions>` 行完整复制（上面是子集示例；以源文件为准补全）。

- [ ] **Step 3: CMakeLists install srdf 目录**

`src/ur5_bringup/CMakeLists.txt` 加（在 `install(DIRECTORY urdf ...)` 附近）：
```cmake
install(DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
```

- [ ] **Step 4: xacro 生成 srdf 验证**

Run:
```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup
xacro $(ros2 pkg prefix ur5_bringup)/share/ur5_bringup/srdf/ur5_with_gripper.srdf.xacro name:=ur ur_type:=ur5 prefix:=\"\" > /tmp/ur5_gripper.srdf 2>&1 && echo "xacro OK"
grep -E "group name=.gripper|end_effector|group_state name=.open|group_state name=.closed" /tmp/ur5_gripper.srdf | head
```
Expected: `xacro OK`；srdf 含 `gripper` group + end_effector + open/closed。若 include ur.srdf.xacro 报错 → 改用复制法（把 ur.srdf.xacro 全文复制进我们的 srdf 再追加 gripper 段）。

- [ ] **Step 5: 提交**

```bash
git add src/ur5_bringup/srdf/ur5_with_gripper.srdf.xacro src/ur5_bringup/CMakeLists.txt
git commit -m "feat(gripper): ur5_with_gripper.srdf.xacro（include ur srdf + gripper group/TCP/open-closed）"
```

---

### Task 3: moveit.launch.py 接 gripper（description_file + srdf + GripperCommand 句柄）

**Files:**
- Modify: `src/ur5_bringup/launch/moveit.launch.py`

**Interfaces:**
- Consumes: Task 1 的 `ur5_with_gripper.urdf.xacro` + Task 2 的 `ur5_with_gripper.srdf.xacro`。
- Produces: move_group 用带 gripper 的 robot_description + srdf（gripper group/TCP）+ moveit_controllers 注入 GripperCommand 句柄（action 名 `lebai_gripper_controller/gripper_cmd`，real+isaac 共用）。

- [ ] **Step 1: 改 robot_description 用我们的 xacro**

在 `moveit.launch.py` 的 `robot_description_content = Command([xacro, ...])`，把 xacro 文件参数从 `ur_description/urdf/ur.urdf.xacro` 改成 `ur5_bringup/urdf/ur5_with_gripper.urdf.xacro`。即 `PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"])` → `PathJoinSubstitution([FindPackageShare("ur5_bringup"), "urdf", "ur5_with_gripper.urdf.xacro"])`。

- [ ] **Step 2: 改 SRDF（moveit_config_file）用我们的**

`robot_description_semantic_content = Command([xacro, ..., srdf/ur.srdf.xacro, name:=ur, ...])` 里，把 srdf 路径从 `ur_moveit_config/srdf/ur.srdf.xacro` 改成 `ur5_bringup/srdf/ur5_with_gripper.srdf.xacro`。即 `PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file])` — 把 `moveit_config_package` 默认值或传入改成 `ur5_bringup`、`moveit_config_file` 默认值改成 `ur5_with_gripper.srdf.xacro`。最简：直接在 PathJoinSubstitution 里写死 `FindPackageShare("ur5_bringup")` + `"srdf"` + `"ur5_with_gripper.srdf.xacro"`（仿 Task 3-pilz 的 description_file 改法）。

- [ ] **Step 3: 注入 GripperCommand 句柄到 moveit_controllers**

在 moveit.launch.py 加载 `controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")` 之后、`moveit_controllers = {...}` 之前，注入 gripper 句柄：
```python
    # 注入 Lebai gripper 的 GripperCommand 句柄（action 名固定，real(serial_gripper) 与
    # isaac(isaac_gripper_action) 各后端起对应 action 提供节点，共用此句柄）
    controllers_yaml.setdefault("lebai_gripper_controller", {
        "action_ns": "lebai_gripper_controller/gripper_cmd",
        "type": "GripperCommand",
        "default": True,
        "joints": ["gripper_r_joint1"],
    })
```

- [ ] **Step 4: 起栈验证（mock，gripper 无驱动但 URDF/SRDF 可见）**

Run:
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=mock
# 另一终端：
ros2 param get /move_group robot_description_semantic 2>&1 | grep -i gripper | head
ros2 run tf2_ros tf2_echo tool0 gripper_tip
```
Expected: RViz 里机械臂 flange 下挂了 gripper（base/fingers/tip）；`tf2_echo tool0 gripper_tip` 解析到（TF 链通）；SRDF 含 gripper group。Ctrl+C。
（mock 下 gripper 无驱动关节发布——gripper_r_joint1 的 TF 可能停在 0，正常；URDF/SRDF/gripper_tip TF 通即此任务成功。）

- [ ] **Step 5: build + 提交**

```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup
git add src/ur5_bringup/launch/moveit.launch.py
git commit -m "feat(gripper): moveit.launch.py 接 ur5_with_gripper xacro+srdf + 注入 GripperCommand 句柄"
```

---

### Task 4: real 后端 gripper（serial_gripper + description_file）

**Files:**
- Modify: `src/ur5_bringup/launch/real_robot.launch.py`（传 description_file）
- Modify: `src/ur5_bringup/launch/full_stack.launch.py`（real 分支起 serial_gripper 节点）

**Interfaces:**
- Consumes: Task 1（ur5_with_gripper.urdf.xacro）+ Task 3（moveit GripperCommand 句柄）。
- Produces: real 后端 robot_description 含 gripper + serial_gripper 节点提供 GripperCommand action。

- [ ] **Step 1: real_robot.launch.py 传 description_file**

在 `real_robot.launch.py` 给 `ur_control.launch.py` 的 `launch_arguments` 加 `"description_file": "ur5_with_gripper.urdf.xacro"`（ur_control 支持该 arg）。若 real_robot 用 `_bringup_include("real_robot.launch.py", {"robot_ip":...})` 模式，则在 real_robot.launch.py 内部 IncludeLaunchDescription ur_control 的 args 加 description_file。

- [ ] **Step 2: full_stack real 分支起 serial_gripper 节点**

在 `full_stack.launch.py` 的 `_setup` real 分支，起 lebai serial_gripper 节点（用 lebai_driver 的 launch 或直接 Node）：
```python
        # real 后端：Lebai 夹爪串口驱动，提供 GripperCommand action
        actions.append(Node(
            package="lebai_driver",
            executable="serial_gripper",
            name="lebai_serial_gripper",
            parameters=[{"port_name": "/dev/ttyUSB0"}],
            output="screen",
        ))
```
（端口 /dev/ttyUSB0 按实际改；可加 ` DeclareLaunchArgument` 让它可配。）

- [ ] **Step 3: build + 验证（需真夹爪串口）**

Run:
```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup
ros2 launch ur5_bringup full_stack.launch.py backend:=real
# 另一终端：
ros2 action list | grep gripper   # 应有 /lebai_gripper_controller/gripper_cmd
```
Expected: `ros2 action list` 含 `/lebai_gripper_controller/gripper_cmd`；RViz 里夹爪挂 flange。MoveIt 规划 gripper open/closed（MotionPlanning → Planning Group 选 gripper → Plan）→ 真夹爪开合。（需真夹爪连串口；没接硬件时 action list 有即可。）

- [ ] **Step 4: 提交**

```bash
git add src/ur5_bringup/launch/real_robot.launch.py src/ur5_bringup/launch/full_stack.launch.py
git commit -m "feat(gripper): real 后端——ur_control description_file 带 gripper + 起 serial_gripper 节点"
```

---

### Task 5: isaac gripper（桥扩展 + isaac_gripper_action 节点）

**Files:**
- Create: `src/ur5_motion/ur5_motion/isaac_gripper_action.py`
- Modify: `src/ur5_motion/setup.py`（entry_point）
- Modify: `src/ur5_bringup/urdf/ur5_isaac.urdf.xacro`（include gripper_macro + fixed joint）
- Modify: `src/ur5_bringup/launch/sim_isaac.launch.py`（起 isaac_gripper_action）
- Modify: `isaac_scripts/ur5_isaac_bridge.py`（OmniGraph 加 gripper articulation 读写）

**Interfaces:**
- Consumes: Task 1（gripper_macro）+ Task 3（moveit GripperCommand 句柄）。
- Produces: isaac 后端 gripper：桥发 `/isaac_gripper_joint_states`、订 `/isaac_gripper_joint_commands` 驱动 gripper articulation；`isaac_gripper_action` 节点暴露 GripperCommand action（同 `lebai_gripper_controller/gripper_cmd`）中转。

- [ ] **Step 1: `ur5_isaac.urdf.xacro` 加 gripper**

在 `ur5_isaac.urdf.xacro` 里 include gripper_macro + 加 fixed joint（同 Task 1）：
```xml
  <xacro:include filename="$(find lebai_lm3_support)/urdf/gripper_macro.xacro"/>
  <xacro:lebai_gripper prefix=""/>
  <joint name="tool0_to_gripper_base" type="fixed">
    <parent link="tool0"/><child link="gripper_base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
```

- [ ] **Step 2: 写 `isaac_gripper_action.py`（GripperCommand 中转节点）**

```python
"""isaac 后端 gripper 中转：暴露 control_msgs/GripperCommand action（名
lebai_gripper_controller/gripper_cmd，与 real 的 lebai_driver 同名），goal→写
/isaac_gripper_joint_commands（gripper_r_joint1 目标），到位（读
/isaac_gripper_joint_states）后 result。让 MoveIt 的 GripperCommand 句柄能控 isaac 夹爪。"""
import math, threading
import rclpy
from rclpy import ActionServer, GoalResponse
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64  # 若用 Float64 命令；否则 JointState

DRIVEN_JOINT = "gripper_r_joint1"
MAX_POS = math.pi / 3.0  # open
CMD_TOPIC = "/isaac_gripper_joint_commands"
STATE_TOPIC = "/isaac_gripper_joint_states"
TOL = 0.05


def main():
    rclpy.init()
    node = rclpy.create_node("isaac_gripper_action")
    node.get_logger().info("isaac_gripper_action: 暴露 GripperCommand action "
                           "lebai_gripper_controller/gripper_cmd")
    pub = node.create_publisher(JointState, CMD_TOPIC, 10)
    state = {"pos": 0.0}

    def on_state(msg):
        try:
            i = msg.name.index(DRIVEN_JOINT)
            state["pos"] = msg.position[i]
        except ValueError:
            pass
    node.create_subscription(JointState, STATE_TOPIC, on_state, 10)

    def execute(goal):
        target = max(0.0, min(MAX_POS, goal.command.position))
        js = JointState()
        js.name = [DRIVEN_JOINT]
        js.position = [target]
        r = rclpy.Duration()
        feedback = GripperCommand.Feedback()
        feedback.position = target
        while rclpy.ok():
            pub.publish(js)
            feedback.position = state["pos"]
            goal.publish_feedback(feedback)
            if abs(state["pos"] - target) < TOL:
                break
            rclpy.spin_once(node, timeout_sec=0.05) if False else None
        result = GripperCommand.Result()
        result.position = state["pos"]
        result.reached_goal = abs(state["pos"] - target) < TOL
        return result

    ActionServer(node, GripperCommand, "lebai_gripper_controller/gripper_cmd",
                 execute_callback=execute,
                 goal_callback=lambda gh: GoalResponse.ACCEPT)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
> 实现时：命令 topic 用 JointState（与桥的 SubscribeJointState 对齐）；到位轮询读 state。若桥的 gripper 命令订阅格式不同（如 Float64），调整 publisher 类型。gripper 关节名若 scene 里不叫 `gripper_r_joint1`，按实际映射。

- [ ] **Step 3: setup.py 加 entry_point**

`src/ur5_motion/setup.py` 的 entry_points 加：
```python
            "isaac_gripper_action = ur5_motion.isaac_gripper_action:main",
```

- [ ] **Step 4: sim_isaac.launch.py 起 isaac_gripper_action 节点**

在 sim_isaac.launch.py 的 LaunchDescription 加：
```python
    Node(
        package="ur5_motion", executable="isaac_gripper_action",
        name="isaac_gripper_action", output="screen",
    ),
```

- [ ] **Step 5: 桥 OmniGraph 加 gripper articulation 读写**

在 `isaac_scripts/ur5_isaac_bridge.py` 的 ActionGraph `og.Controller.edit` 块（现有 UR5 joint graph，~line 262-315），**追加** gripper 的 4 个节点 + 连接 + topic。仿照 UR5 的 ReadJointState/PublishJointState/SubscribeJointState/ArticulationController，但节点名加 `G` 后缀、prim 指向 gripper articulation、topic 用 gripper 的：

在 `CREATE_NODES` 列表追加：
```python
                ("ReadJointStateG", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PublishJointStateG", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointStateG", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationControllerG", "isaacsim.core.nodes.IsaacArticulationController"),
```
在 `CONNECT` 列表追加（仿 UR5 的连接，节点名换 G 后缀）：
```python
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
                ("SubscribeJointStateG.outputs:positionCommand", "ArticulationControllerG.inputs:positionCommand"),
                ("SubscribeJointStateG.outputs:velocityCommand", "ArticulationControllerG.inputs:velocityCommand"),
                ("SubscribeJointStateG.outputs:effortCommand", "ArticulationControllerG.inputs:effortCommand"),
```
在 `SET_VALUES` 列表追加（gripper articulation prim + topic 名）：
```python
                ("ArticulationControllerG.inputs:robotPath", "/World/gripper_abs_stl/Geometry"),
                ("ReadJointStateG.inputs:prim", [usdrt.Sdf.Path("/World/gripper_abs_stl/Geometry")]),
                ("PublishJointStateG.inputs:topicName", "isaac_gripper_joint_states"),
                ("SubscribeJointStateG.inputs:topicName", "isaac_gripper_joint_commands"),
```
> gripper articulation prim：export 报的 articulation root 是 `/World/gripper_abs_stl/Geometry`。若场景里 gripper 挂在 robot 下或 prim 路径不同，按 `--robot_prim` 推导或加 `--gripper_prim` arg。gripper 关节名（scene 里）可能不叫 `gripper_r_joint1`——`isaac_gripper_action` 按 scene 实际关节名映射。

- [ ] **Step 6: 验证（isaac）**

终端 A（桥，带 gripper 扩展）：
```bash
cd ~/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/ur5_isaac_bridge.py --stream --robot_prim /World/ur5
```
终端 B：
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=isaac
# 另一终端：
ros2 topic hz /isaac_gripper_joint_states   # 桥在发 gripper 状态
ros2 action list | grep gripper             # isaac_gripper_action 的 action 在
```
MoveIt（RViz）→ Planning Group 选 gripper → Plan open/closed → Execute → Isaac 里夹爪开合。
Expected: `/isaac_gripper_joint_states` 有数据；gripper action 在；MoveIt 发命令 → Isaac 夹爪动。

- [ ] **Step 7: build + 提交**

```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_motion ur5_bringup
git add src/ur5_motion/ur5_motion/isaac_gripper_action.py src/ur5_motion/setup.py \
        src/ur5_bringup/urdf/ur5_isaac.urdf.xacro src/ur5_bringup/launch/sim_isaac.launch.py \
        isaac_scripts/ur5_isaac_bridge.py
git commit -m "feat(gripper): isaac 后端——桥扩展 gripper articulation 读写 + isaac_gripper_action 中转 + ur5_isaac xacro"
```

---

### Task 6: collider 修正（排除机器人挂载 prim）

**Files:**
- Modify: `isaac_scripts/export_scene_collisions.py`

**Interfaces:**
- Produces: `scene_collisions.json` 只剩真静态环境（table、ur5_elevate_base），无相机机身/夹爪。

- [ ] **Step 1: 改 export 排除 robot_prim 子 prim + 显式名单**

在 `export_scene_collisions.py` 的 collider 遍历循环里（`for ... in ...` 带 `if not prim.HasAPI(UsdPhysics.CollisionAPI): continue` 那段），加排除逻辑。在现有 articulation 排除之后加：
```python
    # 排除 robot_prim 子 prim（腕上相机、gripper 等机器人挂载件，不当静态障碍）
    ROBOT_PRIM = "/World/ur5"  # 与桥 --robot_prim 默认一致；可参数化
    EXCLUDE_NAMES = ("gemini2_body", "back_cover", "solid", "camera_rgb", "camera_ldm",
                     "camera_ir_left", "camera_ir_right", "gripper_abs_stl")
    path_name = path.split("/")[-1]
    if path.startswith(ROBOT_PRIM + "/") or path_name in EXCLUDE_NAMES or "gripper" in path.lower():
        continue
```
（放在 `if not prim.HasAPI(...): continue` 之后、ComputeWorldBound 之前。）

- [ ] **Step 2: 重跑 export 验证**

Run:
```bash
cd ~/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/export_scene_collisions.py 2>&1 | grep -E "table|elevate|gemini|gripper|camera|solid|导出" | tail -15
```
Expected: 只剩 `table` + `ur5_elevate_base`（无 gemini2_*/camera_*/gripper/solid）。

- [ ] **Step 3: 验证 world_sync 只加这俩**

起 `full_stack backend:=isaac`（或 mock，world_sync 都起），RViz PlanningScene 只显示桌面+底座（无冻住的相机/夹爪）。

- [ ] **Step 4: 提交**

```bash
git add isaac_scripts/export_scene_collisions.py
git commit -m "fix(gripper): export_scene_collisions 排除 robot 挂载 prim（相机/夹爪不当静态障碍）"
```

---

## Self-Review

**1. Spec 覆盖**：
- §1 URDF → Task 1 ✓
- §2 SRDF → Task 2 ✓
- §3 moveit 接线 + GripperCommand 句柄 → Task 3 ✓
- §3 real（serial_gripper）→ Task 4 ✓
- §3 isaac（桥扩展 + action 节点）→ Task 5 ✓
- §4 collider 修正 → Task 6 ✓
- §5 验证 → 各 Task 的 verify step ✓
- 组件清单覆盖（package.xml/CMakeLists/setup.py 散在 Task 1/2/5）✓

**2. 占位符扫描**：URDF offset 用 identity（具体值，verify 时调，非遗留 TBD）；isaac_gripper_action 代码完整；桥 OmniGraph 扩展节点/连接/topic 完整给出（仿现有 UR5 graph）；disable_collisions 注明以 lebai_lm3.srdf 源为准补全（给出子集 + 指引）。无 TBD/TODO。

**3. 一致性**：gripper_r_joint1、gripper_tip、tool0→gripper_base fixed joint、action 名 `lebai_gripper_controller/gripper_cmd`、topic `isaac_gripper_joint_states`/`isaac_gripper_joint_commands` 跨任务一致 ✓。`ur5_with_gripper.urdf.xacro` / `.srdf.xacro` 跨任务同名 ✓。

无问题。

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-22-gripper-integration.md`.
