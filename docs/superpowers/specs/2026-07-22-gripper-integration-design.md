# Lebai 夹爪集成设计（UR5 + Lebai gripper，real + isaac）

> 状态：已设计待实现 ｜ 分支：`sp1-isaac-motion-bridge` ｜ 日期：2026-07-22
> 用户更新了 scene.usd（尺寸 + 物理属性 + 加了 Lebai 夹爪 / 腕上 Gemini 2），要把夹爪完整集成进 MoveIt/UR5 栈（real + isaac），并修正 collider 同步（机器人挂载的 prim 不该当静态障碍）。

## 目标

1. **夹爪完整集成**：Lebai gripper 进 UR5 的 robot_description（URDF），MoveIt 有 gripper group + TCP（`gripper_tip`）+ open/closed 预设；夹爪可由 MoveIt 控制开合。
2. **real 后端**：`lebai_driver/serial_gripper`（串口）驱动真夹爪，暴露 `GripperCommand` action；MoveIt 经 GripperCommand 句柄控制。
3. **isaac 后端**：桥扩展读写 `gripper_abs_stl` articulation + 一个 `isaac_gripper_action` 节点中转 GripperCommand；MoveIt 发命令 → Isaac 里夹爪动。
4. **collider 修正**：`export_scene_collisions.py` 排除机器人挂载的 prim（腕上相机机身、夹爪），只导真静态环境（table、底座）。

**非目标**：mock 夹爪（用户未选）、相机碰撞进 URDF（YAGNI，后续）、力控、SP3 抓取编排。

## 关键事实（实现前已验证）

- **夹爪 URDF**：`~/lebai_ws/src/lebai-ros-sdk/lebai_lm3_support/urdf/gripper_macro.xacro` 定义 `lebai_gripper` macro（`prefix` 参数）。1-DOF：`gripper_r_joint1` 驱动，其余全 `mimic`。TCP link = `gripper_tip`。mesh 在 `lebai_lm3_support/meshes/gripper/`。
- **挂载范式**：`lebai_lm3_support/urdf/lm3_with_gripper.xacro`：include gripper_macro + `<joint name="fixed_gripper" type="fixed"><parent link="tool0"/><child link="gripper_base_link"/></joint>`（LM3 用 identity origin）。UR5 同样挂 `tool0→gripper_base_link`，offset 从 scene.usd 量（物理实况）。
- **SRDF 范式**（`lebai_lm3_moveit_config/config/lebai_lm3.srdf`）：
  - `<group name="gripper"><joint name="gripper_r_joint1"/></group>`
  - `<end_effector name="gripper" parent_link="tool0" group="gripper" parent_group="manipulator"/>`
  - `<group_state name="open" group="gripper"><joint name="gripper_r_joint1" value="1.0471975512"/></group_state>`（π/3）
  - `<group_state name="closed" group="gripper"><joint name="gripper_r_joint1" value="0.0"/></group_state>`
  - 一组 `<disable_collisions>`（gripper 各 link 间）照搬。
- **真机驱动**：`lebai_driver/lebai_driver/gripper_action.py` 暴露 `control_msgs/GripperCommand` action，名 `lebai_gripper_controller/gripper_cmd`；`MAX_GRIPPER_POSITION=π/3`。`serial_gripper.launch.py` 起 `serial_gripper` 节点（串口 `/dev/ttyUSB0`）。
- **系统 ur_moveit_config 的 SRDF 是 xacro**（`srdf/ur.srdf.xacro`），可 xacro-include。
- **现有栈**：`moveit.launch.py`（我们的 fork，build robot_description via xacro + srdf xacro）；`real_robot.launch.py` include `ur_robot_driver` 的 `ur_control.launch.py`（有 `description_file` arg）；`sim_isaac.launch.py` 用 `ur5_isaac.urdf.xacro`；`full_stack.launch.py` 按后端 gate。
- **scene.usd**：`gripper_abs_stl` 是独立 articulation（已被 export 排除）；腕上 Gemini 2 机身（`gemini2_body`/`camera_rgb` 等）当前被当静态碰撞体导出（错，要修）。相机 RGBD 由桥的 Camera prim 发（eye-in-hand）。

## §1 URDF 组合（gripper 进 robot_description）

新增 `ur5_bringup/urdf/ur5_with_gripper.urdf.xacro`：
```xml
<robot name="ur5_with_gripper" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
  <xacro:include filename="$(find lebai_lm3_support)/urdf/gripper_macro.xacro"/>
  <xacro:ur name="ur" ur_type="$(arg ur_type)"
            joint_limit_params="$(arg joint_limit_params)"
            kinematics_params="$(arg kinematics_params)"
            physical_params="$(arg physical_params)"
            visual_params="$(arg visual_params)"
            safety_limits="$(arg safety_limits)" safety_pos_margin="$(arg safety_pos_margin)"
            safety_k_position="$(arg safety_k_position)" prefix="$(arg prefix)"/>
  <xacro:lebai_gripper prefix=""/>
  <joint name="tool0_to_gripper_base" type="fixed">
    <parent link="tool0"/><child link="gripper_base_link"/>
    <origin xyz="<从 scene.usd 提取的 offset>" rpy="<同>"/>
  </joint>
</robot>
```
- **三处共用**：
  - `moveit.launch.py` 的 `description_file` 改成 `ur5_with_gripper.urdf.xacro`（连同那些 xacro arg 一起传，仿 ur_moveit）。
  - `real_robot.launch.py` 给 `ur_control.launch.py` 传 `description_file:=ur5_bringup/urdf/ur5_with_gripper.urdf.xacro`（ur_control 支持该 arg）。
  - isaac：`ur5_isaac.urdf.xacro` 里 include `gripper_macro` + 同一个 `tool0→gripper_base_link` fixed joint（offset 一致）。
- **挂载 offset**：实现时 isaac 脚本一把读出 `gripper_abs_stl`（或 gripper_base 几何）相对 UR5 `tool0` 的 transform，填进 fixed joint origin。
- **依赖**：`lebai_lm3_support` 包要可 `$(find)` 到 → `tools/source_env.sh` 加 `~/lebai_ws/install` overlay（仿 Orbbec 的 `~/ros2_ws`），并在 `ur5_bringup/package.xml` 加 `lebai_lm3_support` / `lebai_driver` exec_depend。

## §2 SRDF（gripper group + TCP）

新建 `ur5_bringup/srdf/ur5_with_gripper.srdf.xacro`：xacro-include ur 的 srdf + 追加 gripper 段：
```xml
<?xml version="1.0" ?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find ur_moveit_config)/srdf/ur.srdf.xacro"/>  <!-- 不行则复制其内容 -->
  <!-- gripper group（照搬 lebai_lm3.srdf）-->
  <group name="gripper"><joint name="gripper_r_joint1"/></group>
  <end_effector name="gripper" parent_link="tool0" group="gripper" parent_group="ur_manipulator"/>
  <group_state name="open"   group="gripper"><joint name="gripper_r_joint1" value="1.0471975512"/></group_state>
  <group_state name="closed" group="gripper"><joint name="gripper_r_joint1" value="0.0"/></group_state>
  <!-- disable_collisions：照搬 lebai_lm3.srdf 里 gripper 那一组（gripper_base_link vs 各 finger 等）-->
  ...
</robot>
```
- `moveit.launch.py` 的 `moveit_config_file`（SRDF）改成我们的 `ur5_with_gripper.srdf.xacro`。
- TCP = `gripper_tip`（gripper group 的 tip，MoveIt 自动认）。
- 若 ur.srdf.xacro 不能被 include（xacro 限制），则把 ur.srdf.xacro 的内容复制进我们的 srdf 再追加 gripper 段（实现时确认）。

## §3 控制器（real + isaac）

**real**：
- `full_stack` real 分支起 `lebai_driver/serial_gripper` 节点（`serial_gripper.launch.py`，串口 `/dev/ttyUSB0`，namespace 按需）→ 暴露 `control_msgs/GripperCommand` action `lebai_gripper_controller/gripper_cmd`。
- MoveIt `controllers.yaml`（real 用的那份）加 GripperCommand 句柄：
  ```yaml
  lebai_gripper_controller:
    action_ns: lebai_gripper_controller/gripper_cmd
    type: GripperCommand
    default: true
    joints: [gripper_r_joint1]
  ```
  （`ur5_isaac_controllers.yaml` 是 isaac 用的；real 用 ur_robot_driver 的 controllers.yaml——需确认在哪份注入。实现时定位。）

**isaac**（最复杂）：
- 桥扩展：OmniGraph 同时读/写 **gripper articulation**（`gripper_abs_stl`）的驱动关节 → state 发 `/isaac_gripper_joint_states`，cmd 订 `/isaac_gripper_joint_commands` 驱动 articulation。
- 新建 `ur5_motion/ur5_motion/isaac_gripper_action.py`（rclpy 节点）：暴露 `control_msgs/GripperCommand` action `lebai_gripper_controller/gripper_cmd`；goal → 写 `/isaac_gripper_joint_commands`（gripper_r_joint1 目标值，映射 0..π/3）；轮询 `/isaac_gripper_joint_states` 到位后 result。
- MoveIt isaac 分支的 controllers.yaml 加同款 GripperCommand 句柄，`action_ns` 指向该 action。
- `sim_isaac.launch.py`（或 full_stack isaac 分支）起 `isaac_gripper_action` 节点。
- **mock**：不集成（未选）。mock 下 gripper 关节无驱动，仅 URDF 可见（够看模型；MoveIt 规划 gripper 会失败但不影响机械臂规划）。

## §4 collider 同步修正（排除机器人挂载 prim）

`isaac_scripts/export_scene_collisions.py` 改：
- 排除规则加：**prim 路径在 `robot_prim`（默认 `/World/ur5`）之下**一律跳过（含腕上相机、任何 robot 子 prim）。
- 外加**显式排除名单**（顶层但属机器人部件）：`/World/gripper_abs_stl/**`、`/World/gemini2_*`、`/World/camera_*`、`/World/back_cover`、`/World/solid`（相机机身 + 夹爪 + 杂项）。
- 结果 `scene_collisions.json` 只剩 `table` + `ur5_elevate_base`（真静态环境）。world_sync 加载这俩。
- 夹爪碰撞来自 URDF（gripper_macro 自带 collision mesh），不靠静态导出。
- 相机碰撞暂不进 URDF（YAGNI），但不再当静态障碍。

## §5 验证

- **URDF**：`xacro $(find ur5_bringup)/urdf/ur5_with_gripper.urdf.xacro ur_type:=ur5 ...` 生成 + `check_urdf` 显示 `tool0 → gripper_base_link → … → gripper_tip` 链通。
- **SRDF**：起 moveit，RViz 看到 `gripper` group + end-effector + open/closed 预设；`ros2 param get /move_group robot_description_semantic` 含 gripper。
- **real**：`full_stack backend:=real` + `serial_gripper`；RViz 夹爪挂 flange；MoveIt 规划 gripper open/closed → 真夹爪开合；`ros2 action list` 含 `lebai_gripper_controller/gripper_cmd`。
- **isaac**：桥扩展 + `isaac_gripper_action`；MoveIt 发 gripper 命令 → Isaac 夹爪动；`ros2 topic hz /isaac_gripper_joint_states` 有数据。
- **collider**：重跑 export → `scene_collisions.json` 只有 table + ur5_elevate_base；起 full_stack → world_sync 只加这俩（RViz PlanningScene 干净，无冻住的相机/夹爪障碍）。
- **已知风险**：isaac 桥扩展 gripper articulation 的 OmniGraph（多一个 articulation 读写 + 关节名映射）最易踩坑，实现时重点验证；SRDF xacro-include 若受限则复制内容。

## 组件清单

| 文件 | 动作 |
|---|---|
| `ur5_bringup/urdf/ur5_with_gripper.urdf.xacro` | 新建：ur_macro + gripper_macro + tool0→gripper_base fixed joint |
| `ur5_bringup/srdf/ur5_with_gripper.srdf.xacro` | 新建：include ur srdf + gripper group/TCP/open-closed/disable_collisions |
| `ur5_bringup/launch/moveit.launch.py` | 改：description_file + moveit_config_file 指向我们的 |
| `src/ur5_bringup/launch/real_robot.launch.py` | 改：给 ur_control 传 description_file（带 gripper） |
| `ur5_bringup/urdf/ur5_isaac.urdf.xacro` | 改：include gripper_macro + tool0→gripper fixed joint |
| `ur5_bringup/launch/sim_isaac.launch.py` | 改：起 isaac_gripper_action 节点；controllers.yaml 加 gripper 句柄 |
| `ur5_bringup/config/ur5_isaac_controllers.yaml` | 改：加 lebai_gripper_controller GripperCommand 句柄 |
| `ur5_motion/ur5_motion/isaac_gripper_action.py` | 新建：GripperCommand action → /isaac_gripper_joint_commands 中转 |
| `isaac_scripts/ur5_isaac_bridge.py` | 改：OmniGraph 加 gripper articulation 读写（/isaac_gripper_joint_states + /isaac_gripper_joint_commands） |
| `isaac_scripts/export_scene_collisions.py` | 改：排除 robot_prim 子 prim + 显式排除名单 |
| `ur5_bringup/package.xml` | 加 lebai_lm3_support / lebai_driver exec_depend |
| `tools/source_env.sh` | 加 ~/lebai_ws/install overlay |
| `ur5_motion/setup.py` | 加 isaac_gripper entry_point |

## 范围外 / 推迟

- mock 后端夹爪驱动（用户未选）。
- 相机碰撞进 URDF（YAGNI）。
- SP3 抓取编排（夹爪到位后）。
- 力控抓取。

## 决策记录

1. **gripper 用 lebai 官方 URDF macro + 真机 serial_gripper 驱动**：复用官方包，不重复造轮子。
2. **一个 ur5_with_gripper.urdf.xacro 三后端共用**：一致 robot 模型；real 经 ur_control `description_file` 注入，isaac 经 ur5_isaac xacro。
3. **自己的 SRDF xacro（include ur + 加 gripper 段）**：系统 ur_moveit srdf 无 gripper；照搬 lebai_lm3.srdf 的 gripper 段。
4. **isaac 夹爪用"桥扩展 + GripperCommand action 中转节点"**：桥驱动 articulation（仿 UR 关节），action 节点对齐 MoveIt 的 GripperCommand 句柄接口。
5. **collider 排除 robot 挂载 prim**：腕上相机/夹爪不当静态障碍；夹爪碰撞来自 URDF。
6. **real + isaac 先做，mock 不做**：用户选定；mock 夹爪仅 URDF 可见。
