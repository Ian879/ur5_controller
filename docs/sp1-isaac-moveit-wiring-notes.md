# SP1 — 官方 isaac_moveit 接法笔记（Task 4 产出）

来源：本仓库 vendor 的 `src/isaac_moveit/launch/isaac_moveit.launch.py` +
`src/moveit_resources/.../panda_isaac.urdf.xacro` + `panda.ros2_control.xacro` +
`ros2_controllers.yaml`。这是把官方 Franka 接法翻译成 UR5 的权威依据。

## 官方接线（Franka）核心事实

1. **硬件插件**：`ros2_control_hardware_type:=isaac`（默认）→ 在 xacro 里选
   `topic_based_ros2_control/TopicBasedSystem`（`=mock_components` 时用 `GenericSystem`）。
2. **TopicBasedSystem 的 topic 名**（在 xacro 的 `<param>` 里，不在 launch 里 remap）：
   - `joint_commands_topic` = **`isaac_joint_commands`**
   - `joint_states_topic` = **`isaac_joint_states`**
   - ⇒ Isaac 侧必须：发 `/isaac_joint_states`、订 `/isaac_joint_commands`（与 `moveit.py` 一致）。
3. **ros2_control_node**：通用 `controller_manager`（`controller_manager` 包的 `ros2_control_node` 可执行），
   参数 = `ros2_controllers.yaml` + `use_sim_time:=true`。
4. **控制器**（`ros2_controllers.yaml`，`use_sim_time: true`）：
   - `joint_state_broadcaster`（JointStateBroadcaster）——它把 TopicBasedSystem 的 state 转发成 `/joint_states` 给 move_group。
   - `<arm>_controller` = `joint_trajectory_controller/JointTrajectoryController`（panda 叫 `panda_arm_controller`，7 关节）。
   - 夹爪控制器（panda_hand）——SP1 无夹爪，忽略。
5. **Launch 结构**（`isaac_moveit.launch.py`）：
   - `MoveItConfigsBuilder("<robot>")` 组装 moveit 配置：robot_description 来自 `<robot>_isaac.urdf.xacro`、SRDF、trajectory_execution、planning pipelines（ompl + pilz）。
   - `<robot>_isaac.urdf.xacro` = include 机器人 URDF + include `<robot>.ros2_control.xacro`（含 TopicBasedSystem 宏，由 `ros2_control_hardware_type` 门控）。
   - 节点：move_group、rviz、robot_state_publisher、static TF（world→base_link）、ros2_control_node、spawner(joint_state_broadcaster + arm_controller)。
   - **launch 里没有任何 `/joint_states` remap**——靠 TopicBasedSystem 直接读写 isaac topic。

## UR5 翻译（给 T5/T6 用）

| 官方(Franka) | UR5 等价 |
|---|---|
| `panda_isaac.urdf.xacro` | `ur5_isaac.urdf.xacro`：include `ur_description` 的 `ur_macro.xacro` + 一个 UR5 的 `ros2_control` 宏（TopicBasedSystem，`ros2_control_hardware_type` 门控） |
| `panda.ros2_control.xacro`（7 关节 TopicBasedSystem 宏） | UR5 版：6 关节 `shoulder_pan_joint…wrist_3_joint`，同款 TopicBasedSystem 插件 + `isaac_joint_states`/`isaac_joint_commands` |
| `MoveItConfigsBuilder("moveit_resources_panda")` | 复用系统 `ur_moveit_config`（SRDF/运动学已有）；或 `MoveItConfigsBuilder("ur")` |
| `panda_arm_controller`（7 关节 JTC） | `joint_trajectory_controller`（6 UR 关节，**非 scaled**——scaled 依赖真机速度缩放）；`use_sim_time: true` |
| `panda_hand_controller` + `gripper_to_isaac` | SP1 不需要（无夹爪） |
| static TF world→panda_link0 | world→base_link（或省略，看场景） |

## 关键结论（修正/确认早前设计）

- ✅ 早前 SP1 设计的 TopicBasedSystem + `isaac_joint_states`/`isaac_joint_commands` **与官方完全一致**。
- ✅ T5 的 `ur5_isaac.ros2_control.xacro` 方向正确；按官方 `panda.ros2_control.xacro` 的结构写（`ros2_control_hardware_type` 门控 isaac/mock，TopicBasedSystem 带 `joint_commands_topic`/`joint_states_topic` 两个 `<param>`）。
- ✅ T6 `sim_isaac.launch.py`：通用 `ros2_control_node` + `ros2_controllers.yaml`(use_sim_time) + spawn `joint_state_broadcaster` + `joint_trajectory_controller`；再 include `ur_moveit.launch.py`（use_sim_time:=true）起 move_group+rviz。**无需 /joint_states remap**。
- ⚠️ 前置：`topic_based_ros2_control` 插件必须装（`sudo apt install ros-humble-topic-based-ros2-control`），否则 TopicBasedSystem 加载失败——这是 T3 验证 + T5/T6 跑通的前提。
- ⚠️ Isaac 侧（T7）的 bridge graph 必须用 **`isaac_joint_states`/`isaac_joint_commands`** 这两个确切 topic 名（`moveit.py` 默认就是这名，对得上）。
