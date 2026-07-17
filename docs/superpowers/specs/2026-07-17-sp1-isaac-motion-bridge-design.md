# SP1 设计：Isaac Sim 运动桥（UR5 + MoveIt2 + 相机）

**日期**: 2026-07-17
**子项目**: SP1（Isaac 运动桥）——「Isaac Sim 抓取任务」分解的第一块
**状态**: 设计已批准（brainstorming 通过），待写实现计划

## 背景 / 为什么做这个

整体目标是「Isaac Sim 里 6D 位姿估计 → 抓取桌面指定零食」。这事跨多个子系统，已分解成
顺序子项目（各自 spec→plan→实现）：

| 子项目 | 内容 | 依赖 |
|---|---|---|
| **SP1 Isaac 运动桥（本文档）** | UR5 在 Isaac 被 MoveIt 驱动 + 相机发图 | 用户提供 Isaac 场景 |
| SP2 夹爪接入 | Robotiq 2F-85 + gripper controller | SP1 |
| SP3 抓取编排（GT 打底） | 指定目标→加碰撞体→抓取规划→执行→闭合→抬起，用 Isaac 真值位姿 | SP1+SP2 |
| SP4 真 6D 位姿估计 | 把 GT 换成 FoundationPose | SP3 |

运动栈（mock + 真机 CB3）已验证打通。SP1 是 Isaac 侧的缺口：让 MoveIt 能驱动 Isaac 里的 UR5，
并发布 RGBD 相机给后续感知用。

## 目标 & 验收

- Isaac Sim 里的 UR5（腕部 eye-in-hand 相机）能被 MoveIt 经 ros2_control 驱动：
  `demo_plan_execute` / RViz 下发的轨迹让 sim 机器人精确运动。
- 腕部相机发布 `/camera/color`、`/camera/depth`、`/camera/camera_info`（RGBD）。
- 业务代码零改动：`backend:=isaac` 即切（与 mock/real 一致）。
- **验收**：headless 起 Isaac 桥 + `full_stack backend:=isaac`，`demo_plan_execute` 全绿、
  `/joint_states`（来自 Isaac）、`/clock`、`/camera/*` 都有数据。

## 关键决策（来自 brainstorming）

1. **运行方式**：GUI 搭场景存 `.usd`（用户做）+ standalone 桥接脚本加载（我做）。
2. **相机**：eye-in-hand（腕部）。hand-eye TF 在 Isaac 里是已知真值，省标定；与真机方案一致。
3. **运动接口**：ros2_control 的 **TopicBasedSystem**（读 `isaac_joint_states`、写 `isaac_joint_commands`）。
4. **官方优先**：用 NVIDIA 官方 `isaac_moveit`（来自 `isaac-sim/IsaacSim-ros_workspaces`）当模板和首选实现；
   **若在 Humble 下不稳（官方文档已承认间歇性失败）再降级自写 bridge**。
5. SP1 **不带夹爪**（夹爪在 SP2，避免关节混杂）。

## 架构 & 数据流

```
Isaac Sim (standalone 脚本，加载用户 .usd)
   ├─ PublishClock ───────────────────▶ /clock
   ├─ UR5 Articulation ──ROS2PublishJointState──▶ /isaac_joint_states
   ├─ ROS2SubscribeJointState ◀──/isaac_joint_commands── ▶ IsaacArticulationController
   └─ 腕部 RTX 相机 ──▶ /camera/color, /camera/depth, /camera/camera_info

PC (full_stack backend:=isaac):
   ros2_control_node [硬件插件=TopicBasedSystem]
        ├─ 读 /isaac_joint_states 当状态 ─▶ joint_state_broadcaster ─▶ /joint_states
        └─ 写 /isaac_joint_commands
   move_group (ur_moveit_config，不变)
   RViz
pymoveit2 ─▶ move_group ─▶ joint_trajectory_controller ─▶ TopicBasedSystem ─▶ /isaac_joint_commands ─▶ Isaac 动
```
`use_sim_time:=true`（Isaac 发 /clock）。关节名 Isaac↔ur_description 做映射（`shoulder_pan…wrist_3`）。

## 分阶段实现（官方优先）

### Phase 0：拿官方 isaac_moveit + 用 Franka 验证
1. `git clone -b IsaacSim-6.0.1 https://github.com/isaac-sim/IsaacSim-ros_workspaces.git`
2. 按 repo 的 `build_ros.sh`/README 编 `humble_ws` → 得到官方 `isaac_moveit` 包（含 `isaac_moveit.launch.py`）。
3. Isaac Sim：Window > Examples > Robotics Examples > ROS2 > MoveIt > Franka MoveIt，Play。
4. PC（source 该 workspace overlay）：`ros2 launch isaac_moveit isaac_moveit.launch.py`。
5. **验证 + 风险评估**：RViz 里 Plan/Execute，Franka 动；重点观察 **Humble 下是否间歇性失败**
   （官方教程已警告）。记录稳定性。
   - 若稳定 → 进 Phase 1。
   - 若严重不稳 → 跳到「降级自写 bridge」。

### Phase 1：官方 launch 模板 → 改 UR5
1. 读官方 `isaac_moveit.launch.py`（TopicBasedSystem + move_group + rviz 的接法）当**权威模板**。
2. 做 UR5 等价物：`ur_moveit_config`（已有）+ TopicBasedSystem + 话题 remap 到 Isaac 的
   `isaac_joint_states`/`isaac_joint_commands`，挂进 `src/ur5_bringup/launch/full_stack.launch.py` 的
   `backend:=isaac` 分支（起通用 `ros2_control_node` + `joint_trajectory_controller` + move_group + RViz）。
3. Isaac 侧：照搬本机 `standalone_examples/.../moveit.py` 的 ROS2 bridge graph（ReadJointState /
   Publish/Subscribe JointState / PublishClock / ArticulationController），改三处：
   - 加载用户的 `.usd`（UR5 + 腕部相机 + 桌面 + 零食）而非 Franka+simple_room；
   - `robotPath`/`ReadJointState` 指向 UR5 articulation，关节名映射；
   - **加 RTX 相机发布**（moveit.py 没有）。
   - 另参考 `exts/isaacsim.ros2.examples/.../ros_moveit_sample.py`。

### 降级：自写 bridge（仅当官方不稳）
若 Phase 0 验证 Humble 下官方链路严重不稳，走原备选：`passthrough_trajectory_controller` +
Isaac `ROS2Subscriber` 直接吃整条 JointTrajectory（按时序播放，避免 TopicBasedSystem 抖动），
代价是丢 MoveIt 执行监控。`full_stack backend:=isaac` 加一个 `motion_iface:=passthrough` 开关切换。

## 要新建 / 改的文件

- **外部**：`IsaacSim-ros_workspaces/`（克隆，提供 `isaac_moveit` 模板 + Franka 验证）。
- **新** `isaac_scripts/ur5_isaac_bridge.py`：moveit.py 的 UR5 适配版（加载 .usd + 桥接 graph + 相机）。
- **改** `src/ur5_bringup/launch/full_stack.launch.py`：`backend:=isaac` 分支落地（或新增 `launch/sim_isaac.launch.py`）。
- 可能 **新** UR5 isaac 用的小配置（视官方模板而定，尽量复用 `ur_moveit_config` + remap）。

## 场景契约（用户的 `.usd` 要提供，脚本参数化读取）

- 一个 UR5 Articulation，关节可映射到 `shoulder_pan_joint…wrist_3_joint`（prim 路径 + 关节名作参数）。
- 一个挂在腕部（wrist_3/tool0 下）的 RTX 相机 prim（路径作参数）。
- `.usd` 文件路径。
- **不含夹爪**（SP2 再加）。

## 验证（headless 可自动跑）

1. `python.sh isaac_scripts/ur5_isaac_bridge.py --headless <你的.usd>`（或先 GUI 看效果）。
2. `source tools/source_env.sh && ros2 launch ur5_bringup full_stack.launch.py backend:=isaac launch_rviz:=true`。
3. `ros2 topic hz /joint_states`、`/clock`、`/camera/color` 都有数据。
4. `ros2 run ur5_motion demo_plan_execute` → `/isaac_joint_commands` 变化、Isaac UR5 按规划运动。

## 风险 & 缓解

| 风险 | 缓解 |
|---|---|
| Humble 下官方 MoveIt2 间歇性失败（官方已承认） | Phase 0 先验证稳定性；不稳走 passthrough 降级 |
| TopicBasedSystem 跟随抖动 | 降级 passthrough；或后续评估 Jazzy（但与 ur_robot_driver/Humble 栈绑定） |
| 关节名/方向不匹配 | 桥脚本做映射，验证时对一遍 |
| Isaac 场景 prim/相机路径差异 | 全部参数化，不对就调参 |

## 不在 SP1 范围

- 夹爪（SP2）、抓取编排（SP3）、真位姿估计 FoundationPose（SP4）。
- 真机 Isaac 迁移（Isaac 是开发/验证环境；真机已单独跑通）。
- 桌面/零食物体本身的建模（用户搭场景负责）。
