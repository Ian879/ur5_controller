# UR5 Controller（MoveIt2 + ur_robot_driver 运动栈）

为 UR5 搭建 **MoveIt2 + ur_robot_driver（ros2_control）** 运动栈，并在 mock / Isaac Sim / 真机
三种后端间无缝切换。目标：先打通**规划 + 执行 + 碰撞检测**，为后续 6D 位姿估计抓取打地基。

> 设计支柱：三后端共用**同一 MoveIt 规划执行栈**，仅 URDF xacro 的 ros2_control 硬件插件不同。

## 架构

```
Python 客户端 (ur5_motion, pymoveit2) ─action▶ move_group (规划+碰撞, OMPL/pilz)
                                                        │ FollowJointTrajectory
                                                        ▼
                            ros2_control_node: joint_trajectory_controller + joint_state_broadcaster
                                                        │ 硬件 command/state
                          ┌──────────────────────┼──────────────────────┐
                          ▼                      ▼                      ▼
               mock_components/         TopicBasedSystem         ur_robot_driver
               GenericSystem (mock)     (←→ Isaac Sim)            (真机)
                                        Phase 2                   Phase 3
```

## 环境（本机关键事实）

- ROS2 **Humble**（`/opt/ros/humble`），已含 MoveIt2 全套、`ur_robot_driver 2.13.2`、`ur_moveit_config`、`ur_description`。
- 规划组 `ur_manipulator`，chain `base_link → tool0`，命名位姿 `home`/`up`/`test_configuration`。
- **conda 陷阱（头号）**：`.bashrc` 自动激活 conda `base`，使 `python3` 指向 miniconda，
  与系统 humble C++ 栈 ABI 不符 → ROS2 节点收发失败。**所有 ROS2 命令前必须 `source tools/source_env.sh`**（全项目唯一环境入口）——它会
  deactivate conda、从 PATH 剔除 miniconda、source humble + Orbbec overlay（`~/ros2_ws`）
  + 本仓库 `install/setup.bash` + `robot_ip.sh`。从任意子目录 source 都生效。
- 若想一劳永逸：`conda config --set auto_activate_base false`（一次即可）。

## 目录结构

```
ur5_controller/                 # = colcon workspace 根
├── src/
│   ├── ur5_bringup/            # launch：full_stack(后端选择) / sim_mock / sim_isaac / real_robot
│   │   ├── launch/             #   full_stack / sim_mock / sim_isaac / real_robot
│   │   ├── urdf/               #   ur5_isaac.urdf.xacro（TopicBasedSystem 硬件插件）
│   │   └── config/             #   ur5_isaac_controllers.yaml + rtde 配方
│   ├── ur5_motion/             # pymoveit2 客户端 + demos + world_sync_node
│   │   └── ur5_motion/
│   │       ├── moveit_client.py     # move_to_joint / move_to_pose / add_box
│   │       ├── world_sync_node.py   # 读 scene_collisions.json → compound CollisionObject → PlanningScene
│   │       └── demos/               # demo_plan_execute / demo_collision / sync_scene
│   ├── isaac_moveit/           # （vendor）官方 isaac_moveit 包（Franka MoveIt 参考配置）
│   └── moveit_resources/       # （vendor）panda moveit 配置（isaac_moveit 依赖）
├── isaac_scripts/
│   ├── ur5_isaac_bridge.py     # ★ Isaac 桥：加载 ~/Desktop/scene.usd + ROS2 bridge（关节/clock/相机）
│   ├── export_scene_collisions.py  # 读 scene.usd 所有 collider → ~/Desktop/scene_collisions.json（按父分组）
│   └── introspect_ur5.py       # UR5 USD 关节/链接探查（调试用）
├── tools/                      # source_env.sh / check_topics.sh / robot_ip.sh（★真机IP配置处）
├── docs/                       # 验收清单 + 感知路线图 + 真机安全清单
└── README.md
```

## 安装

```bash
sudo apt update && sudo apt install -y \
    ros-humble-pymoveit2 \
    ros-humble-topic-based-ros2-control \
    ros-humble-ros2controlcli \
    python3-colcon-common-extensions
```

## 构建

```bash
cd /home/gds/projects/shengzhi/ur5_controller
source tools/source_env.sh
colcon build --symlink-install
source install/setup.bash
```

## 快速启动

> 一句话：**build 一次 → 每个终端 `source tools/source_env.sh` → `ros2 launch ur5_bringup full_stack.launch.py backend:=<mock|isaac|real>`**。
> 相机（SP2）+ 静态碰撞同步已随栈自动起，无需额外命令。

### 0. 首次 / 改了代码后（build 一次）
```bash
cd /home/gds/projects/shengzhi/ur5_controller
source tools/source_env.sh
colcon build --symlink-install
```

### 1. 每个新终端（全项目唯一环境入口）
```bash
cd /home/gds/projects/shengzhi/ur5_controller
source tools/source_env.sh   # 退出 conda + source humble + Orbbec overlay(~/ros2_ws) + 本仓库 install
```

### 2. 起运动栈（三选一；都带 RViz + 相机 TF + 碰撞同步）
```bash
# mock —— 无需真机/Isaac，纯 fake hardware，先验证规划+碰撞+相机 TF
ros2 launch ur5_bringup full_stack.launch.py backend:=mock

# real —— 真机：先示教器 上电→松刹车→按 Play 启 external_control（详见"真机控制"节）
ros2 launch ur5_bringup full_stack.launch.py backend:=real

# isaac —— 先在另一终端跑 Isaac 桥（详见"Isaac Sim 仿真后端"节）
ros2 launch ur5_bringup full_stack.launch.py backend:=isaac
```
常用参数：`launch_rviz:=false`（不开 RViz）、`robot_ip:=10.67.64.5`（临时覆盖真机 IP；默认读 `tools/robot_ip.sh` 的 `UR5_HOST`）。

### 3. 验证 + 跑动作（另开终端，先 `source tools/source_env.sh`）
```bash
./tools/check_topics.sh             # 运动栈自检（controller / joint_states）
./tools/check_camera.sh             # 相机自检（4 topic + 2 frame；mock 下只有 TF）
ros2 run ur5_motion demo_plan_execute   # 全自动 plan & execute（关节/笛卡尔）
ros2 run ur5_motion demo_collision      # 加障碍盒，验证碰撞绕行
```

> 各后端细节见下方专节：[真机控制](#真机控制rviz--moveit已验证-) / [Isaac Sim 仿真后端](#isaac-sim-仿真后端已验证-) / [RGBD 相机](#rgbd-相机orbbec-gemini-2eye-in-handsp2)。

---

## 运行示例（mock，演示客户端）

终端 1——起运动栈（含 RViz）：
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=mock
```

终端 2——自检：
```bash
source tools/source_env.sh
./tools/check_topics.sh
```

终端 2——Python 客户端（全自动 plan&execute）：
```bash
ros2 run ur5_motion demo_plan_execute     # 关节/笛卡尔规划+执行
ros2 run ur5_motion demo_collision        # 加障碍盒，验证碰撞检测绕行
```

## 切换后端（不改业务代码）

```bash
# Isaac Sim（需先跑桥 + 导出碰撞体，见下方"Isaac Sim 仿真后端"节）
ros2 launch ur5_bringup full_stack.launch.py backend:=isaac

# 真机 —— 先把 IP 填进 tools/robot_ip.sh（设了 UR5_HOST 后无需再带 robot_ip:=）
ros2 launch ur5_bringup full_stack.launch.py backend:=real
# 或临时覆盖:  ... backend:=real robot_ip:=10.67.64.5
```

## 真机控制（RViz + MoveIt，已验证 ✅）

> 本机已验证：CB3 真机（IP `10.67.64.5`，固件 **Polyscope 3.14.3**）—— driver 连上、
> `scaled_joint_trajectory_controller` active、RViz 内拖目标 → Plan → Execute，真机精确到位。

### 一次性准备（每台机器人一次）
1. **装 External Control URCap**：文件 `/opt/ros/humble/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap`
   拷到机器人 programs 目录（USB，或 `scp ... ur@<IP>:/programs/`，CB3 默认账号 `ur`/`easybot`）
   → 示教器 Welcome → Setup Robot → URCaps → `+` 选该 urcap → 重启机器人。
2. 重启后 Program Robot → Installation → **External Control** → Custom IP 填**本机 IP**（与机器人同网段，如 `10.67.64.64`）→ 新建含 External Control 节点的程序并保存。
3. **填 IP**：编辑 `tools/robot_ip.sh` → `export UR5_HOST="10.67.64.5"`。

### RTDE 配方（视固件而定）
- 固件 **3.14.3+**：用官方原配方即可，**不用改**。
- 旧固件 **3.9**：`tcp_offset`/`payload`/`payload_cog` 不支持需删（备份 `rtde_output_recipe.txt.orig`；详见 `docs/real_robot_bringup.md`）。

### 每次启动
1. 示教器：**上电 → 松刹车**，确认 `RUNNING + NORMAL`（反复异常连接会进 `safetymode: RECOVERY`，断电恢复到 NORMAL）。速度滑块先 **10–25%**。
2. 示教器启动 "external_control"程序
3. PC 起运动栈（带 RViz）：
   ```bash
   cd /home/gds/projects/shengzhi/ur5_controller
   source tools/source_env.sh
   ros2 launch ur5_bringup full_stack.launch.py backend:=real launch_rviz:=true
   ```
4. 等 `scaled_joint_trajectory_controller ... active` 且 RViz 弹出。
5. **若 `Program running: false` → 示教器按 Play** 启动 External Control 程序（driver 不会自动 play）。
   验证接管：另开终端 `ros2 topic hz /joint_states` 有数据 = external_control 已连上。

### 在 RViz 里控制（MoveIt）
- 默认加载 ur_moveit 预设（已含 RobotModel / TF / MotionPlanning / PlanningScene）。缺则 **Add → By display type** 手动加。
- **必检三项**：
  - `Global Options → Fixed Frame` = **`base_link`**
  - `RobotModel → Description Topic` = **`/robot_description`**
  - `MotionPlanning → Planning Request → Planning Group` = **`ur_manipulator`**，并勾 `Query Goal State`（出现可拖的末端球）
- 操作：拖末端球到目标 → **Plan**（先看绿色轨迹会不会撞）→ **手按急停 → Execute**。

### Python 客户端（真机）
```bash
ros2 run ur5_motion demo_plan_execute
```
> ⚠️ 已知：scaled 控制器对快速小目标可能返回 PREEMPTED，pymoveit2 会误报失败但**实际已到位**（控制器日志 `Goal reached, success!`）——以到位判定为准。

**完整安全清单**见 [`docs/real_robot_bringup.md`](docs/real_robot_bringup.md)。

## Isaac Sim 仿真后端（已验证 ✅）

> 本机已验证：UR5 在 Isaac Sim 6.0.1 里被 MoveIt2 驱动（TopicBasedSystem），`demo_plan_execute`
> 全绿、Isaac UR5 精确到位。`--stream` 走 WebRTC 推流（完整 streaming kit 配置）。

### 工作流（三步）

**① 你在 Isaac GUI 里搭场景**：存成 `~/Desktop/scene.usd`（UR5 + 桌子 + 灯光 + collider）。
机器人放在某个 prim 下（如 `/World/ur5`），articulation root 在 `<robot_prim>/root_joint`。

**② 导出碰撞体到 MoveIt**（改了 scene.usd 后跑一次，~10 秒）：
```bash
cd ~/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/export_scene_collisions.py
# → ~/Desktop/scene_collisions.json（按父节点分组的 compound 对象，base_link 系）
```

**③ 起栈**（两个终端）：
```bash
# 终端 A：Isaac 桥（加载 scene.usd + ROS2 bridge + WebRTC 推流）
cd ~/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/ur5_isaac_bridge.py \
  --stream --robot_prim /World/ur5

# 终端 B：PC 运动栈（world_sync_node 自动把碰撞体加进 MoveIt）
cd /home/gds/projects/shengzhi/ur5_controller
source tools/source_env.sh && source install/setup.bash
ros2 launch ur5_bringup full_stack.launch.py backend:=isaac
```

- **`--robot_prim`**：你的 scene.usd 里机器人的 prim 路径（articulation/camera 自动从它推导）。
- **`--stream`**：WebRTC 推流（和 `isaac-sim.streaming.sh` 同一套完整 kit 配置）。去掉则 headless 无画面。
- **`world_sync_node`** 随栈自动启动，读 `scene_collisions.json` → 把 compound 碰撞对象加进 MoveIt
  PlanningScene（table 整体含 top+legs，不是一个一个散的 box）。
- **RViz Fixed Frame**：用 `world`（桥会发 `world→base_link` 的 TF 补偿机器人底座抬高）。

### 桥的关键设计

- 加载用户场景（`open_stage ~/Desktop/scene.usd`），**不在运行时建场景**。
- 发 `/isaac_joint_states`（6 UR 关节）+ `/clock` + 腕部相机 RGBD（如果 scene.usd 里有 Camera prim）。
- 订阅 `/isaac_joint_commands` 驱动 UR5 articulation。
- 启动时自动清理残留 ActionGraph（防止旧 graph 带老 robotPath 报错）。

### 碰撞同步原理

```
scene.usd (Isaac 物理世界)              MoveIt PlanningScene (规划世界)
  │                                       │
  │ export_scene_collisions.py            │ world_sync_node (launch 自动起)
  │ 读所有 collider → base_link 系 ──────▶ │ compound CollisionObject
  │ 按父节点分组（table=1 对象含 5 box）   │ → ApplyPlanningScene
```

**为什么不直接从 Isaac 发 CollisionObject**：Isaac 没有这个 OmniGraph 节点；我们的方式在
frame 控制（base_link）、分组（compound）、launch 集成上更灵活，且不依赖 Isaac 运行时。

## RGBD 相机（Orbbec Gemini 2，eye-in-hand，SP2）

> Orbbec SDK（v2-main）在独立工作空间 `~/ros2_ws`，已编译。相机 eye-in-hand（腕部），
> 当前**未装机**，TF 用占位（M7 手眼标定精修）。真机/Isaac 发布**同一套**标准 ROS RGBD 契约，
> 下游感知（SP3/SP4）写一份即跨后端跑。

### 契约（下游唯一接口）
| Topic | frame |
|---|---|
| `/camera/color/image_raw`、`/camera/color/camera_info` | `camera_color_optical_frame` |
| `/camera/depth/image_rect_raw`、`/camera/depth/camera_info` | `camera_depth_optical_frame` |

### 用法（随 full_stack 自动起）
相机已并入 `full_stack`（`camera.launch.py` 内部按 backend gate）：
- **real**：自动起 Orbbec 驱动 + 占位 TF（`publish_tf:=false`，TF 子树由 launch 统一发）。
- **isaac**：图像来自桥（`ur5_isaac_bridge.py` 已发标准 topic）；占位 TF 同样起。
- **mock**：仅占位 TF，无图像。

```bash
source tools/source_env.sh   # 一步含 Orbbec overlay
ros2 launch ur5_bringup full_stack.launch.py backend:=real
./tools/check_camera.sh       # 核对 4 topic + 2 frame
```

相机**单独跑**（桌面调试，不接机器人）：
```bash
ros2 launch ur5_bringup camera.launch.py backend:=real
```

### Isaac 侧
桥已把相机 topic/frame 对齐标准契约（见 `isaac_scripts/ur5_isaac_bridge.py` 相机 OmniGraph），
并补了 depth 的 camera_info（原来只有 color 的）。Isaac 后端 RViz 用同一套 Image display。

### 已知限制（SP2 范围外）
- 手眼标定（M7）未做：`tool0→camera_link` 是占位，方向/位置待标定。
- pointcloud 未做（FoundationPose 只要 RGB+depth）。
- 真·eye-in-hand 闭环验证待物理安装。

## 验收

见 [`docs/phase1_verification.md`](docs/phase1_verification.md)。**首期硬指标**：
mock 下 `demo_plan_execute` 全绿 + `demo_collision` 绕开障碍 = 运动栈打通。

## 后续：6D 抓取路线图

见 [`docs/perception_roadmap.md`](docs/perception_roadmap.md)（M3–M8：相机 / 6D 位姿 /
手眼标定 / 抓取编排 / 夹爪 / 力控）。

## 关键限制 / 已知点

- ur_rtde **不做规划/碰撞**；它只作为低级调试/标定工具保留（见 `ur5_controller/config.py`）。
- mock 下 MoveIt 必须 `use_sim_time:=true`（否则会去找未激活的 scaled 控制器）——已内置。
- **真机 CB3 固件需 ≥ 3.14**（3.9 的 primary 接口与 urcl 2.13.0 不兼容，会 SIGABRT）；官方最低 3.7。
- 真机 **calibration 尚未校准**（driver 有警告，TCP 有轻微偏差）——抓取前用 `ur_calibration` 工具校准。
- Isaac 桥用 `open_stage` 加载 `~/Desktop/scene.usd`——场景由你在 Isaac GUI 里搭建/编辑。
- Isaac 碰撞同步是**离线两步**（export → JSON → world_sync_node），非 Isaac 实时发布。
- `world→base_link` TF 和碰撞体坐标是手算的（对应当前 scene.usd）；改了场景后需重跑 export。
