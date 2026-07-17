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
  与系统 humble C++ 栈 ABI 不符 → ROS2 节点收发失败。**所有 ROS2 命令前必须 `source tools/source_env.sh`**
  （它会 deactivate conda 并从 PATH 剔除 miniconda，回到系统 `/usr/bin/python3`）。
- 若想一劳永逸：`conda config --set auto_activate_base false`（一次即可）。

## 目录结构

```
ur5_controller/                 # = colcon workspace 根
├── src/
│   ├── ur5_bringup/            # launch：full_stack(后端选择) / sim_mock / real_robot
│   └── ur5_motion/             # pymoveit2 客户端(move_to_joint/pose + add_box/remove_collision) + 2 demo
├── tools/                      # source_env.sh / check_topics.sh / robot_ip.sh（★真机IP配置处）
├── docs/                       # 验收清单 + 感知路线图
├── ur5_controller/config.py    # （保留）ur_rtde 低级调试配置，标定阶段再用
├── isaac_scripts/              # Phase 2：Isaac Sim 启动脚本
└── README.md
```

## 安装

```bash
sudo apt update && sudo apt install -y \
    ros-humble-pymoveit2 \
    python3-colcon-common-extensions
# Phase 2 再加: ros-humble-topic-based-ros2-control
```

## 构建

```bash
cd /home/gds/projects/shengzhi/ur5_controller
source tools/source_env.sh
colcon build --symlink-install
source install/setup.bash
```

## 运行（Phase 1：mock，无需真机/Isaac）

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
# Phase 2 Isaac（需另起 isaac_scripts/ur5_isaac_moveit.py）
ros2 launch ur5_bringup full_stack.launch.py backend:=isaac

# Phase 3 真机 —— 先把 IP 填进 tools/robot_ip.sh（设了 UR5_HOST 后无需再带 robot_ip:=）
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

## 验收

见 [`docs/phase1_verification.md`](docs/phase1_verification.md)。**首期硬指标**：
mock 下 `demo_plan_execute` 全绿 + `demo_collision` 绕开障碍 = 运动栈打通。

## 后续：6D 抓取路线图

见 [`docs/perception_roadmap.md`](docs/perception_roadmap.md)（M3–M8：相机 / 6D 位姿 /
手眼标定 / 抓取编排 / 夹爪 / 力控）。

## 关键限制 / 已知点

- ur_rtde **不做规划/碰撞**；它只作为低级调试/标定工具保留（见 `ur5_controller/config.py`）。
- 此版本 ur_rtde control 侧无 `setStandardDigitalOut`/`setSpeedSlider`（IO 仅可读）。
- mock 下 MoveIt 必须 `use_sim_time:=true`（否则会去找未激活的 scaled 控制器）——已内置。
- **真机 CB3 固件需 ≥ 3.14**（3.9 的 primary 接口与 urcl 2.13.0 不兼容，会 SIGABRT）；官方最低 3.7。
- 真机 **calibration 尚未校准**（driver 有警告，TCP 有轻微偏差）——抓取前用 `ur_calibration` 工具校准。
- scaled 控制器对快速小目标可能返回 PREEMPTED（客户端可能误报失败，实际已到位）。
- Phase 2 的 TopicBasedSystem 在 Isaac Sim 有轨迹抖动报告（NVIDIA 论坛），有降级方案。
