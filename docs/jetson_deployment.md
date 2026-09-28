# UR5 抓取系统真机部署完整教程（方案A）

> 版本：2026-09-28（阶段0 已合入 main：`3cfd20b` `298e324` `6744a83`）
> 架构决策：2026-09-28 确定方案A —— Jetson 只做硬件层，规划与感知留在 GPU 服务器。
> 本文档是**从零到抓取闭环**的完整教程；已在仿真/开发机上验证的写 ✓，
> 需要硬件到场才能验证的写 ⏳（首次执行时按实际情况修正并回填本文档）。

---

## 目录

- [0. 系统总览](#0-系统总览)
- [1. 硬件清单与连线](#1-硬件清单与连线)
- [2. 网络规划](#2-网络规划)
- [3. UR 控制柜准备（一次性）](#3-ur-控制柜准备一次性)
- [4. Jetson 系统准备（从零）](#4-jetson-系统准备从零)
- [5. 构建硬件层镜像](#5-构建硬件层镜像)
- [6. 服务器侧准备（确认清单）](#6-服务器侧准备确认清单)
- [7. 阶段1：真机臂联调 runbook ⏳](#7-阶段1真机臂联调-runbook-)
- [8. 阶段2：夹爪 + 相机 ⏳](#8-阶段2夹爪--相机-)
- [9. 阶段3：FoundationPose 感知闭环 ⏳](#9-阶段3foundationpose-感知闭环-)
- [10. 日常操作速查](#10-日常操作速查)
- [11. 故障排查总表](#11-故障排查总表)

---

## 0. 系统总览

### 0.1 架构图

```
                        ┌────────────────────────────────────────────┐
                        │            GPU 服务器（现有环境）             │
                        │  planning.launch.py  = move_group + world_sync │
                        │  demo_grasp / grasp_planner（pymoveit2 客户端）│
                        │  FoundationPose：fp_server + fp_node + 分割    │
                        └───────────────▲────────────────────────────┘
                                        │  DDS（fastrtps，ROS_DOMAIN_ID=0，
                                        │  有线 LAN，全部 topic/service/action）
┌──────────────┐  以太网  ┌─────────────┴──────────────────────────┐
│  UR5 控制柜   │◄────────►│  Jetson Xavier NX 8G（Docker ros:humble）│
│ (10.67.64.5) │ 500Hz    │  hardware.launch.py backend:=real       │
└──────────────┘  RTDE    │   = ur_robot_driver + controller_manager │
                           │   + Lebai 串口夹爪 + Orbbec Gemini2 相机  │
                           └────────────────────────────────────────┘
```

### 0.2 职责表

| 机器 | 角色 | 跑什么 | 不跑什么 |
|---|---|---|---|
| Jetson NX 8G | **硬件层** | `hardware.launch.py backend:=real`：ur_robot_driver、controller_manager、scaled_joint_trajectory_controller、robot_state_publisher、Lebai 夹爪驱动、Orbbec 相机驱动、world→base TF | MoveIt、planner、demo、FP、RViz、一切 CUDA |
| GPU 服务器 | **规划层 + 感知** | `planning.launch.py use_sim_time:=false`：move_group、world_sync；demo_grasp、grasp_planner；fp_server、fp_node、分割 | 硬件驱动 |
| UR 控制柜 | 执行器 | externalcontrol URCap（受 Jetson 的 driver 指挥） | — |

### 0.3 设计原则（为什么这样分）

1. **FoundationPose 留服务器是唯一解**：register/refine 需 10G+ 显存；NX 是 8G 统一内存（CPU/GPU 共享），跑不动。
2. **全系统统一 ROS2 Humble，零桥接**：Jetson 20.04 宿主原生只有 Noetic(ROS1)/Foxy/Galactic(EOL)，都不用；Humble 走 Docker。跨 distro（Foxy↔Humble）通信官方不支持，不做 ros1_bridge。
3. **两层只靠 DDS**：move_group 不关心 hardware interface 在哪台机器上——`FollowJointTrajectory` action 跨机透明。这个缝在仿真里已经天天演练（`tools/bringup_sim.sh` 的 hw/plan 双窗就是这两层共址跑，双向缝合验证过 ✓）。
4. **RMW 统一 `rmw_fastrtps_cpp`**（cyclonedds 会让 isaac 桥卡顿，gotcha #17）。
5. 内存账：Jetson 硬件层全栈 ~2-3G（宿主 headless ~0.6G + docker ~0.3G + ur_driver/控制器 ~0.5G + 相机 ~0.8G），8G 余量充足——但别往 Jetson 塞任何 CUDA 活。

### 0.4 分阶段路线

| 阶段 | 内容 | 前置 |
|---|---|---|
| 阶段0 | launch 拆分 + 仿真缝合验证 + Jetson 脚手架 | ✅ 已完成 |
| 阶段1 | 真机臂联调（不接夹爪/相机） | 本文 §3–§7 |
| 阶段2 | Lebai 夹爪 + Orbbec 相机 + 手眼标定 | 阶段1 通过 |
| 阶段3 | FP 感知闭环（先解决真机分割） | 阶段2 通过 |

---

## 1. 硬件清单与连线

| 设备 | 数量 | 连接 | 备注 |
|---|---|---|---|
| UR5 + 控制柜 | 1 | 控制柜网口 → 交换机 | IP `10.67.64.5`（现 `tools/robot_ip.sh` 里配的） |
| Jetson Xavier NX 8G（NVMe/SD + 电源） | 1 | 网口 → 交换机 | JetPack 5.x / Ubuntu 20.04 |
| Orbbec Gemini 2 | 1 | USB3 → Jetson；机械安装在腕上（eye-in-hand） | 阶段2 才需要 |
| Lebai 串口夹爪 | 1 | RS485/USB → Jetson（`/dev/ttyUSB0`） | 阶段2 才需要 |
| 交换机 + 网线 | 1 + 3 | 服务器/Jetson/UR 柜 | 全有线，见 §2 |
| 急停按钮 | ≥1 | UR 柜侧 | **联调期间手边必须有** |

连线注意：

- 控制链路（UR 柜 ↔ Jetson、Jetson ↔ 服务器）**全部有线**，禁 WiFi。
- 相机接 Jetson 的 **USB3** 口（深度流带宽）。
- ⏳ Lebai 夹爪供电/接线方式按到场实物确认，回填本节。

---

## 2. 网络规划

### 2.1 IP 分配（示例，按现场调整后回填）

| 设备 | IP | 备注 |
|---|---|---|
| UR5 控制柜 | `10.67.64.5` | 已配在 `tools/robot_ip.sh`（UR5_HOST） |
| Jetson | `10.67.64.xx`（静态） | |
| GPU 服务器 | 现有地址 | 与 Jetson 同网段/同交换机 |

### 2.2 统一约定（缺一不可）

```bash
ROS_DOMAIN_ID=0                    # 三方一致
RMW_IMPLEMENTATION=rmw_fastrtps_cpp # 三方一致（服务器 source_env.sh 已带；镜像/compose 已固化）
```

- 服务器侧 `tools/source_env.sh` 自动 export 这两个 ✓。
- Jetson 侧镜像 `ENV` + compose `environment` 已固化 ✓。

### 2.3 代理（本内网特有）

本内网上网走**认证代理**（宿主机 shell 的 `http_proxy`/`https_proxy`）。要点：

- **运行时不需要代理**：DDS 和 UR 柜都在内网直连。
- **构建时需要**（apt/pip 要出网）：容器默认不继承代理 → 构建命令见 §5.2。
- 凭据只存在于本机 shell 环境，**不要写进任何仓库文件**。
- `no_proxy` 已含 `10.0.0.0/8`（UR 柜、Jetson、服务器互访不受影响）。

### 2.4 时间同步（建议）

三方装 chrony 或至少 `timedatectl` 对时。真机无 sim time，跨机 TF/日志时间戳都来自墙钟；不对时会让排查日志很痛苦（DDS 本身不依赖时间同步）。

---

## 3. UR 控制柜准备（一次性）⏳

在示教器上完成：

1. **装 externalcontrol URCap**（ur_robot_driver 的前提）：
   - 先在 UR 官网确认 URCap 版本与 Polyscope 版本兼容；
   - URCap 里 Host IP 填 **Jetson 的 IP**，端口 `50002`。
2. **远程模式**：设置 → 系统 → 远程控制 → 切 Remote（ROS2 driver 控制的前提）。
3. 上电 → brake release（松刹车）→ 确认 `robot_mode == RUNNING`、保护停机已复位。
4. **保守速度**：首次联调前把关节速度/加速度限值调低（可在 ur 控制器 yaml 或示教器安装配置里）。
5. **量 world→base_link 抬升**：机器人底座/台子装好后，量出 base_link 相对 world 的位姿，
   启动时传给 `real_robot.launch.py`（默认 identity，不传就是"基座贴地"）：
   ```bash
   ros2 launch ur5_bringup hardware.launch.py backend:=real \
     world_to_base_x:=0 world_to_base_y:=0 world_to_base_z:=0.75 \
     world_to_base_roll:=0 world_to_base_pitch:=0 world_to_base_yaw:=0
   ```
6. 安全平面：在示教器里设工作区安全平面（挡住人员侧）。

---

## 4. Jetson 系统准备（从零）

### 4.1 刷机 ⏳

- JetPack 5.x（L4T 35.x，Ubuntu 20.04）。用 SDK Manager 或 SD 卡镜像。
- 装好后**headless 使用**（SSH 进去），省 ~0.7G 内存、少一堆桌面进程抢 CPU。
- 磁盘：镜像 + workspace 要 ~8G，NVMe/SD 至少 32G。

### 4.2 装 Docker ⏳

```bash
# 内网直装（若可达）：
sudo apt-get update && sudo apt-get install -y docker.io
# 若出网要代理：
sudo -E apt-get update && sudo -E apt-get install -y docker.io   # -E 带上代理 env

sudo usermod -aG docker $USER    # 重新登录生效
docker run --rm hello-world      # 验证（需代理时：先 export http_proxy=... 再 docker pull）
```

> 本教程主命令用纯 `docker build/run`，**不依赖 docker compose**（仓库里的
> `deploy/jetson/compose.yaml` 是等价替代，装了 compose 插件可用 `docker compose`）。

### 4.3 DDS 大消息缓冲（一次性，与服务器同款）

```bash
sudo tee /etc/sysctl.d/99-ros2.conf <<'EOF'
net.core.rmem_max=16777216
net.core.wmem_max=16777216
EOF
sudo sysctl --system
```

### 4.4 夹爪串口 udev 别名（阶段2 前）⏳

USB 串口设备名会漂移（ttyUSB0/ttyUSB1）。拿到夹爪的 vendor/product id 后：

```bash
# 示例：插上夹爪后查 idVendor/idProduct
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct' | head -2
sudo tee /etc/udev/rules.d/99-lebai-gripper.rules <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", SYMLINK+="lebai_gripper"
EOF
sudo udevadm control --reload && sudo udevadm trigger
# 之后设备名固定为 /dev/lebai_gripper，启动时 gripper_port:=/dev/lebai_gripper
```

### 4.5 同步仓库到 Jetson

```bash
git clone --recurse-submodules <仓库地址>   # ★ 必须 --recurse-submodules：
#   src/third_party/OrbbecSDK_ROS2（v2-main）和 lebai-ros-sdk（humble-dev）是子模块，
#   不带的话第三步构建直接失败（orbbec_camera/lebai_driver 源码是空的）
```

已有克隆补子模块：`git submodule update --init --recursive`。

---

## 5. 构建硬件层镜像

### 5.1 构建（直连网络环境）

```bash
cd <仓库根>
docker build --network=host -f deploy/jetson/Dockerfile -t ur5-hw:humble .
```

### 5.2 构建（本内网/需代理）

```bash
cd <仓库根>
docker build --network=host \
  --build-arg http_proxy="$http_proxy" --build-arg https_proxy="$https_proxy" \
  --build-arg no_proxy="$no_proxy" \
  -f deploy/jetson/Dockerfile -t ur5-hw:humble .
```

- `http_proxy`/`https_proxy` 是 Docker 预定义 build-arg，只注入 RUN 环境，**不烤进镜像**。
- `--network=host`：BuildKit 沙箱默认无 egress（开发服务器实测 bridge/host 模式下 apt 全超时）。
- arm64 首次构建 15-40min（ur-robot-driver deb + colcon 编 orbbec/lebai）。amd64 开发机上
  烟测构建约 3min ✓（2026-09-28 实测通过）。

### 5.3 起容器

```bash
docker run -d --name ur5_hw \
  --network host --ipc host --privileged \
  --device /dev/ttyUSB0 \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e UR5_HOST=10.67.64.5 \
  ur5-hw:humble sleep infinity
```

硬要求：`--network host`（DDS 发现 + 对 UR 柜 500Hz RTDE 低延迟）、`--ipc host`（fastrtps
共享内存）、`--privileged + --device`（串口夹爪 / 相机）。阶段2 后 `/dev/ttyUSB0` 换成
udev 别名 `/dev/lebai_gripper`。

### 5.4 验证镜像

```bash
docker exec -it ur5_hw bash    # 进容器（.bashrc 已自动 source ROS + workspace ✓）
ros2 pkg list | grep -E 'ur5_bringup|ur_robot_driver|orbbec_camera|lebai_driver'   # 4 个都得在
ros2 launch ur5_bringup hardware.launch.py --show-args    # 能打印参数表
```

### 5.5 镜像内容清单（审阅用）

- apt：`ros-humble-ur-robot-driver`（拉齐 ros2_control/JTC/dashboard 等）、
  `ros-humble-rosidl-default-generators`（lebai_interfaces/orbbec_camera_msgs 要）、
  `ros-humble-tf-transformations`（lebai_driver 运行时依赖；**PyPI 上没有这个包**，
  是 ROS apt 专属，别走 pip）、orbbec 的 15 个构建依赖（cv_bridge/image_transport/
  glog/nlohmann-json3-dev 等，见 Dockerfile 第二个 RUN）。
- colcon 子集（5 包）：`ur5_bringup`、`lebai_interfaces`、`lebai_driver`、
  `orbbec_camera_msgs`、`orbbec_camera`。规划层（ur5_motion/isaac_moveit/
  moveit_resources）**不进**镜像。
- 已知未构建：`lebai_lm3_support`（lebai_driver 的 exec_depend，Lebai 机械臂 URDF，
  UR5 用不到；若 serial_gripper 运行时报缺，加进 Dockerfile 的 `HW_PKGS` 重 build）。

### 5.6 构建 FAQ（开发机烟测实际踩过的）

| 症状 | 原因 | 处置 |
|---|---|---|
| `Unable to locate package` 一大片 | 容器没走代理，apt 拉不到索引 | §5.2 带代理 build-arg |
| `E: Package 'nlohmann-json-dev' has no installation candidate` | jammy 里叫 `nlohmann-json3-dev` | 已修 ✓ |
| `pip3 install tf-transformations` → `from versions: none` | PyPI 上没有此包 | 用 apt 的 `ros-humble-tf-transformations`（已修 ✓） |
| orbbec_camera CMake 找不到 `cv_bridge` 等 | ur 驱动不拉这些依赖 | 已在 Dockerfile 补齐 ✓ |
| orbbec 在 arm64 报缺预编译库 ⏳ | OrbbecSDK 平台库不全 | `--build-arg HW_PKGS="ur5_bringup lebai_driver"` 先验其余，orbbec 单独处理 |

---

## 6. 服务器侧准备（确认清单）

服务器（GPU 开发机）现有环境已就绪，逐项确认即可：

```bash
cd ~/projects/shengzhi/ur5_controller
colcon build                                   # ✓ 已构建（install/ 在）
source tools/source_env.sh                     # ✓ 自动处理 conda 陷阱 + RMW + DOMAIN + UR5_HOST
ros2 launch ur5_bringup planning.launch.py --show-args   # ✓ 能打印
ping 10.67.64.5                                # ⏳ 能通 UR 柜
ping <jetson-ip>                               # ⏳ 能通 Jetson
```

已知服务器侧陷阱（详见 `tools/source_env.sh` 头注释）：

- **conda base 陷阱**：任何 ros2 命令前必须 `source tools/source_env.sh`（把 PATH 里的
  miniconda 剔掉，强制 /usr/bin/python3），否则 rclpy ABI 不符静默收发失败。
- **tmux 环境缓存**：改过 RMW/DOMAIN 的 tmux 必须 `tmux kill-server` 重开，旧 server
  带着旧 env 静默失联。

---

## 7. 阶段1：真机臂联调 runbook ⏳

> 先不接夹爪/相机，只连臂。目标：跨机缝双向打通 + 低速运动一次。

### 7.0 安全须知（先读）

- **急停在手边**（UR 柜上的急停或示教器 Emergency Stop）。
- **断网行为**：跨机 move_group 断连时，控制器会把**当前轨迹执行完才停**——停不下来，
  只能 UR 侧急停。这是此架构的已知特性。
- 首次运动用低速（§3.4 调过限值）；工作区内无人、无障碍。
- 首次联调建议 UR 柜设**虚拟安全墙**（安全平面）。

### 7.1 启动顺序

**① Jetson 上（起硬件层）——一键：**

```bash
cd <仓库根>
./deploy/jetson/jetson_up.sh              # 等价于：确保容器在 + docker exec 起 launch（前台，Ctrl-C 停）
# 变体：
#   ROBOT_IP=10.67.64.5 ./deploy/jetson/jetson_up.sh      # 换 IP
#   ./deploy/jetson/jetson_up.sh --build                  # 镜像不在先构建
#   ./deploy/jetson/jetson_up.sh --shell                  # 只进容器调试
#   ./deploy/jetson/jetson_up.sh world_to_base_z:=0.75    # 抬升补偿等 launch 参数透传（§3.5）
#   ./deploy/jetson/jetson_down.sh                        # 停容器
```

观察点：ur_driver 连上控制柜、`scaled_joint_trajectory_controller` 激活、
`/joint_states` 开始跳。**外部控制**：UR 示教器会自动进入程序运行（externalcontrol 接管）。

**② 服务器上（规划层 + 感知）——一键：**

```bash
cd ~/projects/shengzhi/ur5_controller
./tools/bringup_real.sh                   # tmux 5 窗：plan | fp_server | mask | fp_node | client
# 变体：RVIZ=0 / ATTACH=0 / MASK=sam|gt；停：./tools/bringup_real_down.sh
# 脚本会探测 /joint_states：Jetson 侧没起会 warn（起来后自动互通，无需重启本侧）
```

（手工等价命令：`source tools/source_env.sh && ros2 launch ur5_bringup
planning.launch.py use_sim_time:=false launch_rviz:=true`，fp 三件套见 §9。）

### 7.2 缝合检查表（逐项过）

| # | 检查 | 命令（在服务器） | 期望 |
|---|---|---|---|
| 1 | 感知向 | `ros2 topic info /joint_states` | Publisher ≥1（来自 Jetson） |
| 2 | 感知向 | RViz | 机器人模型实时跟随真机关节 |
| 3 | 感知向 | `ros2 run ur5_motion grasp_plan` | 等到 TF 后打出 33 候选 IK 过滤（不动臂） |
| 4 | 命令向 | `ros2 run ur5_motion go_home` | **低速**回 home，日志"复位完成 ✓" |

3、4 两项分别就是仿真里验证过的双向缝合（`grasp_plan` = plan 层消费 hw 层的
joint_states/TF/`/compute_ik`；`go_home` = move_group→控制器 action 链）。

### 7.3 停止

- 服务器：规划层窗口 Ctrl-C。
- Jetson：硬件层窗口 Ctrl-C；容器留着（`docker stop ur5_hw` 彻底停）。
- UR 柜：确认回到本地模式（示教器可操作）。

### 7.4 阶段1 常见故障

| 症状 | 排查 |
|---|---|
| 两边互相看不见节点/topic | ① RMW 一致？② ROS_DOMAIN_ID 一致？③ 都有线？④ 服务器在 tmux 里改过 env → `tmux kill-server` |
| Jetson 侧控制器不激活 | ur_driver 连上柜子了吗（URCap host IP 填的是 Jetson IP？remote 模式？）；`/ros_control` 日志 |
| move_group 起来了但规划全失败 | `ros2 topic echo /joint_states` 有数据吗；robot_description（服务器 moveit 自己生成，不依赖 Jetson）日志 |
| `goal 未能到达控制器` | 控制器名对不上：真机是 `scaled_joint_trajectory_controller`；moveit_controllers 指向它 |
| RTDE 抖动/丢包 | 换网线/换口；确认没经 WiFi；Jetson `top` 看有无进程抢满 CPU |

---

## 8. 阶段2：夹爪 + 相机 ⏳

### 8.1 Lebai 夹爪

1. 接线 + udev 别名（§4.4），启动加 `gripper_port:=/dev/lebai_gripper`。
2. `hardware.launch.py backend:=real` 已自动起 `lebai_driver/serial_gripper`
   （提供 `gripper/*` services，如 `set_position`）。
3. **必做：action 桥**（demo 的 `GripperClient` 说话的接口是
   `control_msgs/GripperCommand` action，对齐仿真里的
   `lebai_gripper_controller/gripper_cmd`）：写一个小 rclpy 节点把 serial_gripper 的
   services 包成同名 action server，demo 代码零改动。**抓取验证逻辑**（夹住 ≈0.59 rad
   vs 空夹 ≈0）依赖位置反馈——串口协议里确认能读到实际开口，读不到就要换验证手段。
4. **TCP 标定重做**：`GRIPPER_TIP_OFFSET`/`TCP_LATERAL_X` 环境变量现在灌的是 isaac
   lebai 夹爪的仿真标定值（0.17127 / 0.0265），真机装好后必须重新标定（GRASP_INSPECT
   法：相机看手指跨位 + 深度图）。

### 8.2 Orbbec Gemini 2 相机

1. `hardware.launch.py backend:=real` 已自动起 `orbbec_camera/gemini2.launch.py`
   （`publish_tf:=false`、`depth_registration:=true`，TF 子树由本 launch 统一发）。
2. 话题契约（下游 FP 依赖）：
   - `/camera/color/image_raw` + `/camera/color/camera_info`（frame: `camera_color_optical_frame`）
   - `/camera/depth/image_rect_raw` + `/camera/depth/camera_info`
3. 带宽：fp_node 按需触发，观测瞬间几帧压缩 RGB，GbE 绰绰有余；Jetson 上**不开点云**。
4. **手眼标定（eye-in-hand）**：替换 `camera.launch.py` 里 `tool0→camera_link` 占位 TF
   （z=0.05, identity）为实测值（easy_handeye 或手写 AX=XB）。标定质量直接决定 FP 位姿
   在 base 系的准确性——FP 的 z 误差靠侧抓策略兜底（仿真已验证），但外参错了 xy 也歪。

### 8.3 真机分割（本阶段最大风险项）

仿真里用"深度带 GT mask"兜底 SAM 剥不开小物体的问题——**真机没有 GT mask**。动手
顺序建议：

1. 先离线：真机相机拍一组照片，验证 SAM 的 **box 提示**（一个轻量检测器给框，或首帧
   人工点一下）能否稳定剥出可乐罐 mask。
2. 过了再进在线管线（`sam_segmentation.py` 换提示方式；`MASK=sam` 的 bringup 路径已在）。
3. 备选：FP 自身 mask refine；或首帧人工 + FP track 模式接管。

---

## 9. 阶段3：FoundationPose 感知闭环 ⏳

服务器侧照常（bringup 里 fp_server/mask/fp_node 三个窗口的逻辑），区别只是图像来源
从 isaac 桥换成 Jetson 的相机：

```bash
# 服务器（有现成编排，但真机不需要 isaac/hw 窗口——按需手工起 fp 三件套）
cd ~/projects/jiayin && PYTHONPATH=$PWD conda run --no-capture-output -p envs/fp python server/fp_server.py
# mask / fp_node 同 tools/bringup_sim.sh 窗 5/6 的命令（真机不带 use_sim_time）
```

- fp_node 已知 gotcha：阻塞后要重启节点本身（不带 isaac 桥重启）。
- 真机无 /clock：fp_node **不要** `use_sim_time:=true`（那是 isaac 专属）。
- 端到端：

```bash
# 服务器（planning 层已在跑）
POSE_SOURCE=fp ros2 run ur5_motion demo_grasp
```

预期行为与仿真一致：LOOK 观测 → FP 位姿 → v2 planner（吃 6D 姿态，倒罐自动出竖直
下探候选）→ 侧抓 → 抓取验证（开口 ≥0.25 rad 判夹住）→ 搬运放置。

---

## 10. 日常操作速查

### 10.1 每日启动（真机）

```bash
# ① Jetson（硬件层）
./deploy/jetson/jetson_up.sh                       # Ctrl-C 停 launch；jetson_down.sh 停容器
# ② 服务器（规划层 + 感知，一键 5 窗）
cd ~/projects/shengzhi/ur5_controller
./tools/bringup_real.sh                            # 停：./tools/bringup_real_down.sh
# ③ demo（tmux client 窗或任一 source 过的 shell）
POSE_SOURCE=fp ros2 run ur5_motion demo_grasp
```

### 10.2 命令对照（仿真 vs 真机）

| 层 | 仿真 | 真机 |
|---|---|---|
| 一键编排 | `./tools/bringup_sim.sh`（7 窗） | 服务器 `./tools/bringup_real.sh`（5 窗）+ Jetson `./deploy/jetson/jetson_up.sh` |
| 硬件层 | tmux hw 窗（isaac 桥 + `hardware.launch.py backend:=isaac`） | Jetson 容器内 `hardware.launch.py backend:=real` |
| 规划层 | tmux plan 窗（`planning.launch.py use_sim_time:=true`） | `planning.launch.py use_sim_time:=false` |
| 单机全栈 | `ros2 launch ur5_bringup full_stack.launch.py backend:=mock|isaac` | （跨机不用） |
| 停止 | `./tools/bringup_down.sh` | 服务器 `bringup_real_down.sh` + Jetson `jetson_down.sh` |

### 10.3 常用诊断

```bash
ros2 topic info /joint_states            # 缝活了没
ros2 run ur5_motion grasp_plan           # 只查 IK，不动臂
ros2 run ur5_motion go_home              # 最小运动链
ros2 control list_controllers            # 容器内（Jetson）看控制器状态
docker logs ur5_hw --tail 50             # 容器日志
```

---

## 11. 故障排查总表

**通信类**

| 症状 | 根因 | 处置 |
|---|---|---|
| 跨机完全互相看不见 | RMW / ROS_DOMAIN_ID 不一致 | 两端 `env | grep -E 'RMW\|DOMAIN'` 对齐 |
| 服务器 tmux 里改了 env 没生效 | tmux server 缓存旧环境 | `tmux kill-server` 重开 |
| action/service 偶发超时 | UDP buffer 太小 | 服务器已调（gotcha #16）；Jetson 见 §4.3 |
| 构建时 apt 全超时 | 容器没走代理 | §5.2 |

**控制类**

| 症状 | 根因 | 处置 |
|---|---|---|
| goal 发出后机器人不响应就报成功/失败异常 | 控制器名不匹配（isaac 用非 scaled，真机用 scaled） | 检查 moveit_controllers 指向 `scaled_joint_trajectory_controller` |
| 关节值 ±360° 越缠越大 | PhysX 累加不回卷（**仅仿真**） | demo 已有 `_junwrap`；真机无此问题 |
| 断网后臂继续动完轨迹 | 架构已知特性 | UR 侧急停 |

**感知类**

| 症状 | 根因 | 处置 |
|---|---|---|
| FP 位姿 z 偏 ~10cm | FP 已知特性 | 侧抓策略兜底（对 z 不敏感）；标定 `GRIPPER_TIP_OFFSET` |
| fp_node 无响应 | 已知阻塞 gotcha | 重启 fp_node |
| SAM 剥不出小物体 | 单点提示弱 | box/多点提示（§8.3） |
| TF extrapolation 报错 | sim/wall 时戳混用 | 真机一切用墙钟，别带 `use_sim_time` |

---

## 附：相关文档

- 阶段0 实施计划：`docs/superpowers/plans/2026-09-28-real-robot-split-hw-plan.md`
- 仿真 bringup 细节：`tools/bringup_sim.sh` 头注释
- 抓取管线分析：`docs/grasp_pipeline_analysis.md`
- 服务器机器陷阱：`tools/source_env.sh` 头注释 + 项目记忆（conda、tmux、ros2controlcli）
