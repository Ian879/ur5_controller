# 真机部署（方案A）：Jetson 硬件层 + GPU 服务器规划层

> 2026-09-28 确定的部署架构。阶段0（launch 拆分 + 仿真验证）已完成并合入 main；
> 本文档从阶段1（连真机臂）开始是现场 runbook。

## 架构

```
[UR5 控制柜] ←以太网─ [Jetson Xavier NX 8G（Docker ros:humble）]
                        hardware.launch.py backend:=real
                        = ur_robot_driver + controller_manager
                        + Lebai 串口夹爪 + Orbbec 相机
                              ↕ DDS（fastrtps，同 ROS_DOMAIN_ID=0，有线 LAN）
[GPU 服务器（现有环境）]
  planning.launch.py use_sim_time:=false   = move_group + world_sync
  demo_grasp / grasp_planner / FoundationPose
```

关键原则：

- 两层**只靠 DDS 通信**，可独立启停（仿真里天天演练这个缝：`tools/bringup_sim.sh`
  的 hw/plan 两窗就是这两层共址跑）。
- 全系统统一 ROS2 **Humble**。Jetson 20.04 宿主上**不用**原生 Noetic/Foxy/Galactic，
  不做任何 ros1_bridge / 跨 distro 桥（不支持、不稳，见 2026-09-28 会话结论）。
- RMW 统一 `rmw_fastrtps_cpp`（cyclonedds 会让 isaac 桥卡顿，已回退，gotcha #17）。
- 8G 内存余量充足：硬件层全栈 ~2-3G（不含任何 CUDA 活——统一内存，别往 Jetson 塞）。

## 一、Jetson 侧准备（一次性）

1. **系统**：JetPack 5.x（Ubuntu 20.04）。宿主机 headless 用（SSH），省 ~0.7G 内存。
2. **Docker**：
   ```bash
   sudo apt-get update && sudo apt-get install -y docker.io docker-compose-v2
   sudo usermod -aG docker $USER   # 重新登录生效
   ```
3. **构建硬件层镜像**（仓库同步到 Jetson，仓库根目录）：
   ```bash
   docker compose -f deploy/jetson/compose.yaml up -d --build
   ```
   arm64 首次构建 15-40min（ur-robot-driver deb + colcon 编 orbbec/lebai）。
   - **代理**：本内网上网走认证代理（宿主机 `http_proxy`/`https_proxy` 环境变量
     里有凭据），容器/构建默认不继承 → apt 全部超时。构建时显式传：
     ```bash
     docker build --network=host \
       --build-arg http_proxy="$http_proxy" --build-arg https_proxy="$https_proxy" \
       --build-arg no_proxy="$no_proxy" \
       -f deploy/jetson/Dockerfile -t ur5-hw:humble .
     ```
     （`http_proxy`/`https_proxy` 是 Docker 预定义 build-arg，只注入 RUN 环境，
     不烤进镜像；`--network=host` 是因为 BuildKit 沙箱默认无 egress。凭据只存在于
     本机 shell 环境，**不要**写进仓库文件。）
   - 若 OrbbecSDK 缺 arm64 预编译库报错（叠加代理参数，见上条）：`docker build -f deploy/jetson/Dockerfile
     --build-arg HW_PKGS="ur5_bringup lebai_driver" -t ur5-hw:humble .` 先验证其余
     部分，orbbec 单独处理（查 OrbbecSDK_ROS2 的 arm64 支持版本）。
4. **DDS 大消息缓冲**（与服务器侧同款，宿主机执行一次）：
   ```bash
   sudo tee /etc/sysctl.d/99-ros2.conf <<'EOF'
   net.core.rmem_max=16777216
   net.core.wmem_max=16777216
   EOF
   sudo sysctl --system
   ```

## 二、UR 控制柜准备（一次性，示教器上）

1. 装 **externalcontrol URCap**（Polyscope 版本兼容性先在 UR 官网确认），
   Host IP 填 Jetson 的 IP，端口 50002。
2. 上电 → brake release → `robot_mode == RUNNING`、保护停机复位。
3. 切 **remote 模式**（ROS2 driver 控制的前提）。
4. 保守速度：改 `ur_controllers.yaml` 调低 joint 速度/加速度限值再跑首次验证。

## 三、网络规划

- UR 柜、Jetson、GPU 服务器接**同一交换机**，全部静态 IP（示例 192.168.1.x 段）。
- 控制链路**禁 WiFi**（DDS 发现 + RTDE 都吃抖动）。
- 三方 `ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 一致
  （镜像和 compose 已固化这两个 env，服务器侧 `tools/source_env.sh` 已带）。
- 服务器侧的 UDP 大缓冲已调过（gotcha #16）；Jetson 侧见上文 sysctl。

## 四、阶段1 runbook：连真机臂（先不接夹爪/相机）

**Jetson 上**（起硬件层）：

```bash
docker exec -it ur5_hw bash
ros2 launch ur5_bringup hardware.launch.py backend:=real robot_ip:=192.168.1.102
# robot_ip 也可写进 tools/robot_ip.sh 的 UR5_HOST 后省略
```

**服务器上**（起规划层）：

```bash
cd ~/projects/shengzhi/ur5_controller
source tools/source_env.sh
ros2 launch ur5_bringup planning.launch.py use_sim_time:=false launch_rviz:=true
```

**缝合检查**（证明跨机缝通，对应仿真里已验证的两向）：

| 检查 | 命令（在服务器） | 期望 |
|---|---|---|
| 感知向 | `ros2 topic info /joint_states` | Publisher ≥1（来自 Jetson） |
| 感知向 | RViz | 机器人模型跟随真机关节 |
| 感知向 | `ros2 run ur5_motion grasp_plan` | 读到 TF/joint_states、打出 IK 候选 |
| 命令向 | `ros2 run ur5_motion go_home` | 真机低速回 home（首次前确认限速！） |

首次运动注意：手边急停（UI 上的 Emergency Stop 或物理急停）；go_home 成功后再
逐步放开速度。跨机 move_group 断网时控制器会把当前轨迹执行完——**停不下来**，
这是此架构的已知特性，急停必须在 UR 柜侧。

## 五、阶段2 指针（硬件到场后做）

- **Lebai 夹爪 action 桥**：`hardware.launch.py` real 分支起的 `serial_gripper`
  提供 services（`gripper/set_position` 等）而非 `control_msgs/GripperCommand`
  action；demo 的 `GripperClient`（现对 isaac_gripper_action）需要一个真机版
  ——写一个包 serial_gripper services 的 action server，接口对齐
  `lebai_gripper_controller/gripper_cmd`，demo 代码零改动。
  抓取验证逻辑依赖夹爪位置反馈（夹住 ≈0.59 rad vs 空夹 ≈0）——串口协议里
  确认能读到实际开口。
- **Orbbec 手眼标定**（eye-in-hand）：替换 `camera.launch.py` 里 `tool0→camera_link`
  占位 TF（easy_handeye 或手写 AX=XB）。FP 的 z 误差靠侧抓策略兜底（已在仿真验证）。
- **真机分割**：深度 GT mask 在真机不存在——SAM 对小物体需要 box/多点提示
  （轻量检测器给框，或首帧人工点选后 FP track 接管）。这是真机闭环最大的风险项，
  建议在阶段2 开始前先离线在真机图片上验证分割方案。

## 六、镜像内容清单（审阅用）

- apt：`ros-humble-ur-robot-driver`（拉齐 ros2_control/JTC/dashboard 等）、
  `ros-humble-rosidl-default-generators`（lebai_interfaces/orbbec_camera_msgs 要）、
  `ros-humble-tf-transformations`（lebai_driver 运行时依赖；**注意 PyPI 上没有
  tf-transformations 这个包**——是 ROS apt 专属，别走 pip）、colcon/git、
  `libusb-1.0-0-dev libudev-dev`（Orbbec）。
- colcon 子集：`ur5_bringup lebai_driver orbbec_camera`（+ 其构建依赖 lebai_interfaces/
  orbbec_camera_msgs/orbbec_description）。规划层包（ur5_motion/isaac_moveit/
  moveit_resources）**不进**镜像。
- 已知未构建：`lebai_lm3_support`（lebai_driver 的 exec_depend，Lebai 机械臂 URDF，
  UR5 场景用不到；若 serial_gripper 运行时报缺再补进 HW_PKGS）。
