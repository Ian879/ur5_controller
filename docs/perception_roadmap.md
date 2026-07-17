# 6D 位姿抓取路线图（Phase 1 之后）

首期只做运动栈（规划+执行+碰撞）。抓取算法侧后续按以下里程碑推进。下面是选型推荐与依赖，
不是实现细节——到时再设计。

## 选型推荐

### 相机
- **RealSense D435i**（eye-in-hand，腕部）：首选。价廉、有 IMU、ROS2 `realsense2_camera` 成熟、
  深度有效距离 0.3–3 m 适配桌面抓取，UR5 末端负载内可挂。
- RealSense D455（eye-to-hand，外部固定）：更大基线、更准深度，适合大场景；需额外支架。
- Isaac Sim：用 `isaacsim.sensors.rtx` RTX 相机仿真 D435i 参数，与真机无缝替换。

### 6D 位姿估计
- **已知 CAD 工件 → FoundationPose**（`NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation`）：SOTA、
  zero-shot 注册、NVIDIA 官方维护、与 Isaac/ROS2 同栈、无需训练。conda env `robot` 的
  PyTorch+CUDA 正好跑推理。输出 `vision_msgs/Detection3DArray`。
- **未知物体 → GraspNet-1Billion / AnyGrasp**：直接给 SE(3) grasp pose，无需 CAD。

### 手眼标定
- eye-in-hand：`easy_handeye2`（ROS2 版）+ ArUco/ChArUco 板。
- Isaac Sim：用 `ArticulationView.get_link_transforms()` 的 ground-truth TF 闭环校验算法。

### 夹爪
- Robotiq 2F-85（position controller）。首期可用「虚拟盒子夹爪」先验位姿，再接实物。

## 里程碑

| 里程碑 | 内容 | 依赖 |
|---|---|---|
| **M3 感知接入** | 相机驱动 + 6D 位姿节点发 `Detection3DArray`；物体入 PlanningScene 作 CollisionObject | Phase 1 + 相机标定 |
| **M4 抓取位姿生成** | Detection3D 位姿 → pre-grasp(上方 10cm) + grasp + lift 三个笛卡尔点；考虑接近向量(top/side) | M3 |
| **M5 抓取编排** | state machine 或 BehaviorTree.CPP：Detect→Plan pre-grasp→Execute→Plan grasp(Cartesian)→Close→Lift→Place | M4 + M6 |
| **M6 夹爪集成** | ur_description xacro 加夹爪 URDF + ros2_control gripper controller | M5 |
| **M7 手眼标定** | eye-in-hand：15+ 位姿采集；解 AX=XB 得 `camera_link→tool0` 静态 TF；残差 <2mm/<1°（用 `ur5_controller/config.py` + ur_rtde 读真 TCP ↔ MoveIt tool0 比对） | M3 |
| **M8 闭环鲁棒** | 力控接触检测（`tool_contact_controller`/`force_mode_controller`，ur_controllers 已提供）+ 失败重试 | M6 |

## 关键风险
- FoundationPose 推理延迟（~1–2 s/帧）：用 `image_transport` + 异步 service，不阻塞运动栈。
- eye-in-hand 视野受限：必要时加 eye-to-hand 全局相机做初始定位。
- 抓取编排可先用简单 state machine（rclpy），复杂后再上 BehaviorTree.CPP + MoveIt BT actions。
