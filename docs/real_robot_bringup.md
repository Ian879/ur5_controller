# 真机 bring-up 安全清单（Phase 3）

> 真机动起来**不可逆**。第一次上电务必照此走，手不离急停。

## 0. 一次性准备（每台机器人只做一次）

> 本机已确认：机器人 = **CB3 真机，Polyscope 3.9.0**（满足官方最低 3.7）；
> PC 与机器人同网段 **10.67.64.0/24**（PC=`10.67.64.64`，机器人=`10.67.64.5`）。

### 0a. 装 External Control URCap（**强制**，不装 driver 无法接管）
1. 把 URCap 文件弄到机器人（文件在本机：`/opt/ros/humble/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap`）：
   - USB 拷贝（最简单，免密码），或
   - scp：`scp /opt/ros/humble/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap ur@10.67.64.5:/programs/`（CB3 默认账号 `ur` / 密码 `easybot`）。
2. 示教器 Welcome → **Setup Robot → URCaps** → 点 `+` → 选 `externalcontrol-1.0.5.urcap` → 出现 "External Control" + 提示重启 → **重启机器人**。
3. 重启后 Program Robot → **Installation** 标签 → **External Control** → **Custom IP 填 `10.67.64.64`**（=本机 enp0s31f6，与机器人同网段；端口别动）。
4. 新建程序，把 **External Control** 节点拖进程序树 → 保存。

### 0b. 网络
- PC 与机器人同网段、能 `ping` 通（已确认）。
- PC 防火墙放行 30001–30004、50001–50003（机器人需 reverse 连回 PC）。
- ⚠️ 已知坑：若不装 URCap / External Control 程序没跑，driver 连上后 `ur_ros2_control_node` 会
  在 `MasterboardData::parseWith` 抛 `urcl::UrException` → SIGABRT → controller_manager 起不来。

## 1. 配置 IP（**唯一要改的地方**）
编辑 `tools/robot_ip.sh`，把 IP 改成你 UR5 的真实 IP：
```bash
export UR5_HOST="192.168.1.102"
```
`source tools/source_env.sh` 会自动加载它。之后所有真机命令**不用再带 `robot_ip:=`**。

## 2. 示教器准备（每次上电）
按顺序确认（任一项不满足都**不要**让 ROS 下发运动）：
- [ ] **上电**（Power On）
- [ ] **松刹车**（Start / Release Brakes）→ 示教器状态变为 **Running**（或 Idle，可被外部控制）
- [ ] **关闭保护停机**：若有红色保护停机，在示教器点 **Reset** 复位
- [ ] **REMOTE CONTROL** 模式（设置里开启，使能外部控制）—— 否则 driver 不能启动 external_control
- [ ] **安全模式 = NORMAL**（不是 Reduced/Protective Stop）
- [ ] **急停按钮就位、可一把拍下**
- [ ] **工作区清空**，机械臂活动范围内无人、无障碍
- [ ] **速度滑块调低（建议 10%–25%）**——scaled 控制器会按它缩放，首次移动更稳

## 3. 启动（PC 侧）
```bash
cd /home/gds/projects/shengzhi/ur5_controller
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=real launch_rviz:=true
```
RViz 会弹出，显示机器人**当前真实姿态**。

## 4. 连上后先验证（**不动机器人**）
新终端：
```bash
source tools/source_env.sh
ros2 control list_controllers        # 应见 scaled_joint_trajectory_controller + joint_state_broadcaster 均 active
ros2 topic echo /joint_states --once # 关节值 = 真机当前角度（轻轻手动抬起机械臂会变化）
ros2 topic echo /io_and_status_controller/robot_mode --once   # 应为 RUNNING(7)
ros2 topic echo /io_and_status_controller/safety_mode --once # 应为 NORMAL(0)
```
- [ ] 控制器 active、`/joint_states` 反映真实角度、`safety_mode==0`、**无 protective stop / e-stop**

## 5. 第一次移动（**手不离急停**）
在 RViz 的 **MotionPlanning** 面板：
1. Planning Goal 拖一个**很小的**位移（如末端抬高 5–10 cm），先别管朝向。
2. 点 **Plan**（**不**点 Execute）→ 看绿色轨迹**是否合理、会不会撞自己/工作台**。
3. 确认安全后，**手按在急停上**，点 **Execute**。机器人缓慢移动到目标。
4. 反复几次小范围移动，确认一切正常，再逐步扩大范围/跑 demo。

## 6. 用脚本（确认上面都 OK 之后）
```bash
ros2 run ur5_motion demo_plan_execute   # 会做 4 个连续动作，含一个笛卡尔点
```
首次建议把 `demo_plan_execute.py` 里的目标改成更小范围、或注释掉部分步骤再跑。

## 常见问题
- **driver 起不来 / 卡在 connecting**：检查 IP、网络、REMOTE_CONTROL 是否开、URCap 是否装。
- **`robot is not running` / 启动 external_control 失败**：示教器没开 REMOTE CONTROL，或有保护停机。
- **protective stop 频繁**：速度/加速度太大，或工作区碰撞 → 调低 ur_controllers.yaml 的速度限制 + TP 速度滑块。
- **保护停机后 ROS 端报错**：在示教器 Reset 后，driver 的 controller_stopper 会自动重新激活控制器；若没恢复，重启 launch。

## 速度调低（可选，首次更稳）
编辑 `/opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml`（或在自定义控制器配置里）
调低 `scaled_joint_trajectory_controller` 的 `constraints` / 默认速度；或更简单——首次全程用 TP 速度滑块 10%。
