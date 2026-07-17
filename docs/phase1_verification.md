# Phase 1 验收清单：mock 下运动栈打通

目标：在 **mock（fake hardware）** 下验证 MoveIt 规划 + 执行 + 碰撞检测全链路。不需要真机、不需要 Isaac Sim。

## 前置
- [ ] 已 `sudo apt install ros-humble-pymoveit2 python3-colcon-common-extensions`
- [ ] 已 `colcon build --symlink-install` 且 `source install/setup.bash`

## Step 1：起栈
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=mock
```
判定：
- [ ] 终端无致命错误，RViz 弹出，UR5 可见
- [ ] `ros2 control list_controllers` 中 `joint_trajectory_controller` 与 `joint_state_broadcaster` 均 **active**
  （其余 UR 专用控制器如 `io_and_status_controller` 在 mock 下可能 warning，正常）
- [ ] `ros2 topic echo /joint_states --once` 有 6 个关节值
- [ ] `ros2 topic list` 含 `/move_action`、`/execute_trajectory`、`/move_group/apply_planning_scene`

**若 move_group 卡在 TF/时间**：根因是 `use_sim_time:=true` 但无 `/clock` 发布者（mock 没有时钟源）。
- 先确认是否真卡（`ros2 topic echo /clock` 若无发布者即为此因）。
- 临时变通：在 `full_stack.launch.py` 的 mock 分支把 `_moveit_include("true")` 改为 `"false"`，
  并把 `ur_control` 的 `initial_joint_controller` 保持 `joint_trajectory_controller`（mock 仍用它）。
  但 move_group 的控制器默认会回到 scaled——此时需让 MoveIt 显式用 joint_trajectory_controller
  （见 ur_moveit_config/config/controllers.yaml 的 default 字段）。Isaac/真机有 /clock，不存在此问题。

## Step 2：RViz 交互（半自动）
在 RViz 的 **MotionPlanning** 面板：拖动末端球到新位姿 → **Plan & Execute**。
- [ ] 规划成功，轨迹播放，机器人移动到目标

## Step 3：Python 客户端（全自动）
```bash
ros2 run ur5_motion demo_plan_execute
```
- [ ] 每个目标点打印 `[OK]`，无 `ABORTED`
- [ ] `/joint_states` 随执行实时变化

## Step 4：碰撞检测
```bash
ros2 run ur5_motion demo_collision
```
- [ ] RViz 中出现红色障碍盒 `demo_obstacle`
- [ ] 规划出的绿色轨迹**绕开**障碍盒（或日志 `[OK] 找到无碰撞路径并执行`）

## 通过 = 首期硬指标达成
运动栈（规划 + 执行 + 碰撞检测）打通。下一步：Phase 2（Isaac Sim）。
