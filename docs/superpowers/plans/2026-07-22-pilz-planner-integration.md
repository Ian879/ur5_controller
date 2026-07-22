# pilz 规划器集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 MoveIt 接入 pilz 规划器（PTP/LIN）做成 RViz 默认，OMPL 留作兜底——规划轨迹走直路、不再乱绕。

**Architecture:** fork `ur_moveit.launch.py` → `ur5_bringup/launch/moveit.launch.py`，把单 OMPL pipeline 换成多 pipeline（`planning_pipelines.{ompl,pilz}` + `default_planning_pipeline=pilz`），并加载合并的 joint_limits（velocity+accel，pilz 硬前置）；`full_stack` 切到我们的 moveit.launch.py。复用 `ur_moveit_config` 的 SRDF/kinematics/controllers/ompl 配置文件。

**Tech Stack:** ROS2 Humble, MoveIt2 2.5.x, `pilz_industrial_motion_planner 2.5.9`, `ur_moveit_config`, ament_cmake（ur5_bringup）, launch_ros.

**Spec:** `docs/superpowers/specs/2026-07-22-pilz-planner-integration-design.md`

## Global Constraints

- **conda 陷阱**：所有 ROS2 命令前 `source tools/source_env.sh`。
- **pilz pipeline 参数固定**：`planning_plugin=pilz_industrial_motion_planner/CommandPlanner`，`default_planner_config=PTP`，`capabilities=MoveGroupSequenceAction MoveGroupSequenceService`，request_adapters 用 4 个 Fix*（不含 AddTimeOptimal，pilz 自带时间参数化）。字段取自已验证的 `src/moveit_resources/panda_moveit_config/config/pilz_industrial_motion_planner_planning.yaml`。
- **默认 pipeline = pilz**，OMPL 留作 `planning_pipelines.ompl` 兜底。
- **joint_limits 必须 velocity+accel 都有**（pilz 读 `robot_description_planning.joint_limits`）。velocity 3.14159 rad/s（180°/s，ur_description）、accel 5.0 rad/s²（ur_moveit）、position ±6.2832 rad（±360°）。
- **ur5_bringup 是 ament_cmake**：`CMakeLists.txt` 已 `install(DIRECTORY launch config urdf ...)`，新文件自动安装（`--symlink-install` 下即时生效）。**不需改 CMakeLists**，但加 `pilz_industrial_motion_planner` 到 package.xml exec_depend。
- **moveit.launch.py = ur_moveit.launch.py 的 fork**：cp 后做 4 处精确编辑，不要重写整个文件。
- 验证最轻用 **mock 后端**（move_group 与后端无关）。

---

## File Structure

| 文件 | 责任 | 动作 |
|---|---|---|
| `src/ur5_bringup/config/joint_limits_pilz.yaml` | 合并 velocity+accel 的关节限位，加载为 robot_description_planning | 新建 |
| `src/ur5_bringup/package.xml` | 声明 pilz 依赖 | 改（加 exec_depend） |
| `src/ur5_bringup/launch/moveit.launch.py` | fork ur_moveit：move_group+rviz 装配 + 多 pipeline + 我们的 joint_limits | 新建（cp + 4 编辑） |
| `src/ur5_bringup/launch/full_stack.launch.py` | `_moveit_include` 从 ur_moveit.launch.py 切到我们的 moveit.launch.py | 改 |

---

### Task 1: joint_limits_pilz.yaml + pilz 依赖

**Files:**
- Create: `src/ur5_bringup/config/joint_limits_pilz.yaml`
- Modify: `src/ur5_bringup/package.xml`

**Interfaces:**
- Produces: `ur5_bringup/config/joint_limits_pilz.yaml`（6 UR 关节 velocity+accel+position），Task 2 的 moveit.launch.py 用 `load_yaml("ur5_bringup", "config/joint_limits_pilz.yaml")` 加载为 `robot_description_planning`。

- [ ] **Step 1: 写 `src/ur5_bringup/config/joint_limits_pilz.yaml`**

```yaml
# 合并的关节限位（pilz 硬前置：velocity + acceleration 都要有）。
# velocity / position 来自 ur_description/config/ur5/joint_limits.yaml（180°/s、±360°），
# acceleration 来自 ur_moveit_config/config/joint_limits.yaml（5.0 rad/s^2）。
# 加载为 move_group 的 robot_description_planning（见 moveit.launch.py）。
joint_limits:
  shoulder_pan_joint:
    has_position_limits: true
    min_position: -6.2832
    max_position: 6.2832
    has_velocity_limits: true
    max_velocity: 3.14159
    has_acceleration_limits: true
    max_acceleration: 5.0
  shoulder_lift_joint:
    has_position_limits: true
    min_position: -6.2832
    max_position: 6.2832
    has_velocity_limits: true
    max_velocity: 3.14159
    has_acceleration_limits: true
    max_acceleration: 5.0
  elbow_joint:
    has_position_limits: true
    min_position: -6.2832
    max_position: 6.2832
    has_velocity_limits: true
    max_velocity: 3.14159
    has_acceleration_limits: true
    max_acceleration: 5.0
  wrist_1_joint:
    has_position_limits: true
    min_position: -6.2832
    max_position: 6.2832
    has_velocity_limits: true
    max_velocity: 3.14159
    has_acceleration_limits: true
    max_acceleration: 5.0
  wrist_2_joint:
    has_position_limits: true
    min_position: -6.2832
    max_position: 6.2832
    has_velocity_limits: true
    max_velocity: 3.14159
    has_acceleration_limits: true
    max_acceleration: 5.0
  wrist_3_joint:
    has_position_limits: true
    min_position: -6.2832
    max_position: 6.2832
    has_velocity_limits: true
    max_velocity: 3.14159
    has_acceleration_limits: true
    max_acceleration: 5.0
```

- [ ] **Step 2: 加 pilz 依赖到 `src/ur5_bringup/package.xml`**

在现有 `<exec_depend>joint_trajectory_controller</exec_depend>` 之后追加一行：
```xml
  <exec_depend>pilz_industrial_motion_planner</exec_depend>
```

- [ ] **Step 3: 验证 yaml 合法 + 内容对**

Run:
```bash
source tools/source_env.sh
python3 -c "import yaml; d=yaml.safe_load(open('src/ur5_bringup/config/joint_limits_pilz.yaml')); jl=d['joint_limits']; assert set(jl)=={'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; assert all(jl[j]['has_velocity_limits'] and 'max_velocity' in jl[j] and jl[j]['has_acceleration_limits'] and 'max_acceleration' in jl[j] for j in jl); print('joint_limits OK: 6 joints, velocity+accel present')"
```
Expected: `joint_limits OK: 6 joints, velocity+accel present`

- [ ] **Step 4: build（让 config/ 的新文件进 install）**

Run:
```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup
```
Expected: build 成功，`install/ur5_bringup/share/ur5_bringup/config/joint_limits_pilz.yaml` 存在（`ls install/ur5_bringup/share/ur5_bringup/config/ | grep pilz`）。

- [ ] **Step 5: 提交**

```bash
git add src/ur5_bringup/config/joint_limits_pilz.yaml src/ur5_bringup/package.xml
git commit -m "feat(sp2): joint_limits_pilz.yaml（velocity+accel 合并，pilz 硬前置）+ pilz exec_depend"
```

---

### Task 2: moveit.launch.py（fork ur_moveit + 多 pipeline）

**Files:**
- Create: `src/ur5_bringup/launch/moveit.launch.py`（cp ur_moveit.launch.py 后 4 处编辑）

**Interfaces:**
- Consumes: Task 1 的 `ur5_bringup/config/joint_limits_pilz.yaml`；复用 `ur_moveit_config` 的 SRDF/kinematics/controllers/ompl_planning.yaml。
- Produces: `ur5_bringup/launch/moveit.launch.py`，接受与 ur_moveit.launch.py 相同的 args（ur_type/use_sim_time/launch_rviz/launch_servo 等），默认 pipeline=pilz。Task 3 的 full_stack include 它。

- [ ] **Step 1: cp ur_moveit.launch.py 为起点**

Run:
```bash
cp /opt/ros/humble/share/ur_moveit_config/launch/ur_moveit.launch.py src/ur5_bringup/launch/moveit.launch.py
```

- [ ] **Step 2: 编辑①——顶部加 fork 说明**

在 `src/ur5_bringup/launch/moveit.launch.py` 里，把：
```
#
# Author: Denis Stogl
```
替换为：
```
#
# Author: Denis Stogl (upstream ur_moveit.launch.py)
#
# ★ ur5_controller fork (SP2 pilz): 多 planning pipeline (pilz 默认 + OMPL 兜底) + 合并 joint_limits。
#   基于 /opt/ros/humble/share/ur_moveit_config/launch/ur_moveit.launch.py 复制后改动：
#   - planning_pipelines_config 取代单 OMPL pipeline（default_planning_pipeline=pilz）
#   - robot_description_planning 改加载 ur5_bringup/config/joint_limits_pilz.yaml
```

- [ ] **Step 3: 编辑②——robot_description_planning 改加载我们的 joint_limits**

把：
```python
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }
```
替换为：
```python
    # robot_description_planning: 用合并 joint_limits（velocity+accel，pilz 硬前置）。
    # ur_moveit 原版只加载 ur_moveit_config/config/joint_limits.yaml（仅 acceleration），pilz 缺 velocity 会报错。
    robot_description_planning = {
        "robot_description_planning": load_yaml("ur5_bringup", "config/joint_limits_pilz.yaml")
    }
```

- [ ] **Step 4: 编辑③——单 OMPL pipeline 换成多 pipeline**

把：
```python
    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
```
替换为：
```python
    # Planning Configuration —— 多 pipeline（pilz 默认 + OMPL 兜底），见 SP2 pilz 设计。
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    planning_pipelines_config = {
        "planning_pipelines": {
            "ompl": {  # OMPL 兜底（拥挤场景 pilz 解不出时 RViz 手切）
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
                "start_state_max_bounds_error": 0.1,
                **ompl_planning_yaml,
            },
            "pilz": {  # 默认；CommandPlanner 提供 PTP(关节最短)+LIN(末端直线)
                "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
                "request_adapters": "default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
                "default_planner_config": "PTP",
                "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
                "start_state_max_bounds_error": 0.1,
            },
        },
        "default_planning_pipeline": "pilz",
    }
```

- [ ] **Step 5: 编辑④——两处引用改名（move_group + rviz 节点）**

文件里 `            ompl_planning_pipeline_config,` 出现**两次**（move_group_node 和 rviz_node 的 parameters 列表里）。用**全部替换**把两处都改成 `planning_pipelines_config,`：
- old: `            ompl_planning_pipeline_config,`
- new: `            planning_pipelines_config,`
- replace_all: true（两处都要改）

改完后 `grep -n ompl_planning_pipeline_config src/ur5_bringup/launch/moveit.launch.py` 应**无输出**（变量名已全替）；`grep -n planning_pipelines_config src/ur5_bringup/launch/moveit.launch.py` 应有 3 处（定义 + move_group + rviz）。

- [ ] **Step 6: 语法 + 解析自检**

Run:
```bash
source tools/source_env.sh
python3 -c "import ast; ast.parse(open('src/ur5_bringup/launch/moveit.launch.py').read()); print('AST OK')"
ros2 launch ur5_bringup moveit.launch.py --show-args 2>&1 | grep -E "Arguments|'ur_type'|'use_sim_time'" | head
```
Expected: `AST OK`；`--show-args` 列出 ur_type/use_sim_time 等 args（证明 launch 文件被找到且解析通过）。

- [ ] **Step 7: 起 move_group 验证 pilz 加载为默认（standalone，不需控制器）**

终端 A：
```bash
source tools/source_env.sh
ros2 launch ur5_bringup moveit.launch.py ur_type:=ur5 launch_servo:=false launch_rviz:=false use_sim_time:=false
```
等 move_group 起来（日志无 pilz/CommandPlanner 加载错误、无 "planning_pipelines" 相关 error）。

终端 B（验证）：
```bash
source tools/source_env.sh
ros2 param get /move_group default_planning_pipeline
ros2 param get /move_group planning_pipelines 2>&1 | grep -iE "pilz|ompl" | head
ros2 service list | grep -iE "sequence|pilz" | head
```
Expected：
- `default_planning_pipeline` → `pilz`
- `planning_pipelines` 输出含 `pilz` 和 `ompl`
- service list 含 pilz 的 `sequence` action（`/execute_pilz_sequence_move` 或类似）

若 move_group 启动报 pilz 相关错误（如 "could not load plugin"、"joint X has no velocity limit"）→ 检查 §3 joint_limits 是否加载成功、planning_pipelines 字段是否拼对。Ctrl+C 关终端 A。

- [ ] **Step 8: build + 提交**

```bash
source tools/source_env.sh
colcon build --symlink-install --packages-select ur5_bringup
git add src/ur5_bringup/launch/moveit.launch.py
git commit -m "feat(sp2): moveit.launch.py fork——多 pipeline (pilz 默认 + OMPL 兜底) + 合并 joint_limits"
```

---

### Task 3: full_stack 切到我们的 moveit.launch.py + 端到端验证

**Files:**
- Modify: `src/ur5_bringup/launch/full_stack.launch.py`（`_moveit_include` 函数）

**Interfaces:**
- Consumes: Task 2 的 `ur5_bringup/launch/moveit.launch.py`（同 ur_moveit 的 args）。
- Produces: `full_stack` 用我们的 moveit.launch.py 起 move_group（pilz 默认）。

- [ ] **Step 1: 改 `_moveit_include`**

在 `src/ur5_bringup/launch/full_stack.launch.py` 的 `_moveit_include` 函数里，把 `FindPackageShare("ur_moveit_config")` 改成 `FindPackageShare("ur5_bringup")`、`ur_moveit.launch.py` 改成 `moveit.launch.py`。即把：
```python
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
```
替换为：
```python
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur5_bringup"), "launch", "moveit.launch.py"]
            )
        ),
```
（`launch_arguments` 那块——ur_type/use_sim_time/launch_servo/launch_rviz——**不动**，我们的 moveit.launch.py 兼容这些 args。）

- [ ] **Step 2: 解析自检**

Run:
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py --show-args 2>&1 | grep -E "Arguments|backend" | head -3
python3 -c "import ast; ast.parse(open('src/ur5_bringup/launch/full_stack.launch.py').read()); print('AST OK')"
```
Expected: 无解析错误；`AST OK`。

- [ ] **Step 3: 端到端验证（mock 后端，pilz 默认 + 路径变直）**

终端 A：
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=mock
```
等 move_group + rviz 起来。

终端 B（验证 pipeline）：
```bash
source tools/source_env.sh
ros2 param get /move_group default_planning_pipeline      # → pilz
ros2 run ur5_motion demo_plan_execute                     # 关节/位姿规划走 pilz PTP
```
Expected：
- `default_planning_pipeline` → `pilz`
- `demo_plan_execute` 全绿，**轨迹明显比 OMPL 直**（关节空间接近最短路径，不再反方向绕）。

RViz：拖末端球→Plan→看绿色轨迹是直路；MotionPlanning display 下拉切 LIN→末端走直线、切 OMPL→仍能规划（兜底）。

- [ ] **Step 4: 提交**

```bash
git add src/ur5_bringup/launch/full_stack.launch.py
git commit -m "feat(sp2): full_stack 切到我们的 moveit.launch.py（pilz 默认规划器）"
```

---

## Self-Review

**1. Spec 覆盖**：
- §1 架构（moveit.launch.py fork + full_stack 切换）→ Task 2 + Task 3 ✓
- §2 多 pipeline 配置 → Task 2 Step 4（planning_pipelines_config）✓
- §3 joint_limits → Task 1 ✓
- §4 RViz/客户端行为 → Task 3 Step 3 验证（pilz 默认、demo 路径变直、切 LIN/OMPL）✓
- §5 验证 → Task 2 Step 7（standalone param）+ Task 3 Step 3（端到端）✓
- 组件清单 4 项 → Task 1（yaml+package.xml）、Task 2（moveit.launch.py）、Task 3（full_stack）✓

**2. 占位符扫描**：joint_limits 6 关节全写（无 `...`）；4 处 fork 编辑全 old→new 精确给出；无 TBD/TODO ✓

**3. 类型/命名一致**：`planning_pipelines_config`（Task 2 定义）→ Task 2 Step 5 两处引用 + grep 自检；`load_yaml("ur5_bringup","config/joint_limits_pilz.yaml")`（Task 2 Step 3）与 Task 1 产物路径一致；`joint_limits_pilz.yaml` 跨任务同名 ✓

无问题。

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-22-pilz-planner-integration.md`.
