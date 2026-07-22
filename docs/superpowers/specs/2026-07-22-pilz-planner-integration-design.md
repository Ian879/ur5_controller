# pilz 规划器集成设计（多 pipeline，pilz 默认）

> 状态：已设计待实现 ｜ 分支：`sp1-isaac-motion-bridge` ｜ 日期：2026-07-22
> 起因：RViz 里 MoveIt plan 老是"反方向绕路"——根因是默认 OMPL 规划器只找"可行路"不找"好路"。

## 目标

给 MoveIt 接入 **pilz_industrial_motion_planner**（PTP=关节空间最短直走、LIN=末端笛卡尔直线），做成 **RViz 默认规划器**，OMPL 保留作兜底。规划出来的轨迹是直路/确定性路径，不再乱绕。

**非目标**：改 pymoveit2 客户端（planner_id=LIN 等留给 SP3 抓取）、力控、其它后端逻辑。

## 根因（已确认，非 bug）

MoveIt 默认用 OMPL（`RRTConnect`），采样型 RRT，目标=尽快找一条不撞的路，不优化路径质量 → 终点对、但路径绕/反方向兜。系统自带 `ur_moveit_config` **只配了 OMPL**（config 里只有 `ompl_planning.yaml`），pilz 没接入。要直路得用 pilz。

## 关键事实（实现前已验证）

- **pilz 已装**：`ros-humble-pilz-industrial-motion-planner 2.5.9`，`libpilz_industrial_motion_planner.so` 在，`CommandPlanner` 插件可加载。
- **ur_moveit 没有 pilz 钩子**：`ur_moveit.launch.py` 无 pilz/pipeline 相关 arg、无 skip-move_group 开关、只有一个 launch 文件；move_group 用**老式单 pipeline**（`move_group.planning_plugin=ompl_interface/OMPLPlanner`）。`IncludeLaunchDescription` 无法覆盖其内嵌参数。→ **接 pilz 必须接管 move_group 节点装配（fork）。**
- **多 pipeline 结构已验证**（仓库内 `src/moveit_resources/panda_moveit_config`）：panda `demo.launch.py` 用 `planning_pipelines(pipelines=["ompl","chomp","pilz"])`；pilz yaml 模板见 `config/pilz_industrial_motion_planner_planning.yaml`。Humble move_group 支持 `planning_pipelines`（map）+ `default_planning_pipeline`（string）。
- **pilz 要 joint velocity+acceleration**：ur_description 有 `max_velocity: 180°/s`（`has_velocity_limits: true`、`has_acceleration_limits: false`）；ur_moveit 的 `joint_limits.yaml` 只有 `max_acceleration: 5.0`（无 velocity）。pilz 读 `robot_description_planning.joint_limits`，需两者都有 → 要合并。
- **SRDF 是 xacro**：`ur_moveit_config/srdf/ur.srdf.xacro`（带 `ur_type:=ur5`），非裸 `.srdf`。ur_moveit 用 `Command([xacro, …, ur.srdf.xacro, ur_type:=ur5])` 处理，本设计照搬。
- **ur_moveit 配置文件可复用**（`load_yaml` 读 `/opt/ros/humble/share/ur_moveit_config`）：`kinematics.yaml`、`controllers.yaml`、`ompl_planning.yaml`、`srdf/ur.srdf.xacro`。

## §1 架构 / 文件结构

新建 `ur5_bringup/launch/moveit.launch.py`，**忠实镜像** `ur_moveit.launch.py` 的 move_group+rviz 装配（已验证可用），只把单 OMPL pipeline 换成多 pipeline。配置**文件**复用（不复制内容）：

| move_group 参数 | 来源 |
|---|---|
| `robot_description` | xacro(`ur_description/ur.urdf.xacro`, ur_type=ur5) — 同 ur_moveit |
| `robot_description_semantic` | xacro(`ur_moveit_config/srdf/ur.srdf.xacro`, ur_type=ur5) — 同 ur_moveit |
| `robot_description_planning` | **★新建** `ur5_bringup/config/joint_limits_pilz.yaml`（见 §3） |
| kinematics / controllers / trajectory_execution / planning_scene_monitor / warehouse | 同 ur_moveit（复用 yaml + 相同参数） |
| `planning_pipelines` | **★多 pipeline**（见 §2） |

`full_stack.launch.py` 的 `_moveit_include`：从 `IncludeLaunchDescription(ur_moveit.launch.py)` 改成 include 我们的 `moveit.launch.py`（签名兼容：接受 `use_sim_time`、`launch_rviz`）。

**装配方式选"镜像 ur_moveit"而非 `moveit_configs_utils`**：ur_moveit 的 SRDF-xacro / controllers / trajectory_execution 细节已验证，1:1 fork + 单点 delta（多 pipeline）风险最低、最"正式"。

## §2 多 pipeline 配置（核心）

> ⚠️ **实现期修正（2026-07-22）**：下面代码块里的**嵌套 dict 形式是错的**——Humble move_group 把
> `planning_pipelines` 当**名字列表**读，嵌套 dict 会被拒、回退到 CHOMP、pilz 不加载。实现采用的
> 正确形式（已验证，运行时 pilz 默认加载）是 `moveit_configs_builder.py:469-477` 的 canonical 写法：
> ```python
> planning_pipelines_config = {
>     "planning_pipelines": ["ompl", "pilz"],     # 名字列表（非 dict）
>     "default_planning_pipeline": "pilz",
>     "ompl": { ... },   # 每个 pipeline 的参数作为与 planning_pipelines 平级的顶层 key
>     "pilz": { ... },
> }
> ```
> 下面代码块保留作历史记录，以本修正为准。

替换 ur_moveit 的 `ompl_planning_pipeline_config`（单 pipeline）为多 pipeline：

```python
planning_pipelines = {
    "planning_pipelines": {
        "ompl": {  # 原样保留 OMPL 兜底
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
            **load_yaml("ur_moveit_config", "config/ompl_planning.yaml"),
        },
        "pilz": {  # 新增，字段取自已验证的 panda pilz yaml
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints",
            "default_planner_config": "PTP",
            "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction "
                "pilz_industrial_motion_planner/MoveGroupSequenceService",
            "start_state_max_bounds_error": 0.1,
        },
    },
    "default_planning_pipeline": "pilz",   # ← RViz 默认走 pilz
}
```

pilz 的 CommandPlanner 自动提供 **PTP**（关节空间最短，类 moveJ）+ **LIN**（末端直线，类 moveL）。

## §3 joint_limits（pilz 硬前置）

新建 `ur5_bringup/config/joint_limits_pilz.yaml`，**合并** ur_description 的 velocity + ur_moveit 的 acceleration，6 个 UR 关节各填全：

```yaml
joint_limits:
  shoulder_pan_joint:  { has_velocity_limits: true,  max_velocity: 3.14159,  has_acceleration_limits: true, max_acceleration: 5.0, has_position_limits: true, min_position: -6.2832, max_position: 6.2832 }
  shoulder_lift_joint: { has_velocity_limits: true,  max_velocity: 3.14159,  has_acceleration_limits: true, max_acceleration: 5.0, has_position_limits: true, min_position: -6.2832, max_position: 6.2832 }
  elbow_joint:         { has_velocity_limits: true,  max_velocity: 3.14159,  has_acceleration_limits: true, max_acceleration: 5.0, has_position_limits: true, min_position: -6.2832, max_position: 6.2832 }
  wrist_1_joint:       { has_velocity_limits: true,  max_velocity: 3.14159,  has_acceleration_limits: true, max_acceleration: 5.0, has_position_limits: true, min_position: -6.2832, max_position: 6.2832 }
  wrist_2_joint:       { has_velocity_limits: true,  max_velocity: 3.14159,  has_acceleration_limits: true, max_acceleration: 5.0, has_position_limits: true, min_position: -6.2832, max_position: 6.2832 }
  wrist_3_joint:       { has_velocity_limits: true,  max_velocity: 3.14159,  has_acceleration_limits: true, max_acceleration: 5.0, has_position_limits: true, min_position: -6.2832, max_position: 6.2832 }
```
（velocity=180°/s≈3.14159 rad/s 来自 ur_description；accel=5.0 rad/s² 来自 ur_moveit；position±360°。加载为 `robot_description_planning`。）数值以实际 yaml 为准，实现时核对 ur_description/ur5/joint_limits.yaml。

## §4 RViz 与 pymoveit2 客户端行为

- **RViz**：多 pipeline + 默认 pilz → MotionPlanning display 规划器下拉默认 pilz（PTP/LIN），可切 OMPL。拖末端球→Plan 轨迹是直路。rviz 节点传入 `planning_pipelines` 参数（镜像 ur_moveit）。
- **pymoveit2 客户端**：默认走 pilz。`move_to_joint`→PTP、`move_to_pose`→默认 PTP；现有 `demo_plan_execute` 不改即跑、路径变直。LIN（SP3 抓取）需客户端传 `planner_id="LIN"`，本次不改。
- **pilz 严格性**：目标无法用直路/PTP 无碰撞到达时 pilz 失败 → 切 OMPL 兜底（预期）。

## §5 验证

用 **mock 后端**（move_group 与后端无关，最轻）：
```bash
source tools/source_env.sh
ros2 launch ur5_bringup full_stack.launch.py backend:=mock
# 另一终端：
ros2 param get /move_group default_planning_pipeline      # → pilz
ros2 param get /move_group planning_pipelines             # → 含 ompl, pilz
ros2 service list | grep -i sequence                      # pilz MoveGroupSequenceAction 在
ros2 run ur5_motion demo_plan_execute                     # 关节/位姿规划走 pilz PTP，路径直
```
RViz：拖末端球→Plan 看直路（PTP）；下拉切 LIN→末端直线；切 OMPL→仍能规划（兜底）。

**实现时验证的风险点**：pilz 报"缺 velocity limit"→ §3 joint_limits 没合对；pilz 对个别目标失败→切 OMPL（预期）；三后端：mock 验证规划器足矣，isaac/real 抽测仍能规划。

## 组件清单

| 文件 | 动作 |
|---|---|
| `ur5_bringup/launch/moveit.launch.py` | 新建：镜像 ur_moveit 的 move_group+rviz 装配 + 多 pipeline |
| `ur5_bringup/config/joint_limits_pilz.yaml` | 新建：velocity+accel 合并的关节限位 |
| `src/ur5_bringup/launch/full_stack.launch.py` | 改：`_moveit_include` 换成 include 我们的 moveit.launch.py |
| `ur5_bringup/package.xml` / `setup.py` | 确认 data_files 含 config/*.yaml + launch/*（若 setup.py 用 glob 则自动） |

## 范围外 / 推迟

- pymoveit2 客户端传 `planner_id="LIN"`（SP3 抓取接近用）。
- 力控 / 其它后端逻辑。
- CHOMP 等其它 pipeline（YAGNI，只要 pilz+OMPL）。

## 决策记录

1. **多 pipeline（pilz 默认 + OMPL 兜底）** 而非 pilz-only：pilz 严格，留 OMCL 兜底覆盖拥挤场景。
2. **镜像 ur_moveit 装配** 而非 `moveit_configs_utils`：ur_moveit 的 SRDF-xacro/controllers/trajectory_execution 已验证，1:1 fork 风险最低。
3. **新建合并 joint_limits** 而非依赖运行时合并：保证 pilz 一定拿到 velocity+accel，确定性强。
4. **默认 pipeline=pilz**：满足"RViz 默认走直路"诉求，OMCL 仍可在 RViz 手切。
