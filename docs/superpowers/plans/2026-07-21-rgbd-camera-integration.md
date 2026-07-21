# RGBD 相机集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 Orbbec Gemini 2 RGBD 相机接入三后端运动栈，真机驱动进栈 + 后端无关的相机契约（真机/Isaac 同一套 topic+frame）+ 单一环境入口。

**Architecture:** 薄 launch include 复用 `~/ros2_ws` 的 Orbbec 包原样；新增 `camera.launch.py` 按后端 gate（real→起 Orbbec，isaac→靠桥，mock→仅 TF），所有后端共用占位 TF 子树；Isaac 桥把非标准相机 topic 重映射成 Orbbec/ROS 标准约定；`source_env.sh` 加 Orbbec overlay 成全项目唯一入口。

**Tech Stack:** ROS2 Humble, `orbbec_camera` (v2-main, `~/ros2_ws`), pymoveit2 栈（不动）, Isaac Sim 6.0.1 ROS2 bridge (OmniGraph), launch_ros, tf2_ros static_transform_publisher.

**Spec:** `docs/superpowers/specs/2026-07-21-rgbd-camera-integration-design.md`

## Global Constraints

- **conda 陷阱**：所有 ROS2 命令前必须 `source tools/source_env.sh`（退出 conda base + source humble）。
- **相机契约固定**（下游唯一接口）：`/camera/color/image_raw`、`/camera/color/camera_info`（frame `camera_color_optical_frame`）；`/camera/depth/image_rect_raw`、`/camera/depth/camera_info`（frame `camera_depth_optical_frame`）。
- **占位 TF**：`tool0→camera_link`（mount，z=0.05，identity 旋转）+ `camera_link→{color,depth}_optical_frame`（identity）。M7 手眼标定精修。
- **不 vendor Orbbec 包**：从 `~/ros2_ws` overlay 用，不复制进本仓库。
- **pointcloud / 手眼标定 / 物理安装验证**：本计划范围外（deferred）。
- 相机当前**未装机**，验证只到"驱动起 + TF 树连通 + RViz 可见"，不做真·eye-in-hand 闭环。

**测试约定（重要）**：ROS launch/集成没有单测；本计划的"测试"= 精确命令 + 预期输出的集成验证（仿现有 `tools/check_topics.sh` 模式）。`source_env.sh` 有可写的真实断言测试。

---

## File Structure

| 文件 | 责任 | 动作 |
|---|---|---|
| `tools/source_env.sh` | 全项目唯一环境入口（conda 退出 + humble + Orbbec overlay + 我们 overlay + robot_ip） | 改 |
| `ur5_bringup/launch/camera.launch.py` | 相机集成 launch：占位 TF 子树 + 后端 gate（real→include Orbbec） | 新建 |
| `ur5_bringup/launch/full_stack.launch.py` | 在 `_setup` 里 include camera.launch.py | 改 |
| `isaac_scripts/ur5_isaac_bridge.py` | 相机 OmniGraph：重映射 topic/frame + 加 depth camera_info | 改 |
| `tools/check_camera.sh` | 相机 topic/frame 自检（跨后端复用） | 新建 |
| `README.md` | 相机集成章节 + 统一 source 说明 | 改 |

---

### Task 1: 统一环境入口 source_env.sh（Orbbec overlay + 修 cwd bug）

**Files:**
- Modify: `tools/source_env.sh`（全文小改：提前 THIS_DIR、加 Orbbec overlay step、修 step 6 的 `$(pwd)`）

**Interfaces:**
- Produces: `source tools/source_env.sh` 之后，环境含 humble + `~/ros2_ws/install` overlay + 本仓库 `install/setup.bash`，无论从哪个子目录 source 都生效。下游任务依赖此来 `ros2 pkg list | grep orbbec_camera` 能找到包。

- [ ] **Step 1: 写失败测试（shell 断言）**

新建临时测试脚本（验证完删）：

```bash
cat > /tmp/test_source_env.sh <<'EOF'
#!/bin/bash
# 从子目录 source，验证：orbbec 包可见 + 本仓库 install 已 source + 无 conda 在 PATH
set -u
cd /tmp   # ★刻意非仓库根，触发旧 bug
source /home/gds/projects/shengzhi/ur5_controller/tools/source_env.sh >/dev/null 2>&1

fail=0
# 1) orbbec overlay 进来了
ros2 pkg list 2>/dev/null | grep -q '^orbbec_camera$' || { echo "FAIL: orbbec_camera 不在 pkg list"; fail=1; }
# 2) 本仓库 overlay 进来了（ur5_motion 是我们的包）
ros2 pkg list 2>/dev/null | grep -q '^ur5_motion$' || { echo "FAIL: ur5_motion 不在 pkg list（cwd bug 没修）"; fail=1; }
# 3) PATH 无 conda
echo "$PATH" | grep -qE 'miniconda|anaconda' && { echo "FAIL: PATH 残留 conda"; fail=1; }

[ "$fail" = "0" ] && echo "PASS: source_env.sh 从子目录 source 全 OK" || echo "FAIL"
exit $fail
EOF
chmod +x /tmp/test_source_env.sh
```

前置（确保本仓库已 build，否则 ur5_motion 不会在）：`source tools/source_env.sh && colcon build --symlink-install`（若 `install/` 已存在可跳过）。

- [ ] **Step 2: 跑测试验证它失败**

Run: `/tmp/test_source_env.sh`
Expected: `FAIL: ur5_motion 不在 pkg list（cwd bug 没修）`（因为旧脚本 step 6 用 `$(pwd)`=当前目录=/tmp，没 source 到仓库 install；orbbec 那条也可能 FAIL 因为还没加 overlay step）。

- [ ] **Step 3: 实现 — 改 `tools/source_env.sh`**

把 `THIS_DIR` 计算从脚本末尾（step 7 处）提到 step 4 之前，并在 step 4 前插入 Orbbec overlay，把 step 6 改用 `THIS_DIR`。

具体：在 `# 4) source 系统 ROS2 Humble` 这段（行 23-28）之后、`# 5) RMW...`（行 30）之前，插入：

```bash
# 4.5) ★ Orbbec SDK overlay（~/ros2_ws，SP2 相机集成）★
#    Gemini 2 的 orbbec_camera 包在独立工作空间，按 colcon overlay 依赖顺序
#    （humble underlay 之后、本仓库 overlay 之前）source。不存在则跳过。
THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$THIS_DIR/.." && pwd)"
ORBBEC_WS="${ORBBEC_WS:-$HOME/ros2_ws}"
if [ -f "$ORBBEC_WS/install/setup.bash" ]; then
    source "$ORBBEC_WS/install/setup.bash"
else
    echo "[source_env.sh] 提示: $ORBBEC_WS/install/setup.bash 不存在（未装 Orbbec 则忽略）" >&2
fi
```

然后把原 step 6（行 34-37）：

```bash
# 6) 本工作空间的 overlay（若已 colcon build）
if [ -f "$(pwd)/install/setup.bash" ]; then
    source "$(pwd)/install/setup.bash"
fi
```

改为：

```bash
# 6) 本仓库工作空间的 overlay（若已 colcon build）—— 用 THIS_DIR 而非 $(pwd)，
#    使从任意子目录 source 都生效。
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
fi
```

删掉原 step 7 里重复的 `THIS_DIR=...` 那一行（行 41，已提到上面），保留 robot_ip.sh source 那两行（行 42）。

- [ ] **Step 4: 跑测试验证它通过**

Run: `/tmp/test_source_env.sh`
Expected: `PASS: source_env.sh 从子目录 source 全 OK`

- [ ] **Step 5: 清理 + 提交**

```bash
rm /tmp/test_source_env.sh
git add tools/source_env.sh
git commit -m "feat(sp2): source_env.sh 统一入口 — 加 Orbbec overlay + 修 cwd bug"
```

---

### Task 2: camera.launch.py（占位 TF 子树 + 后端 gate）

**Files:**
- Create: `ur5_bringup/launch/camera.launch.py`

**Interfaces:**
- Consumes: Task 1 的 Orbbec overlay（`FindPackageShare("orbbec_camera")` 才能找到 `gemini2.launch.py`）。
- Produces: 一个 launch，接受 `backend:=mock|isaac|real`。real 时起 Orbbec（`publish_tf:=false`），所有后端发占位 TF 子树 `tool0→camera_link→{color,depth}_optical_frame`。Task 3 会 include 它。

- [ ] **Step 1: 写文件 `ur5_bringup/launch/camera.launch.py`**

```python
"""RGBD 相机集成 launch（Orbbec Gemini 2，eye-in-hand）。

按后端 gate：
  real  → 起 Orbbec 驱动（orbbec_camera/gemini2.launch.py, publish_tf:=false）
  isaac → 不起 Orbbec（图像来自 Isaac 桥，桥已把 topic 重映射成标准名）
  mock  → 不起相机（仅 TF 子树，下游收不到图，符合预期）

所有后端发同一个占位 TF 子树，让相机帧挂到机器人 TF 树：
  tool0 → camera_link                       (mount 占位：z=0.05, identity 旋转)
  camera_link → camera_color_optical_frame  (identity —— 定义 camera_link≡光学帧)
  camera_link → camera_depth_optical_frame  (identity)

定义 camera_link 与光学帧重合 ⇒ mount TF (tool0→camera_link) 独占位置+朝向，无需猜
body→optical 旋转。占位值；物理安装后量真实偏移，M7 手眼标定精修。

下游感知（SP3/SP4）契约：
  /camera/color/image_raw, /camera/color/camera_info      frame: camera_color_optical_frame
  /camera/depth/image_rect_raw, /camera/depth/camera_info frame: camera_depth_optical_frame
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, PathJoinSubstitution


def _static_tf(name, frame_id, child_frame_id, x="0.0", y="0.0", z="0.0",
               yaw="0.0", pitch="0.0", roll="0.0"):
    """static_transform_publisher，参数风格与 sim_isaac.launch.py 一致。"""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw, "-P", pitch, "-R", roll,
        ],
    )


def _orbbec_include():
    """real 后端：include Orbbec Gemini 2 驱动。publish_tf:=false（TF 子树由本 launch 发）。"""
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orbbec_camera"), "launch", "gemini2.launch.py"]
            )
        ),
        launch_arguments={
            "publish_tf": "false",          # TF 子树由本 launch 统一发
            "depth_registration": "true",   # 深度对齐到 color 帧（HW）
        }.items(),
    )


def _setup(context, *args, **kwargs):
    backend = LaunchConfiguration("backend").perform(context)
    actions = []

    # ===== 占位 TF 子树（所有后端）=====
    actions.append(_static_tf(
        "tool0_to_camera_link", "tool0", "camera_link",
        x="0.0", y="0.0", z="0.05",  # ★占位 mount；物理安装后改实测值，M7 精标
    ))
    actions.append(_static_tf(
        "camera_link_to_color_optical", "camera_link", "camera_color_optical_frame",
    ))
    actions.append(_static_tf(
        "camera_link_to_depth_optical", "camera_link", "camera_depth_optical_frame",
    ))

    # ===== 后端 gate =====
    if backend == "real":
        actions.append(_orbbec_include())
    # isaac: 图像来自桥（terminal A 的 ur5_isaac_bridge.py，已重映射到标准名）
    # mock:  无相机
    elif backend not in ("isaac", "mock"):
        raise RuntimeError(f"camera.launch.py: 未知 backend '{backend}'")

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend", default_value="real", choices=["mock", "isaac", "real"]
            ),
            OpaqueFunction(function=_setup),
        ]
    )
```

- [ ] **Step 2: 验证 mock 后端（TF 子树，不需相机/不需真机）**

Run:
```bash
source tools/source_env.sh && source install/setup.bash
ros2 launch ur5_bringup camera.launch.py backend:=mock
```
（另一个终端）：
```bash
source tools/source_env.sh
ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame
```
Expected: 打印出 `tool0 → camera_color_optical_frame` 的变换（translation z≈0.05，rotation identity），无 "frame not found"。Ctrl+C 退出 launch。

- [ ] **Step 3: 验证 real 后端（需相机插着 USB）**

Run:
```bash
source tools/source_env.sh && source install/setup.bash
ros2 launch ur5_bringup camera.launch.py backend:=real
```
（另一终端）：
```bash
ros2 topic list | grep /camera/
ros2 topic hz /camera/color/image_raw
```
Expected: 看到 `/camera/color/image_raw`、`/camera/color/camera_info`、`/camera/depth/image_rect_raw`、`/camera/depth/camera_info`；`topic hz` 有稳定频率（~30Hz）。若相机没插 → 跳过此步（标 "需硬件"），Step 2 已证明 launch 结构正确。

- [ ] **Step 4: 提交**

```bash
git add ur5_bringup/launch/camera.launch.py
git commit -m "feat(sp2): camera.launch.py — 后端 gate + 占位 TF 子树（Orbbec real/Isaac/mock）"
```

---

### Task 3: full_stack.launch.py include camera.launch.py

**Files:**
- Modify: `ur5_bringup/launch/full_stack.launch.py`（`_setup` 函数，world_sync 之后加 include）

**Interfaces:**
- Consumes: Task 2 的 `camera.launch.py`（接受 `backend` 参数）。
- Produces: `ros2 launch ur5_bringup full_stack.launch.py backend:=real` 现在自动带相机。

- [ ] **Step 1: 改 `full_stack.launch.py` 的 `_setup`**

在 `_setup` 里 `# ===== world_sync: 所有后端共用 =====` 那个 `actions.append(Node(...world_sync...))` 之后、`return actions` 之前，加相机 include。

加一个 import（文件顶部 `from launch_ros.actions import Node` 附近）：

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import PathJoinSubstitution
```
（`IncludeLaunchDescription`/`AnyLaunchDescriptionSource`/`PathJoinSubstitution` 文件顶部已有，按需只补缺的——`PathJoinSubstitution` 和 `FindPackageShare` 已有；`IncludeLaunchDescription`/`AnyLaunchDescriptionSource` 已有。确认不重复 import。）

在 `_setup` 的 world_sync Node 之后插入：

```python
    # ===== camera: 所有后端带（camera.launch.py 内部 gate real→起 Orbbec）=====
    # real: Orbbec 驱动 + 占位 TF；isaac: 仅占位 TF（图像来自桥）；mock: 仅 TF。
    actions.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ur5_bringup"), "launch", "camera.launch.py"]
                )
            ),
            launch_arguments={"backend": backend}.items(),
        )
    )
```

注意：`backend` 变量在 `_setup` 里已 `perform(context)` 成字符串，可直接传入。

- [ ] **Step 2: 验证 full_stack mock 后端带相机 TF**

Run:
```bash
source tools/source_env.sh && source install/setup.bash
ros2 launch ur5_bringup full_stack.launch.py backend:=mock
```
（另一终端，等栈起来后）：
```bash
ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame
```
Expected: 打印变换（z≈0.05）。证明 full_stack 把 camera.launch.py 拉起来了，TF 树连通到 tool0。Ctrl+C 退栈。

- [ ] **Step 3: 提交**

```bash
git add ur5_bringup/launch/full_stack.launch.py
git commit -m "feat(sp2): full_stack include camera.launch.py（所有后端带相机 TF，real 起 Orbbec）"
```

---

### Task 4: Isaac 桥相机 topic/frame 重映射 + 加 depth camera_info

**Files:**
- Modify: `isaac_scripts/ur5_isaac_bridge.py:320-371`（相机 OmniGraph `og.Controller.edit` 块）+ 顶部 docstring（行 12-13）+ 注释（行 325-326）

**Interfaces:**
- Consumes: 无（独立改动，对齐 Task 2 的契约）。
- Produces: Isaac 桥发布标准契约的 4 个 topic（`/camera/color/image_raw` 等），frame 用 `camera_color_optical_frame`/`camera_depth_optical_frame`，使下游感知在 isaac 后端和 real 后端 topic/frame 完全一致。

- [ ] **Step 1: 改 docstring（行 12-13）**

把：
```
 * publishes wrist-camera RGB + depth + camera_info
   (``/camera/color``, ``/camera/depth``, ``/camera/camera_info``).
```
改为：
```
 * publishes wrist-camera RGB + depth + camera_info（标准 ROS RGBD 契约，与真机 Orbbec 一致）:
   ``/camera/color/image_raw``, ``/camera/color/camera_info``,
   ``/camera/depth/image_rect_raw``, ``/camera/depth/camera_info``
   (frame: camera_color_optical_frame / camera_depth_optical_frame).
```

- [ ] **Step 2: 改注释（行 325-326）**

把：
```
#   straight at it. Topic names are camera/{color,depth,camera_info} as required
#   by the task spec.
```
改为：
```
#   straight at it. Topic/frame 名对齐 Orbbec/ROS 标准 RGBD 契约（SP2 相机集成）：
#   /camera/color/image_raw, /camera/color/camera_info,
#   /camera/depth/image_rect_raw, /camera/depth/camera_info;
#   frame camera_color_optical_frame / camera_depth_optical_frame.
```

- [ ] **Step 3: 改 OmniGraph 块（行 333-371）**

整块替换。新块相对旧块的变化：
1. 删 `camera_frame_id = ...` 那一行（不再用 prim 名做 frame）。
2. `CREATE_NODES` 加 `cameraHelperDepthInfo`（ROS2CameraInfoHelper）。
3. `CONNECT` 加 2 条边给 depth info。
4. `SET_VALUES`：rgb/info/depth 的 topicName + frameId 改成标准名；加 depth info 的 topicName + frameId。

新块（替换从 `camera_frame_id = ...` 到 `keys.SET_VALUES: [...]` 结束 `],` 的整段）：

```python
        keys = og.Controller.Keys
        ros_camera_graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": ROS_CAMERA_GRAPH_PATH,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    ("createRenderProduct.outputs:execOut", "cameraHelperDepthInfo.inputs:execIn"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                    ("createRenderProduct.outputs:renderProductPath", "cameraHelperDepthInfo.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(args.camera_prim)]),
                    # Resolution matches camera_periodic.py; adjust here to throttle GPU cost.
                    ("createRenderProduct.inputs:width", 1280),
                    ("createRenderProduct.inputs:height", 720),
                    # RGB → /camera/color/image_raw
                    ("cameraHelperRgb.inputs:frameId", "camera_color_optical_frame"),
                    ("cameraHelperRgb.inputs:topicName", "camera/color/image_raw"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    # color CameraInfo → /camera/color/camera_info
                    ("cameraHelperInfo.inputs:frameId", "camera_color_optical_frame"),
                    ("cameraHelperInfo.inputs:topicName", "camera/color/camera_info"),
                    # Depth → /camera/depth/image_rect_raw
                    ("cameraHelperDepth.inputs:frameId", "camera_depth_optical_frame"),
                    ("cameraHelperDepth.inputs:topicName", "camera/depth/image_rect_raw"),
                    ("cameraHelperDepth.inputs:type", "depth"),
                    # depth CameraInfo → /camera/depth/camera_info（★新增，原来只有 color 的）
                    ("cameraHelperDepthInfo.inputs:frameId", "camera_depth_optical_frame"),
                    ("cameraHelperDepthInfo.inputs:topicName", "camera/depth/camera_info"),
                ],
            },
        )
```

- [ ] **Step 4: 验证（需 Isaac 环境 + scene.usd）**

终端 A（桥）：
```bash
cd ~/isaacsim && source setup_ros_env.sh
./python.sh /home/gds/projects/shengzhi/ur5_controller/isaac_scripts/ur5_isaac_bridge.py \
  --stream --robot_prim /World/ur5
```
等桥起来、相机 graph 构建成功（日志无 "Failed to build camera RGBD graph"）。

终端 B：
```bash
ros2 topic list | grep /camera/
```
Expected: 看到 4 个标准 topic：
```
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_rect_raw
/camera/depth/camera_info
```
再验 frame：
```bash
ros2 topic echo /camera/color/image_raw --once | grep frame_id
ros2 topic echo /camera/depth/image_rect_raw --once | grep frame_id
```
Expected: 分别 `frame_id: camera_color_optical_frame` / `camera_depth_optical_frame`。

若 `ROS2CameraInfoHelper` 无 `topicName`/`frameId` 输入属性（即 camera_info 不接受这些参数）→ bridge 报错。降级方案：保留旧的单 `cameraHelperInfo`（color only），depth camera_info 用 `ROS2CameraHelper` 的 `type='point_cloud'` 替代不可行；此情况下在本任务备注"Isaac depth camera_info 受 helper 限制暂缺，real 后端完整"，不阻塞。但先按上面试，Isaac 6.0.1 的 ROS2CameraInfoHelper 支持 topicName/frameId。

- [ ] **Step 5: 提交**

```bash
git add isaac_scripts/ur5_isaac_bridge.py
git commit -m "feat(sp2): Isaac 桥相机 topic/frame 对齐 Orbbec 标准契约 + 加 depth camera_info"
```

---

### Task 5: tools/check_camera.sh 自检脚本

**Files:**
- Create: `tools/check_camera.sh`

**Interfaces:**
- Consumes: Task 2/4 的相机契约（4 topic + 2 frame）。
- Produces: 一个跨后端复用的相机自检脚本（仿 `check_topics.sh`），核对 4 标准 topic 在线 + 2 frame 在 TF 树。

- [ ] **Step 1: 写 `tools/check_camera.sh`**

```bash
#!/bin/bash
# 相机自检：核对 4 个标准 topic 在线 + 2 个 frame 在 TF 树。跨后端复用。
# 用法: source tools/source_env.sh && ./tools/check_camera.sh
set -u

echo "=== 相机 topic（应有 4 个标准 topic）==="
for t in /camera/color/image_raw /camera/color/camera_info \
         /camera/depth/image_rect_raw /camera/depth/camera_info; do
    if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
        hz=$(timeout 3 ros2 topic hz "$t" 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
        echo "  OK   $t   ${hz:-(camera_info 无 Hz)} "
    else
        echo "  MISS $t"
    fi
done

echo "=== 相机 frame（TF 树里应能解析到 tool0）==="
for f in camera_color_optical_frame camera_depth_optical_frame; do
    if timeout 2 ros2 run tf2_ros tf2_echo tool0 "$f" 2>/dev/null | grep -q -- '-- At time'; then
        echo "  OK   tool0 → $f"
    else
        echo "  MISS tool0 → $f（TF 树未连通；起 full_stack 或 camera.launch.py 了吗？）"
    fi
done
```

- [ ] **Step 2: chmod + 验证**

```bash
chmod +x tools/check_camera.sh
```
起 full_stack（mock 即可测 TF 部分；real 测 topic 部分）后：
```bash
source tools/source_env.sh && source install/setup.bash
ros2 launch ur5_bringup full_stack.launch.py backend:=mock   # 另一终端
./tools/check_camera.sh
```
Expected: mock 下 topic 全 MISS（正常，mock 无相机），frame 两条 OK（tool0→optical 连通）。real 下 topic 也 OK。

- [ ] **Step 3: 提交**

```bash
git add tools/check_camera.sh
git commit -m "feat(sp2): tools/check_camera.sh — 相机 topic/frame 跨后端自检"
```

---

### Task 6: README 相机集成章节

**Files:**
- Modify: `README.md`（加"RGBD 相机（Orbbec Gemini 2）"章节 + 把环境一节的 source 说明更新为"唯一入口"）

**Interfaces:**
- 无（文档）。

- [ ] **Step 1: 改 README.md 环境一节**

把"## 环境（本机关键事实）"里 conda 陷阱一段末尾那句：
```
**所有 ROS2 命令前必须 `source tools/source_env.sh`**
（它会 deactivate conda 并从 PATH 剔除 miniconda，回到系统 `/usr/bin/python3`）。
```
改为：
```
**所有 ROS2 命令前必须 `source tools/source_env.sh`**（全项目唯一环境入口）——它会
deactivate conda、从 PATH 剔除 miniconda、source humble + Orbbec overlay（`~/ros2_ws`）
+ 本仓库 `install/setup.bash` + `robot_ip.sh`。从任意子目录 source 都生效。
```

- [ ] **Step 2: 加相机章节（插在"## Isaac Sim 仿真后端"节之后、"## 验收"之前）**

```markdown
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
```

- [ ] **Step 3: 提交**

```bash
git add README.md
git commit -m "docs(sp2): README 加 RGBD 相机集成章节 + 统一 source 说明"
```

---

## Self-Review（计划写完后自检）

**1. Spec 覆盖**：
- §1 统一 source → Task 1 ✓
- §2 架构/camera.launch.py → Task 2 ✓；full_stack 接入 → Task 3 ✓
- §3 topic/frame 契约 → Task 2 (real 原生) + Task 4 (Isaac 对齐) ✓
- §4 Isaac 桥改动 → Task 4 ✓
- §5 验证（check_camera.sh）→ Task 5 ✓；README → Task 6 ✓
- 组件清单 6 项 → Task 1-6 一一对应 ✓

**2. 占位符扫描**：无 TBD/TODO；占位 TF 数值（z=0.05, identity）是明确设计决策，非遗留；Isaac `ROS2CameraInfoHelper` 属性不确定处给了降级备注（Task 4 Step 4）✓

**3. 类型/命名一致**：
- 契约 topic/frame 名在 Task 2、4、5、6 全一致（`camera_color_optical_frame` 等）✓
- `camera.launch.py` 的 `backend` 参数 choices 与 full_stack 一致（mock/isaac/real）✓
- `_static_tf` / `_orbbec_include` / `_setup` 命名在 Task 2 内自洽 ✓

无问题。

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-21-rgbd-camera-integration.md`.
