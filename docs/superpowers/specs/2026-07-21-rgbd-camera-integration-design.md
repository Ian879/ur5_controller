# SP2 — RGBD 相机集成设计（Orbbec Gemini 2，eye-in-hand）

> 状态：已设计待实现 ｜ 分支：`sp1-isaac-motion-bridge` ｜ 日期：2026-07-21
> 替换原 SP2（夹爪）——夹爪材料未齐，先把相机接入栈，为 SP3/SP4（抓取编排 + 6D 位姿估计）铺路。

## 目标

把 Orbbec Gemini 2 RGBD 相机接入现有三后端（mock/isaac/real）运动栈，达到：

1. **真机相机驱动进栈**：`full_stack backend:=real` 自动起 Orbbec 驱动。
2. **TF 树完整**：相机帧 (`camera_color_optical_frame` 等) 挂到 `tool0` 下，RViz 里图像能定位到机器人。
3. **后端无关的相机契约**：真机 Orbbec 和 Isaac 桥 RTX 相机发布**同一套 topic + frame**，下游感知代码（SP3/SP4）写一份即可跨后端跑。
4. **单一环境入口**：全项目只 `source tools/source_env.sh` 一步，自动 overlay Orbbec 工作空间。

**非目标（推迟）**：手眼标定（M7）、点云发布、物理安装验证（相机尚未装机）。

## 现状（关键事实）

| 项 | 现状 |
|---|---|
| Orbbec SDK | `~/ros2_ws/src/OrbbecSDK_ROS2`（v2-main 分支），**独立工作空间**，已 `colcon build`，install 在 `~/ros2_ws/install/` |
| 真机 launch | `orbbec_camera/gemini2.launch.py`，namespace `camera` |
| 真机发布 | 标准 ROS RGBD：`/camera/color/image_raw`、`/camera/depth/image_rect_raw`、各 `camera_info`，frame `camera_color_optical_frame`/`camera_depth_optical_frame`；`publish_tf:=true` 时自带内部 TF |
| Isaac 桥现状 | 非标准：`/camera/color`、`/camera/depth`、`/camera/camera_info`，frame=`wrist_camera`，**只有 color 的 camera_info**（缺 depth camera_info） |
| 物理安装 | **未装机**（桌面测试）→ TF 用占位值，装上后量、M7 精标 |
| 安装方式 | eye-in-hand（腕部），与路线图 + Isaac 桥已有腕部相机 prim 一致 |
| 现有 source | `tools/source_env.sh` 已做 conda 退出 + humble + 我们 overlay + robot_ip；**缺 Orbbec overlay**；且 step 6 用 `$(pwd)` 有 cwd 依赖 bug |

## 统一方向

让 **Isaac 对齐 Orbbec/ROS 标准约定**（改桥重映射 + frame），不反过来。Orbbec 原生即标准，零改动。

---

## §1 统一环境入口（`tools/source_env.sh`）

最终 layering（colcon overlay 必须按依赖顺序 source：underlay → overlay）：

```
1. conda deactivate + PATH 清洗 miniconda         (已有)
2. /opt/ros/humble/setup.bash                      (已有, underlay)
3. $ORBBEC_WS/install/setup.bash                   (★新增, Orbbec overlay)
4. <repo>/install/setup.bash                        (已有, ★修 bug: 用 THIS_DIR 而非 $(pwd))
5. tools/robot_ip.sh                                (已有)
```

改动：
- 新增 step 3：`ORBBEC_WS=${ORBBEC_WS:-$HOME/ros2_ws}`；`[ -f "$ORBBEC_WS/install/setup.bash" ] && source`。不存在则跳过（没装 Orbbec 的机器也能用）。
- 修 step 4：`THIS_DIR` 在脚本末尾才计算，提到顶部；用 `"$THIS_DIR/../install/setup.bash"` 替 `$(pwd)/install/setup.bash`，使从任意子目录 source 都生效。
- 顺序硬约束：Orbbec overlay 必须在 humble 之后（依赖 humble）、我们 overlay 之前（我们可能 launch 它）。

之后全项目唯一入口：`source tools/source_env.sh`。

## §2 架构 / launch 结构

新增 `ur5_bringup/launch/camera.launch.py`，按后端 gate：

```
camera.launch.py
├─ 静态 TF 子树（所有后端，占位值）：
│     tool0 → camera_link                         (mount，占位，M7 标定)
│     camera_link → camera_color_optical_frame    (标准光学旋转)
│     camera_link → camera_depth_optical_frame    (标准光学旋转)
├─ backend==real  → IncludeLaunchDescription(orbbec_camera/gemini2.launch.py,
│                                            publish_tf:=false, depth_registration:=true)
├─ backend==isaac → 不起 Orbbec（图像来自桥，桥已重映射到标准名）
└─ backend==mock  → 不起相机（仅 TF 子树，下游收不到图，符合预期）
```

`full_stack.launch.py` 的 `_setup`：world_sync 之后追加 `IncludeLaunchDescription(camera.launch.py, launch_arguments={"backend": backend})`，所有后端都带。

### 关键决策：TF 子树由 camera.launch.py 统一发，Orbbec `publish_tf:=false`

理由：
- 三个后端 TF 树完全一致（真后端无关）。
- 避免 Orbbec 自带 TF 和我们 mount TF 双发冲突。
- 占位值满足 SP2 目标（驱动进栈 + RViz 可见）。
- **M7 标定时**：改回 `publish_tf:=true`（Orbbec 自带准确内部 TF）+ 标定 mount offset，拿精度。

### 光学帧旋转（占位常量）

ROS optical frame 约定：Z 前、X 右、Y 下。`camera_link`（body）→ optical 的固定旋转：
- `(x,y,z,w) = (-0.5, 0.5, -0.5, 0.5)`（body X-forward → optical Z-forward 的标准变换）

Isaac 相机 USD 默认看 -Z，body 约定不同 → 实现时验证并按需调整此四元数（spec 记录占位，实测修正）。

## §3 Topic + frame 契约（下游唯一接口）

| Topic | 类型 | frame |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `camera_color_optical_frame` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image (32FC1)` | `camera_depth_optical_frame` |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | `camera_depth_optical_frame` |

- `depth_registration:=true`：深度对齐到 color 帧（Orbbec HW；Isaac 渲染 depth 到 color 分辨率）。
- **pointcloud 不做**（FoundationPose 只要 RGB+depth；Isaac 侧做点云代价大）→ deferred。
- 真机 Orbbec 原生即此套；Isaac 桥重映射成此套。

## §4 Isaac 桥改动（`isaac_scripts/ur5_isaac_bridge.py`）

相机 OmniGraph `og.Controller.edit` 改动（path/value 改值 + 加一个节点）：

| 节点 | 现状 | 改为 |
|---|---|---|
| `cameraHelperRgb` | topic=`camera/color`, frame=`wrist_camera` | topic=`camera/color/image_raw`, frame=`camera_color_optical_frame` |
| `cameraHelperDepth` | topic=`camera/depth`, frame=`wrist_camera` | topic=`camera/depth/image_rect_raw`, frame=`camera_depth_optical_frame` |
| `cameraHelperInfo` | topic=`camera/camera_info`, frame=`wrist_camera` | topic=`camera/color/camera_info`, frame=`camera_color_optical_frame` |
| **★新增 `cameraHelperDepthInfo`** | （无） | `isaacsim.ros2.bridge.ROS2CameraInfoHelper` → topic=`camera/depth/camera_info`, frame=`camera_depth_optical_frame` |

renderProduct → 原来 3 个 helper 变 4 个（depth 也接 camera_info helper）。SP2 唯一动 Isaac 代码处。

## §5 验证（相机未装机，分阶段）

**阶段 0：相机单独跑通（桌面）**
```bash
ros2 launch orbbec_camera gemini2.launch.py
ros2 topic hz /camera/color/image_raw        # 有数据 = 驱动 OK
rviz2   # Image(/camera/color/image_raw) + CameraInfo，Fixed Frame=camera_color_optical_frame
```

**阶段 1：进栈 + 统一 source 自检**
```bash
cd <任意子目录> && source tools/source_env.sh
ros2 pkg list | grep orbbec_camera            # ★overlay 不分目录 source 进来 = 修 bug 成功
ros2 launch ur5_bringup full_stack.launch.py backend:=real
# RViz Fixed Frame=base_link, Image display=/camera/color/image_raw
#   → tool0→camera_link→camera_color_optical_frame 连通，图像定位到机器人
```

**阶段 2：Isaac 对齐**
```bash
# 桥（已重映射）+ full_stack backend:=isaac
ros2 topic list | grep /camera/               # 4 个标准 topic 齐
# RViz 同一套 Image display 显示 Isaac 相机画面 = 统一成功
```

**新增 `tools/check_camera.sh`**：核对 4 标准 topic + 2 frame 都在，跨后端复用（仿 `check_topics.sh`）。

**不能验证**（推迟）：真·眼在手上闭环、TF 对齐精度 → 物理安装后 + M7。

## 组件清单（实现时改的文件）

| 文件 | 动作 |
|---|---|
| `tools/source_env.sh` | 改：加 Orbbec overlay step + 修 `$(pwd)` bug（THIS_DIR 提前） |
| `ur5_bringup/launch/camera.launch.py` | 新建：静态 TF 子树 + 后端 gate（real→include Orbbec） |
| `ur5_bringup/launch/full_stack.launch.py` | 改：`_setup` 加 include camera.launch.py |
| `isaac_scripts/ur5_isaac_bridge.py` | 改：相机 OmniGraph topic/frame 重映射 + 加 depth camera_info helper |
| `tools/check_camera.sh` | 新建：相机 topic/frame 自检 |
| `README.md` | 改：相机集成章节 + 统一 source 说明 |

## 范围外 / 推迟

- 手眼标定（M7）：`tool0→camera_link` 用占位，标定后换 `publish_tf:=true` + 精确 mount。
- 点云发布：FoundationPose 不需要，Isaac 代价大。
- 物理安装 + 真·eye-in-hand 闭环验证：相机装上后。
- mock 后端相机：不起（mock 只测运动）。

## 决策记录

1. **方案 A（薄 launch include）**：复用 `~/ros2_ws` 的 Orbbec 包原样，不 vendor、不自建包。符合"不重复造轮子"。
2. **Isaac 对齐 Orbbec**，不反向：Orbbec 原生标准，零改动。
3. **TF 子树统一由 camera.launch.py 发**，`publish_tf:=false`：三后端一致、无冲突、M7 可平滑升级精度。
4. **pointcloud 推迟**：YAGNI，FoundationPose 只要 RGB+depth。
5. **统一 source_env.sh**：全项目一个入口，Orbbec overlay 按 colcon 依赖顺序插入。
