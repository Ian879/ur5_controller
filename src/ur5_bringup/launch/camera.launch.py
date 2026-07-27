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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _static_tf(name, frame_id, child_frame_id, x="0.0", y="0.0", z="0.0",
               yaw="0.0", pitch="0.0", roll="0.0"):
    """static_transform_publisher，参数风格与 sim_isaac.launch.py 一致。

    注：Humble 的 static_transform_publisher 只接 long flags（--x/--y/--z/--yaw/--pitch/--roll）；
    短 flag（-x/-Y...）会被拒为 "Extra unparsed arguments"，节点启动即死。
    """
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
            "--x", x, "--y", y, "--z", z,
            "--yaw", yaw, "--pitch", pitch, "--roll", roll,
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

    # 相机 TF（camera_link + camera_color/depth_optical_frame 等）现由 robot URDF 提供——
    # ur5_with_gripper.urdf.xacro / ur5_isaac.urdf.xacro 里 xacro:include 了 Orbbec 官方
    # gemini_2.urdf.xacro（带 mesh + REP-103 光学帧），RSP 直接发相机 TF。故本 launch 不再
    # 手搓 static TF（之前占位/手写的 TF 已删，避免和 URDF 双发冲突）。

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
