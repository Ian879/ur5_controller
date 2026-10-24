"""一键启动（同机组合）：hardware.launch.py + planning.launch.py。

  ros2 launch ur5_bringup full_stack.launch.py backend:=mock
  ros2 launch ur5_bringup full_stack.launch.py backend:=real robot_ip:=192.168.1.102
  ros2 launch ur5_bringup full_stack.launch.py backend:=isaac   # 需另起 Isaac 桥

分层（2026-09-28 方案A）：
  - hardware.launch.py = 硬件层（真机跑 Jetson：ur_driver / Lebai 夹爪 / Orbbec 相机）
  - planning.launch.py = 规划层（跑 GPU 服务器：move_group + world_sync）
本文件把两层合在一台机器起，向后兼容单机工作流；跨机部署时两层分别起
（见 docs/jetson_deployment.md）。三层后端（mock/isaac/real）共用同一 MoveIt
规划+执行栈，仅硬件来源不同。

MoveIt 的 use_sim_time：mock/isaac→true（isaac 发 /clock；mock 下若无 /clock
发布者，move_group 可能卡 TF，见 docs/phase1_verification.md）；real→false（墙钟）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(launch_file, extra_args=None):
    args = extra_args or {}
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur5_bringup"), "launch", launch_file])
        ),
        launch_arguments=args.items(),
    )


def _setup(context, *args, **kwargs):
    backend = LaunchConfiguration("backend").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    if backend not in ("mock", "isaac", "real"):
        raise RuntimeError(f"未知 backend: {backend}")
    if backend == "real" and ("xxx" in robot_ip or not robot_ip):
        raise RuntimeError(
            "backend:=real 需要真实 robot_ip。\n"
            "  方式1：编辑 tools/robot_ip.sh 填 IP（设了 UR5_HOST 后即可直接 backend:=real）\n"
            "  方式2：命令行临时覆盖  robot_ip:=192.168.1.102"
        )

    hw_args = {"backend": backend}
    if backend == "real":
        hw_args["robot_ip"] = robot_ip
        hw_args["gripper_port"] = LaunchConfiguration("gripper_port").perform(context)
    use_sim_time = "false" if backend == "real" else "true"
    return [
        _include("hardware.launch.py", hw_args),
        _include(
            "planning.launch.py",
            {"use_sim_time": use_sim_time, "launch_rviz": launch_rviz},
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend", default_value="mock", choices=["mock", "isaac", "real"]
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value=EnvironmentVariable("UR5_HOST", default_value="192.168.xxx.xxx"),
                description="真机 IP。默认读环境变量 UR5_HOST（在 tools/robot_ip.sh 设置）。",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "gripper_port",
                default_value="/dev/lebai_gripper",
                description="Lebai 串口夹爪设备（udev 别名；容器内由 jetson_up 启动时按 by-id 补建软链）。",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
