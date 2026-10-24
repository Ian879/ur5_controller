"""硬件层：机器人本体后端（mock/isaac/real）+ 相机 + （real）Lebai 夹爪驱动。

部署分层（2026-09-28 方案A）：
  - 硬件层（本文件）：真机部署跑在 Jetson（Docker humble）——ur_robot_driver、
    夹爪、相机都在机器人侧；仿真时跑在本机（Isaac 桥需另起，见 tools/bringup_sim.sh）。
  - 规划层（planning.launch.py）：move_group + world_sync，跑在 GPU 服务器。
  两层只靠 DDS 通信（同 RMW=fastrtps、同 ROS_DOMAIN_ID），可分开启停；
  full_stack.launch.py 是两者在同一台机器上的组合（向后兼容）。

  ros2 launch ur5_bringup hardware.launch.py backend:=mock
  ros2 launch ur5_bringup hardware.launch.py backend:=isaac   # 需先起 Isaac 桥
  ros2 launch ur5_bringup hardware.launch.py backend:=real robot_ip:=192.168.1.102
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(launch_file, extra_args=None):
    args = {"ur_type": "ur5"}
    if extra_args:
        args.update(extra_args)
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur5_bringup"), "launch", launch_file])
        ),
        launch_arguments=args.items(),
    )


def _setup(context, *args, **kwargs):
    backend = LaunchConfiguration("backend").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    if backend == "mock":
        actions = [_include("sim_mock.launch.py")]
    elif backend == "isaac":
        actions = [_include("sim_isaac.launch.py")]
    elif backend == "real":
        # robot_ip 合法性由 real_robot.launch.py 自己把关（缺 IP 时 raise）
        actions = [_include("real_robot.launch.py", {"robot_ip": robot_ip})]
        # Lebai 串口夹爪（硬件层成员）：serial_gripper 提供 gripper/* services +
        # gripper/state（含位置反馈）；real_gripper_action 把它包成与 isaac 版
        # 同名同契的 GripperCommand action（lebai_gripper_controller/gripper_cmd），
        # demo 的 GripperClient 零改动。串口建议 udev 别名（教程 §4.4）。
        actions.append(
            Node(
                package="lebai_driver",
                executable="serial_gripper",
                name="lebai_serial_gripper",
                parameters=[{"port_name": LaunchConfiguration("gripper_port")}],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="ur5_motion",
                executable="real_gripper_action",
                name="real_gripper_action",
                output="screen",
            )
        )
    else:
        raise RuntimeError(f"未知 backend: {backend}")
    # 相机：所有后端带（camera.launch.py 内部 gate：real→起 Orbbec，其余仅占位 TF）
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
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend", default_value="mock", choices=["mock", "isaac", "real"]
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value=EnvironmentVariable("UR5_HOST", default_value="192.168.xxx.xxx"),
                description="真机 IP。默认读环境变量 UR5_HOST（tools/robot_ip.sh）。",
            ),
            DeclareLaunchArgument(
                "gripper_port",
                default_value="/dev/lebai_gripper",
                description="Lebai 串口夹爪设备（udev 别名；容器内由 jetson_up 启动时按 by-id 补建软链）。",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
