"""Phase 3：真机后端。

把 ur_robot_driver 指向真实 UR5：
  - use_fake_hardware:=false              → 起 ur_ros2_control_node（真机硬件接口）
  - initial_joint_controller:=scaled_joint_trajectory_controller
                                          真机用 scaled（按速度缩放，更安全/顺滑）

前置（真机必做，示教器上）：
  - 上电 + brake release（松刹车）
  - robot_mode == RUNNING
  - 保护停机已复位
  - 保守速度（可改 ur_controllers.yaml 调低）

启动： ros2 launch ur5_bringup real_robot.launch.py robot_ip:=192.168.1.102
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PLACEHOLDER_IP = "192.168.xxx.xxx"


def _launch_with_real(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    if not robot_ip or "xxx" in robot_ip or robot_ip == PLACEHOLDER_IP:
        raise RuntimeError(
            "真机后端需要真实 robot_ip。\n"
            "  方式1：编辑 tools/robot_ip.sh 填 IP（设了 UR5_HOST 环境变量）\n"
            "  方式2：命令行  robot_ip:=192.168.1.102\n"
            "  方式3：导出  export UR5_HOST=192.168.1.102"
        )

    ur_control_launch = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
    )
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(ur_control_launch),
            launch_arguments={
                "ur_type": "ur5",
                "robot_ip": robot_ip,
                "use_fake_hardware": "false",
                # 真机用 scaled（按速度缩放，更安全）
                "initial_joint_controller": "scaled_joint_trajectory_controller",
                "launch_rviz": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value=EnvironmentVariable("UR5_HOST", default_value=PLACEHOLDER_IP),
                description="真实 UR5 的 IP。默认读环境变量 UR5_HOST（在 tools/robot_ip.sh 设置）。",
            ),
            OpaqueFunction(function=_launch_with_real),
        ]
    )
