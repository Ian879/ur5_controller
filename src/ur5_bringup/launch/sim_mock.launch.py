"""Phase 1 默认环境：ur_robot_driver 的 mock（fake）hardware。

不连真机、不需要 Isaac Sim，用 ros2_control 的 mock_components/GenericSystem
把下发指令回环成状态，用来先打通 MoveIt 规划+执行链路。

启动它之后，再单独起 ur_moveit（见 full_stack.launch.py，它已自动组合）。

关键参数（已查 ur_control.launch.py 源码确认）：
  - use_fake_hardware:=true            → 起通用 ros2_control_node + mock 硬件
  - initial_joint_controller:=joint_trajectory_controller
                                       fake 硬件不支持 scaled_joint_trajectory_controller
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_control_launch = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(ur_control_launch),
                launch_arguments={
                    "ur_type": "ur5",
                    # mock 不连真机；ur_control.launch.py 的 dashboard 等会因 use_fake_hardware 自动跳过
                    "robot_ip": "0.0.0.0",
                    "use_fake_hardware": "true",
                    # fake 硬件必须用（非 scaled 的）joint_trajectory_controller
                    "initial_joint_controller": "joint_trajectory_controller",
                    "launch_rviz": "false",  # RViz 由 ur_moveit.launch.py 起
                }.items(),
            ),
        ]
    )
