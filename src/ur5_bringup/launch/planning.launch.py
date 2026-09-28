"""规划层：move_group + world_sync（真机部署跑在 GPU 服务器；仿真跑本机）。

与 hardware.launch.py 只靠 DDS 通信，不含任何硬件节点。
use_sim_time：mock/isaac→true（Isaac 发 /clock）；real→false（墙钟）。
world_sync 把 ~/Desktop/scene_collisions.json 灌进 PlanningScene（缺文件只 warn）。

  ros2 launch ur5_bringup planning.launch.py                       # 仿真默认
  ros2 launch ur5_bringup planning.launch.py use_sim_time:=false   # 真机（服务器侧）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur5_bringup"), "launch", "moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_servo": "false",  # 首期不开 servo
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )
    # world_sync：等 move_group 就绪后把静态碰撞体发进 PlanningScene，发完即退
    world_sync = Node(
        package="ur5_motion",
        executable="world_sync",
        name="world_sync_node",
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true",
                description="mock/isaac→true；real→false（墙钟）。",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            moveit,
            world_sync,
        ]
    )
