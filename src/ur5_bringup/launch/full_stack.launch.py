"""一键启动：bringup（机器人硬件/仿真）+ MoveIt + RViz。

  ros2 launch ur5_bringup full_stack.launch.py backend:=mock
  ros2 launch ur5_bringup full_stack.launch.py backend:=real robot_ip:=192.168.1.102
  ros2 launch ur5_bringup full_stack.launch.py backend:=isaac   # Phase 2，需另起 Isaac Sim 脚本

设计：三种后端共用同一 MoveIt 规划+执行栈，仅硬件来源不同。
  - mock : ur_robot_driver 的 fake hardware（Phase 1，无需真机/Isaac）
  - isaac: Isaac Sim 通过 topic_based_ros2_control 暴露机器人（Phase 2）
  - real : ur_robot_driver 指向真实 UR5（Phase 3）

MoveIt 的 use_sim_time 取值：
  mock/isaac → true（ur_moveit.launch.py 靠它把默认控制器切到 joint_trajectory_controller；
              注意 mock 下若无 /clock 发布者，move_group 可能卡 TF，见 docs/phase1_verification.md）
  real      → false（用墙钟）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _moveit_include(use_sim_time: str, launch_rviz: str = "true"):
    """组合系统 ur_moveit.launch.py，固定 ur_type:=ur5。"""
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5",
            "use_sim_time": use_sim_time,
            "launch_servo": "false",  # 首期不开 servo
            "launch_rviz": launch_rviz,
        }.items(),
    )


def _bringup_include(launch_file: str, extra_args=None):
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
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    actions = []

    if backend == "mock":
        actions.append(_bringup_include("sim_mock.launch.py"))
        actions.append(_moveit_include("true", launch_rviz))

    elif backend == "real":
        if "xxx" in robot_ip or not robot_ip:
            raise RuntimeError(
                "backend:=real 需要真实 robot_ip。\n"
                "  方式1：编辑 tools/robot_ip.sh 填 IP（设了 UR5_HOST 后即可直接 backend:=real）\n"
                "  方式2：命令行临时覆盖  robot_ip:=192.168.1.102"
            )
        actions.append(_bringup_include("real_robot.launch.py", {"robot_ip": robot_ip}))
        actions.append(_moveit_include("false", launch_rviz))

    elif backend == "isaac":
        # Phase 2：Isaac Sim 在外部独立运行（isaac_scripts/ur5_isaac_moveit.py, T7），
        # 它通过 ROS2 bridge 发布 /clock、/isaac_joint_states 并订阅 /isaac_joint_commands。
        # 这里起 ROS2 侧 bringup（ros2_control 的 TopicBasedSystem + 控制器 +
        # robot_state_publisher），MoveIt 由 _moveit_include 起。isaac 后端完全不走
        # ur_robot_driver（无 dashboard/urscript/真机硬件节点）——Isaac 就是"硬件"。
        actions.append(_bringup_include("sim_isaac.launch.py"))
        actions.append(_moveit_include("true", launch_rviz))

    else:
        raise RuntimeError(f"未知 backend: {backend}")

    # ===== world_sync: 所有后端共用 =====
    # 自动把 ~/Desktop/scene_collisions.json 里的静态碰撞体（桌面+桌腿+底座…）
    # 加进 MoveIt PlanningScene。等 move_group 就绪后执行，发布后退出。
    # JSON 不存在则 warn 跳过（不影响栈）。
    actions.append(
        Node(
            package="ur5_motion",
            executable="world_sync",
            name="world_sync_node",
            output="screen",
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
                description="真机 IP。默认读环境变量 UR5_HOST（在 tools/robot_ip.sh 设置）。",
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
