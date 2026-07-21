"""Phase 2 仿真后端：Isaac Sim 通过 topic_based_ros2_control 暴露的 UR5。

Isaac Sim 在外部独立运行（isaac_scripts/ur5_isaac_moveit.py, T7），通过 ROS2
bridge 发布 /clock、/isaac_joint_states 并订阅 /isaac_joint_commands。本 launch
只起 ROS2 侧的 bringup：
  - xacro 处理 ur5_isaac.urdf.xacro（<ros2_control>=TopicBasedSystem）
  - 通用 controller_manager/ros2_control_node + 控制器 (joint_trajectory_controller)
  - robot_state_publisher（发 /robot_description，订阅 /joint_states 发 TF）
  - spawner：joint_state_broadcaster + joint_trajectory_controller

与 sim_mock.launch.py 的区别：
  mock 复用 ur_robot_driver 的 ur_control.launch.py（含 dashboard/urscript 等真机
  节点，靠 use_fake_hardware:=true 跳过）；isaac 完全不起 ur_robot_driver 任何
  节点——Isaac 就是"硬件"，只通过 TopicBasedSystem 的两个 topic 与之交互。

use_sim_time 全程 true（Isaac 发 /clock）。MoveIt 由 full_stack.launch.py 的
backend:=isaac 分支另行起（见 _moveit_include）；这里不带 RViz。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ===== args =====
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Isaac /clock) time.",
    )
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur5",
        description="UR robot type (ur5/ur5e/...). Must match MoveIt's ur_type.",
    )

    ur_type = LaunchConfiguration("ur_type")

    # ===== robot_description via xacro =====
    # Explicitly resolve the ur5 config yamls via FindPackageShare("ur_description"),
    # mirroring ur_robot_driver/launch/ur_control.launch.py. (The xacro also has
    # working $(find ur_description) defaults per T5; passing them explicitly is the
    # robust pattern used by the upstream real-robot launch.)
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur5_bringup"), "urdf", "ur5_isaac.urdf.xacro"]
    )

    # ros2_control_hardware_type:=isaac selects TopicBasedSystem
    # (reads isaac_joint_states, writes isaac_joint_commands) per T5.
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "ur_type:=", ur_type, " ",
            "ros2_control_hardware_type:=isaac",
            " ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
        ]
    )

    # ParameterValue(value_type=str): the xacro output is a big URDF string;
    # without this wrap launch_ros tries to YAML-parse it and fails
    # ("Unable to parse the value of parameter robot_description as yaml").
    # Mirrors ur_robot_driver/launch/ur_control.launch.py.
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ===== controllers yaml =====
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ur5_bringup"), "config", "ur5_isaac_controllers.yaml"]
    )

    # ===== generic controller_manager / ros2_control_node =====
    # NOTE: NOT ur_ros2_control_node / dashboard / urscript — Isaac is the "hardware"
    # via TopicBasedSystem topics. Mirrors src/isaac_moveit/launch/isaac_moveit.launch.py.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            # keep parity with the official isaac_moveit launch
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # ===== robot_state_publisher =====
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ===== controller spawners =====
    # The controllers inherit use_sim_time from controller_manager (yaml sets it
    # true there + under each controller), so the spawner nodes don't need it
    # (they are one-shot CLI clients that exit after configuring the controller).
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ===== world -> base_link static TF (让 RViz 视觉对齐 Isaac 场景) =====
    # MoveIt 机器人模型 base_link 在 z=0（URDF 根），但 Isaac 场景里机器人被圆柱底座
    # 抬到世界 (-0.55, 0, 1.211)。发这个 TF 后，RViz Fixed Frame=world 会把机器人
    # 显示在抬高位置（和 Isaac 一致）。⚠️ 这些值对应当前 ~/Desktop/scene.usd；你在
    # GUI 里挪了机器人就得改（或以后自动同步）。
    world_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", "base_link",
            "--x", "-0.55", "--y", "0.0", "--z", "1.211",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            ur_type_arg,
            ros2_control_node,
            robot_state_publisher,
            world_to_base_tf,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
        ]
    )
