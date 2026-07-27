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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _retrying_spawner(controller_name, max_tries=60):
    """起一个 ros2_control 控制器，每 2s 重试直到它 configure+activate 成功。

    isaac 后端的 TopicBasedSystem 要等 /isaac_joint_states 有数据才暴露关节状态接口，
    而 Isaac 桥启动很慢（30-60s）。若 spawner 在桥就绪前跑，configure() 失败、一次性
    spawner 退出非零 → 控制器卡 unconfigured → /joint_states 不发 → RViz 渲染 URDF 默认
    姿态、不同步（2026-07-21 排障根因）。launch 的 respawn=True 在 exit 0 也会重 Spawn
    （execute_local.py:583 不查 returncode，成功会死循环），所以用 bash ``until`` 重试
    spawner——成功（exit 0）即退。封顶 ``max_tries`` 防病态无限刷屏（如重复 launch 致控制器
    已 active，此时 spawner 反而退非零）。"""
    script = (
        f"i=0; until ros2 run controller_manager spawner {controller_name} "
        f"--controller-manager /controller_manager; do "
        f"i=$((i+1)); if [ $i -ge {max_tries} ]; then "
        f"echo '[spawner {controller_name}] {max_tries} 次重试仍失败——检查 "
        f"controller_manager / Isaac 桥是否在发 /isaac_joint_states，或是否已有同名栈在跑'; "
        f"exit 1; fi; echo '[spawner {controller_name}] 未就绪，重试 #'$i'（2s）'; sleep 2; done"
    )
    return ExecuteProcess(
        cmd=["bash", "-c", script],
        name=f"spawner_{controller_name}",
        output="screen",
    )


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

    # ===== controller spawners（自愈：失败重试到成功）=====
    # TopicBasedSystem 要等 /isaac_joint_states 有数据才暴露状态接口，Isaac 桥启动慢，
    # 一次性 spawner 可能 configure 失败而退出 → 控制器卡 unconfigured → /joint_states 不发
    # → RViz 不同步。_retrying_spawner 用 bash until 包住 spawner，每 2s 重试直到 activate 成功。
    joint_state_broadcaster_spawner = _retrying_spawner("joint_state_broadcaster")
    joint_trajectory_controller_spawner = _retrying_spawner("joint_trajectory_controller")

    # world→base_link TF 现由 Isaac 桥发布（ur5_isaac_bridge.py 的 ActionGraph 里
    # WorldTfPub 节点从 USD 读 base 世界 pose 发 tf_static，P2）——GUI 挪机器人后
    # 重启桥即自动跟随，无需这里的硬编码 static_transform_publisher。

    # ===== isaac_gripper_action 节点（Task 5）=====
    # 暴露 control_msgs/GripperCommand action（lebai_gripper_controller/gripper_cmd），
    # 中转 MoveIt 的 GripperCommand 句柄 → /isaac_gripper_joint_commands，读
    # /isaac_gripper_joint_states 判到位。纯 rclpy 节点，不需要 Isaac 桥在场也能起
    # （只是 goal 会一直等到状态有数据；自愈 spawner 没必要——它不依赖 TopicBasedSystem）。
    isaac_gripper_action = Node(
        package="ur5_motion",
        executable="isaac_gripper_action",
        name="isaac_gripper_action",
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            ur_type_arg,
            ros2_control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            isaac_gripper_action,
        ]
    )
