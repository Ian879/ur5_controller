cmake_minimum_required(VERSION 3.8)
project(ur5_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch / config 目录，使 ros2 launch ur5_bringup ... 可用
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# UR5 isaac (TopicBasedSystem) xacro overlay; consumed by T6 launch via
# robot_description_content := xacro ur5_isaac.urdf.xacro ...
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# SRDF (gripper-augmented) xacro; consumed by moveit.launch.py (Task 3) via
# moveit_config_file := ur5_with_gripper.srdf.xacro
install(DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Provide ur_description's ur5 config yamls so the real backend's
# ur_control.launch.py can resolve its non-overridable config
# PathJoinSubstitutions when description_package:=ur5_bringup:
#   FindPackageShare(ur5_bringup)/config/ur5/{joint_limits,physical_parameters,
#   visual_parameters,default_kinematics}.yaml
# (ur_control passes joint_limit_params/physical_params/visual_params as
#  local PathJoinSubstitutions keyed off description_package — NOT overridable
#  launch args. kinematics_params_file defaults to the same pattern. Copying all
#  four makes ur5_with_gripper.urdf.xacro resolve under ur5_bringup.)
execute_process(
  COMMAND ros2 pkg prefix ur_description
  OUTPUT_VARIABLE _ur_desc_prefix
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _ur_desc_result
)
if(NOT _ur_desc_result EQUAL 0)
  message(FATAL_ERROR
    "Cannot locate ur_description (ros2 pkg prefix failed). "
    "Source ROS2 (tools/source_env.sh) before building ur5_bringup.")
endif()
set(_ur_desc_config_dir "${_ur_desc_prefix}/share/ur_description/config/ur5")
# ⚠️ 这些是 ur_description 的拷贝（非符号链接，即使 --symlink-install 也是复制）：
# ur_description 升级（如 apt 升级 ros-humble-ur-description）后，必须重新 colcon build
# ur5_bringup 刷新这些拷贝，否则真机 ur_control 解析 URDF 时会静默用旧的限位/运动学。
install(FILES
  "${_ur_desc_config_dir}/joint_limits.yaml"
  "${_ur_desc_config_dir}/physical_parameters.yaml"
  "${_ur_desc_config_dir}/visual_parameters.yaml"
  "${_ur_desc_config_dir}/default_kinematics.yaml"
  DESTINATION share/${PROJECT_NAME}/config/ur5
)

ament_package()
