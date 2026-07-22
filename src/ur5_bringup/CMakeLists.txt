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

ament_package()
