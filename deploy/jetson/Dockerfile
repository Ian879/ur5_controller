# Jetson Xavier NX（JetPack 5 / Ubuntu 20.04 宿主）硬件层镜像 —— 方案A（2026-09-28）。
#
# 只跑 hardware.launch.py（ur_robot_driver + Lebai 夹爪 + Orbbec 相机）；
# move_group / planner / demo / FoundationPose 在 GPU 服务器——本镜像不含
# MoveIt / pymoveit2 / isaac 相关任何东西。
#
# 纯 CPU 控制：普通 ros:humble 即可（无需 L4T/CUDA 变体镜像）。
# arm64 在 Jetson 上构建（首次 15-40min）；amd64 可在开发服务器烟测构建
# 验证 Dockerfile 正确性：
#   docker build -f deploy/jetson/Dockerfile -t ur5-hw:smoke .
FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
# HW_PKGS：colcon 构建子集。ur5_motion 也要——hardware.launch.py real 分支在
# 容器里拉 ur5_motion/real_gripper_action（夹爪 action 桥），handeye_calib 也
# 在这包里（ament_python，只多几百 KB；demo/planner 入口在里面但容器不会调它们，
# 缺 pymoveit2 不影响 build，入口是惰性 import）。
# 若 OrbbecSDK 在某架构缺预编译库导致构建失败，可
#   --build-arg HW_PKGS="ur5_bringup lebai_driver ur5_motion"
# 先验证其余部分（arm64 现场再处理 orbbec，见 docs/jetson_deployment.md）。
ARG HW_PKGS="ur5_bringup lebai_driver orbbec_camera ur5_motion"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ur-robot-driver \
    ros-humble-rosidl-default-generators \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    git \
    ca-certificates \
    libusb-1.0-0-dev \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# orbbec_camera 的构建依赖（ur_robot_driver 不拉；其 CMakeLists find_package 逐个要）
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    ros-humble-diagnostic-updater \
    ros-humble-rclcpp-components \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-std-srvs \
    ros-humble-backward-ros \
    ros-humble-ament-index-cpp \
    libgflags-dev \
    libgoogle-glog-dev \
    libssl-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# 只拷硬件层需要的包（isaac_moveit/moveit_resources 不进；.dockerignore 已排除
# scenes/docs/tools 等）。⚠ ur5_motion 必须进：real_gripper_action/handeye_calib
# 在里面，hardware.launch.py backend:=real 会在容器里起前者的可执行文件。
COPY src/ur5_bringup /ws/src/ur5_bringup
COPY src/ur5_motion /ws/src/ur5_motion
COPY src/third_party/lebai-ros-sdk/lebai_interfaces /ws/src/lebai_interfaces
COPY src/third_party/lebai-ros-sdk/lebai_driver /ws/src/lebai_driver
COPY src/third_party/OrbbecSDK_ROS2 /ws/src/OrbbecSDK_ROS2
RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --packages-up-to ${HW_PKGS} \
    && rm -rf build log

ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY deploy/jetson/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# docker exec 不走 ENTRYPOINT，靠两处自动 source（缺了 exec 进去找不到 ur5_bringup 包）：
#   /etc/profile.d/*.sh —— `bash -lc`（login，非交互，.bashrc 有早退守卫会被跳过）
#   /root/.bashrc       —— `docker exec -it ... bash`（交互）
RUN echo '. /opt/ros/humble/setup.sh 2>/dev/null; . /ws/install/setup.bash 2>/dev/null' > /etc/profile.d/ros-ws.sh \
    && echo '. /opt/ros/humble/setup.sh 2>/dev/null; . /ws/install/setup.bash 2>/dev/null' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
