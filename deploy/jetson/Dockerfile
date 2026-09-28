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
# HW_PKGS：colcon 构建子集。若 OrbbecSDK 在某架构缺预编译库导致构建失败，可
#   --build-arg HW_PKGS="ur5_bringup lebai_driver"
# 先验证其余部分（arm64 现场再处理 orbbec，见 docs/jetson_deployment.md）。
ARG HW_PKGS="ur5_bringup lebai_driver orbbec_camera"

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
# 只拷硬件层需要的包：规划层（ur5_motion/isaac_moveit/moveit_resources）不进镜像。
# （.dockerignore 已排除 scenes/docs/tools 等，build context 亦只含 src 所需）
COPY src/ur5_bringup /ws/src/ur5_bringup
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
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
