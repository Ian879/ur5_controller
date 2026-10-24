# Jetson Xavier NX（JetPack 5 / Ubuntu 20.04 宿主）硬件层镜像 —— 方案A（2026-09-28）。
#
# 只跑 hardware.launch.py（ur_robot_driver + Lebai 夹爪 + Orbbec 相机）；
# move_group / planner / demo / FoundationPose 在 GPU 服务器——本镜像不含
# MoveIt / pymoveit2 / isaac 相关任何东西。
# （例外：末尾一层装 moveit 的 rviz 插件——见文件底部"RViz 调试视图"。）
#
# 纯 CPU 控制：普通 ros:humble 即可（无需 L4T/CUDA 变体镜像）。
# arm64 在 Jetson 上构建（首次 15-40min）；amd64 可在开发服务器烟测构建
# 验证 Dockerfile 正确性：
#   docker build -f deploy/jetson/Dockerfile -t ur5-hw:smoke .
FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
# HW_PKGS：colcon 构建子集。⚠ 2026-10-10 真机首跑教训：exec_depend 的包
# --packages-up-to 不会带，但运行时全是刚需——
#   ur5_motion（real_gripper_action/handeye_calib 在里面，launch 拉它们）
#   lebai_lm3_support + lebai_resources（URDF 夹爪 macro 及其 mesh）
#   orbbec_description（URDF 相机段）
# 若 OrbbecSDK 在某架构缺预编译库导致构建失败，可
#   --build-arg HW_PKGS="ur5_bringup lebai_driver ur5_motion lebai_lm3_support lebai_resources orbbec_description"
# 先验证其余部分（arm64 现场再处理 orbbec，见 docs/jetson_deployment.md）。
ARG HW_PKGS="ur5_bringup lebai_driver orbbec_camera ur5_motion lebai_lm3_support lebai_resources orbbec_description"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ur-robot-driver \
    ros-humble-rosidl-default-generators \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    ca-certificates \
    libusb-1.0-0-dev \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# ⚠ pylebai（Lebai 夹爪 SDK）必须显式装：lebai_driver 的 setup.py install_requires
# 声明了它，但 colcon build 不会履行——不装的话 serial_gripper 运行时才炸
# （from pylebai.gripper import Gripper，2026-10-09 实测镜像里没有它）。
# PyPI 有 cp310 x86_64/aarch64 双架构 wheel（版本对齐 lebai_driver 的 >=2.0,<3.0）。
RUN pip3 install --no-cache-dir 'pylebai>=2.0.0,<3.0.0'

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

# 8G Jetson 防 OOM：C++ 编译并行度可调。默认空=全核（amd64 烟测无压力）；
# Jetson 上 OOM（cc1plus 被 kill）就 --build-arg JOBS=2 重来。
ARG JOBS=""

WORKDIR /ws
# 只拷硬件层需要的包（isaac_moveit/moveit_resources 不进；.dockerignore 已排除
# scenes/docs/tools 等）。⚠ ur5_motion 必须进：real_gripper_action/handeye_calib
# 在里面，hardware.launch.py backend:=real 会在容器里起前者的可执行文件。
COPY src/ur5_bringup /ws/src/ur5_bringup
COPY src/ur5_motion /ws/src/ur5_motion
COPY src/third_party/lebai-ros-sdk/lebai_interfaces /ws/src/lebai_interfaces
COPY src/third_party/lebai-ros-sdk/lebai_driver /ws/src/lebai_driver
COPY src/third_party/lebai-ros-sdk/lebai_lm3_support /ws/src/lebai_lm3_support
COPY src/third_party/lebai-ros-sdk/lebai_resources /ws/src/lebai_resources
COPY src/third_party/OrbbecSDK_ROS2 /ws/src/OrbbecSDK_ROS2
RUN . /opt/ros/humble/setup.sh \
    && if [ -n "$JOBS" ]; then export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"; fi \
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

# RViz 调试视图（2026-10-24）：Jetson 接显示器当现场调试站——deploy/jetson/jetson_rviz.sh
# 起一次性容器（host 网络，DDS 同域）跑 rviz2，看/管全栈话题（含服务器 move_group，
# MotionPlanning 面板可直接远程规划执行）。裸 rviz2 是 ros:humble 自带，但 MotionPlanning
# 插件（moveit_rviz_plugin）镜像里没有，得显式装；vision-msgs 插件给 /pose_estimation/output
# （Detection3DArray）画 3D 框。
# ⚠ 本层放在 colcon 构建层之后：改这行只重建本层，不动 15min 的构建缓存。
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit-ros-visualization \
    ros-humble-vision-msgs-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
