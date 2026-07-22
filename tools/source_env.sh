#!/bin/bash
# 统一的 ROS2 环境激活脚本。
#
# 解决本机头号陷阱：conda `base` 会被 .bashrc 自动激活，导致 python3 指向
# /opt/miniconda3/bin/python3，与 /opt/ros/humble 的 C++ 栈（rclpy/消息）ABI 不符，
# ROS2 节点会收发失败。本脚本强制回到系统 python(/usr/bin/python3) 再 source ROS2。
#
# 用法（每个新终端、每次跑 ROS2 命令前）：
#   source tools/source_env.sh
# 或在仓库根目录直接：
#   . tools/source_env.sh

# 1) 退出所有 conda 环境
conda deactivate 2>/dev/null

# 2) 从 PATH 里剔除 miniconda / anaconda 条目（conda deactivate 有时清不干净）
export PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -E 'miniconda|anaconda|/conda' | paste -sd ':')"
hash -r 2>/dev/null

# 3) 一劳永逸：禁用 conda base 自动激活（仅在你同意时取消下一行注释）
#    conda config --set auto_activate_base false

# 4) source 系统 ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[source_env.sh] 警告: /opt/ros/humble/setup.bash 不存在" >&2
fi

# 4.5) Orbbec SDK + Lebai gripper 现作为 submodule 在 src/third_party/，随本仓库
#      colcon build 一起进 install/（不再需要独立 ~/ros2_ws / ~/lebai_ws overlay）。
THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$THIS_DIR/.." && pwd)"

# 5) RMW 与 Isaac Sim 的 setup_ros_env.sh 保持一致（默认即此值，显式设置以防被改）
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 6) 本仓库工作空间的 overlay（若已 colcon build）—— 用 THIS_DIR 而非 $(pwd)，
#    使从任意子目录 source 都生效。
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
fi

# 7) ★ 真机 IP（编辑 tools/robot_ip.sh 即可，这里自动加载）★
#    设了 UR5_HOST 后，真机栈命令就不用再带 robot_ip:= 了。
[ -f "$THIS_DIR/robot_ip.sh" ] && source "$THIS_DIR/robot_ip.sh"

# 自检
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    # 被 source 时打印一行确认
    echo "[source_env.sh] python3=$(command -v python3)  ros2=$(command -v ros2)"
fi
