#!/usr/bin/env bash
# Jetson【调试视图】RViz 一键起：显示器接在 Jetson 上时，本机看/管全栈。
#
# 不动 ur5_hw 硬件层容器，另起一次性容器跑 rviz2；host 网络 → DDS 同域
# 看见全部话题（含 GPU 服务器上的 move_group / FP），MotionPlanning 面板
# 可直接远程 Plan & Execute（规划在服务器算，执行经 DDS 回硬件层）。
#
# 用法（Jetson 上，仓库根目录；桌面终端或 SSH 均可——SSH 下脚本自己借
# gnome 会话的 XAUTHORITY 放行 X）：
#   ./deploy/jetson/jetson_rviz.sh                    # 默认 view_robot.rviz
#   CONFIG=/path/x.rviz ./deploy/jetson/jetson_rviz.sh # 自带任意 .rviz
#   SOFTWARE_GL=1 ./deploy/jetson/jetson_rviz.sh       # GPU 渲染崩时退 llvmpipe
# 需要镜像含 moveit rviz 插件（Dockerfile 末层，2026-10-24 起）。
set -euo pipefail

IMAGE="${IMAGE:-ur5-hw:humble}"
NAME=ur5_rviz
DISPLAY_NAME="${DISPLAY:-:0}"   # Jetson GNOME 桌面默认 :0
CONFIG="${CONFIG:-}"

# X 放行本机 docker 容器；SSH 里没 DISPLAY 就借 gnome 会话的 XAUTHORITY（实测
# /run/user/1000/gdm/Xauthority，2026-10-24）
if command -v xhost >/dev/null; then
  if ! DISPLAY="$DISPLAY_NAME" xhost +local:docker >/dev/null 2>&1; then
    GPID="$(pgrep -n gnome-shell || true)"
    if [ -n "$GPID" ]; then
      XA="$(tr '\0' '\n' < "/proc/$GPID/environ" 2>/dev/null | sed -n 's/^XAUTHORITY=//p')"
      [ -n "$XA" ] && XAUTHORITY="$XA" DISPLAY="$DISPLAY_NAME" xhost +local:docker >/dev/null 2>&1 || true
    fi
  fi
fi

# GPU 渲染走 nvidia runtime（Jetson 装了 nvidia-container-runtime；实测 GL 4.5）
GPU_ARGS=(--runtime nvidia -e NVIDIA_DRIVER_CAPABILITIES=all)
[ "${SOFTWARE_GL:-0}" = "1" ] && GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)

docker rm -f "$NAME" >/dev/null 2>&1 || true
exec docker run --rm -it --name "$NAME" \
  --network host --ipc host \
  "${GPU_ARGS[@]}" \
  -e DISPLAY="$DISPLAY_NAME" \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  "$IMAGE" bash -lc "
    source /etc/profile.d/ros-ws.sh
    exec rviz2 -d ${CONFIG:-\$(ros2 pkg prefix ur5_bringup)/share/ur5_bringup/rviz/view_robot.rviz}
  "
