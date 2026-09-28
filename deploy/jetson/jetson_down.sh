#!/usr/bin/env bash
# 停 Jetson 硬件层容器（launch 本身 Ctrl-C 即停；本脚本连容器一起停）。
set -uo pipefail
CONTAINER=ur5_hw
if docker inspect "$CONTAINER" >/dev/null 2>&1; then
  docker stop "$CONTAINER" >/dev/null && echo "[jetson_down] $CONTAINER 已停（镜像保留；docker rm $CONTAINER 可删容器）"
else
  echo "[jetson_down] 容器 $CONTAINER 不在"
fi
