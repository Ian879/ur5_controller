#!/usr/bin/env bash
# Jetson【硬件层】一键启动：确保容器在 → 前台起 hardware.launch.py（Ctrl-C 停 launch，容器保留）。
#
# 用法（在 Jetson 上、仓库根目录）：
#   ./deploy/jetson/jetson_up.sh                     # 默认 backend:=real
#   ROBOT_IP=10.67.64.5 ./deploy/jetson/jetson_up.sh # 或环境变量 UR5_HOST
#   ./deploy/jetson/jetson_up.sh --shell             # 只进容器不开 launch（调试）
#   ./deploy/jetson/jetson_up.sh --build             # 镜像不在先构建（内网先 export http_proxy）
#   ./deploy/jetson/jetson_up.sh world_to_base_z:=0.75   # 额外 launch 参数直接透传
#   ./deploy/jetson/jetson_down.sh                   # 停容器
set -euo pipefail

IMAGE="${IMAGE:-ur5-hw:humble}"
CONTAINER=ur5_hw
UR5_HOST="${ROBOT_IP:-${UR5_HOST:-10.67.64.5}}"
GRIPPER_DEV="${GRIPPER_DEV:-/dev/ttyUSB0}"   # 阶段2 后建议 udev 别名 /dev/lebai_gripper
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

BUILD=0; SHELL_ONLY=0; EXTRA=()
for a in "$@"; do
  case "$a" in
    --build) BUILD=1 ;;
    --shell) SHELL_ONLY=1 ;;
    *) EXTRA+=("$a") ;;
  esac
done

command -v docker >/dev/null || { echo "缺 docker（见 docs/jetson_deployment.md §4.2）"; exit 1; }

# 镜像在吗
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  if [ "$BUILD" = "1" ]; then
    echo "[jetson_up] 镜像 $IMAGE 不在，开始构建（内网需代理：先 export http_proxy/https_proxy）..."
    docker build --network=host \
      ${http_proxy:+--build-arg http_proxy="$http_proxy"} \
      ${https_proxy:+--build-arg https_proxy="$https_proxy"} \
      ${no_proxy:+--build-arg no_proxy="$no_proxy"} \
      -f "$REPO/deploy/jetson/Dockerfile" -t "$IMAGE" "$REPO"
  else
    echo "缺镜像 $IMAGE —— 加 --build 构建，或先看 docs/jetson_deployment.md §5"; exit 1
  fi
fi

# 容器在吗（在就直接复用——launch 参数不烤进容器，每次 exec 时给）
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "[jetson_up] 起容器 $CONTAINER（host 网络 + ipc + privileged，UR5_HOST=$UR5_HOST）..."
  docker run -d --name "$CONTAINER" \
    --network host --ipc host --privileged \
    --device "$GRIPPER_DEV" \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e UR5_HOST="$UR5_HOST" \
    "$IMAGE" sleep infinity
else
  docker start "$CONTAINER" >/dev/null 2>&1 || true
  # UR5_HOST 变了的话更新（容器只能重建才改 env——提示而不是静默用旧 IP）
  cur_ip="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER" | sed -n 's/^UR5_HOST=//p')"
  [ "$cur_ip" = "$UR5_HOST" ] || echo "[jetson_up] ⚠ 容器 env UR5_HOST=$cur_ip 与当前 $UR5_HOST 不一致；以容器内为准。要改：docker rm $CONTAINER 后重跑本脚本"
fi

if [ "$SHELL_ONLY" = "1" ]; then
  exec docker exec -it "$CONTAINER" bash
fi

# 前台起硬件层（Ctrl-C 只停 launch；容器保留）。bash -lc 让 .bashrc 的自动 source 生效。
echo "[jetson_up] 起 hardware.launch.py backend:=real（Ctrl-C 停）..."
exec docker exec -it "$CONTAINER" bash -lc \
  "ros2 launch ur5_bringup hardware.launch.py backend:=real gripper_port:=$GRIPPER_DEV ${EXTRA[*]}"
