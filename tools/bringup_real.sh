#!/usr/bin/env bash
# 真机【服务器侧】一键启动（tmux 5 窗）：plan | fp_server | mask | fp_node | client
#
# 硬件层（Jetson 侧）不在本脚本管辖——另起，见 deploy/jetson/jetson_up.sh。
# 与 bringup_sim.sh 的区别：无 isaac/hw 两窗；planning 用墙钟（use_sim_time:=false）；
# fp_node 不带 use_sim_time；mask 默认 sam（真机没有深度 GT mask 的话）。
#
# 用法：
#   ./tools/bringup_real.sh               # 默认 RViz 开，最后 attach
#   RVIZ=0   ./tools/bringup_real.sh      # 不起 RViz
#   ATTACH=0 ./tools/bringup_real.sh      # 后台起完即退
#   MASK=sam ./tools/bringup_real.sh      # 分割后端（sam 默认；gt=深度带，真机深度也可试）
#   ./tools/bringup_real_down.sh          # 全停
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JIAYIN="$HOME/projects/jiayin"
ROBOT_PY="$HOME/.conda/envs/robot/bin/python"
SESSION="ur5_real"
MASK="${MASK:-sam}"          # sam | gt
RVIZ="${RVIZ:-1}"
ATTACH="${ATTACH:-1}"
# RMW 必须三方一致（服务器/Jetson/UR 侧容器都固化 fastrtps）
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

C_CYAN=$'\033[36m'; C_YEL=$'\033[33m'; C_GRN=$'\033[32m'; C_OFF=$'\033[0m'
log()  { printf "${C_CYAN}[bringup-real]${C_OFF} %s\n" "$*"; }
ok()   { printf "${C_GRN}[bringup-real]${C_OFF} %s\n" "$*"; }
warn() { printf "${C_YEL}[bringup-real]${C_OFF} %s\n" "$*" >&2; }

command -v tmux >/dev/null || { echo "缺 tmux"; exit 1; }
[ -x "$ROBOT_PY" ] || { echo "缺 robot conda env：$ROBOT_PY"; exit 1; }
[ -f "$JIAYIN/ros2_nodes/fp_node.py" ] || { echo "缺 fp_node：$JIAYIN/ros2_nodes/fp_node.py"; exit 1; }

# source 项目环境做健康检查（set -u 下 humble setup.bash 会炸，临时关——bringup_sim 同坑）
set +e +u +o pipefail
# shellcheck disable=SC1091
source "$REPO/tools/source_env.sh" >/dev/null 2>&1
set -e -u -o pipefail
command -v ros2 >/dev/null 2>&1 || warn "source 后 ros2 仍不可用（健康检查会失效）"

_wait_for() {
  local desc="$1" check="$2" secs="$3" i=0
  printf "${C_CYAN}[gate]${C_OFF} 等 %s " "$desc"
  while (( i < secs )); do
    if eval "$check" >/dev/null 2>&1; then printf "${C_GRN}✓${C_OFF}(%ds)\n" "$i"; return 0; fi
    printf "."; sleep 1
    i=$((i + 1))
  done
  printf "${C_YEL}⚠ 超时(%ds)，继续${C_OFF}\n" "$secs" >&2; return 0
}

tmux kill-session -t "$SESSION" 2>/dev/null && log "杀掉旧 session $SESSION" || true

# ── 窗 1: 规划层 plan（move_group + world_sync，墙钟） ─────
log "起规划层 planning.launch.py（窗 plan）..."
tmux new-session -d -s "$SESSION" -n plan
RVIZ_ARG="launch_rviz:=$([ "$RVIZ" = "1" ] && echo true || echo false)"
# RViz 要 X：tmux 服务器环境常无 DISPLAY（本机桌面在 :1）——起 RViz 时显式补，否则静默死
DISP_PREFIX=""
[ "$RVIZ" = "1" ] && DISP_PREFIX='export DISPLAY=${DISPLAY:-:1}; '
tmux send-keys -t "$SESSION:plan" \
  "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; $DISP_PREFIX cd $REPO && source tools/source_env.sh && ros2 launch ur5_bringup planning.launch.py use_sim_time:=false $RVIZ_ARG" C-m
_wait_for "move_group" "timeout 4 ros2 node list 2>/dev/null | grep -q '^/move_group$'" 60

# ── 缝检查：Jetson 硬件层在吗（不在只 warn——可能还没起） ──
_wait_for "/joint_states（Jetson 硬件层）" "timeout 8 ros2 topic info /joint_states 2>/dev/null | grep -q 'Publisher count: [1-9]'" 15
ros2 topic info /joint_states 2>/dev/null | grep -q 'Publisher count: [1-9]' \
  || warn "⏳ 未见 /joint_states —— Jetson 硬件层未起？（deploy/jetson/jetson_up.sh；起来后自动互通，无需重启本侧）"

# ── 窗 2: fp_server（py3.11） ───────────────────────────────
log "起 fp_server（窗 fp_server）..."
tmux new-window -t "$SESSION" -n fp_server
tmux send-keys -t "$SESSION:fp_server" \
  "cd $JIAYIN && PYTHONPATH=\$PWD conda run --no-capture-output -p envs/fp python server/fp_server.py" C-m
_wait_for "fp_server /health" "curl -sf localhost:8765/health" 40

# ── 窗 3: mask（py3.10 robot） ──────────────────────────────
MASK_FILE="sam_segmentation.py"
[ "$MASK" = "gt" ] && MASK_FILE="gt_mask_from_depth.py"
log "起 mask=$MASK（窗 mask）..."
tmux new-window -t "$SESSION" -n mask
tmux send-keys -t "$SESSION:mask" \
  "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $REPO && source tools/source_env.sh && $ROBOT_PY src/ur5_motion/ur5_motion/$MASK_FILE" C-m
# 门禁=节点 banner（构造即建 publisher；高负载下 DDS 发现滞后，topic info 会误报——bringup_sim 同结论）
_wait_for "mask 节点就绪" "tmux capture-pane -t $SESSION:mask -p 2>/dev/null | grep -qE '\[(gt_mask_from_depth|sam_segmentation)\]'" 60

# ── 窗 4: fp_node（py3.10 robot；真机墙钟，不带 use_sim_time） ─
log "起 fp_node（窗 fp_node）..."
tmux new-window -t "$SESSION" -n fp_node
tmux send-keys -t "$SESSION:fp_node" \
  "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; export PYTHONPATH=$JIAYIN:\$PYTHONPATH; $ROBOT_PY -u $JIAYIN/ros2_nodes/fp_node.py" C-m
_wait_for "/foundationpose/track" "timeout 3 ros2 service list 2>/dev/null | grep -q '/foundationpose/track'" 15

# ── 窗 5: client（空 shell 备好环境） ───────────────────────
tmux new-window -t "$SESSION" -n client
tmux send-keys -t "$SESSION:client" "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $REPO && source tools/source_env.sh" C-m
tmux select-window -t "$SESSION:client"

ok "服务器侧就绪 ✓  5 窗：plan | fp_server | mask | fp_node | client"
echo "  Ctrl-b d 脱离 | demo: POSE_SOURCE=fp ros2 run ur5_motion demo_grasp"
echo "  停：./tools/bringup_real_down.sh"

if [ "$ATTACH" = "1" ] && [ -t 1 ]; then
  exec tmux attach-session -t "$SESSION"
fi
