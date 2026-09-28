#!/usr/bin/env bash
# 停真机【服务器侧】栈（配合 bringup_real.sh）。Jetson 侧见 deploy/jetson/jetson_down.sh。
set -uo pipefail

C_CYAN=$'\033[36m'; C_GRN=$'\033[32m'; C_OFF=$'\033[0m'
log() { printf "${C_CYAN}[down-real]${C_OFF} %s\n" "$*"; }

SESSION="ur5_real"
set +e +u +o pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/source_env.sh" >/dev/null 2>&1 || true
set -uo pipefail

log "杀 tmux session $SESSION ..."
tmux kill-session -t "$SESSION" 2>/dev/null && log "session killed" || log "无 session"

log "pkill 孤儿进程 ..."
pkill -f "server/fp_server.py" 2>/dev/null && log "  fp_server" || true
pkill -f "ros2_nodes/fp_node.py" 2>/dev/null && log "  fp_node" || true
pkill -f "gt_mask_from_depth.py" 2>/dev/null && log "  gt_mask" || true
pkill -f "sam_segmentation.py" 2>/dev/null && log "  sam" || true
sleep 1

remain="$(ps -eo args | grep -E "fp_server.py|fp_node.py" | grep -vE "grep|bringup_real_down" || true)"
if [ -z "$remain" ]; then
  printf "${C_GRN}[down-real]${C_OFF} 干净 ✓（服务器侧栈已全停；Jetson 侧另停）\n"
else
  log "还有残留："; echo "$remain"
fi
