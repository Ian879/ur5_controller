#!/usr/bin/env bash
# 停 isaac+感知 全栈（配合 bringup_sim.sh）。
# tmux kill 带走窗内进程；pkill 兜底清 conda-run / isaac kit 孤儿。
set -uo pipefail

C_CYAN=$'\033[36m'; C_GRN=$'\033[32m'; C_OFF=$'\033[0m'
log() { printf "${C_CYAN}[down]${C_OFF} %s\n" "$*"; }

SESSION="ur5_sim"
# 脚本自身要 ros2 列节点验证，尽量 source（失败不阻塞）。
# ⚠️ source source_env.sh 会被 humble setup.bash 的未设变量在 set -u 下杀掉
#    （同 bringup_sim.sh 的坑：unbound variable 是 expansion error，|| true 救不了）
#    ——source 前临时关 -u -o pipefail，source 完再开。
set +u +o pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/source_env.sh" >/dev/null 2>&1 || true
set -uo pipefail

log "杀 tmux session $SESSION ..."
tmux kill-session -t "$SESSION" 2>/dev/null && log "session killed" || log "无 session"

log "pkill 孤儿进程 ..."
pkill -f "ur5_isaac_bridge.py" 2>/dev/null && log "  isaac 桥" || true
pkill -f "server/fp_server.py" 2>/dev/null && log "  fp_server" || true
pkill -f "ros2_nodes/fp_node.py" 2>/dev/null && log "  fp_node" || true
pkill -f "gt_mask_from_depth.py" 2>/dev/null && log "  gt_mask" || true
pkill -f "sam_segmentation.py" 2>/dev/null && log "  sam" || true
# isaac kit（python.sh 起的 kit 子进程）
pkill -f "isaacsim.exp.full.streaming.kit" 2>/dev/null && log "  isaac kit" || true
sleep 1

# 验证
remain="$(ps -eo args | grep -E "ur5_isaac_bridge|fp_server.py|fp_node.py" | grep -vE "grep|bringup_down" || true)"
if [ -z "$remain" ]; then
  printf "${C_GRN}[down]${C_OFF} 干净 ✓（isaac/fp_server/fp_node 都退了）\n"
else
  log "还有残留："
  echo "$remain"
fi
