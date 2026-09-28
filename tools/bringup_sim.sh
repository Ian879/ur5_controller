#!/usr/bin/env bash
# isaac+感知 一键启动（tmux 7 窗口）。把 5-7 个手动终端压成 1 命令。
# 按依赖顺序拉起，每步健康检查（超时只 warn 不阻塞），最后 attach。
#
# 用法：
#   ./tools/bringup_sim.sh              # 默认 GT mask，带 RViz，最后 attach
#   MASK=sam  ./tools/bringup_sim.sh     # 用 SAM 分割代替深度 GT mask
#   RVIZ=0   ./tools/bringup_sim.sh      # 不起 RViz
#   ATTACH=0 ./tools/bringup_sim.sh      # 不 attach（后台起完即退出）
#   ./tools/bringup_down.sh              # 全停
#
# 7 窗口：isaac | hw | plan | fp_server | mask | fp_node | client
# （hw/plan 分窗 = 真机方案A 的缝：硬件层↔规划层只靠 DDS，未来分居 Jetson/服务器）
# 进程隔离是 ABI 硬墙（isaac 3.12 / ros 3.10 / robot-conda 3.10 / fp_server 3.11），无法合并，
# 本脚本只做编排（每窗对应 env + 顺序 + 健康检查），不改任何节点代码。
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC="$HOME/isaacsim"
JIAYIN="$HOME/projects/jiayin"
ROBOT_PY="$HOME/.conda/envs/robot/bin/python"
SESSION="ur5_sim"
MASK="${MASK:-gt}"          # gt | sam
RVIZ="${RVIZ:-1}"
ATTACH="${ATTACH:-1}"
# RMW：cyclonedds（fastdds 的 action goal 投递间歇性超时——service 正常但 /move_action
# goal response 发不回，16MB UDP 缓冲只缓解未根治；cyclone 单机可靠）。所有窗口+健康检查
# 必须同一 RMW。source_env.sh / setup_ros_env.sh 都尊重此预设值。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# 颜色
C_CYAN=$'\033[36m'; C_YEL=$'\033[33m'; C_GRN=$'\033[32m'; C_RED=$'\033[31m'; C_OFF=$'\033[0m'
log()  { printf "${C_CYAN}[bringup]${C_OFF} %s\n" "$*"; }
warn() { printf "${C_YEL}[bringup]${C_OFF} %s\n" "$*" >&2; }
ok()   { printf "${C_GRN}[bringup]${C_OFF} %s\n" "$*"; }
die()  { printf "${C_RED}[bringup]${C_OFF} %s\n" "$*" >&2; exit 1; }

# ── 前置检查 ──────────────────────────────────────────────
log "启动 bringup_sim（MASK=$MASK RVIZ=$RVIZ ATTACH=$ATTACH）"
[ -f "$HOME/Desktop/scene.usd" ] || die "缺 ~/Desktop/scene.usd（先在 Isaac GUI 搭场景）"
command -v tmux >/dev/null       || die "缺 tmux：sudo apt install tmux"
[ -x "$ISAAC/python.sh" ]        || die "缺 isaac：$ISAAC/python.sh"
[ -x "$ROBOT_PY" ]               || die "缺 robot conda env：$ROBOT_PY"
[ -d "$JIAYIN/envs/fp" ]         || die "缺 fp env：$JIAYIN/envs/fp"
[ -f "$JIAYIN/ros2_nodes/fp_node.py" ] || die "缺 fp_node：$JIAYIN/ros2_nodes/fp_node.py"

# 脚本自身要跑 ros2 健康检查 → 先 source 项目环境
# ⚠️ 非交互脚本里 source source_env.sh 有两坑：① 首行 `conda deactivate` 返回非零（conda
#    不是函数）→ set -e 杀脚本；② ROS humble/setup.bash 引用未设变量 → set -u 杀脚本。
#    故 source 前临时关 -e -u -o pipefail，source 完再开。tmux 窗口里是交互 shell，不受影响。
set +e +u +o pipefail
# shellcheck disable=SC1091
source "$REPO/tools/source_env.sh" >/dev/null 2>&1
set -e -u -o pipefail
command -v ros2 >/dev/null 2>&1 || warn "source 后 ros2 仍不可用（健康检查会失效，但窗口仍会起）"

MASK_FILE="gt_mask_from_depth.py"
[ "$MASK" = "sam" ] && MASK_FILE="sam_segmentation.py"

# ── 健康检查封装（超时只 warn，不阻塞） ───────────────────
_wait_for() {
  local desc="$1" check="$2" secs="$3" i=0
  printf "${C_CYAN}[gate]${C_OFF} 等 %s " "$desc"
  while (( i < secs )); do
    if eval "$check" >/dev/null 2>&1; then printf "${C_GRN}✓${C_OFF}(%ds)\n" "$i"; return 0; fi
    printf "."; sleep 1
    # ⚠️ 不能用 ((i++))：i=0 时表达式值为 0 → 返回 exit 1 → set -e 杀脚本（已踩）。
    #    用赋值式 i=$((i+1))（恒 exit 0）。
    i=$((i + 1))
  done
  printf "${C_YEL}⚠ 超时(%ds)，继续${C_OFF}\n" "$secs" >&2; return 0
}

# ── 清旧 session（幂等） ──────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null && log "杀掉旧 session $SESSION" || true

# ── 窗 1: isaac 桥（py3.12） ──────────────────────────────
log "起 isaac 桥（窗 isaac）..."
tmux new-session -d -s "$SESSION" -n isaac
tmux send-keys -t "$SESSION:isaac" \
  "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $ISAAC && conda deactivate 2>/dev/null; source setup_ros_env.sh && ./python.sh $REPO/isaac_scripts/ur5_isaac_bridge.py --stream --robot_prim /World/ur5 --object_prim /World/cola" C-m
_wait_for "isaac /clock" "timeout 3 ros2 topic echo --once /clock 2>/dev/null | grep -q sec" 90

# ── 窗 2: 硬件层 hw（controller_manager+控制器+RSP+相机TF；真机=Jetson 侧） ──
log "起硬件层 hardware.launch.py（窗 hw）..."
tmux new-window -t "$SESSION" -n hw
RVIZ_ARG="launch_rviz:=$([ "$RVIZ" = "1" ] && echo true || echo false)"
tmux send-keys -t "$SESSION:hw" \
  "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $REPO && source tools/source_env.sh && ros2 launch ur5_bringup hardware.launch.py backend:=isaac" C-m
# 门禁 = hw 窗日志里控制器激活行（真实条件，免受 ros2 CLI/DDS 发现延迟影响）。
# 预算 420s：Isaac 桥冷启动出 /isaac_joint_states 实测可到 4-5 分钟（热缓存才 30-60s），
# spawner 会一直重试到激活， TopicBasedSystem 一有数据它就成——门禁等它即可
# （2026-09-28 实测：120s/180s 固定预算门禁两次都在控制器激活前耗尽）。
_wait_for "hw 控制器激活" "tmux capture-pane -t $SESSION:hw -p 2>/dev/null | grep -q 'Configured and activated joint_state_broadcaster'" 420

# ── 窗 3: 规划层 plan（move_group+world_sync；真机=GPU 服务器侧） ──
log "起规划层 planning.launch.py（窗 plan）..."
tmux new-window -t "$SESSION" -n plan
tmux send-keys -t "$SESSION:plan" \
  "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $REPO && source tools/source_env.sh && ros2 launch ur5_bringup planning.launch.py use_sim_time:=true $RVIZ_ARG" C-m
_wait_for "move_group" "timeout 4 ros2 node list 2>/dev/null | grep -q '^/move_group$'" 60

# ── 窗 4: fp_server（py3.11） ─────────────────────────────
log "起 fp_server（窗 fp_server）..."
tmux new-window -t "$SESSION" -n fp_server
tmux send-keys -t "$SESSION:fp_server" \
  "cd $JIAYIN && PYTHONPATH=\$PWD conda run --no-capture-output -p envs/fp python server/fp_server.py" C-m
_wait_for "fp_server /health" "curl -sf localhost:8765/health" 40

# ── 窗 5: mask（py3.10 robot） ────────────────────────────
log "起 mask=$MASK（窗 mask）..."
tmux new-window -t "$SESSION" -n mask
tmux send-keys -t "$SESSION:mask" \
  "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $REPO && source tools/source_env.sh && $ROBOT_PY src/ur5_motion/ur5_motion/$MASK_FILE" C-m
# 门禁 = mask 窗日志里节点首行（构造即建 publisher）。不用 `ros2 topic info`：
# isaac 高负载下 fastrtps 发现滞后实测 65-146s，publisher 明明在但 daemon 图看不见，
# 任何预算都会误报（2026-09-28 实测 60s/90s 两版门禁全期误报超时）。
_wait_for "mask 节点就绪" "tmux capture-pane -t $SESSION:mask -p 2>/dev/null | grep -qE '\[(gt_mask_from_depth|sam_segmentation)\]'" 60

# ── 窗 6: fp_node（py3.10 robot） ─────────────────────────
log "起 fp_node（窗 fp_node）..."
tmux new-window -t "$SESSION" -n fp_node
# 复用 /tmp/start_fp_node.sh 的 env（humble + PYTHONPATH jiayin + robot python）。
# ⚠️ 必须 use_sim_time:=true：isaac 后端 base_link 链是 sim 时戳，fp_object 也要 sim 时戳
#    才能链上（否则 "extrapolation into the past"：fp_object wall vs base_link sim）。
tmux send-keys -t "$SESSION:fp_node" \
  "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; export PYTHONPATH=$JIAYIN:\$PYTHONPATH; $ROBOT_PY -u $JIAYIN/ros2_nodes/fp_node.py --ros-args -p use_sim_time:=true" C-m
_wait_for "/foundationpose/track" "timeout 3 ros2 service list 2>/dev/null | grep -q '/foundationpose/track'" 15

# ── 窗 7: client（空 shell 备好环境） ─────────────────────
tmux new-window -t "$SESSION" -n client
tmux send-keys -t "$SESSION:client" "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; cd $REPO && source tools/source_env.sh" C-m

# 选到 client 窗
tmux select-window -t "$SESSION:client"

ok "全部就绪 ✓  7 窗：isaac | hw | plan | fp_server | mask | fp_node | client"
echo "  Ctrl-b 窗口号 切窗 | Ctrl-b d 脱离（后台继续跑）| demo: POSE_SOURCE=fp ros2 run ur5_motion demo_grasp"
echo "  停：./tools/bringup_down.sh"

if [ "$ATTACH" = "1" ] && [ -t 1 ]; then
  exec tmux attach-session -t "$SESSION"
fi
