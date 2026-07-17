#!/bin/bash
# 运动栈自检：起栈后跑此脚本，确认 controller / topic 是否就绪。
# 用法: source tools/source_env.sh && ./tools/check_topics.sh
set -u

say() { printf '\n===== %s =====\n' "$1"; }

say "1) controller 清单（应有 joint_trajectory_controller 与 joint_state_broadcaster，状态 active）"
timeout 5 ros2 control list_controllers 2>&1 || echo "  [!] 未取到 controller（ros2_control_node 是否已起？）"

say "2) /joint_states 采样一次（应为 6 个关节值）"
timeout 5 ros2 topic echo /joint_states --once 2>&1 | head -20 || echo "  [!] /joint_states 无数据"

say "3) move_group 是否在线（应有 /move_action 等）"
timeout 5 ros2 topic list 2>&1 | grep -E "move_action|/execute_trajectory|apply_planning_scene|/joint_states" || echo "  [!] move_group 话题缺失"

say "4) TF 关键帧（应有 base_link、tool0、wrist_3_link 等）"
timeout 8 ros2 run tf2_ros tf2_echo base_link tool0 2>&1 | head -8 || echo "  [!] base_link->tool0 TF 取不到"

say "自检完成。"
