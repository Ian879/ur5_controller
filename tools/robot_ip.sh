#!/bin/bash
# ============================================================
#  ★ 唯一的真机 IP 配置处 ★
#  上电后，把下面这行的 IP 改成你 UR5 的真实 IP，保存即可。
#  tools/source_env.sh 会自动 source 本文件，所以你不用再做别的。
#
#  之后启动真机栈：
#     ros2 launch ur5_bringup full_stack.launch.py backend:=real
#  （无需再带 robot_ip:=，因为环境变量已设；也可临时 robot_ip:=x.x.x.x 覆盖）
# ============================================================
export UR5_HOST="10.67.64.5"
