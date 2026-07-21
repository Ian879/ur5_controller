#!/bin/bash
# 相机自检：核对 4 个标准 topic 在线 + 2 个 frame 在 TF 树。跨后端复用。
# 用法: source tools/source_env.sh && ./tools/check_camera.sh
set -u

echo "=== 相机 topic（应有 4 个标准 topic）==="
for t in /camera/color/image_raw /camera/color/camera_info \
         /camera/depth/image_rect_raw /camera/depth/camera_info; do
    if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
        hz=$(timeout 3 ros2 topic hz "$t" 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
        echo "  OK   $t   ${hz:-(camera_info 无 Hz)} "
    else
        echo "  MISS $t"
    fi
done

echo "=== 相机 frame（TF 树里应能解析到 tool0）==="
for f in camera_color_optical_frame camera_depth_optical_frame; do
    if timeout 2 ros2 run tf2_ros tf2_echo tool0 "$f" 2>/dev/null | grep -q -- 'At time'; then
        echo "  OK   tool0 → $f"
    else
        echo "  MISS tool0 → $f（TF 树未连通；起 full_stack 或 camera.launch.py 了吗？）"
    fi
done
