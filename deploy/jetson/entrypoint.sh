#!/bin/bash
# 硬件层容器入口：叠 ROS humble + 本工作区环境。
set -e
. /opt/ros/humble/setup.sh
# shellcheck disable=SC1091
[ -f /ws/install/setup.bash ] && . /ws/install/setup.bash
exec "$@"
