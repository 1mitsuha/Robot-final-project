#!/bin/bash
# ============================================
# ROS Noetic Docker 容器启动脚本
# ============================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 允许本地 X11 连接（图形界面转发）
xhost +local:docker > /dev/null 2>&1 || true

echo "启动 ROS Noetic 容器..."
echo ""

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(cd "$SCRIPT_DIR/.." && pwd)/src:/home/rosuser/catkin_ws/src" \
    -v "$(cd "$SCRIPT_DIR/.." && pwd)/log:/home/rosuser/log" \
    --network host \
    --name ros-noetic-sim \
    ros-noetic:full
