#!/bin/bash
# ============================================
# ROS Noetic Docker 镜像构建脚本
# ============================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo " 构建 ROS Noetic Full 仿真镜像"
echo "========================================"
echo ""

cd "$SCRIPT_DIR"

# 构建镜像
docker build \
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    -t ros-noetic:full \
    -f Dockerfile \
    .

echo ""
echo "========================================"
echo " 构建完成！"
echo " 镜像: ros-noetic:full"
echo "========================================"
echo ""
echo "启动容器:"
echo "  docker run -it --rm \\"
echo "    -e DISPLAY=\$DISPLAY \\"
echo "    -v /tmp/.X11-unix:/tmp/.X11-unix \\"
echo "    -v $(cd "$SCRIPT_DIR/.." && pwd)/src:/home/rosuser/catkin_ws/src \\"
echo "    --network host \\"
echo "    ros-noetic:full"
echo ""
echo "或使用 docker-compose:"
echo "  cd docker && docker compose up -d && docker exec -it ros-noetic-sim bash"
