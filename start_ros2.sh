#!/bin/bash
# =============================================================================
# 一键启动 ROS2 Humble + TurtleBot3 仿真
# 自动处理 conda 冲突、环境变量、编译和启动
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$SCRIPT_DIR"

echo "=============================================="
echo "  ROS2 TurtleBot3 仿真一键启动"
echo "=============================================="

# 1. 移除 conda 的 Python，确保使用系统 Python 3.10
echo "[1/4] 清理 conda 环境..."
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | grep -v anaconda | tr '\n' ':')

if which python3 | grep -q miniconda; then
    echo "  ✗ 仍有 conda Python 在 PATH 中，请手动处理"
    exit 1
fi
echo "  ✓ Python: $(which python3) ($(python3 --version))"

# 2. 加载 ROS2 环境
echo "[2/4] 加载 ROS2 环境..."
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "  ✗ 找不到 /opt/ros/humble/setup.bash，请确认 ROS2 Humble 已安装"
    exit 1
fi
source /opt/ros/humble/setup.bash
echo "  ✓ ROS_VERSION = $ROS_VERSION ($ROS_DISTRO)"

# 3. 编译（增量，有变化才重编）
echo "[3/4] 编译工作空间..."
cd "$WORKSPACE"
if [ -d install ]; then
    source install/setup.bash 2>/dev/null || true
fi
colcon build --symlink-install 2>&1 | tail -5
source install/setup.bash
echo "  ✓ 编译完成"

# 4. 启动仿真
echo "[4/4] 启动仿真..."
export TURTLEBOT3_MODEL=waffle
echo "  TURTLEBOT3_MODEL = $TURTLEBOT3_MODEL"
echo ""
echo "  等待 Gazebo 启动 (约15秒)..."
echo "  等待地图加载 (约60秒)..."
echo "  启动后用 RViz2 的 2D Nav Goal 设置目标点"
echo "=============================================="
echo ""

ros2 launch Robot-Planner obs_world_ros2.launch.py
