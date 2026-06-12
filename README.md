# 机器人导论期末大作业 — 自主导航综合实践

## 项目简介

本项目是 2026 年度《机器人导论》课程的期末课程设计，基于 **ROS Noetic / ROS2 Humble + Gazebo** 仿真环境，实现移动机器人的**全局路径规划**与**轨迹跟踪控制**两大核心模块。

系统工作流程：在 Gazebo 加载含随机障碍物的仿真环境 → 占据栅格地图构建 → RRT* 算法进行无碰全局路径规划 → PID 双通道控制器驱动机器人沿路径移动并到达目标。

> **新特性**：代码已适配 ROS1 和 ROS2 双版本，通过 `ros_compat.py` 兼容层实现**同一份 Python 代码**在两个 ROS 版本上均可运行，无需维护两套分支。

## 项目结构

```
final_project/                         # ROS 工作空间
├── src/
│   ├── Robot-Planner/                 # 核心功能包
│   │   ├── scripts/
│   │   │   ├── planner.py             # RRT* 全局路径规划器
│   │   │   ├── controller.py          # PID 轨迹跟踪控制器
│   │   │   ├── gazebo_to_tf.py        # TF 坐标变换广播
│   │   │   ├── ros_compat.py          # ROS1/ROS2 兼容层
│   │   │   └── utils.py               # 共享工具函数（无 ROS 依赖）
│   │   ├── launch/
│   │   │   ├── obs_world.launch       # ROS1 主启动文件
│   │   │   └── obs_world_ros2.launch.py  # ROS2 主启动文件
│   │   └── maps/                      # 占据栅格地图（运行时动态生成）
│   ├── random_map_generator/          # 随机障碍物 & 地图生成器
│   │   ├── src/spawn_obstacles.py
│   │   ├── src/ros_compat.py          # ROS1/ROS2 兼容层（副本）
│   │   └── launch/
│   │       ├── spawn_obstacles.launch      # ROS1 启动文件
│   │       └── spawn_obstacles_ros2.launch.py  # ROS2 启动文件
│   ├── turtlebot3/                    # TurtleBot3 机器人模型
│   ├── turtlebot3_msgs/               # TurtleBot3 自定义消息
│   └── turtlebot3_simulations/        # TurtleBot3 Gazebo 仿真
```

## 环境要求

### ROS1 (Noetic)

| 组件 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | Noetic (完整桌面版) |
| Python | 3.8+ |
| Gazebo | 11.x |
| 机器人模型 | TurtleBot3 Waffle |

### ROS2 (Humble)

| 组件 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS | Humble (完整桌面版) |
| Python | 3.10+ |
| Gazebo | Ignition Fortress (或 Classic 11.x) |
| 机器人模型 | TurtleBot3 Waffle |

---

## Docker 环境（推荐：Ubuntu 22.04 / 24.04 用户）

ROS Noetic 最高只支持 Ubuntu 20.04。如果你使用 Ubuntu 22.04 或更高版本，可通过 Docker 容器运行完整 ROS 环境。

### 构建镜像

```bash
# 构建 ROS Noetic Full 镜像（约 7 GB，首次需 10-30 分钟）
cd docker
bash build.sh
```

镜像已预装 `ros-noetic-desktop-full`、Gazebo 11、TurtleBot3 仿真及所有依赖。

### 启动容器

```bash
# 允许 Docker 访问 X11 图形界面
xhost +local:docker

# 启动容器（src 目录自动挂载到 ~/catkin_ws/src）
bash docker/run.sh

# 或手动启动
sudo docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e TURTLEBOT3_MODEL=waffle \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/src:/home/rosuser/catkin_ws/src \
    --network host \
    ros-noetic:full
```

### 容器内编译与运行

```bash
# 进入容器后，先安装缺失的依赖（首次启动需要）
sudo apt update && sudo apt install -y python3-catkin-tools ros-noetic-map-server

cd ~/catkin_ws

# 编译
catkin build
```

编译成功后，每次启动容器都需要 **同时 source 两个 setup 文件**：

```bash
source /opt/ros/noetic/setup.bash   # ROS 基础环境
source ~/catkin_ws/devel/setup.bash # 你的工作空间（Robot-Planner, random_map_generator）

export TURTLEBOT3_MODEL=waffle

# 启动仿真
roslaunch Robot-Planner obs_world.launch
```

> ⚠️ **重要**：两个 `source` 缺一不可。忘记 `source devel/setup.bash` 会导致 `[obs_world.launch] is neither a launch file...` 错误。忘记 `source /opt/ros/noetic/setup.bash` 会导致 `roslaunch: command not found`。

### Docker Hub 镜像加速（国内用户）

如果 `docker build` 无法拉取 `ubuntu:20.04`，需配置镜像加速器：

```bash
sudo tee /etc/docker/daemon.json << 'EOF'
{
    "registry-mirrors": [
        "https://docker.m.daocloud.io",
        "https://dockerproxy.com",
        "https://docker.mirrors.ustc.edu.cn"
    ]
}
EOF
sudo systemctl restart docker
```

---

## 安装依赖

### ROS1 (Noetic)

```bash
# ROS Noetic 完整安装
sudo apt install ros-noetic-desktop-full

# TurtleBot3 仿真包
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
sudo apt install ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-gazebo
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# 必备工具包（map_server 为规划器提供地图，xacro 解析 URDF）
sudo apt install ros-noetic-map-server ros-noetic-xacro
sudo apt install ros-noetic-navigation ros-noetic-rviz ros-noetic-robot-state-publisher

# 构建工具与 Python 依赖
sudo apt install python3-pip python-is-python3 python3-catkin-tools
pip3 install numpy pillow pyyaml
```

### ROS2 (Humble)

```bash
# ROS2 Humble 完整安装
sudo apt install ros-humble-desktop-full

# TurtleBot3 仿真包
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-bringup ros-humble-rviz2
sudo apt install python3-pip
pip3 install numpy pillow pyyaml
```

---

## 编译与运行

### ROS1 (Noetic)

```bash
# 1. 设置环境变量
export TURTLEBOT3_MODEL=waffle
source /opt/ros/noetic/setup.bash

# 2. 编译（必须用 catkin build，不能用 catkin_make）
cd Robot-final-project
catkin build
source devel/setup.bash

# 3. 启动仿真
roslaunch Robot-Planner obs_world.launch
```

> **注意**：`package.xml` 的 `<build_type>` 为 `ament_cmake`（ROS2/colcon 要求）。  
> ROS1 编译必须使用 `catkin build`（catkin_tools）—— **`catkin_make` 不支持**。  
> 若使用 Docker 容器，`catkin_tools` 已预装，可直接 `catkin build`。

### ROS2 (Humble)

```bash
# 1. 设置环境变量
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash

# 2. 编译
cd Robot-final-project
colcon build --symlink-install
source install/setup.bash

# 直接使用一键启动脚本
bash start_ros2.sh
```

启动后（两种 ROS 版本操作相同）：

1. Gazebo 弹出（3D 仿真世界 + 随机障碍物）
2. 等待 ~60 秒地图加载完成（终端看到 `Map loaded...`）
3. RViz 弹出，Fixed Frame 选择 `map`
4. 点击工具栏 **"2D Nav Goal"** 在地图上设定目标点
5. RRT* 自动规划路径（绿色树状结构），机器人沿蓝色路径移动至目标

> **关于 `src/` 下的第三方包**：turtlebot3 / turtlebot3_msgs / turtlebot3_simulations 是
> ROS1 版本的源码包，供 `catkin build`（ROS1）使用。每个包目录下放置了 `COLCON_IGNORE` 
> 空文件，使得 `colcon build`（ROS2）自动跳过它们。ROS2 用户应通过 apt 安装对应的
> `ros-humble-turtlebot3*` 包（见上方安装依赖）。

---

## ROS1 / ROS2 兼容层说明

### 设计思路

本项目使用 `ros_compat.py` 实现同一份 Python 代码在 ROS1 和 ROS2 上运行，核心思想：

- **运行时检测**：`ros_compat.py` 在 `import` 时通过 `try: import rclpy` 自动判断 ROS 版本
- **统一接口**：提供 `CompatNode` 类封装 publisher / subscriber / timer / service / logger / clock / rate / TF 等所有 ROS API
- **消息类型直接导入**：`Point`、`Twist`、`Odometry`、`Path` 等在两个 ROS 版本中包名一致，无需适配
- **构建时分离**：`CMakeLists.txt` 通过 `ROS_VERSION` 环境变量判断使用 `ament_cmake`（ROS2）还是 `catkin`（ROS1）；`package.xml` 只列双平台共有的依赖

### CompatNode 提供的统一 API

| 类别 | API | ROS1 实现 | ROS2 实现 |
|------|-----|-----------|-----------|
| 初始化 | `CompatNode(name)` | `rospy.init_node()` | `rclpy.create_node()` |
| 发布 | `node.create_publisher(...)` | `rospy.Publisher(...)` | `node.create_publisher(...)` |
| 订阅 | `node.create_subscriber(...)` | `rospy.Subscriber(...)` | `node.create_subscription(...)` |
| 时间 | `node.now()` | `rospy.Time.now()` | `node.get_clock().now()` |
| 频率 | `node.create_rate(hz)` | `rospy.Rate(hz)` | `node.create_rate(hz)` |
| 日志 | `node.loginfo/warn/err(...)` | `rospy.loginfo(...)` | `logger.info(...)` |
| 参数 | `node.get_parameter(...)` | `rospy.get_param(...)` | `node.get_parameter(...)` |
| 服务 | `node.create_client(...)` | `rospy.ServiceProxy(...)` | `node.create_client(...)` |
| TF | `node.create_transform_broadcaster()` | `tf2_ros.TransformBroadcaster()` | `tf2_ros.TransformBroadcaster(node)` |
| 循环 | `spin(node)` | `rospy.spin()` | `rclpy.spin_once()` 循环 |
| 阻塞读取 | `wait_for_message(topic, type, node)` | `rospy.wait_for_message()` | 临时订阅 + spin_once |

### 在其他脚本中使用

```python
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_compat import CompatNode, spin, wait_for_message, logerr_throttle

node = CompatNode('my_node')
node.create_publisher(Twist, 'cmd_vel', queue_size=10)
node.create_subscriber(Odometry, '/odom', my_callback)
node.loginfo("Hello from ROS%d", 1 if not _ROS2 else 2)
spin(node)
```

### `ros_compat.py` 在项目中的位置

两个包各自包含一份 `ros_compat.py`，内容完全相同：

- `src/Robot-Planner/scripts/ros_compat.py` — 供 planner / controller / gazebo_to_tf 导入
- `src/random_map_generator/src/ros_compat.py` — 供 spawn_obstacles 导入（同时也会 fallback 到 Robot-Planner 目录）

---

## 核心算法说明

### RRT* 全局路径规划（planner.py）

| 特性 | 实现 |
|------|------|
| 采样策略 | 自适应：前40次起点附近 + 40-100次目标附近 + 25%目标偏向 |
| 步长 | 0.8m 固定步长 |
| 碰撞检测 | Bresenham 直线光栅化（障碍物膨胀 5 cells） |
| 最近邻搜索 | 线性遍历 O(N) |
| 重连（Rewiring） | 动态搜索半径 `min(2.0, 20*sqrt(log(n)/n))` |
| 路径平滑 | 贪心 LOS 剪枝（最大段长 ≤1.5m） |
| 早停 | 首次找到路径 + 300 次优化迭代 |
| 最大迭代 | 12000 |

### PID 轨迹跟踪控制（controller.py）

| 特性 | 实现 |
|------|------|
| 架构 | 双通道独立 PID（线速度 + 角速度）+ Pure Pursuit 前馈 |
| 控制频率 | 50 Hz |
| 最大线速度 | 0.80 m/s |
| 最大角速度 | 1.2 rad/s |
| 自适应速度 | `线速度 × cos(航向误差)` 动态调节 + 近目标减速 |
| 抗积分饱和 | 积分项钳位 + 路径点切换时复位 |
| 加速度限制 | 线 1.2 m/s²，角 5.0 rad/s² |
| 卡住恢复 | 检测 2 秒无位移 → 后退 → 转向 → 前进 |
| 多目标 | Subscriber 持续监听，新路径自动中断旧路径 |

### PID 参数

| 通道 | Kp | Ki | Kd | 输出范围 |
|------|-----|-----|-----|---------|
| 线速度 | 0.5 | 0.02 | 0.02 | [0, 0.80] m/s |
| 角速度 | 0.35 | 0.005 | 0.03 | [-1.2, 1.2] rad/s |

---

## 当前进度

- [x] 项目框架搭建
- [x] `planner.py` — RRT* 规划器完整实现（Bresenham 碰撞检测 / 障碍物膨胀 / RRT* 核心 / LOS 路径平滑 / 双格式路径发布）
- [x] `controller.py` — PID 控制器完整实现（双通道 PID / 航向优先 / 加速度限制 / 卡住检测恢复 / 多目标跟踪）
- [x] `spawn_obstacles.py` — 随机障碍物生成（Gazebo + PGM 地图同步）
- [x] 仿真启动配置调优（时序同步、延迟控制）
- [x] ROS1/ROS2 双版本兼容（`ros_compat.py` + 双 build system + 双 launch 文件）
- [ ] PID 参数精细整定（根据实际跟踪效果调优）
- [ ] 多场景测试数据收集（规划时间 / 路径长度 / 跟踪偏差 / 速度平滑度）
- [ ] 实验报告撰写

---

## 评分标准

| 指标 | 分值 | 说明 |
|------|------|------|
| 路径规划时间 | 20 | RRT* 搜索耗时，越短越高 |
| 路径长度 | 19 | 规划路径总长，越短越高 |
| 轨迹跟踪精度 | 18 | 实际轨迹与规划路径偏差，越小越高 |
| 速度平滑度 | 18 | 速度变化平稳性，越平越高 |
| 实验报告 | 25 | 完整性、逻辑性、分析深度 |
| **合计** | **100** | |

---

## 调参指南

**planner.py**

| 参数 | 位置 | 说明 |
|------|------|------|
| `step_size` | `plan_path()` | 扩展步长，大=搜索快但可能跳过窄通道 |
| `goal_sample_rate` | `plan_path()` | 目标偏向概率，大=更快收敛但可能陷入局部 |
| `max_seg_len` | `_smooth_path()` | 平滑段长上限，小=路径更密更安全 |
| `inflation_cells` | `MapProcessor` | 障碍物膨胀半径，大=更安全但可能堵死通道 |

**controller.py**

| 参数 | 位置 | 说明 |
|------|------|------|
| `linear_pid` kp/ki/kd | `Controller.__init__()` | 线速度 PID，kp 大=更快但可能超调 |
| `angular_pid` kp/ki/kd | `Controller.__init__()` | 角速度 PID，kd 大=更平滑但响应慢 |
| `max_lin_accel` | `Controller.run()` | 加速度限制，大=更快但更抖 |

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| `catkin_make` 报 `non-catkin packages` | package.xml `<build_type>` 是 `ament_cmake`，catkin_make 不支持 | 改用 `catkin build`（需 `python3-catkin-tools`） |
| `catkin build` 报 `catkin: command not found` | 未安装 `catkin_tools` | `sudo apt install python3-catkin-tools` |
| `roslaunch` 报 `is neither a launch file...` | 忘记 `source devel/setup.bash` | 确保同时 source 两个文件：`source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash` |
| `roslaunch` 报 `command not found` | 忘记 `source /opt/ros/noetic/setup.bash` | `source /opt/ros/noetic/setup.bash` |
| 报 `TURTLEBOT3_MODEL is not set` | 未设置 TurtleBot3 型号 | `export TURTLEBOT3_MODEL=waffle` |
| 报 `turtlebot3_description` 找不到 | TurtleBot3 仿真包未安装 | `sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-gazebo` |
| `colcon build` 报 `find_package(catkin)` 失败 | colcon 尝试编译 `src/` 下的 turtlebot3 ROS1 源码包 | 每个子包已放置 `COLCON_IGNORE`，colcon 自动跳过。确认：`find src -name COLCON_IGNORE` |
| `colcon build` 报 `can't find .../maps/` | `maps/` 目录在源码中不存在（运行时动态生成） | `mkdir -p src/Robot-Planner/maps && touch src/Robot-Planner/maps/.gitkeep` |
| 启动时报 `cannot launch node of type [map_server/map_server]` | `ros-noetic-map-server` 未安装 | `sudo apt install ros-noetic-map-server` |
| 启动后有 Gazebo/RViz 界面，但机器人不动 | map_server 未运行或规划器未收到地图，导致 `/path` 无数据 | 确认 `map_server` 已安装；终端查看是否有 `Map loaded` 日志；用 `rostopic echo /path` 检查路径 |
| RViz 显示 `Unknown frame map` | map_server 未就绪 | 等待终端出现 `Map loaded...` 后刷新 |
| 机器人卡在障碍物旁 | 膨胀距离不够或物理碰撞 | 增大 `inflation_cells` 至 7 |
| 点击目标点后无反应 | 目标点可能位于障碍物内或不可达 | 终端查看是否有 `Goal is inside obstacle` 日志，换一个无障碍区域点击 |
| Gazebo 闪退 | 显存不足或进程残留 | `killall gzserver gzclient` 后重试 |
| Docker 构建时 `connection reset by peer` | Docker Hub 在国内被墙 | 配置镜像加速器（见上方 Docker 章节） |

---

## 参考

- 2026年度《机器人导论》期末课程设计与自主导航综合实践规范详细指南
- ROS Noetic 官方文档: https://wiki.ros.org/noetic
- ROS2 Humble 官方文档: https://docs.ros.org/en/humble/
- TurtleBot3 仿真: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
