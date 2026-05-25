# 机器人导论期末大作业 — 自主导航综合实践

## 项目简介

本项目是 2026 年度《机器人导论》课程的期末课程设计，基于 **ROS Noetic + Gazebo** 仿真环境，实现移动机器人的**全局路径规划**与**轨迹跟踪控制**两大核心模块。

系统工作流程：在 Gazebo 加载含随机障碍物的仿真环境 → 占据栅格地图构建 → RRT* 算法进行无碰全局路径规划 → PID 双通道控制器驱动机器人沿路径移动并到达目标。

## 项目结构

```
final_project/                         # ROS 工作空间
├── src/
│   ├── Robot-Planner/                 # ★ 核心功能包（包名: turtle）
│   │   ├── scripts/
│   │   │   ├── planner.py             # RRT* 全局路径规划器
│   │   │   ├── controller.py          # PID 轨迹跟踪控制器
│   │   │   ├── gazebo_to_tf.py        # TF 坐标变换广播
│   │   │   └── utils.py               # 共享工具函数
│   │   ├── launch/obs_world.launch    # 主启动文件
│   │   └── maps/                      # 占据栅格地图（运行时动态生成）
│   ├── random_map_generator/          # ★ 随机障碍物 & 地图生成器
│   │   └── src/spawn_obstacles.py
│   ├── turtlebot3/                    # TurtleBot3 机器人模型
│   ├── turtlebot3_msgs/               # TurtleBot3 自定义消息
│   └── turtlebot3_simulations/        # TurtleBot3 Gazebo 仿真
```

## 环境要求

| 组件 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 20.04 LTS (WSL2 / 双系统) |
| ROS | Noetic (完整桌面版) |
| Python | 3.8+ |
| Gazebo | 11.x |
| 机器人模型 | TurtleBot3 Waffle |

### 安装依赖

```bash
# ROS Noetic 完整安装
sudo apt install ros-noetic-desktop-full

# TurtleBot3 仿真包
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-map-server ros-noetic-navigation ros-noetic-rviz
sudo apt install python3-pip python-is-python3
pip3 install numpy pillow pyyaml
```

### 编译与运行

```bash
# 设置环境变量
export TURTLEBOT3_MODEL=waffle
source /opt/ros/noetic/setup.bash

# 编译
cd final_project
catkin_make
source devel/setup.bash

# 启动仿真
roslaunch turtle obs_world.launch
```

启动后：
1. Gazebo 弹出（3D 仿真世界 + 随机障碍物）
2. 等待 ~60 秒地图加载完成
3. RViz 弹出，Fixed Frame 选择 `map`
4. 点击工具栏 **"2D Nav Goal"** 在地图上设定目标点
5. RRT* 自动规划路径，机器人沿路径移动至目标

## 核心算法说明

### RRT* 全局路径规划（planner.py）

| 特性 | 实现 |
|------|------|
| 采样策略 | 前50次起点附近密集采样 + 15%目标偏向 |
| 步长 | 0.8m 固定步长 |
| 碰撞检测 | Bresenham 直线光栅化（障碍物膨胀 0.5m） |
| 最近邻搜索 | 线性遍历 O(N) |
| 重连（Rewiring） | 动态搜索半径 `min(1.6, 15*sqrt(log(n)/n))` |
| 路径平滑 | 贪心 LOS 剪枝（最大段长 ≤1.5m） |
| 早停 | 首次找到路径 + 500 次优化迭代 |
| 最大迭代 | 5000 |

### PID 轨迹跟踪控制（controller.py）

| 特性 | 实现 |
|------|------|
| 架构 | 双通道独立 PID（线速度 + 角速度） |
| 控制频率 | 50 Hz |
| 最大线速度 | 0.60 m/s |
| 最大角速度 | 2.0 rad/s |
| 自适应速度 | `线速度 × cos(航向误差)` 动态调节 |
| 抗积分饱和 | 积分项钳位 + 路径点切换时复位 |
| 加速度限制 | 线 0.8 m/s²，角 3.0 rad/s² |
| 卡住恢复 | 检测 2 秒无位移 → 后退 → 旋转 → 重试 |
| 多目标 | Subscriber 持续监听，新路径自动中断旧路径 |

### PID 参数

| 通道 | Kp | Ki | Kd | 输出范围 |
|------|-----|-----|-----|---------|
| 线速度 | 0.6 | 0.01 | 0.02 | [0, 0.60] m/s |
| 角速度 | 1.2 | 0.02 | 0.04 | [-2.0, 2.0] rad/s |

## 当前进度

- [x] 项目框架搭建（ROS workspace + 包结构）
- [x] `planner.py` — RRT* 规划器完整实现
  - [x] Bresenham 碰撞检测
  - [x] 障碍物膨胀（5格 = 0.5m安全距离）
  - [x] RRT* 核心算法（采样/扩展/重连/早停）
  - [x] LOS 贪心路径平滑
  - [x] 双格式路径发布（Path + Float32MultiArray）
- [x] `controller.py` — PID 控制器完整实现
  - [x] 双通道独立 PID
  - [x] 航向优先 + 自适应速度
  - [x] 加速度限制
  - [x] 卡住检测与自动恢复
  - [x] 多目标连续跟踪（Subscriber 模式）
- [x] `spawn_obstacles.py` — 随机障碍物生成（Gazebo + PGM 地图同步）
- [x] 仿真启动配置调优（时序同步、延迟控制）
- [x] 编译通过，仿真正常运行
- [ ] PID 参数精细整定（根据实际跟踪效果调优）
- [ ] 多场景测试数据收集（规划时间/路径长度/跟踪偏差/速度平滑度）
- [ ] 实验报告撰写

## 评分标准

| 指标 | 分值 | 说明 |
|------|------|------|
| 路径规划时间 | 20 | RRT* 搜索耗时，越短越高 |
| 路径长度 | 19 | 规划路径总长，越短越高 |
| 轨迹跟踪精度 | 18 | 实际轨迹与规划路径偏差，越小越高 |
| 速度平滑度 | 18 | 速度变化平稳性，越平越高 |
| 实验报告 | 25 | 完整性、逻辑性、分析深度 |
| **合计** | **100** | |

## 调参指南

如果运行效果不理想，主要调整以下参数：

**planner.py**
- `step_size`（L208）：扩展步长，大=快但可能跳过窄通道
- `goal_sample_rate`（L207）：目标偏向概率，大=更快找到路径但探索不足
- `max_seg_len`（L174）：平滑段长上限，小=路径更密更安全

**controller.py**
- `linear_pid` kp/ki/kd（L57-58）：线速度 PID，kp 大=更快但可能超调
- `angular_pid` kp/ki/kd（L59-60）：角速度 PID，kd 大=更平滑但响应慢
- `max_lin_accel`（L95）：加速度限制，大=更快但更抖

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| RViz 显示 `Unknown frame map` | map_server 未就绪 | 等待终端出现 `Map loaded...` 后刷新 |
| 机器人不动 | controller 未收到路径 | 检查 `/path` 话题 `rostopic echo /path` |
| 机器人卡在障碍物旁 | 膨胀距离不够或物理碰撞 | 增大 `inflation_cells` 至 7 |
| Gazebo 闪退 | 显存不足或进程残留 | `killall gzserver gzclient` 后重试 |

## 参考

- 2026年度《机器人导论》期末课程设计与自主导航综合实践规范详细指南
- ROS Noetic 官方文档: https://wiki.ros.org/noetic
- TurtleBot3 仿真: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
