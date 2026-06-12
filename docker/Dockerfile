FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ========== 替换为 USTC 镜像源（国内加速） ==========
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's@//.*security.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list

# ========== 安装基础工具 ==========
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    wget \
    vim \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ========== 添加 ROS Noetic 安装源（USTC 镜像） ==========
RUN sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# ========== 安装 ROS Noetic Desktop Full（含 Gazebo 仿真） ==========
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# ========== 安装构建工具与运行时依赖 ==========
# catkin-tools: 替代 catkin_make 的构建工具
# map_server: 运行时提供占据栅格地图
# xacro: 解析 TurtleBot3 URDF 模型
# turtlebot3-*: TurtleBot3 仿真（apt 安装，避免源码嵌套目录问题）
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-map-server \
    ros-noetic-xacro \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    ros-noetic-navigation \
    ros-noetic-tf2-ros \
    python3-numpy \
    python3-pillow \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# ========== 初始化 rosdep ==========
# rosdep 在国内可能因 GitHub 访问超时失败，添加重试和容错
RUN rosdep init 2>/dev/null || true; \
    for i in 1 2 3; do \
        echo "rosdep update attempt $i/3..."; \
        timeout 120 rosdep update 2>&1 && break || \
        echo "Attempt $i failed, retrying..."; \
        sleep 3; \
    done; \
    echo "rosdep setup done (errors above are non-fatal if packages already installed)"

# ========== 创建普通用户（避免 root 运行图形界面） ==========
ARG USER=rosuser
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USER} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER}

# ========== 设置 ROS 环境变量 ==========
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USER}/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# ========== 创建工作空间目录 ==========
RUN mkdir -p /home/${USER}/catkin_ws/src && chown -R ${USER}:${USER} /home/${USER}/catkin_ws

USER ${USER}
WORKDIR /home/${USER}

# ========== 容器启动时加载 ROS 环境 ==========
CMD ["/bin/bash"]
