# ==================== 阶段1: 基础环境 (Ubuntu 18.04) ====================
# 使用官方 Ubuntu 18.04 作为基础镜像
FROM ubuntu:18.04

# 设置环境变量，确保非交互模式安装
ENV DEBIAN_FRONTEND=noninteractive

# 更新并安装依赖包
RUN apt-get update && apt-get upgrade -y && \


# 添加 ROS 清华镜像源
RUN sh -c '. /etc/os-release && \
    echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加 ROS 密钥
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# 安装 ROS Melodic 相关包
RUN apt-get update && \
    apt-get install -y ros-melodic-desktop-full && \
    apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# 初始化 rosdep
RUN rosdep init && rosdep update

# 安装常用的 ROS 工具
RUN apt-get install -y ros-melodic-rqt* ros-melodic-navigation ros-melodic-robot-state-publisher

# 创建一个工作目录
WORKDIR /root/catkin_ws

# 设置 ROS 环境变量
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# 安装其他依赖库（例如第三方库，使用 Shell 脚本）
COPY install_libraries.sh /root/install_libraries.sh
RUN chmod +x /root/install_libraries.sh && /root/install_libraries.sh

# 设置工作目录
WORKDIR /root/catkin_ws

# 启动 ROS 工作空间
CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && roscore"]

