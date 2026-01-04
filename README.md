# ROS Docker 自动化构建与部署方案 (Ubuntu 18.04 + ROS Melodic)

## 📋 方案概述

本方案针对 **Ubuntu 18.04** 系统设计，使用 **ROS Melodic**。包含完整的 Docker 安装配置、镜像源优化、以及 ROS 环境的自动化构建。

robot_qs/
├── Dockerfile                    # 主构建文件
├── docker-compose.yml           # 多服务编排
├── install_libraries.sh                     # 第三方文件
├── run.sh                       # 一键构建并启动docker脚本
├──qs_robot                         # ROS工程
        └── / src

---

## 第一部分：Ubuntu 18.04 系统准备

### 1.1 安装 Docker 服务

```bash
# 更新系统包列表
sudo apt update

# 安装 Docker 社区版
sudo apt install -y docker.io

# 配置 iptables（解决 Docker 网络问题）
sudo update-alternatives --set iptables /usr/sbin/iptables-legacy
sudo update-alternatives --set ip6tables /usr/sbin/ip6tables-legacy

# 验证安装
docker --version
# 预期输出：Docker version 20.10.x 或更高

# 添加当前用户到 docker 组（避免每次使用 sudo）
sudo usermod -aG docker $USER
# 重要：需要重新登录或执行以下命令使组权限生效
newgrp docker
```

### 1.2 配置 Docker 镜像加速器

#### 创建 Docker 配置文件

sudo tee /etc/docker/daemon.json << 'EOF'
{
  "registry-mirrors": [
    "https://h6lej6c4.mirror.aliyuncs.com",
    "https://docker.mirrors.ustc.edu.cn",
    "https://hub-mirror.c.163.com",
    "https://mirror.baidubce.com",
    "https://docker.nju.edu.cn"
  ],
  "exec-opts": ["native.cgroupdriver=systemd"],
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "100m"
  },
  "storage-driver": "overlay2"
}
EOF

#### 重新加载并重启 Docker

sudo systemctl daemon-reload
sudo systemctl restart docker

#### 验证配置

docker info | grep -A5 "Registry Mirrors"



挂载ROS工程到docker中

```
sudo docker run -it --name modest_keldysh -v /home/radxa/qs_robot:/qs_robot ubuntu:18.04
```

```
# 删除所有编译缓存
cd /qs_robot
rm -rf build devel install
catkin_make
```



## 第二部分：Dockerfile - 自动构建

### 2.1 主 Dockerfile (ROS Melodic)

见Dockerfile.txt文件

### 2. 2 Shell 脚本：instll_libraries.sh

这个脚本将会安装第三方库的源码。你可以根据需求修改其中的库和安装方式。

见install_libraries.txt

##  第三部分：构建和运行Docker镜像

### 3.1构建 Docker 镜像

```
docker build -t ros-melodic-auto-build .
```

### 3.2自动化部署

使用 Docker 容器和 Docker Compose 可以进一步简化自动化部署的流程。你可以创建一个 `docker-compose.yml` 文件来方便地管理和部署多个容器或服务。

见 `docker-compose.txt

### 3.3启动容器

见run.sh(txt)

##  第四部分：编译运行ROS工程

cd /root/catkin_ws

```
rm -rf build devel install
```

catkin_make

source devel/setup.bash

roslaunch xleg_lowersys   xleg_lowersys.launch
