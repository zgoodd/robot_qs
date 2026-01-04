#!/bin/bash

# 构建 Docker 镜像
docker build -t ros-melodic-auto-build .

# 使用 docker-compose 启动服务
docker-compose up -d

# 查看运行中的容器
docker ps

