# RK3576 Docker 部署方案

项目已使用 Docker 将现有 ROS1 四足机器人程序部署到 RK3576 并完成运行。本文说明如何使用已有的板端镜像运行仓库代码。仓库历史构建配置以 Ubuntu 18.04 和 ROS Melodic 为基础。

## 1. 准备板卡与镜像

在 RK3576 的 Linux 宿主机准备 Docker，确认电机串口及所需传感器已被宿主机识别。使用已经在 RK3576 验证的 ARM64 镜像。若通过归档迁移镜像，使用 `docker load -i <镜像归档>`，然后用 `docker image ls` 确认实际标签。

仓库中的 Dockerfile、依赖脚本和 Compose 是历史配置。Dockerfile 有未完成的 RUN 续行，Compose 末尾含未注释的说明文字。从零构建和直接运行历史 Compose 尚待修订和板端验证。

## 2. 启动容器

在板卡克隆仓库并进入根目录。下例使用 Linux host 网络和电机设备映射，镜像标签与设备路径按板端实际环境填写。只映射实际启用的外设。

```bash
git clone git@github.com:zgoodd/robot_qs.git
cd robot_qs
IMAGE='<已验证的 ARM64 镜像名称和标签>'
docker run --rm -it --name robot-qs \
  --network host \
  --device /dev/dxl_port:/dev/dxl_port \
  --mount "type=bind,source=$PWD/qs_robot,target=/root/catkin_ws/src" \
  --workdir /root/catkin_ws \
  --entrypoint /bin/bash "$IMAGE"
```

如需足底力或 IMU，在运行参数中加入对应的 `--device /dev/force_port:/dev/force_port` 或 `--device /dev/imu_port:/dev/imu_port`。若宿主机使用其他设备名，将冒号左侧替换为实际路径。独立 IMU launch 使用的设备名也需与映射一致。

## 3. 构建并启动 ROS 节点

以下按仓库 ROS Melodic 配置示例。若镜像使用其他 ROS1 版本，加载该版本的环境。

```bash
source /opt/ros/melodic/setup.bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
roslaunch xleg_lowersys xleg_lowersys.launch
```

Qt 上位机可在具备显示环境且已构建同一工程的控制端运行。跨机使用时，将控制端的 `ROS_MASTER_URI` 指向 `http://<RK3576地址>:11311`，两端分别配置对方能访问的本机 `ROS_IP`，并检查节点能双向连接。

```bash
roslaunch xleg_controller xleg_controller.launch
```

## 4. 检查运行

在已加载 ROS 环境的另一个终端检查节点、话题与服务，再按实机调试步骤验证动作。

```bash
rosnode list
rostopic list
rosservice list
rostopic hz /xleg/jointState
```

[观看板端运行演示](./演示视频.mp4)。文档中的容器命令为按仓库结构整理的复现示例，具体镜像标签和启用设备以实际板端记录为准。
