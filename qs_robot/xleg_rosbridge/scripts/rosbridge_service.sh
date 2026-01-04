#!/bin/bash
# rosbridge_service.sh
# rosbridge通信服务启动脚本
# 用于开机自动加载并稳定运行rosbridge通信服务

# 设置ROS环境变量（根据实际情况修改）
source /opt/ros/melodic/setup.bash  # 如果是ROS Melodic，否则改为对应版本
source ~/catkin_ws/devel/setup.bash  # 根据实际工作空间路径修改

# 检查roscore是否运行
if ! rostopic list &>/dev/null; then
    echo "ERROR: roscore is not running. Starting roscore..."
    roscore &
    sleep 3
fi

# 检查rosbridge是否已经在运行
if pgrep -f "rosbridge_websocket" > /dev/null; then
    echo "rosbridge_websocket is already running"
    exit 0
fi

# 启动rosbridge
echo "Starting rosbridge_websocket server..."
roslaunch xleg_rosbridge rosbridge.launch

# 保持脚本运行
wait

