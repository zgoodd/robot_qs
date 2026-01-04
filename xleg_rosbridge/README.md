# Kuavo机器人Rosbridge通信接口使用说明

## 概述

本包提供了Kuavo下位机ROS应用层rosbridge节点应用开发，构建ROS层与Web端的高效通信接口，提升系统之间互操作性和性能。

## 功能特性

1. **WebSocket接口通信**：通过rosbridge_websocket实现ROS与Web端的双向通信
2. **动态话题管理**：支持话题的动态添加、删除和配置调整
3. **开机自启动服务**：通过systemd实现rosbridge服务的自动加载和稳定运行
4. **服务管理优化**：提供完整的服务管理层流程

## 安装依赖

### 1. 安装rosbridge_suite

```bash
sudo apt-get install ros-melodic-rosbridge-suite  # ROS Melodic
# 或
sudo apt-get install ros-kinetic-rosbridge-suite  # ROS Kinetic
```

### 2. 安装rosapi（可选，用于ROS API服务）

```bash
sudo apt-get install ros-melodic-rosapi
```

## 使用方法

### 方法一：手动启动rosbridge

```bash
# 启动roscore（如果未运行）
roscore

# 启动rosbridge
roslaunch xleg_rosbridge rosbridge.launch
```

### 方法二：使用启动脚本

```bash
# 赋予执行权限
chmod +x xleg_rosbridge/scripts/rosbridge_service.sh

# 运行脚本
./xleg_rosbridge/scripts/rosbridge_service.sh
```

### 方法三：使用systemd服务（推荐，实现开机自启动）

#### 1. 配置systemd服务文件

编辑 `xleg_rosbridge/systemd/rosbridge.service` 文件，修改以下内容：
- `YOUR_USERNAME`: 替换为实际用户名
- `YOUR_GROUP`: 替换为实际用户组
- `/home/YOUR_USERNAME/catkin_ws`: 替换为实际工作空间路径
- `melodic`: 如果是其他ROS版本，修改为对应版本（如kinetic、noetic等）

#### 2. 安装systemd服务

```bash
# 复制服务文件到systemd目录
sudo cp xleg_rosbridge/systemd/rosbridge.service /etc/systemd/system/

# 重新加载systemd配置
sudo systemctl daemon-reload

# 启用服务（开机自启动）
sudo systemctl enable rosbridge.service

# 启动服务
sudo systemctl start rosbridge.service

# 查看服务状态
sudo systemctl status rosbridge.service

# 查看服务日志
sudo journalctl -u rosbridge.service -f
```

#### 3. 服务管理命令

```bash
# 启动服务
sudo systemctl start rosbridge.service

# 停止服务
sudo systemctl stop rosbridge.service

# 重启服务
sudo systemctl restart rosbridge.service

# 禁用开机自启动
sudo systemctl disable rosbridge.service

# 查看服务状态
sudo systemctl status rosbridge.service
```

## Web端连接

### WebSocket连接地址

```
ws://localhost:9090
```

如果从远程访问，将`localhost`替换为机器人IP地址：
```
ws://192.168.1.100:9090
```

### 使用roslibjs连接示例

```javascript
// 创建ROS连接
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

// 连接成功回调
ros.on('connection', function() {
    console.log('Connected to rosbridge');
});

// 连接错误回调
ros.on('error', function(error) {
    console.log('Error connecting to rosbridge:', error);
});

// 订阅话题
var footForceListener = new ROSLIB.Topic({
    ros: ros,
    name: '/xleg/footForce',
    messageType: 'xleg_msgs/FootsForce'
});

footForceListener.subscribe(function(message) {
    console.log('Foot Force:', message);
});

// 调用服务
var servoClient = new ROSLIB.Service({
    ros: ros,
    name: '/xleg/servoControl',
    serviceType: 'xleg_msgs/servoControl'
});

var request = new ROSLIB.ServiceRequest({
    servoCmd: 1,  // TORQUE_ON
    allServos: true,
    servoID: 0
});

servoClient.callService(request, function(result) {
    console.log('Service call result:', result);
});
```

## 动态话题管理

### 使用动态重配置

```bash
# 启动动态重配置GUI
rosrun rqt_reconfigure rqt_reconfigure

# 选择 rosbridge_topic_manager 节点
# 可以动态调整以下参数：
# - enable_footforce: 启用/禁用footForce话题
# - enable_jointstate: 启用/禁用jointState话题
# - enable_bodystate: 启用/禁用bodyState话题
# - enable_cameraimage: 启用/禁用cameraImage话题
# - publish_rate: 发布频率（Hz）
# - max_connections: 最大WebSocket连接数
```

### 启动话题管理器节点

```bash
rosrun xleg_rosbridge rosbridge_topic_manager
```

## 可用的话题和服务

### Topics（话题）

- `/xleg/footForce` - 足底力信息 (xleg_msgs/FootsForce)
- `/xleg/jointState` - 关节状态信息 (xleg_msgs/JointsState)
- `/xleg/bodyState` - 机身状态信息 (xleg_msgs/BodyState)
- `/cameraImage` - 相机图像 (sensor_msgs/Image)

### Services（服务）

- `/xleg/servoControl` - 舵机控制服务 (xleg_msgs/servoControl)
- `/xleg/bodyControl` - 机身控制服务 (xleg_msgs/bodyControl)
- `/xleg/gaitControl` - 步态控制服务 (xleg_msgs/gaitControl)
- `/xleg/motorCode` - 电机控制服务（测试用）(xleg_msgs/motorCode)
- `/xleg/dataSave` - 数据保存服务 (xleg_msgs/dataSave)

## 性能优化建议

1. **调整发布频率**：根据实际需求调整话题发布频率，避免过度占用带宽
2. **限制连接数**：通过`max_connections`参数限制同时连接的Web客户端数量
3. **选择性发布**：使用动态重配置只启用需要的话题，减少不必要的数据传输
4. **网络优化**：在局域网环境下使用，避免跨网络的高延迟

## 故障排查

### 问题1：rosbridge无法启动

**解决方案**：
- 检查roscore是否运行：`rostopic list`
- 检查端口9090是否被占用：`netstat -tuln | grep 9090`
- 查看日志：`rosrun rosbridge_server rosbridge_websocket`

### 问题2：Web端无法连接

**解决方案**：
- 检查防火墙设置：`sudo ufw allow 9090`
- 检查IP地址是否正确
- 检查rosbridge服务是否运行：`sudo systemctl status rosbridge.service`

### 问题3：话题数据无法接收

**解决方案**：
- 检查话题是否存在：`rostopic list`
- 检查话题是否有数据：`rostopic echo /xleg/footForce`
- 检查动态重配置是否启用了对应话题

## 安全注意事项

1. **生产环境建议启用认证**：修改`rosbridge.launch`中的`authenticate`参数为`true`
2. **使用SSL/TLS**：在生产环境中启用SSL加密
3. **防火墙配置**：限制9090端口的访问范围
4. **访问控制**：使用反向代理（如nginx）实现访问控制

## 参考资料

- [rosbridge_suite官方文档](http://wiki.ros.org/rosbridge_suite)
- [roslibjs文档](https://github.com/RobotWebTools/roslibjs)
- [WebSocket协议](https://tools.ietf.org/html/rfc6455)

