# Kuavo机器人Rosbridge实现总结

## 实现概述

本次实现完成了Kuavo下位机ROS应用层rosbridge节点应用开发，构建了ROS层与Web端的高效通信接口，提升了系统之间互操作性和性能。

所有rosbridge相关功能已独立到 `xleg_rosbridge` 包中。

## 实现内容

### 1. Rosbridge WebSocket通信接口 ✅

**文件**: `xleg_rosbridge/launch/rosbridge.launch`

- 实现了基于WebSocket的rosbridge通信接口
- 默认端口：9090
- 支持Topics和Services的完整访问
- 可配置认证和SSL加密

**使用方法**:
```bash
roslaunch xleg_rosbridge rosbridge.launch
```

### 2. 动态话题发布节点 ✅

**文件**: 
- `xleg_rosbridge/src/rosbridge_topic_manager.cpp` - 话题管理器实现
- `xleg_rosbridge/cfg/RosbridgeTopic.cfg` - 动态重配置文件

**功能特性**:
- 支持话题的动态添加、删除和配置调整
- 通过dynamic_reconfigure实现运行时配置
- 支持以下话题的动态管理：
  - `/xleg/footForce` - 足底力信息
  - `/xleg/jointState` - 关节状态信息
  - `/xleg/bodyState` - 机身状态信息
  - `/cameraImage` - 相机图像

**使用方法**:
```bash
# 启动话题管理器
rosrun xleg_rosbridge rosbridge_topic_manager

# 使用动态重配置GUI调整参数
rosrun rqt_reconfigure rqt_reconfigure
```

### 3. 开机自启动服务 ✅

**文件**: 
- `xleg_rosbridge/systemd/rosbridge.service` - systemd服务文件
- `xleg_rosbridge/scripts/rosbridge_service.sh` - 启动脚本

**功能特性**:
- 通过systemd实现开机自动加载
- 自动重启机制，确保服务稳定运行
- 完整的日志记录功能

**安装步骤**:
1. 编辑`rosbridge.service`文件，修改用户名、路径等信息
2. 复制到systemd目录：`sudo cp xleg_rosbridge/systemd/rosbridge.service /etc/systemd/system/`
3. 启用服务：`sudo systemctl enable rosbridge.service`
4. 启动服务：`sudo systemctl start rosbridge.service`

### 4. 完整启动配置 ✅

**文件**: `xleg_rosbridge/launch/rosbridge_complete.launch`

- 包含rosbridge_websocket和话题管理器节点
- 一键启动所有rosbridge相关服务

## 包结构

```
xleg_rosbridge/
├── launch/
│   ├── rosbridge.launch              # 基础rosbridge启动文件
│   └── rosbridge_complete.launch     # 完整rosbridge启动文件（包含话题管理器）
├── src/
│   └── rosbridge_topic_manager.cpp   # 话题管理器节点实现
├── cfg/
│   └── RosbridgeTopic.cfg            # 动态重配置文件
├── scripts/
│   └── rosbridge_service.sh          # 启动脚本
├── systemd/
│   └── rosbridge.service             # systemd服务文件
├── CMakeLists.txt                    # 构建配置
├── package.xml                       # 包配置
├── README.md                         # 详细使用说明
└── IMPLEMENTATION_SUMMARY.md         # 本文件
```

## 当前通信架构

### 下位机 (xleg_lowersys)
- **发布Topics**:
  - `xleg/footForce` - 足底力传感器数据
  - `xleg/jointState` - 12个关节的状态信息
  - `xleg/bodyState` - 机身姿态（roll, pitch, yaw）
  - `/cameraImage` - 相机图像流

- **提供Services**:
  - `xleg/servoControl` - 舵机控制（启停、错误清除等）
  - `xleg/bodyControl` - 机身姿态控制
  - `xleg/gaitControl` - 步态控制（前进、转向、越障等）
  - `xleg/motorCode` - 电机控制（测试用）
  - `xleg/dataSave` - 数据保存服务

### 上位机 (xleg_controller)
- Qt GUI应用程序
- 订阅Topics显示机器人状态
- 调用Services控制机器人运动

### Rosbridge通信层 (xleg_rosbridge) - 新增
- WebSocket接口：`ws://localhost:9090`
- Web端可通过roslibjs等库访问所有Topics和Services
- 支持多客户端同时连接

## Web端连接示例

### JavaScript连接代码

```javascript
// 1. 创建ROS连接
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // 或机器人IP地址
});

// 2. 订阅话题
var footForceListener = new ROSLIB.Topic({
    ros: ros,
    name: '/xleg/footForce',
    messageType: 'xleg_msgs/FootsForce'
});

footForceListener.subscribe(function(message) {
    console.log('Foot Force:', message);
    // 处理数据...
});

// 3. 调用服务
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

## 编译和部署

### 1. 安装依赖

```bash
sudo apt-get install ros-melodic-rosbridge-suite
sudo apt-get install ros-melodic-rosapi
sudo apt-get install ros-melodic-dynamic-reconfigure
```

### 2. 编译项目

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 测试运行

```bash
# 终端1：启动roscore
roscore

# 终端2：启动下位机节点
roslaunch xleg_lowersys xleg_lowersys.launch

# 终端3：启动rosbridge
roslaunch xleg_rosbridge rosbridge.launch
```

### 4. 验证连接

在浏览器中打开Web页面，使用roslibjs连接`ws://localhost:9090`，测试Topics和Services的访问。

## 性能优化建议

1. **话题发布频率**：根据实际需求调整，避免过度占用带宽
2. **连接数限制**：通过配置限制同时连接的Web客户端数量
3. **选择性发布**：只启用需要的话题，减少不必要的数据传输
4. **网络环境**：建议在局域网环境下使用，避免跨网络的高延迟

## 安全注意事项

1. **生产环境启用认证**：修改launch文件中的`authenticate`参数
2. **使用SSL/TLS**：在生产环境中启用SSL加密
3. **防火墙配置**：限制9090端口的访问范围
4. **访问控制**：使用反向代理实现更细粒度的访问控制

## 故障排查

### 常见问题

1. **rosbridge无法启动**
   - 检查roscore是否运行
   - 检查端口9090是否被占用
   - 查看日志：`rosrun rosbridge_server rosbridge_websocket`

2. **Web端无法连接**
   - 检查防火墙设置
   - 检查IP地址是否正确
   - 检查rosbridge服务状态

3. **话题数据无法接收**
   - 检查话题是否存在：`rostopic list`
   - 检查话题是否有数据：`rostopic echo /xleg/footForce`
   - 检查动态重配置是否启用了对应话题

## 下一步改进建议

1. **添加认证机制**：实现用户认证和权限管理
2. **性能监控**：添加通信性能监控和统计功能
3. **消息压缩**：对大数据量话题（如图像）实现压缩传输
4. **Web界面**：开发完整的Web控制界面
5. **数据记录**：添加Web端数据记录和回放功能

## 参考资料

- [rosbridge_suite官方文档](http://wiki.ros.org/rosbridge_suite)
- [roslibjs文档](https://github.com/RobotWebTools/roslibjs)
- [dynamic_reconfigure文档](http://wiki.ros.org/dynamic_reconfigure)
- [systemd服务管理](https://www.freedesktop.org/software/systemd/man/systemd.service.html)

