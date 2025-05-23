# Multibotnet v4.0.0

[![ROS Version](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-4.0.0-brightgreen.svg)](https://github.com/nanwanuser/multibotnet/releases)

## 🎉 v4.0.0 重大更新

### 🚀 核心特性

1. **动态消息/服务类型支持**
   - 无需修改代码即可支持任意ROS消息和服务类型
   - 基于ROS反射机制的运行时类型识别
   - 配置文件驱动，灵活扩展

2. **高性能优化**
   - **消息压缩**: 支持LZ4、ZLIB等多种压缩算法，自动选择最优方案
   - **批处理机制**: 智能消息批处理，大幅提升吞吐量
   - **连接池管理**: 复用连接，减少建立连接开销
   - **多线程处理**: 充分利用多核CPU，并行处理消息

3. **企业级特性**
   - **负载均衡**: 支持轮询、最少负载、随机、加权等策略
   - **健康检查**: 自动检测和恢复失效连接
   - **智能重试**: 可配置的重试策略，提高可靠性
   - **性能监控**: 实时统计和性能分析

4. **模块化架构**
   - 清晰的分层设计：核心层、传输层、管理层、工具层
   - 易于维护和扩展
   - 完善的异常处理和日志系统

## 📋 系统要求

- **ROS版本**: Kinetic、Melodic、Noetic
- **编译器**: C++14或更高
- **依赖库**:
  - ZeroMQ 3.x 或 4.x
  - yaml-cpp
  - LZ4 (可选，用于高速压缩)
  - zlib (可选，用于高压缩率)

## 🛠️ 安装

### 1. 安装依赖

```bash
# 必需依赖
sudo apt-get install libzmq3-dev libyaml-cpp-dev

# 可选依赖（推荐安装以获得完整功能）
sudo apt-get install liblz4-dev zlib1g-dev

# ROS依赖
sudo apt-get install ros-$ROS_DISTRO-topic-tools
```

### 2. 克隆仓库

```bash
cd ~/catkin_ws/src
git clone https://github.com/SWUST-ICAA/Multibotnet.git
```

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make
# 或使用 catkin build
catkin build multibotnet
```

## 🚀 快速开始

### 1. 基础配置

创建配置文件 `config/my_robot.yaml`:

```yaml
# IP映射
IP:
  self: '*'
  robot_peer: '192.168.1.100'

# 发送本机的里程计数据
send_topics:
  - topic: /odom
    message_type: nav_msgs/Odometry
    max_frequency: 30
    bind_address: self
    port: 3001
    compression: true    # 自动压缩

# 接收对方的里程计数据
recv_topics:
  - topic: /robot_peer/odom
    message_type: nav_msgs/Odometry
    connect_address: robot_peer
    port: 3001
```

### 2. 启动节点

```bash
roslaunch multibotnet multibotnet.launch config_file:=config/my_robot.yaml
```

### 3. 高级用法

#### 使用自定义消息类型

无需修改代码，直接在配置文件中指定：

```yaml
send_topics:
  - topic: /my_custom_data
    message_type: my_package/MyCustomMsg
    max_frequency: 10
    bind_address: self
    port: 3002
```

#### 启用性能监控

```bash
roslaunch multibotnet multibotnet.launch \
  config_file:=config/my_robot.yaml \
  print_statistics:=true \
  statistics_interval:=5.0
```

## 📊 性能优化指南

### 1. 压缩策略

根据数据特点选择合适的压缩算法：

```yaml
advanced:
  compression:
    type: lz4    # 实时性要求高的场景
    # type: zlib  # 带宽受限，压缩率优先
    # type: none  # 低延迟要求，不压缩
```

### 2. 批处理优化

适合小消息高频发送场景：

```yaml
send_topics:
  - topic: /sensor_data
    message_type: std_msgs/Float32
    max_frequency: 100
    bind_address: self
    port: 3003
    batch: true
    batch_size: 20        # 每批20条消息
    batch_timeout_ms: 50  # 或50ms超时
```

### 3. 连接池配置

减少连接建立开销：

```yaml
advanced:
  connection_pool:
    min_connections: 2    # 保持最少2个连接
    max_connections: 20   # 最多20个连接
    idle_timeout_ms: 300000  # 5分钟空闲超时
```

## 🔧 高级特性

### 负载均衡

配置多个目标节点，自动分配负载：

```yaml
# 在服务请求中使用负载均衡
load_balancing:
  enable: true
  strategy: least_loaded
  targets:
    - address: server1
      weight: 2    # 性能好的服务器权重更高
    - address: server2
      weight: 1
```

### 服务容错

自动重试和故障转移：

```yaml
request_services:
  - service_name: /critical_service
    service_type: std_srvs/Trigger
    connect_address: primary_server
    port: 5001
    timeout_ms: 3000
    max_retries: 5        # 最多重试5次
```

## 📈 性能基准

在典型配置下的性能数据：

| 场景 | 消息大小 | 频率 | 延迟 | 吞吐量 |
|------|---------|------|------|---------|
| IMU数据 | 1KB | 100Hz | <1ms | 100KB/s |
| 激光雷达 | 100KB | 10Hz | <5ms | 1MB/s |
| 点云 | 10MB | 1Hz | <50ms | 10MB/s |
| 批处理小消息 | 100B | 1000Hz | <2ms | 100KB/s |

*测试环境：千兆以太网，Intel i7 CPU*

## 🛡️ 故障排除

### 常见问题

1. **连接失败**
   - 检查防火墙设置
   - 确认IP地址和端口配置正确
   - 使用 `netstat -an | grep <port>` 检查端口占用

2. **性能问题**
   - 启用统计信息查看瓶颈
   - 调整批处理和压缩参数
   - 检查网络带宽限制

3. **消息丢失**
   - 增加ZMQ缓冲区大小
   - 降低发送频率
   - 检查网络稳定性

### 调试工具

```bash
# 查看话题流量
rostopic bw /topic_name

# 监控节点状态
rosnode info /multibotnet_topic_node

# 查看详细日志
export ROSCONSOLE_CONFIG_FILE=`rospack find multibotnet`/config/rosconsole_debug.conf
```

## 🤝 贡献指南

欢迎贡献代码、报告问题或提出建议！

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Pull Request

## 📄 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🙏 致谢

- ROS社区提供的优秀框架
- ZeroMQ项目提供的高性能消息库
- 所有贡献者和用户的支持

## 📞 联系方式

- 项目主页: https://github.com/SWUST-ICAA/Multibotnet
- 问题反馈: https://github.com/SWUST-ICAA/Multibotnet/issues
- 邮箱: nanwan2004@126.com

---

**Multibotnet** - 让多机器人协作更简单、更高效！