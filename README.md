# Multibotnet v4.0.0

[![ROS Version](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-4.0.0-brightgreen.svg)](https://github.com/SWUST-ICAA/Multibotnet/releases)

## 🎉 v4.0.0 重大更新

### 🚀 核心特性

1. **灵活的消息和服务支持**
   - **话题通信**: 支持任意ROS消息类型，无需修改代码
   - **服务通信**: 内置常用服务类型，可方便扩展新类型
   - 基于ROS反射机制的运行时类型识别
   - 配置文件驱动，灵活配置

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

## 🙏 致谢

**特别感谢 Anthropic Claude 4 的发布！** 本项目 v4.0.0 版本的重大重构得益于 Claude 4 强大的代码理解和生成能力。Claude 4 在以下方面提供了关键支持：

- 🔧 **架构重构**: 协助设计了更加模块化和可扩展的系统架构
- 📝 **代码优化**: 提供了高质量的C++代码实现和性能优化建议
- 🐛 **问题定位**: 帮助快速定位和解决复杂的技术问题
- 📚 **文档编写**: 协助编写了详细的技术文档和使用说明

Claude 4 的出色表现极大地提升了开发效率，使得 Multibotnet v4.0.0 能够在短时间内完成如此大规模的升级。这充分展示了 AI 辅助编程的巨大潜力！

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

#### 使用任意ROS消息类型（话题通信）

话题通信支持任意ROS消息类型，无需修改代码，直接在配置文件中指定：

```yaml
send_topics:
  - topic: /my_custom_data
    message_type: my_package/MyCustomMsg  # 支持任意消息类型
    max_frequency: 10
    bind_address: self
    port: 3002

recv_topics:
  - topic: /remote_custom_data
    message_type: my_package/MyCustomMsg
    connect_address: robot_peer
    port: 3002
```

#### 使用服务通信

服务通信支持以下内置类型，使用其他服务类型需要按照下文说明添加：

```yaml
# 使用内置服务类型
provide_services:
  - service_name: /set_mode
    service_type: std_srvs/SetBool    # 内置支持
    bind_address: self
    port: 5001

request_services:
  - service_name: /remote/set_mode
    service_type: std_srvs/SetBool
    connect_address: robot_peer
    port: 5001
```

#### 启用性能监控

```bash
roslaunch multibotnet multibotnet.launch \
  config_file:=config/my_robot.yaml \
  print_statistics:=true \
  statistics_interval:=5.0
```

## 📦 内置支持的类型

### 话题消息类型
- **支持任意ROS消息类型**：使用 `topic_tools::ShapeShifter` 动态处理，无需预先定义

### 服务类型（内置支持）
- `std_srvs/SetBool`
- `std_srvs/Trigger`
- `std_srvs/Empty`
- `nav_msgs/GetPlan`
- `nav_msgs/GetMap`

> **注意**：使用其他服务类型需要手动添加支持，请参考下一节。

## 🔧 添加新的服务类型

由于ROS服务的特殊性，新的服务类型需要手动添加支持。以下是详细步骤，以添加 `geometry_msgs/SetPose` 服务为例：

### 步骤 1: 修改 `core/service_factory.hpp`

#### 1.1 添加服务头文件
在文件顶部的include部分添加：

```cpp
// 现有的头文件
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetMap.h>

// 添加新的服务头文件
#include <geometry_msgs/SetPose.h>  // 新增这一行
```

#### 1.2 声明处理函数
在 `ServiceFactory` 类的私有成员部分添加：

```cpp
private:
    // ... 现有的成员变量 ...
    
    // 注册内置服务类型
    void registerBuiltinServices();
    
    // std_srvs服务处理函数
    LocalServiceHandler createSetBoolHandler(const std::string& service_name);
    LocalServiceHandler createTriggerHandler(const std::string& service_name);
    LocalServiceHandler createEmptyHandler(const std::string& service_name);
    
    // nav_msgs服务处理函数
    LocalServiceHandler createGetPlanHandler(const std::string& service_name);
    LocalServiceHandler createGetMapHandler(const std::string& service_name);
    
    // 新增：geometry_msgs服务处理函数
    LocalServiceHandler createSetPoseHandler(const std::string& service_name);  // 新增这一行
};
```

### 步骤 2: 修改 `core/service_factory.cpp`

#### 2.1 在 `registerBuiltinServices()` 中注册新服务

找到 `registerBuiltinServices()` 函数，添加新服务的注册：

```cpp
void ServiceFactory::registerBuiltinServices() {
    // 注册 std_srvs 服务类型
    registerServiceType("std_srvs/SetBool", 
        [this](const std::string& name) { return createSetBoolHandler(name); });
    
    registerServiceType("std_srvs/Trigger",
        [this](const std::string& name) { return createTriggerHandler(name); });
    
    registerServiceType("std_srvs/Empty",
        [this](const std::string& name) { return createEmptyHandler(name); });
    
    // 注册 nav_msgs 服务类型
    registerServiceType("nav_msgs/GetPlan",
        [this](const std::string& name) { return createGetPlanHandler(name); });
    
    registerServiceType("nav_msgs/GetMap",
        [this](const std::string& name) { return createGetMapHandler(name); });
    
    // 新增：注册 geometry_msgs 服务类型
    registerServiceType("geometry_msgs/SetPose",
        [this](const std::string& name) { return createSetPoseHandler(name); });
    
    LOG_INFO("Registered built-in service types");
}
```

#### 2.2 实现处理函数

在文件末尾添加新服务的处理函数实现：

```cpp
// geometry_msgs/SetPose 处理器
ServiceFactory::LocalServiceHandler ServiceFactory::createSetPoseHandler(
    const std::string& service_name) {
    
    return [this, service_name](const std::vector<uint8_t>& req_data,
                               std::vector<uint8_t>& res_data) -> bool {
        try {
            // 反序列化请求
            geometry_msgs::SetPose::Request req;
            if (!deserializeRequest(req_data, req)) {
                LOG_ERROR("Failed to deserialize SetPose request");
                return false;
            }
            
            // 调用本地ROS服务
            ros::ServiceClient client = nh_.serviceClient<geometry_msgs::SetPose>(service_name);
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return false;
            }
            
            geometry_msgs::SetPose srv;
            srv.request = req;
            
            if (client.call(srv)) {
                // 序列化响应
                res_data = serializeResponse(srv.response);
                return true;
            } else {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Exception in SetPose handler: %s", e.what());
            return false;
        }
    };
}
```

#### 2.3 在 `createServiceServer()` 中添加服务类型处理

找到 `createServiceServer()` 函数，在服务类型判断的最后（`else` 语句之前）添加：

```cpp
    } else if (service_type == "geometry_msgs/SetPose") {
        auto server = nh_.advertiseService(service_name,
            boost::function<bool(geometry_msgs::SetPose::Request&,
                               geometry_msgs::SetPose::Response&)>(
                [remote_handler](geometry_msgs::SetPose::Request& req,
                               geometry_msgs::SetPose::Response& res) -> bool {
                    // 序列化请求
                    auto req_data = serializeRequest(req);
                    
                    // 调用远程服务
                    auto res_data = remote_handler(req_data);
                    if (res_data.empty()) {
                        return false;
                    }
                    
                    // 反序列化响应
                    if (!deserializeResponse(res_data, res)) {
                        return false;
                    }
                    
                    return true;
                }
            )
        );
        servers_[service_name] = server;
        return server;
        
    } else {
        LOG_WARNF("Unsupported service type for server: %s", service_type.c_str());
        return ros::ServiceServer();
    }
```

### 步骤 3: 更新 CMakeLists.txt（如果需要）

如果新服务类型来自外部包，需要在 `CMakeLists.txt` 中添加依赖：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs  # 通常已经包含
  sensor_msgs
  nav_msgs
  std_srvs
  topic_tools
  roslib
  # your_package  # 如果服务来自自定义包，添加这一行
)
```

### 步骤 4: 重新编译

```bash
cd ~/catkin_ws
catkin_make
```

### 步骤 5: 在配置文件中使用新服务

现在可以在配置文件中使用新添加的服务类型：

```yaml
# 提供 geometry_msgs/SetPose 服务
provide_services:
  - service_name: /robot/set_pose
    service_type: geometry_msgs/SetPose
    bind_address: self
    port: 5010

# 请求远程的 geometry_msgs/SetPose 服务
request_services:
  - service_name: /remote_robot/set_pose
    service_type: geometry_msgs/SetPose
    connect_address: robot_peer
    port: 5010
    timeout_ms: 5000
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

4. **批处理错误**
   - 确保发送和接收端的配置一致
   - 检查消息大小是否适合批处理
   - 查看日志中的错误信息

5. **服务类型不支持**
   - 检查服务类型是否在内置列表中
   - 按照"添加新的服务类型"章节添加支持
   - 确保正确编译和配置

### 调试工具

```bash
# 查看话题流量
rostopic bw /topic_name

# 监控节点状态
rosnode info /multibotnet_topic_node

# 查看详细日志
export ROSCONSOLE_CONFIG_FILE=`rospack find multibotnet`/config/rosconsole_debug.conf

# 测试ZMQ连接
cd ~/catkin_ws/src/multibotnet/Scripts
python zmq_diagnostic.py

# 测试话题通信
python topic_test.py

# 测试服务通信
python service_test.py
```

## 📋 架构说明

### 话题通信架构
- 使用 `topic_tools::ShapeShifter` 实现动态消息类型支持
- 发送时自动序列化消息并附加类型信息
- 接收时根据类型信息动态创建发布者
- 支持任意ROS消息类型，无需预定义

### 服务通信架构
- 使用工厂模式管理服务类型
- 每种服务类型需要注册处理函数
- 支持本地服务和远程服务的双向代理
- 可通过简单步骤扩展新服务类型

## 🤝 贡献指南

欢迎贡献代码、报告问题或提出建议！

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Pull Request

### 贡献新的服务类型支持
如果您添加了新的服务类型支持，请：
1. 遵循现有代码风格
2. 添加必要的注释
3. 在PR中说明添加的服务类型
4. 提供使用示例

## 📄 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件

## 📞 联系方式

- 项目主页: https://github.com/SWUST-ICAA/Multibotnet
- 问题反馈: https://github.com/SWUST-ICAA/Multibotnet/issues
- 邮箱: nanwan2004@126.com

---

**Multibotnet** - 让多机器人协作更简单、更高效！

**Powered by ZeroMQ and Enhanced by Claude 4** 🚀