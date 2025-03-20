# Multibotnet

Multibotnet 是一个基于 ROS 和 ZeroMQ 的分布式通信包，旨在实现多个机器人或计算机之间的高效 ROS 消息传递。通过 ZeroMQ 的发布-订阅模式，multibotnet 能够在分布式系统中无缝地共享 ROS 话题数据。

## 功能特性

- **发送话题（send_topics）**：从 ROS 节点订阅指定的话题，并通过 ZeroMQ 发布到配置的 IP 和端口。
- **接收话题（recv_topics）**：通过 ZeroMQ 从配置的 IP 和端口接收消息，并发布到 ROS 话题中。
- **服务管理**：提供基于 ZeroMQ REP 套接字和 ROS 服务的简单服务管理功能。
- **支持的消息类型**：
  - sensor_msgs/Imu
  - geometry_msgs/Twist
  - std_msgs/String
  - nav_msgs/Odometry
  - sensor_msgs/LaserScan
  - sensor_msgs/Image
  - geometry_msgs/Pose
  - geometry_msgs/Point
  - std_msgs/Float32
  - std_msgs/Int32
  - 自定义消息类型（需手动修改代码）

## 安装指南

### 依赖

- ZeroMQ
- yaml-cpp

### 安装步骤

1. 安装 ZeroMQ 和 yaml-cpp：
   ```bash
   sudo apt-get install libzmq3-dev libyaml-cpp-dev
   ```

2. 克隆 multibotnet 仓库到您的 catkin 工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your-repo/multibotnet.git
   ```

3. 编译 multibotnet：
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## 使用方法

1. 配置 `config/default.yaml` 文件以定义发送和接收的话题、IP 地址和端口。
2. 启动 multibotnet 节点：
   ```bash
   roslaunch multibotnet multibotnet.launch
   ```

## 配置说明

配置文件 `config/default.yaml` 包含以下部分：

- **IP**：定义 IP 地址的映射。
- **send_topics**：配置要发送的 ROS 话题。
- **recv_topics**：配置要接收的 ROS 话题。


## 支持自定义消息类型

要支持自定义消息类型，请按照以下步骤操作：

1. 在 `include/multibotnet/ros_sub_pub.hpp` 中包含自定义消息的头文件：
   ```cpp
   #include <your_package/YourMessage.h>
   ```

2. 在 `getMsgType` 函数中添加自定义类型的映射：
   ```cpp
   if (type == "your_package/YourMessage") return "your_package::YourMessage";
   ```

3. 在 `src/zmq_manager.cpp` 的 `sendTopic` 和 `recvTopic` 函数中添加对自定义类型的处理逻辑。

4. 更新 `CMakeLists.txt` 和 `package.xml` 以包含对自定义包的依赖。

5. 重新编译 multibotnet：
   ```bash
   catkin_make
   ```

## 示例

假设您有两个机器人，robot1 和 robot2，分别运行在不同的 IP 地址上。您可以在 robot1 上配置发送 `/imu` 话题，并在 robot2 上配置接收该话题。

### robot1 的配置（发送 /imu）
```yaml
send_topics:
- topic_name: /imu
  msg_type: sensor_msgs/Imu
  max_freq: 50
  srcIP: self
  srcPort: 3001
```

### robot2 的配置（接收 /imu）
```yaml
recv_topics:
- topic_name: /imu_recv
  msg_type: sensor_msgs/Imu
  srcIP: robot1
  srcPort: 3001
```

在 robot2 上，您可以通过 `/imu_recv` 话题接收来自 robot1 的 IMU 数据。

## 贡献指南

欢迎对 multibotnet 进行贡献！请遵循以下步骤：

1. Fork 本仓库。
2. 创建一个新的分支 (`git checkout -b feature/your-feature`)。
3. 提交您的更改 (`git commit -am 'Add your feature'`)。
4. 推送到分支 (`git push origin feature/your-feature`)。
5. 创建一个 Pull Request。

## 许可证

本项目采用 Apache-2.0 许可证。详情请参阅 LICENSE 文件。
