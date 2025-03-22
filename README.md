# Multibotnet

Multibotnet 是一个 ROS 包，它使用 ZeroMQ 实现高效的分布式通信，适用于多个机器人或计算机之间的通信。它通过网络无缝共享 ROS 话题和服务，非常适合多机器人系统。

## 功能

- **话题共享**：
  - 发送话题：使用 ZeroMQ 在网络上发布 ROS 话题。
  - 接收话题：通过 ZeroMQ 订阅来自其他节点的话题。
- **服务管理**：
  - 使用 ZeroMQ REP/REQ 套接字提供和请求 ROS 服务。
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
  - 自定义类型（需要修改代码）
- **支持的服务类型**：
  - std_srvs/SetBool
  - nav_msgs/GetPlan

## 安装

### 依赖项

- ZeroMQ
- yaml-cpp
- ROS（确保您的 ROS 环境已设置）

### 步骤

1. 安装依赖项：
   ```bash
   sudo apt-get install libzmq3-dev libyaml-cpp-dev
   ```

2. 将仓库克隆到您的 catkin 工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/nanwanuser/multibotnet.git
   ```

3. 构建包：
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## 使用方法

1. 编辑 config/default.yaml 以指定话题和服务，包括 IP 地址和端口。
2. 启动 Multibotnet：
   ```bash
   roslaunch multibotnet multibotnet.launch
   ```

## 配置

config/default.yaml 文件包括以下内容：

- **IP**：将名称映射到 IP 地址（例如，self: '*' 表示所有本地 IP）。
- **send_topics**：要通过网络发布的话题。
- **recv_topics**：要从网络订阅的话题。
- **provide_services**：通过 ZeroMQ REP 套接字提供的服务。
- **request_services**：通过 ZeroMQ REQ 套接字调用的服务。

请参阅默认配置以获取示例。

## 添加自定义消息类型

要支持自定义 ROS 消息类型，请按以下步骤修改代码：

1. 添加头文件：在 include/multibotnet/ros_sub_pub.hpp 中，包含您的消息头文件：
   ```cpp
   #include <your_package/YourMessage.h>
   ```

2. 更新 getMsgType：在同一文件中，向 getMsgType 函数添加：
   ```cpp
   if (type == "your_package/YourMessage") return "your_package::YourMessage";
   ```

3. 处理发送：在 src/zmq_manager.cpp 中，更新 sendTopic：
   ```cpp
   else if (message_type == "your_package/YourMessage") {
       sub = nh.subscribe<your_package::YourMessage>(topic, 1, 
           [this, &current_socket, index, topic](const your_package::YourMessage::ConstPtr& msg) {
               if (send_freq_control(index)) {
                   auto buffer = serializeMsg(*msg);
                   zmq::message_t zmq_msg(buffer.size());
                   memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                   if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                       ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                   }
               }
           });
   }
   ```

4. 处理接收：在 src/zmq_manager.cpp 中，更新 recvTopic：
   ```cpp
   else if (message_type == "your_package/YourMessage") {
       pub = nh.advertise<your_package::YourMessage>(topic, 1);
       // 在线程 lambda 中：
       your_package::YourMessage msg = deserializeMsg<your_package::YourMessage>(
           static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
       pub.publish(msg);
   }
   ```

5. 更新依赖项：在 package.xml 中，添加：
   ```xml
   <depend>your_package</depend>
   ```

6. 确保您的自定义包已在工作空间中构建并 sourced。

7. 重新编译：
   ```bash
   catkin_make
   ```

## 添加自定义服务类型

要支持自定义 ROS 服务类型：

1. 添加头文件：在 include/multibotnet/ros_sub_pub.hpp 中，包含您的服务头文件：
   ```cpp
   #include <your_package/YourService.h>
   ```

2. 更新 getMsgType：向 getMsgType 添加：
   ```cpp
   if (type == "your_package/YourService") return "your_package::YourService";
   ```

3. 实现服务处理程序：在 src/service_manager.cpp 中，添加一个处理函数：
   ```cpp
   bool ServiceManager::handleYourService(your_package::YourService::Request& req, 
                                        your_package::YourService::Response& res) {
       // 您的逻辑在此处
       return true;
   }
   ```

4. 在 ServiceManager::init 中，注册服务：
   ```cpp
   else if (service_type == "your_package/YourService") {
       service_servers_.push_back(nh.advertiseService(service_name, &ServiceManager::handleYourService, this));
   }
   ```

5. 在 processRequests 中，处理请求：
   ```cpp
   else if (req_data.service_type == "your_package/YourService") {
       auto req = deserializeMsg<your_package::YourService::Request>(
           static_cast<uint8_t*>(req_data.request.data()), req_data.request.size());
       your_package::YourService::Response res;
       handleYourService(req, res);
       auto buffer = serializeMsg(res);
       zmq::message_t reply(buffer.size());
       memcpy(reply.data(), buffer.data(), buffer.size());
       rep_socket->send(reply, zmq::send_flags::none);
   }
   ```

6. 更新 callService：在 service_manager.cpp 中添加模板实例化：
   ```cpp
   template bool ServiceManager::callService<your_package/YourService>(
       const std::string&, your_package::YourService::Request&, your_package::YourService::Response&);
   ```

7. 更新依赖项：在 package.xml 中添加：
   ```xml
   <depend>your_package</depend>
   ```

8. 重新编译：
   ```bash
   catkin_make
   ```

## 示例

### 场景

- Robot1（IP：192.168.1.101）：发送 /imu 并提供 /set_bool 服务。
- Robot2（IP：192.168.1.102）：接收 /imu 作为 /imu_recv 并请求 /set_bool 服务。

### Robot1 配置
```yaml
IP:
  self: '*'
  robot2: 192.168.1.102

send_topics:
- topic: /imu
  message_type: sensor_msgs/Imu
  max_frequency: 50
  bind_address: self
  port: 3001

provide_services:
- service_name: /set_bool
  service_type: std_srvs/SetBool
  bind_address: self
  port: 5555
```

### Robot2 配置
```yaml
IP:
  self: '*'
  robot1: 192.168.1.101

recv_topics:
- topic: /imu_recv
  message_type: sensor_msgs/Imu
  connect_address: robot1
  port: 3001

request_services:
- service_name: /set_bool
  service_type: std_srvs/SetBool
  connect_address: robot1
  port: 5555
```

在两个机器人上启动 Multibotnet 以启用话题共享和服务调用。
