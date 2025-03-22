# Multibotnet

Multibotnet 是一个专为多机器人系统设计的 ROS 包，利用 ZeroMQ 技术实现高效的分布式通信。简单来说，它能让多台机器人或电脑之间轻松共享 ROS 话题和服务，哪怕它们不在同一个网络环境下也能协作工作。无论是机器人团队协同任务，还是跨设备的数据共享，Multibotnet 都能派上用场！

## 主要功能

### 话题共享
- **发送话题**：把本地的 ROS 话题通过网络发出去，其他机器人就能收到。
- **接收话题**：从网络上抓取其他机器人发来的话题，融入本地 ROS 系统。

### 服务管理
- **提供服务**：让你的机器人通过网络为别人提供 ROS 服务，比如远程开关控制。
- **请求服务**：调用其他机器人提供的服务，实现跨设备的功能交互。

### 支持的消息类型
支持常见的 ROS 消息类型，包括但不限于：
- sensor_msgs/Imu（IMU 数据）
- geometry_msgs/Twist（速度指令）
- std_msgs/String（字符串）
- nav_msgs/Odometry（里程计）
- sensor_msgs/LaserScan（激光雷达）
- sensor_msgs/Image（图像）
- geometry_msgs/Pose（位姿）
- geometry_msgs/Point（点坐标）
- std_msgs/Float32（浮点数）
- std_msgs/Int32（整数）
- 自定义类型（稍作修改就能支持，超灵活！）

### 支持的服务类型
- std_srvs/SetBool（布尔开关服务）
- nav_msgs/GetPlan（路径规划服务）
- 自定义服务（同样支持扩展）

## 项目优势

- **简单配置，一键搞定**
  通过一个 YAML 文件就能设置话题、服务、IP 和端口，想改啥改啥，完全不用碰代码。
  
- **通信超快，效率爆表**
  用 ZeroMQ 技术，支持多对多通信，哪怕是大规模机器人集群也能hold住。
  
- **频率可控，不怕卡顿**
  发送话题时可以限制频率，避免网络堵塞，带宽利用率刚刚好。
  
- **扩展方便，随心所欲**
  想加新的消息或服务类型？改几行代码就行，完美适配你的项目需求。
  
- **跨平台无压力**
  不管是机器人还是普通电脑，只要有 ROS 环境，就能跑起来，分布式系统so easy！

## 安装步骤

### 准备工作
需要先装好以下依赖：
- ZeroMQ：网络通信核心
- yaml-cpp：解析配置文件用
- ROS：确保你的 ROS 环境已经配置好

### 安装命令
1. 安装依赖：
   ```bash
   sudo apt-get install libzmq3-dev libyaml-cpp-dev
   ```

2. 克隆项目到你的 catkin 工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/nanwanuser/multibotnet.git
   ```

3. 编译项目：
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## 使用方法

1. **配置一下**
   打开 config/default.yaml 文件，填入你想要共享的话题和服务信息，比如 IP 地址、端口号等。

2. **启动程序**
   一行命令搞定：
   ```bash
   roslaunch multibotnet multibotnet.launch
   ```

## 配置说明
config/default.yaml 是你的“控制中心”，里面有这些关键项：
- **IP**：给 IP 地址起个别名，比如 self: '*' 表示本机所有 IP。
- **send_topics**：设置要发出去的话题（话题名、类型、频率、地址、端口）。
- **recv_topics**：设置要接收的话题（话题名、类型、地址、端口）。
- **provide_services**：定义你要提供的服务（服务名、类型、地址、端口）。
- **request_services**：定义你要调用的远程服务（服务名、类型、地址、端口）。

具体格式可以参考默认文件，照着改就行！

## 如何扩展自定义类型
想用自己的消息或服务类型？很简单，按以下步骤操作：

### 添加自定义消息类型
1. 引入头文件
   在 include/multibotnet/ros_sub_pub.hpp 中加一行：
   ```cpp
   #include <your_package/YourMessage.h>
   ```

2. 映射类型
   在 getMsgType 函数中添加：
   ```cpp
   if (type == "your_package/YourMessage") return "your_package::YourMessage";
   ```

3. 发送逻辑
   在 src/zmq_manager.cpp 的 sendTopic 函数中加一段：
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

4. 接收逻辑
   在 src/zmq_manager.cpp 的 recvTopic 函数中加一段：
   ```cpp
   else if (message_type == "your_package/YourMessage") {
       pub = nh.advertise<your_package::YourMessage>(topic, 1);
       // 在接收线程中：
       your_package::YourMessage msg = deserializeMsg<your_package::YourMessage>(
           static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
       pub.publish(msg);
   }
   ```

5. 更新依赖
   在 package.xml 中添加：
   ```xml
   <depend>your_package</depend>
   ```

6. 重新编译
   ```bash
   catkin_make
   ```

### 添加自定义服务类型
1. 引入头文件
   在 include/multibotnet/ros_sub_pub.hpp 中加一行：
   ```cpp
   #include <your_package/YourService.h>
   ```

2. 映射类型
   在 getMsgType 函数中添加：
   ```cpp
   if (type == "your_package/YourService") return "your_package::YourService";
   ```

3. 实现服务逻辑
   在 src/service_manager.cpp 中加一个处理函数：
   ```cpp
   bool ServiceManager::handleYourService(your_package::YourService::Request& req, 
                                        your_package::YourService::Response& res) {
       // 你的服务逻辑
       return true;
   }
   ```

4. 注册服务
   在 ServiceManager::init 中添加：
   ```cpp
   else if (service_type == "your_package/YourService") {
       service_servers_.push_back(nh.advertiseService(service_name, &ServiceManager::handleYourService, this));
   }
   ```

5. 处理请求
   在 processRequests 中添加：
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

6. 更新调用
   在 service_manager.cpp 中添加模板实例化：
   ```cpp
   template bool ServiceManager::callService<your_package/YourService>(
       const std::string&, your_package::YourService::Request&, your_package::YourService::Response&);
   ```

7. 更新依赖
   在 package.xml 中添加：
   ```xml
   <depend>your_package</depend>
   ```

8. 重新编译
   ```bash
   catkin_make
   ```

## 应用示例
### 场景描述
- Robot1（IP: 192.168.1.101）：发送 /imu 话题，提供 /set_bool 服务。
- Robot2（IP: 192.168.1.102）：接收 /imu 话题（显示为 /imu_recv），调用 /set_bool 服务。

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

启动两台机器上的 Multibotnet 后，Robot2 就能收到 Robot1 的 IMU 数据，并远程控制它的开关服务。
## 总结

Multibotnet 是一个简单又强大的工具，能让多机器人系统高效协作。无论是话题共享还是服务调用，它都能通过灵活的配置和高性能通信满足你的需求。快来试试吧，让你的机器人团队更聪明、更协同！

## 3.1.2版本更新特性

支持了服务通信的功能并且通过了测试
