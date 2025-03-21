# Multibotnet

Multibotnet is a ROS package that uses ZeroMQ to enable efficient distributed communication across multiple robots or computers. It facilitates seamless sharing of ROS topics and services over a network, making it perfect for multi-robot systems.

## Features

- **Topic Sharing:**
  - Send Topics: Publish ROS topics over the network using ZeroMQ.
  - Receive Topics: Subscribe to topics from other nodes via ZeroMQ.
- **Service Management:**
  - Provide and request ROS services using ZeroMQ REP/REQ sockets.
- **Supported Message Types:**
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
  - Custom types (requires code modification)
- **Supported Service Types:**
  - std_srvs/SetBool
  - nav_msgs/GetPlan

## Installation

### Dependencies

- ZeroMQ
- yaml-cpp
- ROS (ensure your ROS environment is set up)

### Steps

1. Install dependencies:
   ```bash
   sudo apt-get install libzmq3-dev libyaml-cpp-dev
   ```

2. Clone the repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/nanwanuser/multibotnet.git
   ```

3. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## Usage

1. Edit config/default.yaml to specify topics and services, including IP addresses and ports.
2. Launch Multibotnet:
   ```bash
   roslaunch multibotnet multibotnet.launch
   ```

## Configuration

The config/default.yaml file includes:

- **IP:** Maps names to IP addresses (e.g., self: '*' for all local IPs).
- **send_topics:** Topics to publish over the network.
- **recv_topics:** Topics to subscribe to from the network.
- **provide_services:** Services to offer via ZeroMQ REP sockets.
- **request_services:** Services to call via ZeroMQ REQ sockets.

See the default configuration for examples.

## Adding Custom Message Types

To support custom ROS message types, modify the code as follows:

1. **Add Header File:**
   In include/multibotnet/ros_sub_pub.hpp, include your message header:
   ```cpp
   #include <your_package/YourMessage.h>
   ```

2. **Update getMsgType:**
   In the same file, add to the getMsgType function:
   ```cpp
   if (type == "your_package/YourMessage") return "your_package::YourMessage";
   ```

3. **Handle Sending:**
   In src/zmq_manager.cpp, update sendTopic:
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

4. **Handle Receiving:**
   In src/zmq_manager.cpp, update recvTopic:
   ```cpp
   else if (message_type == "your_package/YourMessage") {
       pub = nh.advertise<your_package::YourMessage>(topic, 1);
       // Inside the thread lambda:
       your_package::YourMessage msg = deserializeMsg<your_package::YourMessage>(
           static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
       pub.publish(msg);
   }
   ```

5. **Update Dependencies:**
   In package.xml, add:
   ```xml
   <depend>your_package</depend>
   ```
   Ensure your custom package is built and sourced in your workspace.

6. **Recompile:**
   ```bash
   catkin_make
   ```

## Adding Custom Service Types

To support custom ROS service types:

1. **Add Header File:**
   In include/multibotnet/ros_sub_pub.hpp, include your service header:
   ```cpp
   #include <your_package/YourService.h>
   ```

2. **Update getMsgType:**
   Add to getMsgType:
   ```cpp
   if (type == "your_package/YourService") return "your_package::YourService";
   ```

3. **Implement Service Handler:**
   In src/service_manager.cpp, add a handler function:
   ```cpp
   bool ServiceManager::handleYourService(your_package::YourService::Request& req, 
                                        your_package::YourService::Response& res) {
       // Your logic here
       return true;
   }
   ```
   In ServiceManager::init, register the service:
   ```cpp
   else if (service_type == "your_package/YourService") {
       service_servers_.push_back(nh.advertiseService(service_name, &ServiceManager::handleYourService, this));
   }
   ```
   In processRequests, handle the request:
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

4. **Update callService:**
   Add a template instantiation in service_manager.cpp:
   ```cpp
   template bool ServiceManager::callService<your_package::YourService>(
       const std::string&, your_package::YourService::Request&, your_package::YourService::Response&);
   ```

5. **Update Dependencies:**
   Add to package.xml:
   ```xml
   <depend>your_package</depend>
   ```

6. **Recompile:**
   ```bash
   catkin_make
   ```

## Example

### Scenario

- Robot1 (IP: 192.168.1.101) sends /imu and provides /set_bool.
- Robot2 (IP: 192.168.1.102) receives /imu as /imu_recv and requests /set_bool.

### Robot1 Configuration
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

### Robot2 Configuration
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

Launch Multibotnet on both robots to enable topic sharing and service calls.
