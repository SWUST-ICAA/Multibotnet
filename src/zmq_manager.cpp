// zmq_manager.cpp

#include <multibotnet/zmq_manager.hpp> 
#include <multibotnet/ros_sub_pub.hpp> 
#include <yaml-cpp/yaml.h>             
#include <chrono>                      
#include <thread>                      
#include <ifaddrs.h>                   
#include <netinet/in.h>                
#include <arpa/inet.h>                 
#include <cstring>                     
#include <unordered_set>               
#include <iostream>                    


#define RESET   "\033[0m"  
#define RED     "\033[31m" 
#define GREEN   "\033[32m" 
#define YELLOW  "\033[33m" 
#define BLUE    "\033[34m" 

namespace multibotnet {

// 构造函数，初始化ZeroMQ上下文
ZmqManager::ZmqManager() : context_(1) {}

// 析构函数，确保所有接收线程在对象销毁前结束
ZmqManager::~ZmqManager() {
    for (auto& th : recv_threads_) {
        if (th.joinable()) th.join(); // 等待线程结束
    }
}

// 初始化函数，加载配置文件并启动发送和接收话题
void ZmqManager::init(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file); // 加载YAML配置文件
        displayConfig(config); // 显示配置信息

        // 解析IP映射
        std::map<std::string, std::string> ip_map;
        for (const auto& ip : config["IP"]) {
            ip_map[ip.first.as<std::string>()] = ip.second.as<std::string>();
        }

        // 处理send_topics配置
        if (config["send_topics"]) {
            for (const auto& topic : config["send_topics"]) {
                std::string topic_name = topic["topic_name"].as<std::string>(); // 话题名称
                std::string msg_type = topic["msg_type"].as<std::string>();     // 消息类型
                int max_freq = topic["max_freq"].as<int>();                     // 最大频率
                std::string src_ip = topic["srcIP"].as<std::string>();          // 源IP
                int src_port = topic["srcPort"].as<int>();                      // 源端口

                // 处理srcIP
                if (src_ip == "self") {
                    src_ip = "*"; // 绑定到所有接口
                } else if (ip_map.find(src_ip) != ip_map.end()) {
                    src_ip = ip_map[src_ip]; // 使用配置中的IP映射
                } else {
                    ROS_ERROR("Invalid srcIP '%s' for send_topic %s, skipping", src_ip.c_str(), topic_name.c_str());
                    continue; // 跳过无效配置
                }

                // 启动发送话题
                sendTopic(topic_name, msg_type, max_freq, src_ip, src_port);
            }
        }

        // 处理recv_topics配置
        if (config["recv_topics"]) {
            for (const auto& topic : config["recv_topics"]) {
                std::string topic_name = topic["topic_name"].as<std::string>(); // 话题名称
                std::string msg_type = topic["msg_type"].as<std::string>();     // 消息类型
                std::string src_ip = topic["srcIP"].as<std::string>();          // 源IP
                int src_port = topic["srcPort"].as<int>();                      // 源端口

                // 处理srcIP
                if (src_ip == "self") {
                    src_ip = "127.0.0.1"; // 连接到本地环回接口
                } else if (ip_map.find(src_ip) != ip_map.end()) {
                    src_ip = ip_map[src_ip]; // 使用配置中的IP映射
                } else {
                    ROS_ERROR("Invalid srcIP '%s' for recv_topic %s, skipping", src_ip.c_str(), topic_name.c_str());
                    continue; // 跳过无效配置
                }

                // 启动接收话题
                recvTopic(topic_name, msg_type, src_ip, src_port);
            }
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse config file %s: %s", config_file.c_str(), e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Unexpected error in ZmqManager::init: %s", e.what());
    }
}

// 获取本机IP地址
std::string ZmqManager::getLocalIP() {
    struct ifaddrs *ifaddr;
    std::string local_ip = "127.0.0.1"; // 默认使用环回地址

    if (getifaddrs(&ifaddr) == -1) {
        ROS_ERROR("Failed to get local IP address: %s", strerror(errno));
        return local_ip; // 获取失败返回默认值
    }

    // 遍历网络接口
    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) { // IPv4地址
            struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
            char *ip = inet_ntoa(addr->sin_addr);
            if (strcmp(ip, "127.0.0.1") != 0) { // 优先选择非环回地址
                local_ip = ip;
                break;
            }
        }
    }

    freeifaddrs(ifaddr); // 释放接口信息
    if (local_ip == "127.0.0.1") {
        ROS_WARN("No non-loopback IP found, using 127.0.0.1");
    }
    return local_ip;
}

// 发送话题
void ZmqManager::sendTopic(const std::string& topic, const std::string& msg_type,
    int max_freq, const std::string& src_ip, int src_port) {
    // 创建ZeroMQ PUB套接字，用于发布消息
    zmq::socket_t pub_socket(context_, ZMQ_PUB);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);

    // 绑定套接字到指定地址
    pub_socket.bind(address);
    pub_sockets_.emplace_back(std::move(pub_socket));
    auto& current_socket = pub_sockets_.back(); // 获取当前套接字引用

    // 创建ROS订阅者
    ros::NodeHandle nh;
    ros::Subscriber sub;

    try {
        // 根据消息类型订阅ROS话题并发送
        if (msg_type == "sensor_msgs/Imu") {
            sub = nh.subscribe<sensor_msgs::Imu>(topic, 1, [this, &current_socket, max_freq, topic](const sensor_msgs::Imu::ConstPtr& msg) {
                auto buffer = serializeMsg(*msg); // 序列化消息
                zmq::message_t zmq_msg(buffer.size());
                memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                    ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                }
                ros::Rate(max_freq).sleep(); // 控制发送频率
            });
        } else if (msg_type == "geometry_msgs/Twist") {
            sub = nh.subscribe<geometry_msgs::Twist>(topic, 1, [this, &current_socket, max_freq, topic](const geometry_msgs::Twist::ConstPtr& msg) {
                auto buffer = serializeMsg(*msg); // 序列化消息
                zmq::message_t zmq_msg(buffer.size());
                memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                    ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                }
                ros::Rate(max_freq).sleep(); // 控制发送频率
            });
        } else if (msg_type == "std_msgs/String") {
            sub = nh.subscribe<std_msgs::String>(topic, 1, [this, &current_socket, max_freq, topic](const std_msgs::String::ConstPtr& msg) {
                auto buffer = serializeMsg(*msg); // 序列化消息
                zmq::message_t zmq_msg(buffer.size());
                memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                    ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                }
                ros::Rate(max_freq).sleep(); // 控制发送频率
            });
        }
        subscribers_.push_back(sub); // 保存订阅者对象
    } catch (const std::exception& e) {
        ROS_ERROR("Error subscribing to topic %s: %s", topic.c_str(), e.what());
    }
}

// 接收话题
void ZmqManager::recvTopic(const std::string& topic, const std::string& msg_type,
    const std::string& src_ip, int src_port) {
    // 创建ZeroMQ SUB套接字，用于订阅消息
    zmq::socket_t sub_socket(context_, ZMQ_SUB);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);

    try {
        // 连接到远程地址并订阅所有消息
        sub_socket.connect(address);
        sub_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    } catch (const zmq::error_t& e) {
        ROS_ERROR("Failed to connect to %s for recv_topic %s: %s", address.c_str(), topic.c_str(), e.what());
        return; // 连接失败，直接返回
    }

    sub_sockets_.emplace_back(std::move(sub_socket));
    auto& current_socket = sub_sockets_.back(); // 获取当前套接字引用

    // 创建ROS发布者
    ros::NodeHandle nh;
    ros::Publisher pub;
    if (msg_type == "sensor_msgs/Imu") {
        pub = nh.advertise<sensor_msgs::Imu>(topic, 1);
    } else if (msg_type == "geometry_msgs/Twist") {
        pub = nh.advertise<geometry_msgs::Twist>(topic, 1);
    } else if (msg_type == "std_msgs/String") {
        pub = nh.advertise<std_msgs::String>(topic, 1);
    }

    // 用于记录已接收话题的集合，防止重复日志
    static std::unordered_set<std::string> logged_topics;
    static std::mutex log_mutex;

    // 启动接收线程
    recv_threads_.emplace_back([this, &current_socket, pub, msg_type, topic, &logged_topics, &log_mutex]() {
        bool first_message = true;
        while (ros::ok()) {
            zmq::pollitem_t items[] = {{static_cast<void*>(current_socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, 100); // 100ms超时轮询
            if (!ros::ok()) break; // ROS关闭时退出循环
            if (items[0].revents & ZMQ_POLLIN) { // 有数据可读
                zmq::message_t zmq_msg;
                if (current_socket.recv(zmq_msg)) { // 接收消息
                    if (first_message) {
                        std::lock_guard<std::mutex> lock(log_mutex);
                        if (logged_topics.find(topic) == logged_topics.end()) {
                            ROS_INFO("[multibotnet_topic_node] \"%s\" received!", topic.c_str());
                            logged_topics.insert(topic); // 记录已接收话题
                        }
                        first_message = false;
                    }
                    // 根据消息类型反序列化并发布
                    if (msg_type == "sensor_msgs/Imu") {
                        sensor_msgs::Imu msg = deserializeMsg<sensor_msgs::Imu>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (msg_type == "geometry_msgs/Twist") {
                        geometry_msgs::Twist msg = deserializeMsg<geometry_msgs::Twist>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (msg_type == "std_msgs/String") {
                        std_msgs::String msg = deserializeMsg<std_msgs::String>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    }
                }
            }
        }
    });
}

// 显示配置信息到终端
void ZmqManager::displayConfig(const YAML::Node& config) {
    // IP部分
    std::cout << BLUE << "-------------IP------------" << RESET << std::endl;
    for (const auto& ip : config["IP"]) {
        std::cout << YELLOW << ip.first.as<std::string>() << " : " << ip.second.as<std::string>() << RESET << std::endl;
    }

    // 发送话题部分
    std::cout << BLUE << "--------send topics--------" << RESET << std::endl;
    if (config["send_topics"]) {
        for (const auto& topic : config["send_topics"]) {
            std::string topic_name = topic["topic_name"].as<std::string>();
            int max_freq = topic["max_freq"].as<int>();
            std::cout << GREEN << topic_name << "  " << max_freq << "Hz(max)" << RESET << std::endl;
        }
    }

    // 接收话题部分
    std::cout << BLUE << "-------receive topics------" << RESET << std::endl;
    if (config["recv_topics"]) {
        for (const auto& topic : config["recv_topics"]) {
            std::string topic_name = topic["topic_name"].as<std::string>();
            std::string src_ip = topic["srcIP"].as<std::string>();
            std::cout << GREEN << topic_name << "  (from " << src_ip << ")" << RESET << std::endl;
        }
    }
}

} // namespace multibotnet