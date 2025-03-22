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

ZmqManager::ZmqManager() : context_(1) {
}

ZmqManager::~ZmqManager() {
    for (auto& th : recv_threads_) {
        if (th.joinable()) th.join();
    }
}

void ZmqManager::init(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        displayConfig(config);

        std::map<std::string, std::string> ip_map;
        for (const auto& ip : config["IP"]) {
            ip_map[ip.first.as<std::string>()] = ip.second.as<std::string>();
        }

        // 计算发送和接收话题数量以预分配空间
        size_t send_topic_count = config["send_topics"] ? config["send_topics"].size() : 0;

        // 初始化频率控制变量
        send_t_last_.resize(send_topic_count);
        send_num_.resize(send_topic_count, 0);
        max_frequencies_.resize(send_topic_count);

        int send_index = 0;
        if (config["send_topics"]) {
            for (const auto& topic : config["send_topics"]) {
                std::string topic_name = topic["topic"].as<std::string>();
                std::string message_type = topic["message_type"].as<std::string>();
                int max_frequency = topic["max_frequency"].as<int>();
                std::string bind_address = topic["bind_address"].as<std::string>();
                int port = topic["port"].as<int>();

                if (bind_address == "self") {
                    bind_address = "*";
                } else if (ip_map.find(bind_address) != ip_map.end()) {
                    bind_address = ip_map[bind_address];
                } else {
                    ROS_ERROR("Invalid bind_address '%s' for send_topic %s, skipping", bind_address.c_str(), topic_name.c_str());
                    continue;
                }

                max_frequencies_[send_index] = max_frequency;  // 存储最大频率
                send_t_last_[send_index] = ros::Time::now();  // 初始化时间
                sendTopic(topic_name, message_type, max_frequency, bind_address, port);
                send_index++;
            }
        }

        if (config["recv_topics"]) {
            for (const auto& topic : config["recv_topics"]) {
                std::string topic_name = topic["topic"].as<std::string>();
                std::string message_type = topic["message_type"].as<std::string>();
                std::string connect_address = topic["connect_address"].as<std::string>();
                int port = topic["port"].as<int>();

                if (connect_address == "self") {
                    connect_address = "127.0.0.1";
                } else if (ip_map.find(connect_address) != ip_map.end()) {
                    connect_address = ip_map[connect_address];
                } else {
                    ROS_ERROR("Invalid connect_address '%s' for recv_topic %s, skipping", connect_address.c_str(), topic_name.c_str());
                    continue;
                }

                recvTopic(topic_name, message_type, connect_address, port);
            }
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse config file %s: %s", config_file.c_str(), e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Unexpected error in ZmqManager::init: %s", e.what());
    }
}

void ZmqManager::sendTopic(const std::string& topic, 
                           const std::string& message_type, 
                           int max_frequency,
                           const std::string& bind_address, 
                           int port) {
    zmq::socket_t pub_socket(context_, ZMQ_PUB);
    std::string address = "tcp://" + bind_address + ":" + std::to_string(port);
    pub_socket.bind(address);
    pub_sockets_.emplace_back(std::move(pub_socket));
    auto& current_socket = pub_sockets_.back();

    ros::NodeHandle nh;
    ros::Subscriber sub;

    // 获取当前话题的索引
    int index = subscribers_.size();

    try {
        if (message_type == "sensor_msgs/Imu") {
            sub = nh.subscribe<sensor_msgs::Imu>(topic, 1, 
                [this, &current_socket, index, topic](const sensor_msgs::Imu::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "geometry_msgs/Twist") {
            sub = nh.subscribe<geometry_msgs::Twist>(topic, 1, 
                [this, &current_socket, index, topic](const geometry_msgs::Twist::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "std_msgs/String") {
            sub = nh.subscribe<std_msgs::String>(topic, 1, 
                [this, &current_socket, index, topic](const std_msgs::String::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "nav_msgs/Odometry") {
            sub = nh.subscribe<nav_msgs::Odometry>(topic, 1, 
                [this, &current_socket, index, topic](const nav_msgs::Odometry::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "sensor_msgs/LaserScan") {
            sub = nh.subscribe<sensor_msgs::LaserScan>(topic, 1, 
                [this, &current_socket, index, topic](const sensor_msgs::LaserScan::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "sensor_msgs/Image") {
            sub = nh.subscribe<sensor_msgs::Image>(topic, 1, 
                [this, &current_socket, index, topic](const sensor_msgs::Image::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "geometry_msgs/Pose") {
            sub = nh.subscribe<geometry_msgs::Pose>(topic, 1, 
                [this, &current_socket, index, topic](const geometry_msgs::Pose::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "geometry_msgs/Point") {
            sub = nh.subscribe<geometry_msgs::Point>(topic, 1, 
                [this, &current_socket, index, topic](const geometry_msgs::Point::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "std_msgs/Float32") {
            sub = nh.subscribe<std_msgs::Float32>(topic, 1, 
                [this, &current_socket, index, topic](const std_msgs::Float32::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else if (message_type == "std_msgs/Int32") {
            sub = nh.subscribe<std_msgs::Int32>(topic, 1, 
                [this, &current_socket, index, topic](const std_msgs::Int32::ConstPtr& msg) {
                    if (send_freq_control(index)) {
                        auto buffer = serializeMsg(*msg);
                        zmq::message_t zmq_msg(buffer.size());
                        memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                        if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                            ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                        }
                    }
                });
        } else {
            ROS_ERROR("Unsupported message type '%s' for topic %s", message_type.c_str(), topic.c_str());
            return;
        }
        subscribers_.push_back(sub);
    } catch (const std::exception& e) {
        ROS_ERROR("Error subscribing to topic %s: %s", topic.c_str(), e.what());
    }
}

void ZmqManager::recvTopic(const std::string& topic, 
                           const std::string& message_type,
                           const std::string& connect_address, 
                           int port) {
    zmq::socket_t sub_socket(context_, ZMQ_SUB);
    std::string address = "tcp://" + connect_address + ":" + std::to_string(port);

    try {
        sub_socket.connect(address);
        sub_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    } catch (const zmq::error_t& e) {
        ROS_ERROR("Failed to connect to %s for recv_topic %s: %s", address.c_str(), topic.c_str(), e.what());
        return;
    }

    sub_sockets_.emplace_back(std::move(sub_socket));
    auto& current_socket = sub_sockets_.back();

    ros::NodeHandle nh;
    ros::Publisher pub;

    // 为不同消息类型创建发布者
    if (message_type == "sensor_msgs/Imu") {
        pub = nh.advertise<sensor_msgs::Imu>(topic, 1);
    } else if (message_type == "geometry_msgs/Twist") {
        pub = nh.advertise<geometry_msgs::Twist>(topic, 1);
    } else if (message_type == "std_msgs/String") {
        pub = nh.advertise<std_msgs::String>(topic, 1);
    } else if (message_type == "nav_msgs/Odometry") {
        pub = nh.advertise<nav_msgs::Odometry>(topic, 1);
    } else if (message_type == "sensor_msgs/LaserScan") {
        pub = nh.advertise<sensor_msgs::LaserScan>(topic, 1);
    } else if (message_type == "sensor_msgs/Image") {
        pub = nh.advertise<sensor_msgs::Image>(topic, 1);
    } else if (message_type == "geometry_msgs/Pose") {
        pub = nh.advertise<geometry_msgs::Pose>(topic, 1);
    } else if (message_type == "geometry_msgs/Point") {
        pub = nh.advertise<geometry_msgs::Point>(topic, 1);
    } else if (message_type == "std_msgs/Float32") {
        pub = nh.advertise<std_msgs::Float32>(topic, 1);
    } else if (message_type == "std_msgs/Int32") {
        pub = nh.advertise<std_msgs::Int32>(topic, 1);
    } else {
        ROS_ERROR("Unsupported message type '%s' for topic %s", message_type.c_str(), topic.c_str());
        return;
    }

    static std::unordered_set<std::string> logged_topics;
    static std::mutex log_mutex;

    recv_threads_.emplace_back([this, &current_socket, pub, message_type, topic]() {
        bool first_message = true;
        while (ros::ok()) {
            zmq::pollitem_t items[] = {{static_cast<void*>(current_socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, 100);  // 100ms超时，避免忙等待
            if (!ros::ok()) break;
            if (items[0].revents & ZMQ_POLLIN) {
                zmq::message_t zmq_msg;
                if (current_socket.recv(zmq_msg)) {
                    if (first_message) {
                        std::lock_guard<std::mutex> lock(log_mutex);
                        if (logged_topics.find(topic) == logged_topics.end()) {
                            ROS_INFO("[multibotnet_topic_node] \"%s\" received!", topic.c_str());
                            logged_topics.insert(topic);
                        }
                        first_message = false;
                    }
                    if (message_type == "sensor_msgs/Imu") {
                        sensor_msgs::Imu msg = deserializeMsg<sensor_msgs::Imu>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "geometry_msgs/Twist") {
                        geometry_msgs::Twist msg = deserializeMsg<geometry_msgs::Twist>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "std_msgs/String") {
                        std_msgs::String msg = deserializeMsg<std_msgs::String>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "nav_msgs/Odometry") {
                        nav_msgs::Odometry msg = deserializeMsg<nav_msgs::Odometry>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "sensor_msgs/LaserScan") {
                        sensor_msgs::LaserScan msg = deserializeMsg<sensor_msgs::LaserScan>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "sensor_msgs/Image") {
                        sensor_msgs::Image msg = deserializeMsg<sensor_msgs::Image>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "geometry_msgs/Pose") {
                        geometry_msgs::Pose msg = deserializeMsg<geometry_msgs::Pose>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "geometry_msgs/Point") {
                        geometry_msgs::Point msg = deserializeMsg<geometry_msgs::Point>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "std_msgs/Float32") {
                        std_msgs::Float32 msg = deserializeMsg<std_msgs::Float32>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    } else if (message_type == "std_msgs/Int32") {
                        std_msgs::Int32 msg = deserializeMsg<std_msgs::Int32>(
                            static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                        pub.publish(msg);
                    }
                }
            }
        }
    });
}

void ZmqManager::displayConfig(const YAML::Node& config) {
    std::cout << BLUE << "-------------IP------------" << RESET << std::endl;
    for (const auto& ip : config["IP"]) {
        std::cout << YELLOW << ip.first.as<std::string>() << " : " << ip.second.as<std::string>() << RESET << std::endl;
    }

    // 创建 IP 映射表
    std::map<std::string, std::string> ip_map;
    for (const auto& ip : config["IP"]) {
        ip_map[ip.first.as<std::string>()] = ip.second.as<std::string>();
    }

    std::cout << BLUE << "--------send topics--------" << RESET << std::endl;
    if (config["send_topics"]) {
        for (const auto& topic : config["send_topics"]) {
            std::string topic_name = topic["topic"].as<std::string>();
            int max_frequency = topic["max_frequency"].as<int>();
            std::string bind_address_key = topic["bind_address"].as<std::string>();
            int port = topic["port"].as<int>();

            // 确定显示的 IP 地址
            std::string display_ip;
            if (bind_address_key == "self") {
                display_ip = this->getLocalIP(); // 当为 "self" 时，使用本机 IP
            } else if (ip_map.find(bind_address_key) != ip_map.end()) {
                display_ip = ip_map[bind_address_key]; // 从 IP 映射中查找
            } else {
                display_ip = "Invalid: " + bind_address_key; // 无效地址
            }

            // 组合 IP 和端口
            std::string display_address = display_ip + ":" + std::to_string(port);

            // 显示话题信息
            std::cout << GREEN << topic_name << "  " << max_frequency 
                      << "Hz(max_frequency)  bind: " << display_address << RESET << std::endl;
        }
    }

    std::cout << BLUE << "-------receive topics------" << RESET << std::endl;
    if (config["recv_topics"]) {
        for (const auto& topic : config["recv_topics"]) {
            std::string topic_name = topic["topic"].as<std::string>();
            std::string connect_address = topic["connect_address"].as<std::string>();
            std::cout << GREEN << topic_name << "  (IP from " << connect_address << ")" << RESET << std::endl;
        }
    }
}

bool ZmqManager::send_freq_control(int index) {
    ros::Time t_now = ros::Time::now();
    double elapsed = (t_now - send_t_last_[index]).toSec();
    int max_freq = max_frequencies_[index];

    if ((send_num_[index] + 1) / elapsed > max_freq) {
        return false;  // 超过频率限制，不发送
    } else {
        send_num_[index]++;
        if (elapsed > 1.0) {  // 每秒重置一次计数器
            send_t_last_[index] = t_now;
            send_num_[index] = 0;
        }
        return true;  // 允许发送
    }
}

std::string ZmqManager::getLocalIP() {
    struct ifaddrs *ifaddr;
    std::string local_ip = "127.0.0.1";

    if (getifaddrs(&ifaddr) == -1) {
        ROS_ERROR("Failed to get local IP address: %s", strerror(errno));
        return local_ip;
    }

    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
            char *ip = inet_ntoa(addr->sin_addr);
            if (strcmp(ip, "127.0.0.1") != 0) {
                local_ip = ip;
                break;
            }
        }
    }

    freeifaddrs(ifaddr);
    if (local_ip == "127.0.0.1") {
        ROS_WARN("No non-loopback IP found, using 127.0.0.1");
    }
    return local_ip;
}

} // namespace multibotnet