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

#define RESET   "\033[0m"  // Reset to default color
#define RED     "\033[31m" // Red text
#define GREEN   "\033[32m" // Green text
#define YELLOW  "\033[33m" // Yellow text
#define BLUE    "\033[34m" // Blue text

namespace multibotnet {

ZmqManager::ZmqManager() : context_(1) {}

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

        if (config["send_topics"]) {
            for (const auto& topic : config["send_topics"]) {
                std::string topic_name = topic["topic_name"].as<std::string>();
                std::string msg_type = topic["msg_type"].as<std::string>();
                int max_freq = topic["max_freq"].as<int>();
                std::string src_ip = topic["srcIP"].as<std::string>();
                int src_port = topic["srcPort"].as<int>();

                if (src_ip == "self") {
                    src_ip = "*"; // Bind to all interfaces
                } else if (ip_map.find(src_ip) != ip_map.end()) {
                    src_ip = ip_map[src_ip];
                } else {
                    ROS_ERROR("Invalid srcIP '%s' for send_topic %s, skipping", src_ip.c_str(), topic_name.c_str());
                    continue;
                }

                sendTopic(topic_name, msg_type, max_freq, src_ip, src_port);
            }
        }

        if (config["recv_topics"]) {
            for (const auto& topic : config["recv_topics"]) {
                std::string topic_name = topic["topic_name"].as<std::string>();
                std::string msg_type = topic["msg_type"].as<std::string>();
                std::string src_ip = topic["srcIP"].as<std::string>();
                int src_port = topic["srcPort"].as<int>();

                if (src_ip == "self") {
                    src_ip = "127.0.0.1"; // Connect to loopback for local "self"
                } else if (ip_map.find(src_ip) != ip_map.end()) {
                    src_ip = ip_map[src_ip];
                } else {
                    ROS_ERROR("Invalid srcIP '%s' for recv_topic %s, skipping", src_ip.c_str(), topic_name.c_str());
                    continue;
                }

                recvTopic(topic_name, msg_type, src_ip, src_port);
            }
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to parse config file %s: %s", config_file.c_str(), e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Unexpected error in ZmqManager::init: %s", e.what());
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

void ZmqManager::sendTopic(const std::string& topic, const std::string& msg_type,
    int max_freq, const std::string& src_ip, int src_port) {
    zmq::socket_t pub_socket(context_, ZMQ_PUB);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);


    pub_socket.bind(address);
    pub_sockets_.emplace_back(std::move(pub_socket));
    auto& current_socket = pub_sockets_.back();

    ros::NodeHandle nh;
    ros::Subscriber sub;

    try {
        if (msg_type == "sensor_msgs/Imu") {
            sub = nh.subscribe<sensor_msgs::Imu>(topic, 1, [this, &current_socket, max_freq, topic](const sensor_msgs::Imu::ConstPtr& msg) {
                auto buffer = serializeMsg(*msg);
                zmq::message_t zmq_msg(buffer.size());
                memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                    ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                }
                ros::Rate(max_freq).sleep();
            });
        } else if (msg_type == "geometry_msgs/Twist") {
            sub = nh.subscribe<geometry_msgs::Twist>(topic, 1, [this, &current_socket, max_freq, topic](const geometry_msgs::Twist::ConstPtr& msg) {
                auto buffer = serializeMsg(*msg);
                zmq::message_t zmq_msg(buffer.size());
                memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                    ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                }
                ros::Rate(max_freq).sleep();
            });
        } else if (msg_type == "std_msgs/String") {
            sub = nh.subscribe<std_msgs::String>(topic, 1, [this, &current_socket, max_freq, topic](const std_msgs::String::ConstPtr& msg) {
                auto buffer = serializeMsg(*msg);
                zmq::message_t zmq_msg(buffer.size());
                memcpy(zmq_msg.data(), buffer.data(), buffer.size());
                if (!current_socket.send(zmq_msg, zmq::send_flags::none)) {
                    ROS_ERROR("Failed to send message on topic %s", topic.c_str());
                }
                ros::Rate(max_freq).sleep();
            });
        }
        subscribers_.push_back(sub);
    } catch (const std::exception& e) {
        ROS_ERROR("Error subscribing to topic %s: %s", topic.c_str(), e.what());
    }
}

void ZmqManager::recvTopic(const std::string& topic, const std::string& msg_type,
    const std::string& src_ip, int src_port) {
    zmq::socket_t sub_socket(context_, ZMQ_SUB);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);

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
    if (msg_type == "sensor_msgs/Imu") {
        pub = nh.advertise<sensor_msgs::Imu>(topic, 1);
    } else if (msg_type == "geometry_msgs/Twist") {
        pub = nh.advertise<geometry_msgs::Twist>(topic, 1);
    } else if (msg_type == "std_msgs/String") {
        pub = nh.advertise<std_msgs::String>(topic, 1);
    }

    static std::unordered_set<std::string> logged_topics;
    static std::mutex log_mutex;

    recv_threads_.emplace_back([this, &current_socket, pub, msg_type, topic, &logged_topics, &log_mutex]() {
        bool first_message = true;
        while (ros::ok()) {
            zmq::pollitem_t items[] = {{static_cast<void*>(current_socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, 100); // 100ms timeout
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

void ZmqManager::displayConfig(const YAML::Node& config) {
    // IP Section
    std::cout << BLUE << "-------------IP------------" << RESET << std::endl;
    for (const auto& ip : config["IP"]) {
        std::cout << YELLOW << ip.first.as<std::string>() << " : " << ip.second.as<std::string>() << RESET << std::endl;
    }

    // Send Topics Section
    std::cout << BLUE << "--------send topics--------" << RESET << std::endl;
    if (config["send_topics"]) {
        for (const auto& topic : config["send_topics"]) {
            std::string topic_name = topic["topic_name"].as<std::string>();
            int max_freq = topic["max_freq"].as<int>();
            std::cout << GREEN << topic_name << "  " << max_freq << "Hz(max)" << RESET << std::endl;
        }
    }

    // Receive Topics Section
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