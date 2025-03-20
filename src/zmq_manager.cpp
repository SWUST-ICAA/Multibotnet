#include <multibotnet/zmq_manager.hpp>
#include <multibotnet/ros_sub_pub.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

namespace multibotnet {

ZmqManager::ZmqManager() : context_(1) {}

ZmqManager::~ZmqManager() {
    for (auto& th : recv_threads_) {
        if (th.joinable()) th.join();
    }
}

void ZmqManager::init(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);

    // 解析 IP 映射
    std::map<std::string, std::string> ip_map;
    for (const auto& ip : config["IP"]) {
        ip_map[ip.first.as<std::string>()] = ip.second.as<std::string>();
    }

    // 处理发送话题 (send_topics)
    if (config["send_topics"]) {
        for (const auto& topic : config["send_topics"]) {
            std::string topic_name = topic["topic_name"].as<std::string>();
            std::string msg_type = topic["msg_type"].as<std::string>();
            int max_freq = topic["max_freq"].as<int>();
            std::string src_ip = topic["srcIP"].as<std::string>();
            int src_port = topic["srcPort"].as<int>();

            src_ip = (src_ip == "self") ? "*" : ip_map[src_ip];
            sendTopic(topic_name, msg_type, max_freq, src_ip, src_port);
        }
    }

    // 处理接收话题 (recv_topics)
    if (config["recv_topics"]) {
        for (const auto& topic : config["recv_topics"]) {
            std::string topic_name = topic["topic_name"].as<std::string>();
            std::string msg_type = topic["msg_type"].as<std::string>();
            std::string src_ip = topic["srcIP"].as<std::string>();
            int src_port = topic["srcPort"].as<int>();

            src_ip = ip_map[src_ip];
            recvTopic(topic_name, msg_type, src_ip, src_port);
        }
    }
}

void ZmqManager::sendTopic(const std::string& topic, const std::string& msg_type,
    int max_freq, const std::string& src_ip, int src_port) {
    zmq::socket_t pub_socket(context_, ZMQ_PUB);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);
    pub_socket.bind(address);
    pub_sockets_.emplace_back(std::move(pub_socket));
    zmq::socket_t& current_socket = pub_sockets_.back();  // Reference to the specific socket

    ros::NodeHandle nh;
    ros::Subscriber sub;

    if (msg_type == "sensor_msgs/Imu") {
        sub = nh.subscribe<sensor_msgs::Imu>(topic, 1, [this, &current_socket, max_freq](const sensor_msgs::Imu::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "geometry_msgs/Twist") {
        sub = nh.subscribe<geometry_msgs::Twist>(topic, 1, [this, &current_socket, max_freq](const geometry_msgs::Twist::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "std_msgs/String") {
        sub = nh.subscribe<std_msgs::String>(topic, 1, [this, &current_socket, max_freq](const std_msgs::String::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "nav_msgs/Odometry") {
        sub = nh.subscribe<nav_msgs::Odometry>(topic, 1, [this, &current_socket, max_freq](const nav_msgs::Odometry::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "sensor_msgs/LaserScan") {
        sub = nh.subscribe<sensor_msgs::LaserScan>(topic, 1, [this, &current_socket, max_freq](const sensor_msgs::LaserScan::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "sensor_msgs/Image") {
        sub = nh.subscribe<sensor_msgs::Image>(topic, 1, [this, &current_socket, max_freq](const sensor_msgs::Image::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "geometry_msgs/Pose") {
        sub = nh.subscribe<geometry_msgs::Pose>(topic, 1, [this, &current_socket, max_freq](const geometry_msgs::Pose::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "geometry_msgs/Point") {
        sub = nh.subscribe<geometry_msgs::Point>(topic, 1, [this, &current_socket, max_freq](const geometry_msgs::Point::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "std_msgs/Float32") {
        sub = nh.subscribe<std_msgs::Float32>(topic, 1, [this, &current_socket, max_freq](const std_msgs::Float32::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else if (msg_type == "std_msgs/Int32") {
        sub = nh.subscribe<std_msgs::Int32>(topic, 1, [this, &current_socket, max_freq](const std_msgs::Int32::ConstPtr& msg) {
            auto buffer = serializeMsg(*msg);
            zmq::message_t zmq_msg(buffer.size());
            memcpy(zmq_msg.data(), buffer.data(), buffer.size());
            current_socket.send(zmq_msg, zmq::send_flags::none);
            ros::Rate(max_freq).sleep();
        });
    } else {
        // 对于不支持的消息类型，打印错误信息
        ROS_ERROR("Unsupported message type in sendTopic: %s", msg_type.c_str());
        return;
    }

    subscribers_.push_back(sub);
}

void ZmqManager::recvTopic(const std::string& topic, const std::string& msg_type,
    const std::string& src_ip, int src_port) {
    zmq::socket_t sub_socket(context_, ZMQ_SUB);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);
    sub_socket.connect(address);
    sub_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub_sockets_.emplace_back(std::move(sub_socket));
    zmq::socket_t& current_socket = sub_sockets_.back();  // Reference to the specific socket

    ros::NodeHandle nh;
    ros::Publisher pub;

    if (msg_type == "sensor_msgs/Imu") {
        pub = nh.advertise<sensor_msgs::Imu>(topic, 1);
    } else if (msg_type == "geometry_msgs/Twist") {
        pub = nh.advertise<geometry_msgs::Twist>(topic, 1);
    } else if (msg_type == "std_msgs/String") {
        pub = nh.advertise<std_msgs::String>(topic, 1);
    } else if (msg_type == "nav_msgs/Odometry") {
        pub = nh.advertise<nav_msgs::Odometry>(topic, 1);
    } else if (msg_type == "sensor_msgs/LaserScan") {
        pub = nh.advertise<sensor_msgs::LaserScan>(topic, 1);
    } else if (msg_type == "sensor_msgs/Image") {
        pub = nh.advertise<sensor_msgs::Image>(topic, 1);
    } else if (msg_type == "geometry_msgs/Pose") {
        pub = nh.advertise<geometry_msgs::Pose>(topic, 1);
    } else if (msg_type == "geometry_msgs/Point") {
        pub = nh.advertise<geometry_msgs::Point>(topic, 1);
    } else if (msg_type == "std_msgs/Float32") {
        pub = nh.advertise<std_msgs::Float32>(topic, 1);
    } else if (msg_type == "std_msgs/Int32") {
        pub = nh.advertise<std_msgs::Int32>(topic, 1);
    } else {
        // 对于不支持的消息类型，打印错误信息
        ROS_ERROR("Unsupported message type in recvTopic: %s", msg_type.c_str());
        return;
    }

    recv_threads_.emplace_back([this, &current_socket, pub, msg_type]() {
        while (ros::ok()) {
            zmq::message_t zmq_msg;
            current_socket.recv(zmq_msg, zmq::recv_flags::none);
            if (msg_type == "sensor_msgs/Imu") {
                auto msg = deserializeMsg<sensor_msgs::Imu>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "geometry_msgs/Twist") {
                auto msg = deserializeMsg<geometry_msgs::Twist>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "std_msgs/String") {
                auto msg = deserializeMsg<std_msgs::String>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "nav_msgs/Odometry") {
                auto msg = deserializeMsg<nav_msgs::Odometry>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "sensor_msgs/LaserScan") {
                auto msg = deserializeMsg<sensor_msgs::LaserScan>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "sensor_msgs/Image") {
                auto msg = deserializeMsg<sensor_msgs::Image>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "geometry_msgs/Pose") {
                auto msg = deserializeMsg<geometry_msgs::Pose>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "geometry_msgs/Point") {
                auto msg = deserializeMsg<geometry_msgs::Point>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "std_msgs/Float32") {
                auto msg = deserializeMsg<std_msgs::Float32>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            } else if (msg_type == "std_msgs/Int32") {
                auto msg = deserializeMsg<std_msgs::Int32>(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
                pub.publish(msg);
            }
        }
    });
}

} // namespace multibotnet