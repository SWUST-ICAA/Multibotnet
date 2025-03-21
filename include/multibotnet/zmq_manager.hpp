// zmq_manager.hpp

#ifndef ZMQ_MANAGER_HPP
#define ZMQ_MANAGER_HPP

#include <zmq.hpp>         // ZeroMQ库
#include <string>          // 字符串支持
#include <list>            // 列表容器
#include <vector>          // 向量容器
#include <thread>          // 线程支持
#include <mutex>           // 互斥锁
#include <ros/ros.h>       // ROS核心库
#include <yaml-cpp/yaml.h> // YAML解析库

namespace multibotnet {

class ZmqManager {
public:
    // 构造函数，初始化ZeroMQ上下文
    ZmqManager();
    
    // 析构函数，清理资源
    ~ZmqManager();

    // 初始化函数，加载配置文件并启动话题管理
    void init(const std::string& config_file);

private:
    zmq::context_t context_;                  // ZeroMQ上下文
    std::list<zmq::socket_t> pub_sockets_;    // PUB套接字列表，用于发送消息
    std::list<zmq::socket_t> sub_sockets_;    // SUB套接字列表，用于接收消息
    std::vector<std::thread> recv_threads_;   // 接收线程列表
    std::vector<ros::Subscriber> subscribers_; // ROS订阅者列表
    std::mutex mutex_;                        // 互斥锁，用于线程安全

    // 发送话题，设置发布者并通过ZeroMQ发送消息
    void sendTopic(const std::string& topic, const std::string& msg_type, int max_freq,
                   const std::string& src_ip, int src_port);
    
    // 接收话题，设置订阅者并通过ROS发布接收到的消息
    void recvTopic(const std::string& topic, const std::string& msg_type,
                   const std::string& src_ip, int src_port);
    
    // 显示配置信息到终端
    void displayConfig(const YAML::Node& config);
    
    // 获取本机IP地址
    std::string getLocalIP();
};

} // namespace multibotnet

#endif // ZMQ_MANAGER_HPP