#ifndef ZMQ_MANAGER_HPP
#define ZMQ_MANAGER_HPP

#include <zmq.hpp>
#include <string>
#include <list>
#include <vector>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace multibotnet {

class ZmqManager {
public:
    ZmqManager();
    ~ZmqManager();

    void init(const std::string& config_file);

private:
    zmq::context_t context_;
    std::list<zmq::socket_t> pub_sockets_;
    std::list<zmq::socket_t> sub_sockets_;
    std::vector<std::thread> recv_threads_;
    std::vector<ros::Subscriber> subscribers_;
    std::mutex mutex_;

    // 添加用于频率控制的成员变量
    std::vector<ros::Time> send_t_last_;  // 上一次频率重置时间
    std::vector<int> send_num_;           // 当前周期内的发送计数
    std::vector<int> max_frequencies_;    // 每个话题的最大频率

    void sendTopic(const std::string& topic, 
                   const std::string& message_type, 
                   int max_frequency,
                   const std::string& bind_address, 
                   int port);
    void recvTopic(const std::string& topic, 
                   const std::string& message_type,
                   const std::string& connect_address, 
                   int port);
    void displayConfig(const YAML::Node& config);
    std::string getLocalIP();

    // 添加频率控制函数
    bool send_freq_control(int index);
};

} // namespace multibotnet

#endif // ZMQ_MANAGER_HPP