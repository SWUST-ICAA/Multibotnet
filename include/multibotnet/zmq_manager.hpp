// zmq_manager.hpp

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
};

} // namespace multibotnet

#endif // ZMQ_MANAGER_HPP