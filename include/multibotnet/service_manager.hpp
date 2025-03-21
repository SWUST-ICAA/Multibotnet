#ifndef SERVICE_MANAGER_HPP
#define SERVICE_MANAGER_HPP

#include <zmq.hpp>
#include <string>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/GetPlan.h>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <map>
#include <queue>
#include <mutex>

namespace multibotnet {

class ServiceManager {
public:
    ServiceManager();
    ~ServiceManager();

    void init(const std::string& config_file);

    // 模板函数，用于调用远程服务
    template<typename ServiceType>
    bool callService(const std::string& service_name, 
                     typename ServiceType::Request& req, 
                     typename ServiceType::Response& res);

private:
    zmq::context_t context_;                    // ZeroMQ 上下文
    std::vector<zmq::socket_t> rep_sockets_;    // REP 套接字列表，用于提供服务
    std::vector<ros::ServiceServer> service_servers_; // ROS 服务服务器列表
    std::vector<std::thread> service_threads_;  // 服务线程列表
    std::map<std::string, zmq::socket_t> req_sockets_; // REQ 套接字映射，用于请求服务

    // 请求队列及其同步机制
    struct ServiceRequest {
        std::string service_type;
        zmq::message_t request;
        zmq::socket_t* rep_socket; // 指向对应的 REP 套接字，用于发送响应
    };
    std::queue<ServiceRequest> request_queue_;
    std::mutex queue_mutex_;

    void startProvideService(const std::string& service_name, 
                            const std::string& service_type,
                            const std::string& bind_address, 
                            int port);
    void startRequestService(const std::string& service_name, 
                            const std::string& service_type,
                            const std::string& connect_address, 
                            int port);

    // ROS 服务回调函数
    bool handleSetBool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool handleGetPlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);

    // 处理请求队列的函数
    void processRequests();

    void displayConfig(const YAML::Node& config);
    std::string getLocalIP();
};

} // namespace multibotnet

#endif // SERVICE_MANAGER_HPP