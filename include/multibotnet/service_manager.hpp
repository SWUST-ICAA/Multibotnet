// service_manager.hpp

#ifndef SERVICE_MANAGER_HPP
#define SERVICE_MANAGER_HPP

#include <zmq.hpp>          // ZeroMQ库
#include <string>           // 字符串支持
#include <ros/ros.h>        // ROS核心库
#include <std_srvs/SetBool.h> // SetBool服务
#include <nav_msgs/GetPlan.h> // GetPlan服务
#include <thread>           // 线程支持
#include <yaml-cpp/yaml.h>  // YAML解析库

namespace multibotnet {

class ServiceManager {
public:
    // 构造函数，初始化ZeroMQ上下文
    ServiceManager();
    
    // 析构函数，清理资源
    ~ServiceManager();

    // 初始化函数，加载配置文件并启动服务
    void init(const std::string& config_file);

private:
    zmq::context_t context_;                    // ZeroMQ上下文
    std::vector<zmq::socket_t> rep_sockets_;    // REP套接字列表，用于请求-响应通信
    std::vector<ros::ServiceServer> service_servers_; // ROS服务服务器列表
    std::vector<std::thread> service_threads_;  // 服务线程列表

    // 启动服务，设置ZeroMQ和ROS服务
    void startService(const std::string& service_name, const std::string& service_type,
                      const std::string& src_ip, int src_port);
    
    // 处理SetBool服务请求
    bool handleSetBool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    
    // 处理GetPlan服务请求
    bool handleGetPlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);
    
    // 显示配置信息到终端
    void displayConfig(const YAML::Node& config);
    
    // 获取本机IP地址
    std::string getLocalIP();
};

} // namespace multibotnet

#endif // SERVICE_MANAGER_HPP