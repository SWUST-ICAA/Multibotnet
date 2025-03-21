#include <multibotnet/service_manager.hpp> 
#include <multibotnet/ros_sub_pub.hpp>    
#include <yaml-cpp/yaml.h>               
#include <thread>                        
#include <ifaddrs.h>                  
#include <netinet/in.h>             
#include <arpa/inet.h>                    
#include <cstring>                         
#include <zmq.hpp>                         
#include <iostream>                        


#define RESET   "\033[0m"  
#define RED     "\033[31m" 
#define GREEN   "\033[32m" 
#define YELLOW  "\033[33m" 
#define BLUE    "\033[34m" 

namespace multibotnet {

// 构造函数，初始化ZeroMQ上下文
ServiceManager::ServiceManager() : context_(1) {}

// 析构函数，确保所有服务线程在对象销毁前结束
ServiceManager::~ServiceManager() {
    for (auto& th : service_threads_) {
        if (th.joinable()) th.join(); // 等待线程结束
    }
}

// 初始化函数，加载配置文件并启动服务
void ServiceManager::init(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file); // 加载YAML配置文件
    displayConfig(config); // 显示配置信息

    // 解析IP映射
    std::map<std::string, std::string> ip_map;
    for (const auto& ip : config["IP"]) {
        ip_map[ip.first.as<std::string>()] = ip.second.as<std::string>();
    }

    // 处理services配置
    if (config["services"]) {
        size_t num_services = config["services"].size();
        rep_sockets_.reserve(num_services);      // 预分配REP套接字空间
        service_servers_.reserve(num_services);  // 预分配服务服务器空间
        service_threads_.reserve(num_services);  // 预分配线程空间

        for (const auto& service : config["services"]) {
            std::string service_name = service["service_name"].as<std::string>(); // 服务名称
            std::string service_type = service["service_type"].as<std::string>(); // 服务类型
            std::string src_ip = service["srcIP"].as<std::string>();              // 源IP
            int src_port = service["srcPort"].as<int>();                          // 源端口

            // 处理srcIP
            if (src_ip == "self") {
                src_ip = "*"; // 绑定到所有接口
            } else if (ip_map.find(src_ip) != ip_map.end()) {
                src_ip = ip_map[src_ip]; // 使用配置中的IP映射
            } else {
                ROS_ERROR("Invalid srcIP '%s' for service %s, skipping", src_ip.c_str(), service_name.c_str());
                continue; // 跳过无效配置
            }

            // 启动服务
            startService(service_name, service_type, src_ip, src_port);
        }
    }
}

// 获取本机IP地址
std::string ServiceManager::getLocalIP() {
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

// 启动一个服务
void ServiceManager::startService(const std::string& service_name, const std::string& service_type,
                                  const std::string& src_ip, int src_port) {
    // 创建ZeroMQ REP套接字，用于请求-响应通信
    zmq::socket_t rep_socket(context_, ZMQ_REP);
    std::string address = "tcp://" + src_ip + ":" + std::to_string(src_port);

    // 绑定套接字到指定地址
    rep_socket.bind(address);
    rep_sockets_.emplace_back(std::move(rep_socket));
    auto& current_socket = rep_sockets_.back(); // 获取当前套接字引用

    // 创建ROS服务服务器
    ros::NodeHandle nh;
    ros::ServiceServer server;

    // 根据服务类型注册服务回调函数
    if (service_type == "std_srvs/SetBool") {
        server = nh.advertiseService(service_name, &ServiceManager::handleSetBool, this);
    } else if (service_type == "nav_msgs/GetPlan") {
        server = nh.advertiseService(service_name, &ServiceManager::handleGetPlan, this);
    } else {
        ROS_ERROR("Unsupported service type: %s", service_type.c_str());
        return; // 不支持的服务类型，直接返回
    }
    service_servers_.push_back(server); // 保存服务服务器对象

    // 启动服务线程，处理ZeroMQ请求
    service_threads_.emplace_back([this, &current_socket, service_name, service_type]() {
        while (ros::ok()) {
            zmq::pollitem_t items[] = {{static_cast<void*>(current_socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, 100); // 100ms超时轮询

            if (!ros::ok()) break; // ROS关闭时退出循环

            if (items[0].revents & ZMQ_POLLIN) { // 有数据可读
                zmq::message_t request;
                if (current_socket.recv(request)) { // 接收请求
                    if (service_type == "std_srvs/SetBool") {
                        // 反序列化请求并处理
                        auto req = deserializeMsg<std_srvs::SetBool::Request>(
                            static_cast<uint8_t*>(request.data()), request.size());
                        std_srvs::SetBool::Response res;
                        handleSetBool(req, res);
                        // 序列化响应并发送
                        auto buffer = serializeMsg(res);
                        zmq::message_t reply(buffer.size());
                        memcpy(reply.data(), buffer.data(), buffer.size());
                        current_socket.send(reply);
                    } else if (service_type == "nav_msgs/GetPlan") {
                        // 反序列化请求并处理
                        auto req = deserializeMsg<nav_msgs::GetPlan::Request>(
                            static_cast<uint8_t*>(request.data()), request.size());
                        nav_msgs::GetPlan::Response res;
                        handleGetPlan(req, res);
                        // 序列化响应并发送
                        auto buffer = serializeMsg(res);
                        zmq::message_t reply(buffer.size());
                        memcpy(reply.data(), buffer.data(), buffer.size());
                        current_socket.send(reply);
                    }
                }
            }
        }
    });
}

// 处理SetBool服务请求
bool ServiceManager::handleSetBool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    res.success = true; // 设置成功标志
    res.message = "Service called with data: " + std::to_string(req.data); // 返回请求数据
    return true; // 表示处理成功
}

// 处理GetPlan服务请求
bool ServiceManager::handleGetPlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res) {
    res.plan.header.stamp = ros::Time::now(); // 设置当前时间戳
    res.plan.header.frame_id = "map";         // 设置坐标系
    return true; // 表示处理成功
}

// 显示配置信息到终端
void ServiceManager::displayConfig(const YAML::Node& config) {
    std::cout << BLUE << "-------services-------" << RESET << std::endl;
    if (config["services"]) {
        for (const auto& service : config["services"]) {
            std::string service_name = service["service_name"].as<std::string>();
            std::string src_ip = service["srcIP"].as<std::string>();
            std::cout << GREEN << service_name << "  (at " << src_ip << ")" << RESET << std::endl;
        }
    }
}

} // namespace multibotnet