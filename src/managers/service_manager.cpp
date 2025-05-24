#include "multibotnet/managers/service_manager.hpp"
#include "multibotnet/utils/logger.hpp"
#include "multibotnet/utils/config_parser.hpp"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

namespace multibotnet {

ServiceManager::ServiceManager() 
    : context_(1), running_(false) {
    service_factory_ = std::make_unique<ServiceFactory>();
    thread_pool_ = std::make_unique<ThreadPool>();
}

ServiceManager::~ServiceManager() {
    stop();
}

bool ServiceManager::init(const std::string& config_file) {
    if (!loadConfig(config_file)) {
        return false;
    }
    
    displayConfig();
    return true;
}

void ServiceManager::start() {
    if (running_) {
        LOG_WARN("ServiceManager already running");
        return;
    }
    
    running_ = true;
    
    // 启动请求处理线程
    request_processor_thread_ = std::thread(
        &ServiceManager::processRequestQueue, this);
    
    LOG_INFO("ServiceManager started");
}

void ServiceManager::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // 通知所有等待的线程
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_cv_.notify_all();
    }
    
    // 等待请求处理线程结束
    if (request_processor_thread_.joinable()) {
        request_processor_thread_.join();
    }
    
    // 停止所有服务线程
    for (auto& provide_service : provide_services_) {
        provide_service->active = false;
        if (provide_service->service_thread.joinable()) {
            provide_service->service_thread.join();
        }
    }
    
    LOG_INFO("ServiceManager stopped");
}

bool ServiceManager::callService(const std::string& service_name,
                               const std::vector<uint8_t>& request,
                               std::vector<uint8_t>& response,
                               int timeout_ms) {
    auto it = request_services_.find(service_name);
    if (it == request_services_.end()) {
        LOG_ERROR("Service " + service_name + " not found in request_services");
        return false;
    }
    
    return callRemoteService(it->second.get(), request, response, timeout_ms);
}

std::unordered_map<std::string, Statistics> ServiceManager::getStatistics() const {
    std::unordered_map<std::string, Statistics> stats;
    
    // 收集提供服务统计
    for (const auto& provide_service : provide_services_) {
        stats["provide:" + provide_service->config.service_name] = 
            provide_service->stats;
    }
    
    // 收集请求服务统计
    for (const auto& pair : request_services_) {
        stats["request:" + pair.first] = pair.second->stats;
    }
    
    return stats;
}

bool ServiceManager::loadConfig(const std::string& config_file) {
    try {
        ConfigParser parser;
        if (!parser.parse(config_file)) {
            LOG_ERROR("Failed to parse config file: " + parser.getError());
            return false;
        }
        
        if (!parser.validate()) {
            LOG_ERROR("Invalid configuration: " + parser.getError());
            return false;
        }
        
        // 加载IP映射
        ip_map_ = parser.getIpMap();
        
        // 设置提供服务
        for (const auto& config : parser.getProvideServices()) {
            setupProvideService(config);
        }
        
        // 设置请求服务
        for (const auto& config : parser.getRequestServices()) {
            setupRequestService(config);
        }
        
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to load config: " + std::string(e.what()));
        return false;
    }
}

void ServiceManager::displayConfig() {
    // 显示提供服务
    std::cout << BLUE << "-------provide_services-------" << RESET << std::endl;
    for (const auto& provide_service : provide_services_) {
        const auto& config = provide_service->config;
        std::string address_key = (config.address == "*") ? "self" : config.address;
        std::cout << GREEN << config.service_name << " (bind at " 
                  << address_key << ":" << config.port << ")" << RESET << std::endl;
    }
    
    // 显示请求服务
    std::cout << BLUE << "-------request_services-------" << RESET << std::endl;
    for (const auto& pair : request_services_) {
        const auto& config = pair.second->config;
        std::cout << GREEN << config.service_name << " (connect to " 
                  << config.address << ":" << config.port << ")" << RESET << std::endl;
    }
}

std::string ServiceManager::resolveAddress(const std::string& address_key) {
    if (address_key == "self" || address_key == "*") {
        return "*";
    }
    
    auto it = ip_map_.find(address_key);
    if (it != ip_map_.end()) {
        return it->second;
    }
    
    return address_key;
}

std::string ServiceManager::getLocalIP() {
    struct ifaddrs *ifaddr;
    std::string local_ip = "127.0.0.1";
    
    if (getifaddrs(&ifaddr) == -1) {
        LOG_ERROR("Failed to get local IP address");
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
    return local_ip;
}

void ServiceManager::setupProvideService(const ServiceConfig& config) {
    try {
        auto info = std::make_unique<ProvideServiceInfo>();
        info->config = config;
        info->config.address = resolveAddress(config.address);
        info->active = true;
        info->stats.start_time = std::chrono::steady_clock::now();
        
        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::REP);
        
        // 设置套接字选项
        int linger = 0;
        info->transport->setOption(ZMQ_LINGER, linger);
        
        // 设置接收超时
        int rcv_timeout = 100;  // 100ms
        info->transport->setOption(ZMQ_RCVTIMEO, rcv_timeout);
        
        // 绑定地址
        std::string bind_address = "tcp://" + info->config.address + ":" + 
                                  std::to_string(config.port);
        if (!info->transport->bind(bind_address)) {
            LOG_ERROR("Failed to bind service " + config.service_name);
            return;
        }
        
        // 创建服务处理函数，但不创建实际的 ROS 服务
        // 因为我们要支持任意服务类型，所以完全通过 ZMQ 处理
        LOG_INFO("Service provider registered for " + config.service_name + 
                " (type: " + config.service_type + ")");
        
        // 启动服务线程
        info->service_thread = std::thread(
            &ServiceManager::serviceProviderLoop, this, info.get());
        
        provide_services_.push_back(std::move(info));
        LOG_INFO("Setup provide service: " + config.service_name);
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup provide service " + config.service_name + 
                 ": " + e.what());
    }
}

void ServiceManager::serviceProviderLoop(ProvideServiceInfo* info) {
    while (info->active && running_ && ros::ok()) {
        try {
            // 接收请求
            std::vector<uint8_t> request_data;
            if (!info->transport->receive(request_data, 100)) {  // 100ms超时
                continue;
            }
            
            info->stats.messages_received++;
            info->stats.bytes_received += request_data.size();
            
            // 处理请求
            handleServiceRequest(info, request_data);
            
        } catch (const std::exception& e) {
            LOG_ERROR("Error in service provider loop: " + std::string(e.what()));
            info->stats.errors++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void ServiceManager::handleServiceRequest(ProvideServiceInfo* info, 
                                        const std::vector<uint8_t>& request) {
    try {
        // 创建服务代理来调用本地 ROS 服务
        auto proxy = service_factory_->createServiceProxy(
            info->config.service_name, info->config.service_type);
        
        if (!proxy) {
            LOG_ERROR("Failed to create service proxy for " + info->config.service_name);
            // 发送错误响应
            std::vector<uint8_t> error_response = {0, 0, 0, 0, 0};  // 默认错误响应
            info->transport->send(error_response);
            info->stats.errors++;
            return;
        }
        
        // 调用服务代理（它会处理序列化/反序列化）
        auto response_data = proxy(request);
        
        if (response_data.empty()) {
            LOG_ERROR("Service proxy returned empty response");
            // 发送默认错误响应
            response_data = {0, 0, 0, 0, 0};  // success=false, empty message
        }
        
        // 发送响应
        if (!info->transport->send(response_data)) {
            LOG_ERROR("Failed to send service response");
            info->stats.errors++;
        } else {
            info->stats.messages_sent++;
            info->stats.bytes_sent += response_data.size();
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error handling service request: " + std::string(e.what()));
        info->stats.errors++;
        // 发送错误响应
        std::vector<uint8_t> error_response = {0, 0, 0, 0, 0};  // success=false, empty message
        info->transport->send(error_response);
    }
}

void ServiceManager::setupRequestService(const ServiceConfig& config) {
    try {
        auto info = std::make_unique<RequestServiceInfo>();
        info->config = config;
        info->config.address = resolveAddress(config.address);
        info->stats.start_time = std::chrono::steady_clock::now();
        
        // 创建连接池
        ConnectionPoolConfig pool_config;
        pool_config.min_connections = 1;
        pool_config.max_connections = 5;
        pool_config.connection_timeout_ms = config.timeout_ms;
        
        info->connection_pool = std::make_shared<ConnectionPool>(
            context_, ZmqTransport::SocketType::REQ, pool_config);
        
        // 特殊处理localhost
        std::string connect_ip = info->config.address;
        if (connect_ip == "127.0.0.1" || connect_ip == "localhost") {
            connect_ip = "127.0.0.1";
        }
        
        // 预创建连接
        std::string connect_address = "tcp://" + connect_ip + ":" + 
                                     std::to_string(config.port);
        info->connection_pool->preCreateConnections(connect_address, 1);
        
        request_services_[config.service_name] = std::move(info);
        LOG_INFO("Setup request service: " + config.service_name + 
                " (type: " + config.service_type + ")");
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup request service " + config.service_name + 
                 ": " + e.what());
    }
}

bool ServiceManager::callRemoteService(RequestServiceInfo* info,
                                     const std::vector<uint8_t>& request,
                                     std::vector<uint8_t>& response,
                                     int timeout_ms) {
    return retryServiceCall(info, request, response, 
                           info->config.max_retries, timeout_ms);
}

bool ServiceManager::retryServiceCall(RequestServiceInfo* info,
                                    const std::vector<uint8_t>& request,
                                    std::vector<uint8_t>& response,
                                    int retries,
                                    int timeout_ms) {
    // 特殊处理localhost
    std::string connect_ip = info->config.address;
    if (connect_ip == "127.0.0.1" || connect_ip == "localhost") {
        connect_ip = "127.0.0.1";
    }
    
    std::string connect_address = "tcp://" + connect_ip + ":" + 
                                 std::to_string(info->config.port);
    
    for (int attempt = 0; attempt <= retries; attempt++) {
        try {
            // 从连接池获取连接
            auto conn = info->connection_pool->getConnection(
                connect_address, timeout_ms);
            
            if (!conn.get()) {
                LOG_ERROR("Failed to get connection from pool");
                continue;
            }
            
            // 发送请求
            if (!conn->send(request)) {
                LOG_ERROR("Failed to send service request");
                info->stats.errors++;
                continue;
            }
            info->stats.messages_sent++;
            info->stats.bytes_sent += request.size();
            
            // 接收响应
            if (!conn->receive(response, timeout_ms)) {
                LOG_ERROR("Service call timeout");
                info->stats.errors++;
                continue;
            }
            
            // 检查响应是否为空（表示错误）
            if (response.empty()) {
                LOG_ERROR("Received empty response, service call failed");
                info->stats.errors++;
                
                if (attempt < retries) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(100 * (attempt + 1)));
                    continue;
                }
                return false;
            }
            
            info->stats.messages_received++;
            info->stats.bytes_received += response.size();
            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR("Exception in service call: " + std::string(e.what()));
            info->stats.errors++;
            
            if (attempt < retries) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100 * (attempt + 1)));
            }
        }
    }
    
    LOG_ERROR("Service call failed after all retries");
    return false;
}

void ServiceManager::processRequestQueue() {
    while (running_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        queue_cv_.wait_for(lock, std::chrono::milliseconds(100),
                          [this] { return !request_queue_.empty() || !running_; });
        
        if (!running_) break;
        
        while (!request_queue_.empty()) {
            auto request = std::move(request_queue_.front());
            request_queue_.pop();
            lock.unlock();
            
            // 处理请求（这里可以添加更复杂的逻辑）
            // 目前直接在服务线程中处理
            
            lock.lock();
        }
    }
}

} // namespace multibotnet