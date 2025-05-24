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
    
    // 启动所有服务提供线程
    for (auto& provide_service : provide_services_) {
        if (provide_service->thread_should_start && !provide_service->service_thread.joinable()) {
            provide_service->active = true;
            provide_service->service_thread = std::thread(
                &ServiceManager::serviceProviderLoop, this, provide_service.get());
        }
    }
    
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

void ServiceManager::printStatistics() const {
    auto stats = getStatistics();
    
    std::cout << BLUE << "========= Service Statistics =========" << RESET << std::endl;
    
    // 分别收集提供和请求的统计信息
    std::vector<std::pair<std::string, Statistics>> provide_stats;
    std::vector<std::pair<std::string, Statistics>> request_stats;
    
    for (const auto& pair : stats) {
        if (pair.first.find("provide:") == 0) {
            provide_stats.push_back(pair);
        } else if (pair.first.find("request:") == 0) {
            request_stats.push_back(pair);
        }
    }
    
    // 打印请求服务统计
    for (const auto& pair : request_stats) {
        const auto& name = pair.first;
        const auto& stat = pair.second;
        
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - stat.start_time).count();
        
        double calls_per_sec = elapsed > 0 ? 
            static_cast<double>(stat.messages_sent) / elapsed : 0;
        
        std::cout << GREEN << name << ":" << RESET << std::endl;
        std::cout << "  Calls: " << YELLOW << "sent=" << stat.messages_sent 
                  << " (" << calls_per_sec << " calls/s)" << RESET << std::endl;
        if (stat.errors > 0) {
            std::cout << RED << "  Errors: " << stat.errors << RESET << std::endl;
        }
    }
    
    // 打印提供服务统计
    for (const auto& pair : provide_stats) {
        const auto& name = pair.first;
        const auto& stat = pair.second;
        
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - stat.start_time).count();
        
        double calls_per_sec = elapsed > 0 ? 
            static_cast<double>(stat.messages_received) / elapsed : 0;
        
        std::cout << GREEN << name << ":" << RESET << std::endl;
        std::cout << "  Calls: " << YELLOW << "recv=" << stat.messages_received 
                  << " (" << calls_per_sec << " calls/s)" << RESET << std::endl;
        if (stat.errors > 0) {
            std::cout << RED << "  Errors: " << stat.errors << RESET << std::endl;
        }
    }
    
    std::cout << BLUE << "=====================================" << RESET << std::endl;
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
    // 使用字符串流收集所有输出
    std::stringstream ss;
    
    // 添加空行分隔
    ss << std::endl;
    
    ss << BLUE << "=========== Service Node Configuration ===========" << RESET << std::endl;
    
    // 显示提供服务
    ss << BLUE << "Provide Services:" << RESET << std::endl;
    for (const auto& provide_service : provide_services_) {
        const auto& config = provide_service->config;
        std::string address_key = (config.address == "*") ? "self" : config.address;
        ss << "  " << GREEN << config.service_name << RESET 
           << " bind: " << address_key << ":" << config.port << std::endl;
    }
    
    // 显示请求服务
    ss << std::endl << BLUE << "Request Services:" << RESET << std::endl;
    for (const auto& pair : request_services_) {
        const auto& config = pair.second->config;
        ss << "  " << GREEN << config.service_name << RESET 
           << " -> " << config.address << ":" << config.port << std::endl;
    }
    
    ss << BLUE << "=================================================" << RESET << std::endl;
    
    // 原子化输出所有内容
    std::cout << ss.str() << std::flush;
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
        info->active = false;  // 初始化为false，在start()中设置为true
        info->thread_should_start = true;  // 标记线程应该启动
        info->stats.start_time = std::chrono::steady_clock::now();
        
        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::REP);
        
        // 设置套接字选项
        int linger = 0;
        info->transport->setOption(ZMQ_LINGER, linger);
        
        // 绑定地址
        std::string bind_address = "tcp://" + info->config.address + ":" + 
                                  std::to_string(config.port);
        if (!info->transport->bind(bind_address)) {
            LOG_ERROR("Failed to bind service " + config.service_name);
            return;
        }
        
        LOG_INFOF("Successfully bound service %s to %s", 
                 config.service_name.c_str(), bind_address.c_str());
        
        // 不在这里启动线程，而是在start()函数中启动
        
        provide_services_.push_back(std::move(info));
        LOG_INFO("Setup provide service: " + config.service_name + 
                " on " + bind_address);
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup provide service " + config.service_name + 
                 ": " + e.what());
    }
}

void ServiceManager::serviceProviderLoop(ProvideServiceInfo* info) {
    LOG_INFOF("Service provider loop started for %s", info->config.service_name.c_str());
    
    while (info->active && running_ && ros::ok()) {
        try {
            // 接收请求
            std::vector<uint8_t> request_data;
            
            // 添加调试信息
            LOG_DEBUGF("Waiting for request on %s...", info->config.service_name.c_str());
            
            if (!info->transport->receive(request_data, 100)) {  // 100ms超时
                // 这是正常的超时，不需要记录
                continue;
            }
            
            LOG_INFOF("Received request for %s (%zu bytes)", 
                     info->config.service_name.c_str(), request_data.size());
            
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
    
    LOG_INFOF("Service provider loop stopped for %s", info->config.service_name.c_str());
}

void ServiceManager::handleServiceRequest(ProvideServiceInfo* info, 
                                        const std::vector<uint8_t>& request) {
    try {
        LOG_DEBUGF("Handling service request for %s", info->config.service_name.c_str());
        
        // 创建服务代理来调用本地 ROS 服务
        auto proxy = service_factory_->createServiceProxy(
            info->config.service_name, info->config.service_type);
        
        if (!proxy) {
            LOG_ERROR("Failed to create service proxy for " + info->config.service_name);
            // 发送错误响应 - 对于REP套接字，必须发送响应
            std::vector<uint8_t> error_response = {0};  // 空响应表示错误
            info->transport->send(error_response);
            info->stats.errors++;
            return;
        }
        
        LOG_DEBUGF("Calling service proxy for %s", info->config.service_name.c_str());
        
        // 调用服务代理
        auto response_data = proxy(request);
        
        if (response_data.empty()) {
            LOG_ERROR("Service proxy returned empty response");
            // 发送默认错误响应
            response_data = {0};  // 空响应表示错误
        }
        
        LOG_DEBUGF("Sending service response for %s (%zu bytes)", 
                  info->config.service_name.c_str(), response_data.size());
        
        // 发送响应 - REP套接字必须发送响应
        if (!info->transport->send(response_data)) {
            LOG_ERROR("Failed to send service response");
            info->stats.errors++;
        } else {
            info->stats.messages_sent++;
            info->stats.bytes_sent += response_data.size();
            LOG_INFOF("Service %s handled successfully", info->config.service_name.c_str());
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error handling service request: " + std::string(e.what()));
        info->stats.errors++;
        // 即使出错也要发送响应，否则REP套接字会进入错误状态
        std::vector<uint8_t> error_response = {0};
        info->transport->send(error_response);
    }
}

void ServiceManager::setupRequestService(const ServiceConfig& config) {
    try {
        auto info = std::make_unique<RequestServiceInfo>();
        info->config = config;
        info->config.address = resolveAddress(config.address);
        info->stats.start_time = std::chrono::steady_clock::now();
        
        // 不使用连接池，为每个服务创建独立的REQ套接字
        // 连接池对REQ-REP模式不适用
        
        // 特殊处理localhost
        std::string connect_ip = info->config.address;
        if (connect_ip == "127.0.0.1" || connect_ip == "localhost") {
            connect_ip = "127.0.0.1";
        }
        
        // 创建ROS服务代理
        createROSServiceProxy(config.service_name, config.service_type, info.get());
        
        request_services_[config.service_name] = std::move(info);
        LOG_INFO("Setup request service: " + config.service_name + 
                " -> tcp://" + connect_ip + ":" + std::to_string(config.port));
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup request service " + config.service_name + 
                 ": " + e.what());
    }
}

void ServiceManager::createROSServiceProxy(const std::string& service_name,
                                         const std::string& service_type,
                                         RequestServiceInfo* info) {
    // 创建远程服务调用的lambda函数
    auto remote_handler = [this, info](const std::vector<uint8_t>& request) 
        -> std::vector<uint8_t> {
        std::vector<uint8_t> response;
        callRemoteService(info, request, response, info->config.timeout_ms);
        return response;
    };
    
    // 使用service_factory创建ROS服务服务器
    info->server = service_factory_->createServiceServer(
        service_name, service_type, remote_handler);
    
    if (info->server) {
        LOG_INFOF("Created ROS service proxy: %s (type: %s)", 
                 service_name.c_str(), service_type.c_str());
    } else {
        LOG_ERRORF("Failed to create ROS service proxy: %s", service_name.c_str());
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
            // 为每次调用创建新的REQ套接字
            // REQ-REP模式要求严格的请求-响应配对
            auto transport = std::make_shared<ZmqTransport>(
                context_, ZmqTransport::SocketType::REQ);
            
            // 设置套接字选项
            int linger = 0;
            transport->setOption(ZMQ_LINGER, linger);
            
            // 连接到服务器
            if (!transport->connect(connect_address)) {
                LOG_ERRORF("Failed to connect to %s", connect_address.c_str());
                info->stats.errors++;
                continue;
            }
            
            // 给连接一点时间建立（ZMQ的异步连接特性）
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            LOG_DEBUGF("Sending service request to %s (%zu bytes)", 
                      connect_address.c_str(), request.size());
            
            // 发送请求
            if (!transport->send(request)) {
                LOG_ERROR("Failed to send service request");
                info->stats.errors++;
                continue;
            }
            info->stats.messages_sent++;
            info->stats.bytes_sent += request.size();
            
            // 接收响应
            if (!transport->receive(response, timeout_ms)) {
                LOG_ERROR("Service call timeout");
                info->stats.errors++;
                continue;
            }
            
            LOG_DEBUGF("Received service response from %s (%zu bytes)", 
                      connect_address.c_str(), response.size());
            
            // 检查响应是否为空或错误标记
            if (response.empty() || (response.size() == 1 && response[0] == 0)) {
                LOG_ERROR("Received error response");
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
            
            lock.lock();
        }
    }
}

} // namespace multibotnet