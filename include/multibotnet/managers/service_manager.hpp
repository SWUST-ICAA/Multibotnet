#ifndef MULTIBOTNET_MANAGERS_SERVICE_MANAGER_HPP
#define MULTIBOTNET_MANAGERS_SERVICE_MANAGER_HPP

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include "multibotnet/core/types.hpp"
#include "multibotnet/core/service_factory.hpp"
#include "multibotnet/transport/zmq_transport.hpp"
#include "multibotnet/transport/connection_pool.hpp"
#include "multibotnet/utils/thread_pool.hpp"

namespace multibotnet {

/**
 * @brief 服务管理器，负责管理所有的服务提供和请求
 */
class ServiceManager {
public:
    ServiceManager();
    ~ServiceManager();
    
    /**
     * @brief 初始化服务管理器
     * @param config_file 配置文件路径
     * @return 是否成功
     */
    bool init(const std::string& config_file);
    
    /**
     * @brief 启动所有服务处理
     */
    void start();
    
    /**
     * @brief 停止所有服务处理
     */
    void stop();
    
    /**
     * @brief 动态调用远程服务
     * @param service_name 服务名称
     * @param request 请求数据
     * @param response 响应数据
     * @param timeout_ms 超时时间
     * @return 是否成功
     */
    bool callService(const std::string& service_name,
                    const std::vector<uint8_t>& request,
                    std::vector<uint8_t>& response,
                    int timeout_ms = 5000);
    
    /**
     * @brief 获取服务调用统计
     * @return 统计数据
     */
    std::unordered_map<std::string, Statistics> getStatistics() const;

    /**
     * @brief 打印服务统计信息
     */
    void printStatistics() const;
    
private:
    // 提供服务信息
    struct ProvideServiceInfo {
        ServiceConfig config;
        std::shared_ptr<ZmqTransport> transport;
        ros::ServiceServer server;
        std::thread service_thread;
        std::atomic<bool> active;
        Statistics stats;
        bool thread_should_start;  // 新增：标记线程是否应该启动
    };
    
    // 请求服务信息
    struct RequestServiceInfo {
        ServiceConfig config;
        std::shared_ptr<ConnectionPool> connection_pool;
        ros::ServiceServer server;  // 添加ROS服务服务器（作为代理）
        std::mutex mutex;
        Statistics stats;
    };
    
    // 服务请求队列项
    struct ServiceRequest {
        std::vector<uint8_t> request_data;
        std::shared_ptr<ZmqTransport> transport;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    // 成员变量
    zmq::context_t context_;
    std::unique_ptr<ServiceFactory> service_factory_;
    std::unique_ptr<ThreadPool> thread_pool_;
    
    std::vector<std::unique_ptr<ProvideServiceInfo>> provide_services_;
    std::unordered_map<std::string, std::unique_ptr<RequestServiceInfo>> request_services_;
    
    std::unordered_map<std::string, std::string> ip_map_;
    std::atomic<bool> running_;
    
    // 请求处理队列
    std::queue<ServiceRequest> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread request_processor_thread_;
    
    // 内部方法
    bool loadConfig(const std::string& config_file);
    void displayConfig();
    std::string resolveAddress(const std::string& address_key);
    std::string getLocalIP();
    
    // 服务提供相关
    void setupProvideService(const ServiceConfig& config);
    void serviceProviderLoop(ProvideServiceInfo* info);
    void handleServiceRequest(ProvideServiceInfo* info, 
                            const std::vector<uint8_t>& request);
    
    // 服务请求相关
    void setupRequestService(const ServiceConfig& config);
    void createROSServiceProxy(const std::string& service_name,
                             const std::string& service_type,
                             RequestServiceInfo* info);
    bool callRemoteService(RequestServiceInfo* info,
                          const std::vector<uint8_t>& request,
                          std::vector<uint8_t>& response,
                          int timeout_ms);
    
    // 请求处理
    void processRequestQueue();
    
    // 错误处理和重试
    bool retryServiceCall(RequestServiceInfo* info,
                         const std::vector<uint8_t>& request,
                         std::vector<uint8_t>& response,
                         int retries,
                         int timeout_ms);
};

} // namespace multibotnet

#endif // MULTIBOTNET_MANAGERS_SERVICE_MANAGER_HPP