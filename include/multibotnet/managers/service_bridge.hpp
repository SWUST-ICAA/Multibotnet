#ifndef MULTIBOTNET_MANAGERS_SERVICE_BRIDGE_HPP
#define MULTIBOTNET_MANAGERS_SERVICE_BRIDGE_HPP

#include <ros/ros.h>
#include <ros/service.h>
#include <xmlrpcpp/XmlRpc.h>
#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace multibotnet {

/**
 * @brief 服务桥接器，用于动态创建服务代理
 * 
 * 这个类提供了一种通用的方式来桥接本地和远程服务，
 * 而不需要知道具体的服务类型
 */
class ServiceBridge {
public:
    using ServiceHandler = std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>;
    
    ServiceBridge();
    ~ServiceBridge();
    
    /**
     * @brief 创建本地服务代理，将远程服务暴露为本地服务
     * @param local_service_name 本地服务名称
     * @param service_type 服务类型（如 "std_srvs/SetBool"）
     * @param handler 处理函数，用于转发到远程服务
     * @return 是否成功
     */
    bool createLocalProxy(const std::string& local_service_name,
                         const std::string& service_type,
                         const ServiceHandler& handler);
    
    /**
     * @brief 调用本地 ROS 服务
     * @param service_name 服务名称
     * @param service_type 服务类型
     * @param request 请求数据（序列化的）
     * @param response 响应数据（序列化的）
     * @return 是否成功
     */
    bool callLocalService(const std::string& service_name,
                         const std::string& service_type,
                         const std::vector<uint8_t>& request,
                         std::vector<uint8_t>& response);
    
    /**
     * @brief 检查服务是否存在
     * @param service_name 服务名称
     * @return 是否存在
     */
    bool serviceExists(const std::string& service_name);
    
private:
    ros::NodeHandle nh_;
    
    // 存储创建的服务代理
    struct ProxyInfo {
        std::string service_type;
        ServiceHandler handler;
        ros::ServiceServer server;
    };
    std::unordered_map<std::string, std::unique_ptr<ProxyInfo>> proxies_;
    
    /**
     * @brief 通用的服务处理回调
     */
    bool genericServiceCallback(const std::string& service_name,
                               ros::ServiceEvent<ros::SerializedMessage, ros::SerializedMessage>& event);
};

} // namespace multibotnet

#endif // MULTIBOTNET_MANAGERS_SERVICE_BRIDGE_HPP