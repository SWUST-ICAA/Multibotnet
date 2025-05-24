#ifndef MULTIBOTNET_CORE_SERVICE_FACTORY_HPP
#define MULTIBOTNET_CORE_SERVICE_FACTORY_HPP

#include <ros/ros.h>
#include <memory>
#include <functional>
#include <unordered_map>
#include <mutex>
#include "multibotnet/core/types.hpp"

// 包含常用的服务类型
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetMap.h>

namespace multibotnet {

/**
 * @brief 静态服务工厂类，支持预定义的ROS服务类型
 */
class ServiceFactory {
public:
    // 服务处理函数类型
    using ServiceHandler = std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>;
    
    // 本地服务处理函数类型
    using LocalServiceHandler = std::function<bool(const std::vector<uint8_t>&, std::vector<uint8_t>&)>;
    
    // 服务处理器工厂函数类型
    using ServiceHandlerFactory = std::function<LocalServiceHandler(const std::string&)>;
    
    ServiceFactory();
    ~ServiceFactory();
    
    /**
     * @brief 注册服务类型
     * @param service_type 服务类型字符串
     * @param factory 服务处理器工厂函数
     */
    void registerServiceType(const std::string& service_type,
                           const ServiceHandlerFactory& factory);
    
    /**
     * @brief 创建服务代理（调用本地ROS服务）
     * @param service_name 服务名称
     * @param service_type 服务类型
     * @return 服务调用函数
     */
    ServiceHandler createServiceProxy(const std::string& service_name,
                                    const std::string& service_type);
    
    /**
     * @brief 创建ROS服务服务器（作为远程服务的代理）
     * @param service_name 服务名称
     * @param service_type 服务类型
     * @param remote_handler 远程服务处理函数
     * @return ROS服务服务器
     */
    ros::ServiceServer createServiceServer(const std::string& service_name,
                                         const std::string& service_type,
                                         const ServiceHandler& remote_handler);
    
    /**
     * @brief 序列化服务请求
     * @tparam T 服务请求类型
     * @param request 请求对象
     * @return 序列化的字节数组
     */
    template<typename T>
    static std::vector<uint8_t> serializeRequest(const T& request);
    
    /**
     * @brief 反序列化服务请求
     * @tparam T 服务请求类型
     * @param data 字节数组
     * @param request 请求对象
     * @return 是否成功
     */
    template<typename T>
    static bool deserializeRequest(const std::vector<uint8_t>& data, T& request);
    
    /**
     * @brief 序列化服务响应
     * @tparam T 服务响应类型
     * @param response 响应对象
     * @return 序列化的字节数组
     */
    template<typename T>
    static std::vector<uint8_t> serializeResponse(const T& response);
    
    /**
     * @brief 反序列化服务响应
     * @tparam T 服务响应类型
     * @param data 字节数组
     * @param response 响应对象
     * @return 是否成功
     */
    template<typename T>
    static bool deserializeResponse(const std::vector<uint8_t>& data, T& response);
    
private:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, ServiceHandlerFactory> service_factories_;
    std::unordered_map<std::string, ros::ServiceServer> servers_;
    std::mutex mutex_;
    
    // 注册内置服务类型
    void registerBuiltinServices();
    
    // std_srvs服务处理函数
    LocalServiceHandler createSetBoolHandler(const std::string& service_name);
    LocalServiceHandler createTriggerHandler(const std::string& service_name);
    LocalServiceHandler createEmptyHandler(const std::string& service_name);
    
    // nav_msgs服务处理函数
    LocalServiceHandler createGetPlanHandler(const std::string& service_name);
    LocalServiceHandler createGetMapHandler(const std::string& service_name);
};

// 模板实现
template<typename T>
std::vector<uint8_t> ServiceFactory::serializeRequest(const T& request) {
    // 使用ROS序列化
    uint32_t serial_size = ros::serialization::serializationLength(request);
    std::vector<uint8_t> buffer(serial_size);
    ros::serialization::OStream stream(buffer.data(), serial_size);
    ros::serialization::serialize(stream, request);
    return buffer;
}

template<typename T>
bool ServiceFactory::deserializeRequest(const std::vector<uint8_t>& data, T& request) {
    try {
        ros::serialization::IStream stream(const_cast<uint8_t*>(data.data()), data.size());
        ros::serialization::deserialize(stream, request);
        return true;
    } catch (...) {
        return false;
    }
}

template<typename T>
std::vector<uint8_t> ServiceFactory::serializeResponse(const T& response) {
    uint32_t serial_size = ros::serialization::serializationLength(response);
    std::vector<uint8_t> buffer(serial_size);
    ros::serialization::OStream stream(buffer.data(), serial_size);
    ros::serialization::serialize(stream, response);
    return buffer;
}

template<typename T>
bool ServiceFactory::deserializeResponse(const std::vector<uint8_t>& data, T& response) {
    try {
        ros::serialization::IStream stream(const_cast<uint8_t*>(data.data()), data.size());
        ros::serialization::deserialize(stream, response);
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace multibotnet

#endif // MULTIBOTNET_CORE_SERVICE_FACTORY_HPP