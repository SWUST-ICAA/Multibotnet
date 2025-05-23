#ifndef MULTIBOTNET_CORE_SERVICE_FACTORY_HPP
#define MULTIBOTNET_CORE_SERVICE_FACTORY_HPP

#include <ros/ros.h>
#include <ros/service_traits.h>
#include <memory>
#include <functional>
#include <unordered_map>
#include <mutex>
#include "multibotnet/core/types.hpp"

namespace multibotnet {

/**
 * @brief 动态服务工厂类，用于处理任意类型的ROS服务
 * 
 * 使用运行时类型信息和反射机制处理服务请求和响应
 */
class ServiceFactory {
public:
    using ServiceHandler = std::function<bool(ros::SerializedMessage&, ros::SerializedMessage&)>;
    
    ServiceFactory();
    ~ServiceFactory();
    
    /**
     * @brief 创建服务客户端代理
     * @param service_name 服务名称
     * @param service_type 服务类型字符串（如 "std_srvs/SetBool"）
     * @return 服务代理函数
     */
    ServiceCallback createServiceProxy(const std::string& service_name,
                                     const std::string& service_type);
    
    /**
     * @brief 创建服务服务器
     * @param service_name 服务名称
     * @param service_type 服务类型字符串
     * @param handler 处理函数
     * @return ROS服务服务器
     */
    ros::ServiceServer createServiceServer(const std::string& service_name,
                                         const std::string& service_type,
                                         const ServiceHandler& handler);
    
    /**
     * @brief 序列化服务请求
     * @param req 序列化的请求消息
     * @return 字节数组
     */
    std::vector<uint8_t> serializeRequest(const ros::SerializedMessage& req);
    
    /**
     * @brief 序列化服务响应
     * @param res 序列化的响应消息
     * @return 字节数组
     */
    std::vector<uint8_t> serializeResponse(const ros::SerializedMessage& res);
    
    /**
     * @brief 反序列化服务请求
     * @param data 字节数组
     * @param service_type 服务类型
     * @return 序列化的请求消息
     */
    ros::SerializedMessage deserializeRequest(const std::vector<uint8_t>& data,
                                            const std::string& service_type);
    
    /**
     * @brief 反序列化服务响应
     * @param data 字节数组
     * @param service_type 服务类型
     * @return 序列化的响应消息
     */
    ros::SerializedMessage deserializeResponse(const std::vector<uint8_t>& data,
                                             const std::string& service_type);
    
    /**
     * @brief 获取服务类型信息
     * @param service_name 服务名称
     * @return 服务类型字符串
     */
    std::string getServiceType(const std::string& service_name);
    
private:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, ros::ServiceClient> clients_;
    std::unordered_map<std::string, ros::ServiceServer> servers_;
    std::mutex mutex_;
    
    // 服务信息缓存
    struct ServiceInfo {
        std::string req_datatype;
        std::string res_datatype;
        std::string req_md5sum;
        std::string res_md5sum;
    };
    std::unordered_map<std::string, ServiceInfo> service_info_cache_;
    
    // 动态获取服务信息
    ServiceInfo getServiceInfo(const std::string& service_type);
};

} // namespace multibotnet

#endif // MULTIBOTNET_CORE_SERVICE_FACTORY_HPP