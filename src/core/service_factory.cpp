#include "multibotnet/core/service_factory.hpp"
#include "multibotnet/utils/logger.hpp"
#include <ros/service_manager.h>

namespace multibotnet {

ServiceFactory::ServiceFactory() {
}

ServiceFactory::~ServiceFactory() {
}

ServiceCallback ServiceFactory::createServiceProxy(const std::string& service_name,
                                                 const std::string& service_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 获取服务信息
    ServiceInfo info = getServiceInfo(service_type);
    if (info.req_datatype.empty()) {
        LOG_ERRORF("Unknown service type: %s", service_type.c_str());
        return nullptr;
    }
    
    // 创建通用服务处理函数
    return [this, service_name, service_type](const std::vector<uint8_t>& req_data) 
        -> std::vector<uint8_t> {
        try {
            // 直接调用ROS服务
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<ros::ServiceClient>(service_name);
            
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return {};
            }
            
            // 使用通用的服务调用机制
            // 注意：这需要特殊的处理，因为ROS的模板化服务系统
            // 这里简化处理，返回空结果
            LOG_WARN("Generic service proxy not fully implemented");
            return {};
            
        } catch (const std::exception& e) {
            LOG_ERRORF("Error calling service %s: %s", service_name.c_str(), e.what());
            return {};
        }
    };
}

ros::ServiceServer ServiceFactory::createServiceServer(const std::string& service_name,
                                                      const std::string& service_type,
                                                      const ServiceHandler& handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 获取服务信息
    ServiceInfo info = getServiceInfo(service_type);
    if (info.req_datatype.empty()) {
        LOG_ERRORF("Unknown service type: %s", service_type.c_str());
        return ros::ServiceServer();
    }
    
    // 使用通用的服务广告机制
    // 注意：这是一个简化的实现
    ros::NodeHandle nh;
    
    // 创建一个占位服务
    // 实际实现需要更复杂的动态类型处理
    LOG_WARN("Generic service server not fully implemented");
    
    return ros::ServiceServer();
}

std::vector<uint8_t> ServiceFactory::serializeRequest(const ros::SerializedMessage& req) {
    std::vector<uint8_t> data(req.num_bytes);
    memcpy(data.data(), req.message_start, req.num_bytes);
    return data;
}

std::vector<uint8_t> ServiceFactory::serializeResponse(const ros::SerializedMessage& res) {
    std::vector<uint8_t> data(res.num_bytes);
    memcpy(data.data(), res.message_start, res.num_bytes);
    return data;
}

ros::SerializedMessage ServiceFactory::deserializeRequest(const std::vector<uint8_t>& data,
                                                         const std::string& service_type) {
    ros::SerializedMessage msg;
    msg.num_bytes = data.size();
    msg.buf.reset(new uint8_t[msg.num_bytes]);
    memcpy(msg.buf.get(), data.data(), msg.num_bytes);
    msg.message_start = msg.buf.get();
    
    // 设置类型信息
    ServiceInfo info = getServiceInfo(service_type);
    msg.type_info = &typeid(ros::SerializedMessage);
    
    return msg;
}

ros::SerializedMessage ServiceFactory::deserializeResponse(const std::vector<uint8_t>& data,
                                                          const std::string& service_type) {
    ros::SerializedMessage msg;
    msg.num_bytes = data.size();
    msg.buf.reset(new uint8_t[msg.num_bytes]);
    memcpy(msg.buf.get(), data.data(), msg.num_bytes);
    msg.message_start = msg.buf.get();
    
    // 设置类型信息
    ServiceInfo info = getServiceInfo(service_type);
    msg.type_info = &typeid(ros::SerializedMessage);
    
    return msg;
}

std::string ServiceFactory::getServiceType(const std::string& service_name) {
    // 查询ROS master获取服务类型
    ros::NodeHandle nh;
    
    // 使用service::waitForService来检查服务是否存在
    if (!ros::service::waitForService(service_name, ros::Duration(0.1))) {
        LOG_WARNF("Service %s not found", service_name.c_str());
        return "";
    }
    
    // 注意：ROS没有直接的API来获取服务类型
    // 需要使用其他方法或存储服务类型映射
    return "";
}

ServiceFactory::ServiceInfo ServiceFactory::getServiceInfo(const std::string& service_type) {
    // 检查缓存
    auto it = service_info_cache_.find(service_type);
    if (it != service_info_cache_.end()) {
        return it->second;
    }
    
    ServiceInfo info;
    
    // 解析服务类型字符串，例如 "std_srvs/SetBool"
    size_t slash_pos = service_type.find('/');
    if (slash_pos == std::string::npos) {
        LOG_ERRORF("Invalid service type format: %s", service_type.c_str());
        return info;
    }
    
    std::string package = service_type.substr(0, slash_pos);
    std::string name = service_type.substr(slash_pos + 1);
    
    // 构造请求和响应类型名
    info.req_datatype = service_type + "Request";
    info.res_datatype = service_type + "Response";
    
    // TODO: 动态获取MD5值
    // 这里需要通过反射或其他方式获取实际的MD5值
    // 目前先使用占位符
    info.req_md5sum = "*";
    info.res_md5sum = "*";
    
    // 缓存信息
    service_info_cache_[service_type] = info;
    
    return info;
}

} // namespace multibotnet