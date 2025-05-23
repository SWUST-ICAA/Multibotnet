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
    // 注意：由于ROS的限制，我们不能真正创建一个完全通用的服务代理
    // 这里返回一个占位函数，实际使用时需要根据具体服务类型处理
    return [this, service_name, service_type](const std::vector<uint8_t>& req_data) 
        -> std::vector<uint8_t> {
        try {
            // 检查服务是否存在
            if (!ros::service::exists(service_name, false)) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return {};
            }
            
            // 这里需要实际的服务调用实现
            // 由于ROS的模板化限制，我们不能动态创建服务客户端
            // 实际项目中，可能需要为每种服务类型注册特定的处理器
            LOG_WARNF("Generic service proxy not fully implemented for %s", service_name.c_str());
            
            // 返回空响应表示未实现
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
    // 由于ROS的限制，我们需要使用特定的方法来创建服务
    ros::NodeHandle nh;
    
    // 创建占位服务服务器
    // 实际实现中，需要使用服务类型特定的处理
    LOG_WARNF("Generic service server not fully implemented for %s", service_name.c_str());
    
    // 返回空的服务服务器
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
    // 检查服务是否存在
    if (!ros::service::exists(service_name, false)) {
        LOG_WARNF("Service %s not found", service_name.c_str());
        return "";
    }
    
    // 注意：ROS没有直接的API来获取服务类型
    // 可以通过以下方法之一：
    // 1. 从配置中读取服务类型映射
    // 2. 使用ROS参数服务器存储服务类型信息
    // 3. 在服务创建时缓存类型信息
    
    // 这里返回空字符串，表示需要从其他地方获取类型信息
    LOG_WARN("Service type discovery not implemented, returning empty type");
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