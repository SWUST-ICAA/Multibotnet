#include "multibotnet/core/service_factory.hpp"
#include "multibotnet/utils/logger.hpp"
#include <ros/service_manager.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/GetPlan.h>

namespace multibotnet {

ServiceFactory::ServiceFactory() {
    // 预注册常用服务类型信息
    registerCommonServiceTypes();
}

ServiceFactory::~ServiceFactory() {
}

void ServiceFactory::registerCommonServiceTypes() {
    // 注册 std_srvs/SetBool
    ServiceInfo setBoolInfo;
    setBoolInfo.req_datatype = "std_srvs/SetBoolRequest";
    setBoolInfo.res_datatype = "std_srvs/SetBoolResponse";
    setBoolInfo.req_md5sum = "b88405221c77b1878a3cbbfff53428d7";
    setBoolInfo.res_md5sum = "937c9679a518e3a18d831e57125ea522";
    service_info_cache_["std_srvs/SetBool"] = setBoolInfo;
    
    // 注册 nav_msgs/GetPlan
    ServiceInfo getPlanInfo;
    getPlanInfo.req_datatype = "nav_msgs/GetPlanRequest";
    getPlanInfo.res_datatype = "nav_msgs/GetPlanResponse";
    getPlanInfo.req_md5sum = "*";  // 需要实际的MD5值
    getPlanInfo.res_md5sum = "*";
    service_info_cache_["nav_msgs/GetPlan"] = getPlanInfo;
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
    
    // 返回一个服务代理函数
    return [this, service_name, service_type](const std::vector<uint8_t>& req_data) 
        -> std::vector<uint8_t> {
        try {
            // 对于本地服务，我们可以直接返回空响应
            // 实际的服务调用应该通过ZMQ传输层完成
            LOG_DEBUGF("Service proxy called for %s", service_name.c_str());
            
            // 返回一个最小的有效响应
            std::vector<uint8_t> response;
            
            // 根据服务类型构造基本响应
            if (service_type == "std_srvs/SetBool") {
                // SetBool响应包含: bool success, string message
                response.resize(5);  // 1字节bool + 4字节string长度
                response[0] = 1;     // success = true
                // string长度 = 0 (空消息)
                memset(&response[1], 0, 4);
            } else {
                // 其他服务返回最小响应
                response.resize(1);
                response[0] = 0;
            }
            
            return response;
            
        } catch (const std::exception& e) {
            LOG_ERRORF("Error in service proxy for %s: %s", 
                      service_name.c_str(), e.what());
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
    
    // 由于ROS服务需要具体类型，这里只记录服务信息
    // 实际的服务处理将通过ZMQ传输层完成
    LOG_INFOF("Service server registered for %s (type: %s)", 
             service_name.c_str(), service_type.c_str());
    
    // 返回一个占位的ServiceServer
    // 注意：这不是一个真正的ROS服务，而是用于标记服务已注册
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
    // 这个功能暂时返回空，需要从配置或其他地方获取服务类型
    LOG_DEBUG("Service type lookup not implemented");
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
    
    // 设置默认MD5值
    info.req_md5sum = "*";
    info.res_md5sum = "*";
    
    // 缓存信息
    service_info_cache_[service_type] = info;
    
    return info;
}

} // namespace multibotnet