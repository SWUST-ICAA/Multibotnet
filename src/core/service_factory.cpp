#include "multibotnet/core/service_factory.hpp"
#include "multibotnet/utils/logger.hpp"
#include <ros/service_manager.h>
#include <ros/service_callback_helper.h>

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
    
    // 创建服务客户端
    ros::ServiceClient client = nh_.serviceClient<ros::SerializedMessage>(
        service_name, false);
    clients_[service_name] = client;
    
    // 返回代理函数
    return [this, client, service_name, info](const std::vector<uint8_t>& req_data) 
        -> std::vector<uint8_t> {
        try {
            // 创建序列化的请求消息
            ros::SerializedMessage req;
            req.num_bytes = req_data.size();
            req.buf.reset(new uint8_t[req.num_bytes]);
            memcpy(req.buf.get(), req_data.data(), req.num_bytes);
            req.message_start = req.buf.get();
            
            // 创建响应消息
            ros::SerializedMessage res;
            
            // 调用服务
            if (!const_cast<ros::ServiceClient&>(client).call(req, res)) {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return {};
            }
            
            // 将响应转换为vector
            std::vector<uint8_t> res_data(res.num_bytes);
            memcpy(res_data.data(), res.message_start, res.num_bytes);
            
            return res_data;
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
    
    // 创建服务回调
    auto callback = [handler, info](ros::SerializedMessage& req, 
                                   ros::SerializedMessage& res) -> bool {
        try {
            return handler(req, res);
        } catch (const std::exception& e) {
            LOG_ERRORF("Error in service handler: %s", e.what());
            return false;
        }
    };
    
    // 创建服务服务器
    ros::AdvertiseServiceOptions opts;
    opts.service = service_name;
    opts.datatype = service_type;
    opts.req_datatype = info.req_datatype;
    opts.res_datatype = info.res_datatype;
    opts.md5sum = info.req_md5sum;  // 这里需要计算正确的MD5
    opts.helper = boost::make_shared<ros::ServiceCallbackHelperT<
        ros::AdvertiseServiceOptions::Callback>>(callback);
    
    ros::ServiceServer server = nh_.advertiseService(opts);
    servers_[service_name] = server;
    
    LOG_INFOF("Created service server for %s with type %s", 
             service_name.c_str(), service_type.c_str());
    
    return server;
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
    // 从ROS服务管理器获取服务类型
    std::string service_type;
    if (!ros::service::getService(service_name, service_type)) {
        LOG_WARNF("Service %s not found", service_name.c_str());
        return "";
    }
    return service_type;
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