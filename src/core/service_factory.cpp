#include "multibotnet/core/service_factory.hpp"
#include "multibotnet/utils/logger.hpp"
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

namespace multibotnet {

// 服务调用的辅助类
class GenericServiceCaller {
public:
    GenericServiceCaller(ros::NodeHandle& nh) : nh_(nh) {}
    
    // 调用本地ROS服务并返回序列化的响应
    std::vector<uint8_t> callLocalService(const std::string& service_name,
                                         const std::string& service_type,
                                         const std::vector<uint8_t>& request_data) {
        try {
            // 根据服务类型动态调用
            if (service_type == "std_srvs/SetBool") {
                return callSetBoolService(service_name, request_data);
            } else if (service_type == "std_srvs/Trigger") {
                return callTriggerService(service_name, request_data);
            } else if (service_type == "std_srvs/Empty") {
                return callEmptyService(service_name, request_data);
            } else {
                // 对于未知类型，尝试通用方法
                return callGenericService(service_name, service_type, request_data);
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Error calling local service %s: %s", 
                      service_name.c_str(), e.what());
            return {};
        }
    }
    
private:
    ros::NodeHandle& nh_;
    
    std::vector<uint8_t> callSetBoolService(const std::string& service_name,
                                           const std::vector<uint8_t>& request_data) {
        // 反序列化请求
        if (request_data.size() < 1) {
            LOG_ERROR("Invalid SetBool request data");
            return {};
        }
        
        bool data = request_data[0] != 0;
        
        // 调用服务
        ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(service_name);
        std_srvs::SetBool srv;
        srv.request.data = data;
        
        std::vector<uint8_t> response;
        
        if (client.call(srv)) {
            // 序列化响应: [1字节 success][4字节 message长度][message内容]
            response.push_back(srv.response.success ? 1 : 0);
            
            uint32_t msg_len = srv.response.message.size();
            response.insert(response.end(), 
                          reinterpret_cast<uint8_t*>(&msg_len),
                          reinterpret_cast<uint8_t*>(&msg_len) + 4);
            response.insert(response.end(), 
                          srv.response.message.begin(), 
                          srv.response.message.end());
        } else {
            LOG_ERRORF("Failed to call service %s", service_name.c_str());
            // 返回错误响应
            response.push_back(0);  // success = false
            uint32_t zero = 0;
            response.insert(response.end(), 
                          reinterpret_cast<uint8_t*>(&zero),
                          reinterpret_cast<uint8_t*>(&zero) + 4);
        }
        
        return response;
    }
    
    std::vector<uint8_t> callTriggerService(const std::string& service_name,
                                           const std::vector<uint8_t>& request_data) {
        // Trigger服务没有请求参数
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(service_name);
        std_srvs::Trigger srv;
        
        std::vector<uint8_t> response;
        
        if (client.call(srv)) {
            // 序列化响应
            response.push_back(srv.response.success ? 1 : 0);
            
            uint32_t msg_len = srv.response.message.size();
            response.insert(response.end(), 
                          reinterpret_cast<uint8_t*>(&msg_len),
                          reinterpret_cast<uint8_t*>(&msg_len) + 4);
            response.insert(response.end(), 
                          srv.response.message.begin(), 
                          srv.response.message.end());
        } else {
            LOG_ERRORF("Failed to call service %s", service_name.c_str());
            // 返回错误响应
            response.push_back(0);
            uint32_t zero = 0;
            response.insert(response.end(), 
                          reinterpret_cast<uint8_t*>(&zero),
                          reinterpret_cast<uint8_t*>(&zero) + 4);
        }
        
        return response;
    }
    
    std::vector<uint8_t> callEmptyService(const std::string& service_name,
                                         const std::vector<uint8_t>& request_data) {
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>(service_name);
        std_srvs::Empty srv;
        
        std::vector<uint8_t> response;
        
        if (client.call(srv)) {
            // Empty服务没有响应数据
            response.push_back(1);  // 表示成功
        } else {
            response.push_back(0);  // 表示失败
        }
        
        return response;
    }
    
    std::vector<uint8_t> callGenericService(const std::string& service_name,
                                           const std::string& service_type,
                                           const std::vector<uint8_t>& request_data) {
        // 对于未知类型，返回空响应
        LOG_WARNF("Unsupported service type: %s", service_type.c_str());
        return {};
    }
};

ServiceFactory::ServiceFactory() {
    registerCommonServiceTypes();
}

ServiceFactory::~ServiceFactory() {
}

void ServiceFactory::registerCommonServiceTypes() {
    // 保持原有实现
    ServiceInfo setBoolInfo;
    setBoolInfo.req_datatype = "std_srvs/SetBoolRequest";
    setBoolInfo.res_datatype = "std_srvs/SetBoolResponse";
    setBoolInfo.req_md5sum = "b88405221c77b1878a3cbbfff53428d7";
    setBoolInfo.res_md5sum = "937c9679a518e3a18d831e57125ea522";
    service_info_cache_["std_srvs/SetBool"] = setBoolInfo;
    
    ServiceInfo triggerInfo;
    triggerInfo.req_datatype = "std_srvs/TriggerRequest";
    triggerInfo.res_datatype = "std_srvs/TriggerResponse";
    triggerInfo.req_md5sum = "d41d8cd98f00b204e9800998ecf8427e";
    triggerInfo.res_md5sum = "937c9679a518e3a18d831e57125ea522";
    service_info_cache_["std_srvs/Trigger"] = triggerInfo;
    
    ServiceInfo emptyInfo;
    emptyInfo.req_datatype = "std_srvs/EmptyRequest";
    emptyInfo.res_datatype = "std_srvs/EmptyResponse";
    emptyInfo.req_md5sum = "d41d8cd98f00b204e9800998ecf8427e";
    emptyInfo.res_md5sum = "d41d8cd98f00b204e9800998ecf8427e";
    service_info_cache_["std_srvs/Empty"] = emptyInfo;
}

ServiceCallback ServiceFactory::createServiceProxy(const std::string& service_name,
                                                 const std::string& service_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 创建一个lambda函数，用于调用本地ROS服务
    return [this, service_name, service_type](const std::vector<uint8_t>& req_data) 
        -> std::vector<uint8_t> {
        
        GenericServiceCaller caller(nh_);
        return caller.callLocalService(service_name, service_type, req_data);
    };
}

ros::ServiceServer ServiceFactory::createServiceServer(const std::string& service_name,
                                                      const std::string& service_type,
                                                      const ServiceHandler& handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 根据服务类型创建ROS服务服务器
    if (service_type == "std_srvs/SetBool") {
        return nh_.advertiseService(
            service_name,
            boost::function<bool(std_srvs::SetBool::Request&, 
                               std_srvs::SetBool::Response&)>(
                [handler, service_type](std_srvs::SetBool::Request& req,
                                      std_srvs::SetBool::Response& res) -> bool {
                    // 序列化请求
                    std::vector<uint8_t> req_data;
                    req_data.push_back(req.data ? 1 : 0);
                    
                    ros::SerializedMessage req_msg;
                    req_msg.num_bytes = req_data.size();
                    req_msg.buf.reset(new uint8_t[req_msg.num_bytes]);
                    memcpy(req_msg.buf.get(), req_data.data(), req_msg.num_bytes);
                    req_msg.message_start = req_msg.buf.get();
                    
                    ros::SerializedMessage res_msg;
                    
                    // 调用处理器
                    if (handler(req_msg, res_msg)) {
                        // 反序列化响应
                        if (res_msg.num_bytes >= 5) {
                            res.success = res_msg.message_start[0] != 0;
                            
                            uint32_t msg_len;
                            memcpy(&msg_len, res_msg.message_start + 1, 4);
                            
                            if (res_msg.num_bytes >= 5 + msg_len) {
                                res.message = std::string(
                                    reinterpret_cast<char*>(res_msg.message_start + 5),
                                    msg_len);
                            }
                        }
                        return true;
                    }
                    return false;
                }
            )
        );
    } else if (service_type == "std_srvs/Trigger") {
        return nh_.advertiseService(
            service_name,
            boost::function<bool(std_srvs::Trigger::Request&, 
                               std_srvs::Trigger::Response&)>(
                [handler, service_type](std_srvs::Trigger::Request& req,
                                      std_srvs::Trigger::Response& res) -> bool {
                    // Trigger没有请求数据
                    ros::SerializedMessage req_msg;
                    req_msg.num_bytes = 0;
                    
                    ros::SerializedMessage res_msg;
                    
                    // 调用处理器
                    if (handler(req_msg, res_msg)) {
                        // 反序列化响应
                        if (res_msg.num_bytes >= 5) {
                            res.success = res_msg.message_start[0] != 0;
                            
                            uint32_t msg_len;
                            memcpy(&msg_len, res_msg.message_start + 1, 4);
                            
                            if (res_msg.num_bytes >= 5 + msg_len) {
                                res.message = std::string(
                                    reinterpret_cast<char*>(res_msg.message_start + 5),
                                    msg_len);
                            }
                        }
                        return true;
                    }
                    return false;
                }
            )
        );
    } else {
        LOG_WARNF("Cannot create ROS service server for unsupported type: %s", 
                 service_type.c_str());
        return ros::ServiceServer();
    }
}

std::vector<uint8_t> ServiceFactory::serializeRequest(const ros::SerializedMessage& req) {
    std::vector<uint8_t> data(req.num_bytes);
    if (req.num_bytes > 0) {
        memcpy(data.data(), req.message_start, req.num_bytes);
    }
    return data;
}

std::vector<uint8_t> ServiceFactory::serializeResponse(const ros::SerializedMessage& res) {
    std::vector<uint8_t> data(res.num_bytes);
    if (res.num_bytes > 0) {
        memcpy(data.data(), res.message_start, res.num_bytes);
    }
    return data;
}

ros::SerializedMessage ServiceFactory::deserializeRequest(const std::vector<uint8_t>& data,
                                                         const std::string& service_type) {
    ros::SerializedMessage msg;
    msg.num_bytes = data.size();
    if (msg.num_bytes > 0) {
        msg.buf.reset(new uint8_t[msg.num_bytes]);
        memcpy(msg.buf.get(), data.data(), msg.num_bytes);
        msg.message_start = msg.buf.get();
    }
    msg.type_info = &typeid(ros::SerializedMessage);
    return msg;
}

ros::SerializedMessage ServiceFactory::deserializeResponse(const std::vector<uint8_t>& data,
                                                          const std::string& service_type) {
    ros::SerializedMessage msg;
    msg.num_bytes = data.size();
    if (msg.num_bytes > 0) {
        msg.buf.reset(new uint8_t[msg.num_bytes]);
        memcpy(msg.buf.get(), data.data(), msg.num_bytes);
        msg.message_start = msg.buf.get();
    }
    msg.type_info = &typeid(ros::SerializedMessage);
    return msg;
}

std::string ServiceFactory::getServiceType(const std::string& service_name) {
    // 暂时返回空
    return "";
}

ServiceFactory::ServiceInfo ServiceFactory::getServiceInfo(const std::string& service_type) {
    // 检查缓存
    auto it = service_info_cache_.find(service_type);
    if (it != service_info_cache_.end()) {
        return it->second;
    }
    
    ServiceInfo info;
    
    // 解析服务类型字符串
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
    info.req_md5sum = "*";
    info.res_md5sum = "*";
    
    // 缓存信息
    service_info_cache_[service_type] = info;
    
    return info;
}

} // namespace multibotnet