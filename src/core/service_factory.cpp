#include "multibotnet/core/service_factory.hpp"
#include "multibotnet/utils/logger.hpp"

namespace multibotnet {

ServiceFactory::ServiceFactory() {
    registerBuiltinServices();
}

ServiceFactory::~ServiceFactory() {
}

void ServiceFactory::registerBuiltinServices() {
    // 注册 std_srvs 服务类型
    registerServiceType("std_srvs/SetBool", 
        [this](const std::string& name) { return createSetBoolHandler(name); });
    
    registerServiceType("std_srvs/Trigger",
        [this](const std::string& name) { return createTriggerHandler(name); });
    
    registerServiceType("std_srvs/Empty",
        [this](const std::string& name) { return createEmptyHandler(name); });
    
    // 注册 nav_msgs 服务类型
    registerServiceType("nav_msgs/GetPlan",
        [this](const std::string& name) { return createGetPlanHandler(name); });
    
    registerServiceType("nav_msgs/GetMap",
        [this](const std::string& name) { return createGetMapHandler(name); });
    
    LOG_INFO("Registered built-in service types");
}

void ServiceFactory::registerServiceType(const std::string& service_type,
                                       const ServiceHandlerFactory& factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    service_factories_[service_type] = factory;
}

ServiceFactory::ServiceHandler ServiceFactory::createServiceProxy(
    const std::string& service_name,
    const std::string& service_type) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 查找服务工厂
    auto it = service_factories_.find(service_type);
    if (it == service_factories_.end()) {
        LOG_ERRORF("Unknown service type: %s", service_type.c_str());
        return nullptr;
    }
    
    // 使用工厂创建处理器
    LocalServiceHandler handler = it->second(service_name);
    
    // 返回调用本地服务的函数
    return [handler](const std::vector<uint8_t>& request) 
        -> std::vector<uint8_t> {
        std::vector<uint8_t> response;
        if (handler(request, response)) {
            return response;
        }
        return {};
    };
}

ros::ServiceServer ServiceFactory::createServiceServer(
    const std::string& service_name,
    const std::string& service_type,
    const ServiceHandler& remote_handler) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 根据服务类型创建相应的ROS服务
    if (service_type == "std_srvs/SetBool") {
        auto server = nh_.advertiseService(service_name,
            boost::function<bool(std_srvs::SetBool::Request&,
                               std_srvs::SetBool::Response&)>(
                [remote_handler](std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& res) -> bool {
                    // 序列化请求
                    auto req_data = serializeRequest(req);
                    
                    // 调用远程服务
                    auto res_data = remote_handler(req_data);
                    if (res_data.empty()) {
                        res.success = false;
                        res.message = "Remote service call failed";
                        return false;
                    }
                    
                    // 反序列化响应
                    if (!deserializeResponse(res_data, res)) {
                        res.success = false;
                        res.message = "Failed to deserialize response";
                        return false;
                    }
                    
                    return true;
                }
            )
        );
        servers_[service_name] = server;
        return server;
        
    } else if (service_type == "std_srvs/Trigger") {
        auto server = nh_.advertiseService(service_name,
            boost::function<bool(std_srvs::Trigger::Request&,
                               std_srvs::Trigger::Response&)>(
                [remote_handler](std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res) -> bool {
                    // 序列化请求
                    auto req_data = serializeRequest(req);
                    
                    // 调用远程服务
                    auto res_data = remote_handler(req_data);
                    if (res_data.empty()) {
                        res.success = false;
                        res.message = "Remote service call failed";
                        return false;
                    }
                    
                    // 反序列化响应
                    if (!deserializeResponse(res_data, res)) {
                        res.success = false;
                        res.message = "Failed to deserialize response";
                        return false;
                    }
                    
                    return true;
                }
            )
        );
        servers_[service_name] = server;
        return server;
        
    } else if (service_type == "std_srvs/Empty") {
        auto server = nh_.advertiseService(service_name,
            boost::function<bool(std_srvs::Empty::Request&,
                               std_srvs::Empty::Response&)>(
                [remote_handler](std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res) -> bool {
                    // 序列化请求
                    auto req_data = serializeRequest(req);
                    
                    // 调用远程服务
                    auto res_data = remote_handler(req_data);
                    return !res_data.empty();
                }
            )
        );
        servers_[service_name] = server;
        return server;
        
    } else if (service_type == "nav_msgs/GetPlan") {
        auto server = nh_.advertiseService(service_name,
            boost::function<bool(nav_msgs::GetPlan::Request&,
                               nav_msgs::GetPlan::Response&)>(
                [remote_handler](nav_msgs::GetPlan::Request& req,
                               nav_msgs::GetPlan::Response& res) -> bool {
                    // 序列化请求
                    auto req_data = serializeRequest(req);
                    
                    // 调用远程服务
                    auto res_data = remote_handler(req_data);
                    if (res_data.empty()) {
                        return false;
                    }
                    
                    // 反序列化响应
                    if (!deserializeResponse(res_data, res)) {
                        return false;
                    }
                    
                    return true;
                }
            )
        );
        servers_[service_name] = server;
        return server;
        
    } else if (service_type == "nav_msgs/GetMap") {
        auto server = nh_.advertiseService(service_name,
            boost::function<bool(nav_msgs::GetMap::Request&,
                               nav_msgs::GetMap::Response&)>(
                [remote_handler](nav_msgs::GetMap::Request& req,
                               nav_msgs::GetMap::Response& res) -> bool {
                    // 序列化请求
                    auto req_data = serializeRequest(req);
                    
                    // 调用远程服务
                    auto res_data = remote_handler(req_data);
                    if (res_data.empty()) {
                        return false;
                    }
                    
                    // 反序列化响应
                    if (!deserializeResponse(res_data, res)) {
                        return false;
                    }
                    
                    return true;
                }
            )
        );
        servers_[service_name] = server;
        return server;
        
    } else {
        LOG_WARNF("Unsupported service type for server: %s", service_type.c_str());
        return ros::ServiceServer();
    }
}

// std_srvs/SetBool 处理器
ServiceFactory::LocalServiceHandler ServiceFactory::createSetBoolHandler(
    const std::string& service_name) {
    
    return [this, service_name](const std::vector<uint8_t>& req_data,
                               std::vector<uint8_t>& res_data) -> bool {
        try {
            // 反序列化请求
            std_srvs::SetBool::Request req;
            if (!deserializeRequest(req_data, req)) {
                LOG_ERROR("Failed to deserialize SetBool request");
                return false;
            }
            
            // 调用本地ROS服务
            ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(service_name);
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return false;
            }
            
            std_srvs::SetBool srv;
            srv.request = req;
            
            if (client.call(srv)) {
                // 序列化响应
                res_data = serializeResponse(srv.response);
                return true;
            } else {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Exception in SetBool handler: %s", e.what());
            return false;
        }
    };
}

// std_srvs/Trigger 处理器
ServiceFactory::LocalServiceHandler ServiceFactory::createTriggerHandler(
    const std::string& service_name) {
    
    return [this, service_name](const std::vector<uint8_t>& req_data,
                               std::vector<uint8_t>& res_data) -> bool {
        try {
            // 反序列化请求
            std_srvs::Trigger::Request req;
            if (!deserializeRequest(req_data, req)) {
                LOG_ERROR("Failed to deserialize Trigger request");
                return false;
            }
            
            // 调用本地ROS服务
            ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(service_name);
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return false;
            }
            
            std_srvs::Trigger srv;
            srv.request = req;
            
            if (client.call(srv)) {
                // 序列化响应
                res_data = serializeResponse(srv.response);
                return true;
            } else {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Exception in Trigger handler: %s", e.what());
            return false;
        }
    };
}

// std_srvs/Empty 处理器
ServiceFactory::LocalServiceHandler ServiceFactory::createEmptyHandler(
    const std::string& service_name) {
    
    return [this, service_name](const std::vector<uint8_t>& req_data,
                               std::vector<uint8_t>& res_data) -> bool {
        try {
            // 反序列化请求
            std_srvs::Empty::Request req;
            if (!deserializeRequest(req_data, req)) {
                LOG_ERROR("Failed to deserialize Empty request");
                return false;
            }
            
            // 调用本地ROS服务
            ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>(service_name);
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return false;
            }
            
            std_srvs::Empty srv;
            srv.request = req;
            
            if (client.call(srv)) {
                // 序列化响应
                res_data = serializeResponse(srv.response);
                return true;
            } else {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Exception in Empty handler: %s", e.what());
            return false;
        }
    };
}

// nav_msgs/GetPlan 处理器
ServiceFactory::LocalServiceHandler ServiceFactory::createGetPlanHandler(
    const std::string& service_name) {
    
    return [this, service_name](const std::vector<uint8_t>& req_data,
                               std::vector<uint8_t>& res_data) -> bool {
        try {
            // 反序列化请求
            nav_msgs::GetPlan::Request req;
            if (!deserializeRequest(req_data, req)) {
                LOG_ERROR("Failed to deserialize GetPlan request");
                return false;
            }
            
            // 调用本地ROS服务
            ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>(service_name);
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return false;
            }
            
            nav_msgs::GetPlan srv;
            srv.request = req;
            
            if (client.call(srv)) {
                // 序列化响应
                res_data = serializeResponse(srv.response);
                return true;
            } else {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Exception in GetPlan handler: %s", e.what());
            return false;
        }
    };
}

// nav_msgs/GetMap 处理器
ServiceFactory::LocalServiceHandler ServiceFactory::createGetMapHandler(
    const std::string& service_name) {
    
    return [this, service_name](const std::vector<uint8_t>& req_data,
                               std::vector<uint8_t>& res_data) -> bool {
        try {
            // 反序列化请求
            nav_msgs::GetMap::Request req;
            if (!deserializeRequest(req_data, req)) {
                LOG_ERROR("Failed to deserialize GetMap request");
                return false;
            }
            
            // 调用本地ROS服务
            ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>(service_name);
            if (!client.exists()) {
                LOG_ERRORF("Service %s does not exist", service_name.c_str());
                return false;
            }
            
            nav_msgs::GetMap srv;
            srv.request = req;
            
            if (client.call(srv)) {
                // 序列化响应
                res_data = serializeResponse(srv.response);
                return true;
            } else {
                LOG_ERRORF("Failed to call service %s", service_name.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERRORF("Exception in GetMap handler: %s", e.what());
            return false;
        }
    };
}

} // namespace multibotnet