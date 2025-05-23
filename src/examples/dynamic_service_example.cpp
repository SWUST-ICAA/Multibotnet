/**
 * 动态服务处理示例
 * 
 * 由于ROS的服务系统是强类型的，完全动态的服务处理比较困难。
 * 这里展示一种可行的方案：为常用的服务类型预先注册处理器。
 */

 #include <ros/ros.h>
 #include <std_srvs/SetBool.h>
 #include <std_srvs/Trigger.h>
 #include <std_srvs/Empty.h>
 #include <nav_msgs/GetPlan.h>
 #include <functional>
 #include <unordered_map>
 
 namespace multibotnet {
 namespace examples {
 
 // 服务处理器基类
 class ServiceHandlerBase {
 public:
     virtual ~ServiceHandlerBase() = default;
     virtual bool handle(const std::vector<uint8_t>& request,
                        std::vector<uint8_t>& response) = 0;
 };
 
 // 模板化的服务处理器
 template<typename ServiceType>
 class ServiceHandler : public ServiceHandlerBase {
 private:
     ros::ServiceClient client_;
     
 public:
     ServiceHandler(const std::string& service_name) {
         ros::NodeHandle nh;
         client_ = nh.serviceClient<ServiceType>(service_name);
     }
     
     bool handle(const std::vector<uint8_t>& request,
                std::vector<uint8_t>& response) override {
         typename ServiceType::Request req;
         typename ServiceType::Response res;
         
         // 反序列化请求
         ros::serialization::IStream stream(
             const_cast<uint8_t*>(request.data()), request.size());
         ros::serialization::deserialize(stream, req);
         
         // 调用服务
         if (!client_.call(req, res)) {
             return false;
         }
         
         // 序列化响应
         uint32_t serial_size = ros::serialization::serializationLength(res);
         response.resize(serial_size);
         ros::serialization::OStream ostream(response.data(), serial_size);
         ros::serialization::serialize(ostream, res);
         
         return true;
     }
 };
 
 // 服务工厂注册表
 class ServiceRegistry {
 private:
     std::unordered_map<std::string, 
         std::function<std::unique_ptr<ServiceHandlerBase>(const std::string&)>> 
         creators_;
     
 public:
     static ServiceRegistry& getInstance() {
         static ServiceRegistry instance;
         return instance;
     }
     
     // 注册服务类型
     template<typename ServiceType>
     void registerServiceType(const std::string& service_type) {
         creators_[service_type] = [](const std::string& service_name) {
             return std::make_unique<ServiceHandler<ServiceType>>(service_name);
         };
     }
     
     // 创建服务处理器
     std::unique_ptr<ServiceHandlerBase> 
     createHandler(const std::string& service_type,
                   const std::string& service_name) {
         auto it = creators_.find(service_type);
         if (it != creators_.end()) {
             return it->second(service_name);
         }
         return nullptr;
     }
     
     // 初始化常用服务类型
     void initCommonServices() {
         registerServiceType<std_srvs::SetBool>("std_srvs/SetBool");
         registerServiceType<std_srvs::Trigger>("std_srvs/Trigger");
         registerServiceType<std_srvs::Empty>("std_srvs/Empty");
         registerServiceType<nav_msgs::GetPlan>("nav_msgs/GetPlan");
         // 添加更多服务类型...
     }
 };
 
 // 使用示例
 void exampleUsage() {
     // 初始化注册表
     ServiceRegistry::getInstance().initCommonServices();
     
     // 创建服务处理器
     auto handler = ServiceRegistry::getInstance().createHandler(
         "std_srvs/SetBool", "/my_service");
     
     if (handler) {
         // 准备请求数据
         std_srvs::SetBool::Request req;
         req.data = true;
         
         // 序列化请求
         uint32_t req_size = ros::serialization::serializationLength(req);
         std::vector<uint8_t> req_data(req_size);
         ros::serialization::OStream req_stream(req_data.data(), req_size);
         ros::serialization::serialize(req_stream, req);
         
         // 调用服务
         std::vector<uint8_t> res_data;
         if (handler->handle(req_data, res_data)) {
             // 反序列化响应
             std_srvs::SetBool::Response res;
             ros::serialization::IStream res_stream(
                 const_cast<uint8_t*>(res_data.data()), res_data.size());
             ros::serialization::deserialize(res_stream, res);
             
             ROS_INFO("Service call success: %s", res.message.c_str());
         }
     }
 }
 
 } // namespace examples
 } // namespace multibotnet
 
 // 宏定义简化服务注册
 #define REGISTER_SERVICE_TYPE(ServiceType) \
     multibotnet::examples::ServiceRegistry::getInstance() \
         .registerServiceType<ServiceType>(#ServiceType)
 
 // 使用宏的示例
 void registerCustomServices() {
     // 注册自定义服务类型
     // REGISTER_SERVICE_TYPE(my_msgs::MyService);
 }