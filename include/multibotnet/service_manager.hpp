#ifndef SERVICE_MANAGER_HPP
#define SERVICE_MANAGER_HPP

#include <zmq.hpp>
#include <string>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/GetPlan.h>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace multibotnet {

class ServiceManager {
public:
    ServiceManager();
    ~ServiceManager();

    void init(const std::string& config_file);

private:
    zmq::context_t context_;
    std::vector<zmq::socket_t> rep_sockets_;
    std::vector<ros::ServiceServer> service_servers_;
    std::vector<std::thread> service_threads_;

    void startService(const std::string& service_name, const std::string& service_type,
                      const std::string& src_ip, int src_port);
    bool handleSetBool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool handleGetPlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);
    void displayConfig(const YAML::Node& config);
    std::string getLocalIP();  // 新增的声明
};

} // namespace multibotnet

#endif // SERVICE_MANAGER_HPP