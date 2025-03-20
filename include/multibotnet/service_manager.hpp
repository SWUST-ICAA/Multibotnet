#ifndef SERVICE_MANAGER_HPP
#define SERVICE_MANAGER_HPP

#include <zmq.hpp>
#include <string>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>

namespace multibotnet {

class ServiceManager {
public:
    ServiceManager();
    ~ServiceManager();

    void init();

private:
    zmq::context_t context_;
    zmq::socket_t rep_socket_;
    ros::ServiceServer service_server_;
    std::thread service_thread_;  // Thread for handling ZeroMQ requests

    bool handleService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
};

} // namespace multibotnet

#endif // SERVICE_MANAGER_HPP