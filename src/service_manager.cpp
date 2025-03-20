#include <multibotnet/service_manager.hpp>
#include <multibotnet/ros_sub_pub.hpp>
#include <std_srvs/SetBool.h>
#include <thread>

namespace multibotnet {

ServiceManager::ServiceManager() : context_(1) {}

ServiceManager::~ServiceManager() {
    if (service_thread_.joinable()) {
        service_thread_.join();  // Wait for the thread to finish
    }
}

void ServiceManager::init() {
    rep_socket_ = zmq::socket_t(context_, ZMQ_REP);
    rep_socket_.bind("tcp://*:5555");

    ros::NodeHandle nh;
    service_server_ = nh.advertiseService("example_service", &ServiceManager::handleService, this);

    service_thread_ = std::thread([this]() {
        while (ros::ok()) {
            zmq::message_t request;
            rep_socket_.recv(request, zmq::recv_flags::none);
            std::string req_str(static_cast<char*>(request.data()), request.size());
            std::string response = "Received: " + req_str;
            zmq::message_t reply(response.size());
            memcpy(reply.data(), response.data(), response.size());
            rep_socket_.send(reply, zmq::send_flags::none);
        }
    });
}

bool ServiceManager::handleService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    res.success = true;
    res.message = "Service called with data: " + std::to_string(req.data);
    return true;
}

} // namespace multibotnet