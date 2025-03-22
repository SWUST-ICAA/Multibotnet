#ifndef SERVICE_MANAGER_HPP
#define SERVICE_MANAGER_HPP

#include <zmq.hpp>
#include <string>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/GetPlan.h>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <map>
#include <queue>
#include <mutex>
#include <memory> // for shared_ptr

#include "ros_sub_pub.hpp" // For serializeMsg and deserializeMsg

namespace multibotnet {

class ServiceHandler {
public:
    virtual ~ServiceHandler() = default;
    virtual void handleRequest(zmq::message_t& request, zmq::socket_t& rep_socket) = 0;
};

template<typename ServiceType>
class SpecificServiceHandler : public ServiceHandler {
private:
    ros::ServiceClient client_;

public:
    SpecificServiceHandler(const std::string& service_name) {
        ros::NodeHandle nh;
        client_ = nh.serviceClient<ServiceType>(service_name);
    }

    void handleRequest(zmq::message_t& request, zmq::socket_t& rep_socket) override {
        auto req = deserializeMsg<typename ServiceType::Request>(
            static_cast<uint8_t*>(request.data()), request.size());
        typename ServiceType::Response res;

        if (client_.call(req, res)) {
            auto buffer = serializeMsg(res);
            zmq::message_t reply(buffer.size());
            memcpy(reply.data(), buffer.data(), buffer.size());
            rep_socket.send(reply, zmq::send_flags::none);
        } else {
            ROS_ERROR("Failed to call local ROS service %s", client_.getService().c_str());
            // Send an empty response as an error indicator
            zmq::message_t reply(0);
            rep_socket.send(reply, zmq::send_flags::none);
        }
    }
};

class ServiceManager {
public:
    ServiceManager();
    ~ServiceManager();

    void init(const std::string& config_file);

    // Template function for calling remote services
    template<typename ServiceType>
    bool callService(const std::string& service_name,
                     typename ServiceType::Request& req,
                     typename ServiceType::Response& res);

private:
    zmq::context_t context_;                    // ZeroMQ context
    std::vector<zmq::socket_t> rep_sockets_;    // REP sockets for providing services
    std::vector<std::thread> service_threads_;  // Service threads
    std::map<std::string, zmq::socket_t> req_sockets_; // REQ sockets for requesting services
    std::map<zmq::socket_t*, std::shared_ptr<ServiceHandler>> socket_to_handler_; // Map REP sockets to handlers

    // Request queue and synchronization
    struct ServiceRequest {
        zmq::message_t request;
        zmq::socket_t* rep_socket;
    };
    std::queue<ServiceRequest> request_queue_;
    std::mutex queue_mutex_;

    void startProvideService(const std::string& service_name,
                            const std::string& service_type,
                            const std::string& bind_address,
                            int port);
    void startRequestService(const std::string& service_name,
                            const std::string& service_type,
                            const std::string& connect_address,
                            int port);

    // Process requests from the queue
    void processRequests();

    std::shared_ptr<ServiceHandler> createHandler(const std::string& service_type, const std::string& service_name);

    void displayConfig(const YAML::Node& config);
    std::string getLocalIP();
};

} // namespace multibotnet

#endif // SERVICE_MANAGER_HPP