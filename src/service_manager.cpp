#include <multibotnet/service_manager.hpp>
#include <multibotnet/ros_sub_pub.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <zmq.hpp>
#include <iostream>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

namespace multibotnet {

ServiceManager::ServiceManager() : context_(1) {}

ServiceManager::~ServiceManager() {
    for (auto& th : service_threads_) {
        if (th.joinable()) th.join();
    }
}

void ServiceManager::init(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    displayConfig(config);

    std::map<std::string, std::string> ip_map;
    for (const auto& ip : config["IP"]) {
        ip_map[ip.first.as<std::string>()] = ip.second.as<std::string>();
    }

    // Handle provide_services
    if (config["provide_services"]) {
        size_t num_services = config["provide_services"].size();
        rep_sockets_.reserve(num_services);
        service_threads_.reserve(num_services);

        for (const auto& service : config["provide_services"]) {
            std::string service_name = service["service_name"].as<std::string>();
            std::string service_type = service["service_type"].as<std::string>();
            std::string bind_address = service["bind_address"].as<std::string>();
            int port = service["port"].as<int>();

            if (bind_address == "self") {
                bind_address = "*";
            } else if (ip_map.find(bind_address) != ip_map.end()) {
                bind_address = ip_map[bind_address];
            } else {
                ROS_ERROR("Invalid bind_address '%s' for provide_service %s", bind_address.c_str(), service_name.c_str());
                continue;
            }

            startProvideService(service_name, service_type, bind_address, port);
            auto& current_socket = rep_sockets_.back();
            auto handler = createHandler(service_type, service_name);
            if (handler) {
                socket_to_handler_[&current_socket] = handler;
            } else {
                ROS_ERROR("Failed to create handler for service %s with type %s", service_name.c_str(), service_type.c_str());
            }
        }
    }

    // Handle request_services
    if (config["request_services"]) {
        for (const auto& service : config["request_services"]) {
            std::string service_name = service["service_name"].as<std::string>();
            std::string service_type = service["service_type"].as<std::string>();
            std::string connect_address = service["connect_address"].as<std::string>();
            int port = service["port"].as<int>();

            if (connect_address == "self") {
                connect_address = "127.0.0.1";
            } else if (ip_map.find(connect_address) != ip_map.end()) {
                connect_address = ip_map[connect_address];
            } else {
                ROS_ERROR("Invalid connect_address '%s' for request_service %s", connect_address.c_str(), service_name.c_str());
                continue;
            }

            startRequestService(service_name, service_type, connect_address, port);
        }
    }

    // Start request processing thread
    service_threads_.emplace_back([this]() {
        while (ros::ok()) {
            processRequests();
            ros::Duration(0.01).sleep(); // 10ms interval to avoid high CPU usage
        }
    });
}

std::shared_ptr<ServiceHandler> ServiceManager::createHandler(const std::string& service_type, const std::string& service_name) {
    if (service_type == "std_srvs/SetBool") {
        return std::make_shared<SpecificServiceHandler<std_srvs::SetBool>>(service_name);
    } else if (service_type == "nav_msgs/GetPlan") {
        return std::make_shared<SpecificServiceHandler<nav_msgs::GetPlan>>(service_name);
    } else {
        ROS_ERROR("Unsupported service type: %s", service_type.c_str());
        return nullptr;
    }
}

void ServiceManager::startProvideService(const std::string& service_name,
                                        const std::string& service_type,
                                        const std::string& bind_address,
                                        int port) {
    zmq::socket_t rep_socket(context_, ZMQ_REP);
    std::string address = "tcp://" + bind_address + ":" + std::to_string(port);
    try {
        rep_socket.bind(address);
    } catch (const zmq::error_t& e) {
        ROS_ERROR("Failed to bind service %s to %s: %s", service_name.c_str(), address.c_str(), e.what());
        return;
    }

    rep_sockets_.emplace_back(std::move(rep_socket));
    auto& current_socket = rep_sockets_.back();

    service_threads_.emplace_back([this, &current_socket]() {
        while (ros::ok()) {
            zmq::pollitem_t items[] = {{static_cast<void*>(current_socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(items, 1, 100); // 100ms timeout
            if (!ros::ok()) break;
            if (items[0].revents & ZMQ_POLLIN) {
                zmq::message_t request;
                if (current_socket.recv(request)) {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    request_queue_.push({std::move(request), &current_socket});
                }
            }
        }
    });
}

void ServiceManager::startRequestService(const std::string& service_name,
                                        const std::string& service_type,
                                        const std::string& connect_address,
                                        int port) {
    zmq::socket_t req_socket(context_, ZMQ_REQ);
    std::string address = "tcp://" + connect_address + ":" + std::to_string(port);
    try {
        req_socket.connect(address);
    } catch (const zmq::error_t& e) {
        ROS_ERROR("Failed to connect to service %s at %s: %s", service_name.c_str(), address.c_str(), e.what());
        return;
    }
    req_sockets_[service_name] = std::move(req_socket);
}

void ServiceManager::processRequests() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!request_queue_.empty()) {
        auto req_data = std::move(request_queue_.front());
        request_queue_.pop();

        zmq::socket_t* rep_socket = req_data.rep_socket;
        auto it = socket_to_handler_.find(rep_socket);
        if (it != socket_to_handler_.end()) {
            it->second->handleRequest(req_data.request, *rep_socket);
        } else {
            ROS_ERROR("No handler found for socket");
            zmq::message_t reply(0);
            rep_socket->send(reply, zmq::send_flags::none);
        }
    }
}

template<typename ServiceType>
bool ServiceManager::callService(const std::string& service_name,
                                typename ServiceType::Request& req,
                                typename ServiceType::Response& res) {
    auto it = req_sockets_.find(service_name);
    if (it == req_sockets_.end()) {
        ROS_ERROR("Service %s not found in request_services", service_name.c_str());
        return false;
    }
    zmq::socket_t& req_socket = it->second;

    auto buffer = serializeMsg(req);
    zmq::message_t zmq_req(buffer.size());
    memcpy(zmq_req.data(), buffer.data(), buffer.size());

    if (!req_socket.send(zmq_req, zmq::send_flags::none)) {
        ROS_ERROR("Failed to send request for service %s", service_name.c_str());
        return false;
    }

    zmq::message_t zmq_res;
    if (!req_socket.recv(zmq_res)) {
        ROS_ERROR("Failed to receive response for service %s", service_name.c_str());
        return false;
    }

    res = deserializeMsg<typename ServiceType::Response>(
        static_cast<uint8_t*>(zmq_res.data()), zmq_res.size());
    return true;
}

// Explicit template instantiations
template bool ServiceManager::callService<std_srvs::SetBool>(
    const std::string&, std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
template bool ServiceManager::callService<nav_msgs::GetPlan>(
    const std::string&, nav_msgs::GetPlan::Request&, nav_msgs::GetPlan::Response&);

void ServiceManager::displayConfig(const YAML::Node& config) {
    std::cout << BLUE << "-------provide_services-------" << RESET << std::endl;
    if (config["provide_services"]) {
        for (const auto& service : config["provide_services"]) {
            std::string service_name = service["service_name"].as<std::string>();
            std::string bind_address = service["bind_address"].as<std::string>();
            std::cout << GREEN << service_name << " (bind at " << bind_address << ")" << RESET << std::endl;
        }
    }
    std::cout << BLUE << "-------request_services-------" << RESET << std::endl;
    if (config["request_services"]) {
        for (const auto& service : config["request_services"]) {
            std::string service_name = service["service_name"].as<std::string>();
            std::string connect_address = service["connect_address"].as<std::string>();
            std::cout << GREEN << service_name << " (connect to " << connect_address << ")" << RESET << std::endl;
        }
    }
}

std::string ServiceManager::getLocalIP() {
    struct ifaddrs *ifaddr;
    std::string local_ip = "127.0.0.1";

    if (getifaddrs(&ifaddr) == -1) {
        ROS_ERROR("Failed to get local IP address: %s", strerror(errno));
        return local_ip;
    }

    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
            char *ip = inet_ntoa(addr->sin_addr);
            if (strcmp(ip, "127.0.0.1") != 0) {
                local_ip = ip;
                break;
            }
        }
    }

    freeifaddrs(ifaddr);
    if (local_ip == "127.0.0.1") {
        ROS_WARN("No non-loopback IP found, using 127.0.0.1");
    }
    return local_ip;
}

} // namespace multibotnet