// main_service.cpp

#include <ros/ros.h>
#include <multibotnet/service_manager.hpp>
#include <std_srvs/SetBool.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "multibotnet_service_node");
    ros::NodeHandle nh("~");

    std::string config_file;
    if (!nh.getParam("config_file", config_file)) {
        ROS_ERROR("参数配置问题，请确保在 launch 文件中正确设置该参数");
        return 1;
    }

    multibotnet::ServiceManager service_manager;
    service_manager.init(config_file);

    // 示例：调用远程服务
    std_srvs::SetBool::Request req;
    req.data = true;
    std_srvs::SetBool::Response res;
    if (service_manager.callService<std_srvs::SetBool>("/remote_set_bool", req, res)) {
        ROS_INFO("Service call successful: %s", res.message.c_str());
    } else {
        ROS_ERROR("Service call failed");
    }

    ros::spin();
    return 0;
}