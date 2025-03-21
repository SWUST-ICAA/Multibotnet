#include <ros/ros.h>
#include <multibotnet/service_manager.hpp>

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

    ros::spin();
    return 0;
}