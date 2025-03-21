#include <ros/ros.h>              
#include <multibotnet/service_manager.hpp> 

int main(int argc, char** argv) {

    ros::init(argc, argv, "multibotnet_service_node");
    

    ros::NodeHandle nh("~");
    
    // 从参数服务器获取配置文件路径
    std::string config_file;
    if (!nh.getParam("config_file", config_file)) {
        // 如果获取失败，输出错误信息并退出
        ROS_ERROR("参数配置问题，请确保在 launch 文件中正确设置该参数");
        return 1;
    }
    
    // 创建ServiceManager对象，用于管理服务
    multibotnet::ServiceManager service_manager;
    
    // 初始化ServiceManager，传入配置文件路径以加载配置
    service_manager.init(config_file);
    
    // 进入ROS事件循环，处理服务请求
    ros::spin();
    return 0;
}