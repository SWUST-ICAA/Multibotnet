#include <ros/ros.h>            
#include <multibotnet/zmq_manager.hpp> 
#include <iostream>

#define PURPLE "\033[35m"
#define RESET "\033[0m"

int main(int argc, char** argv) {

    ros::init(argc, argv, "multibotnet_topic_node");
    

    ros::NodeHandle nh("~");
    
    // 从参数服务器获取配置文件路径
    std::string config_file;
    if (!nh.getParam("config_file", config_file)) {
        // 如果获取失败，输出错误信息并退出
        ROS_ERROR("参数配置问题，请确保在 launch 文件中正确设置该参数");
        return 1;
    }
    
    std::cout << PURPLE
              << " _   _                                \n"
              << "| \\ | | __ _ _ ____      ____ _ _ __  \n"
              << "|  \\| |/ _` | '_ \\ \\ /\\ / / _` | '_ \\ \n"
              << "| |\\  | (_| | | | \\ V  V / (_| | | | |\n"
              << "|_| \\_|\\__,_|_| |_|\\_/\\_/ \\__,_|_| |_|\n"
              << RESET << std::endl;

    // 创建ZmqManager对象，用于管理ZeroMQ通信
    multibotnet::ZmqManager zmq_manager;
    
    // 初始化ZmqManager，传入配置文件路径以加载配置
    zmq_manager.init(config_file);
    
    // 进入ROS事件循环，处理回调和消息
    ros::spin();
    return 0;
}