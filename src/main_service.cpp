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
    ros::Duration(1).sleep();//延时启动服务通信
    multibotnet::ServiceManager service_manager;
    service_manager.init(config_file);
    
        // 指定远程服务名称并准备请求数据
        std::string service_name = "/remote_set_bool";
        std_srvs::SetBool::Request req;
        std_srvs::SetBool::Response res;
        req.data = true;  // 设置请求数据，例如这里设置为true
        while (ros::ok())
        {
                // 调用远程服务
            if (service_manager.callService<std_srvs::SetBool>(service_name, req, res)) {
                ROS_INFO("OK!!!: %s", res.message.c_str());
            } else {
                ROS_ERROR("Fault");
            }
            ros::Duration(1).sleep();
        }
        
    ros::spin();
    return 0;
}
