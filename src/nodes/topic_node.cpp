#include <ros/ros.h>
#include <signal.h>
#include "multibotnet/managers/topic_manager.hpp"
#include "multibotnet/utils/logger.hpp"

// ANSI颜色代码
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

namespace {
    std::unique_ptr<multibotnet::TopicManager> g_topic_manager;
    ros::Timer g_stats_timer;
}

// 信号处理函数
void signalHandler(int sig) {
    if (sig == SIGINT) {
        ROS_INFO("Shutting down multibotnet_topic_node...");
        if (g_topic_manager) {
            g_topic_manager->stop();
        }
        ros::shutdown();
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "multibotnet_topic_node", 
              ros::init_options::NoSigintHandler);
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    
    ros::NodeHandle nh("~");
    
    // 从参数服务器获取配置文件路径
    std::string config_file;
    if (!nh.getParam("config_file", config_file)) {
        ROS_ERROR("The parameter configuration is incorrect. "
                  "Please ensure that the parameter is correctly set in the launch file");
        return 1;
    }
    
    // 设置日志级别
    std::string log_level_str = "info";
    nh.param("log_level", log_level_str, std::string("info"));
    
    if (log_level_str == "debug") {
        multibotnet::Logger::getInstance().setLevel(multibotnet::LogLevel::DEBUG);
        ROS_INFO("Log level set to DEBUG");
    } else if (log_level_str == "info") {
        multibotnet::Logger::getInstance().setLevel(multibotnet::LogLevel::INFO);
    } else if (log_level_str == "warn") {
        multibotnet::Logger::getInstance().setLevel(multibotnet::LogLevel::WARN);
    } else if (log_level_str == "error") {
        multibotnet::Logger::getInstance().setLevel(multibotnet::LogLevel::ERROR);
    }
    
    // 打印启动横幅
    std::cout << CYAN
              << " __  __       _ _   _ _           _   _   _      _   \n"
              << "|  \\/  |_   _| | |_(_) |__   ___ | |_| \\ | | ___| |_ \n"
              << "| |\\/| | | | | | __| | '_ \\ / _ \\| __|  \\| |/ _ \\ __|\n"
              << "| |  | | |_| | | |_| | |_) | (_) | |_| |\\  |  __/ |_ \n"
              << "|_|  |_|\\__,_|_|\\__|_|_.__/ \\___/ \\__|_| \\_|\\___|\\__|\n"
              << "                                                      \n"
              << "            Topic Communication Node v4.0.0           \n"
              << RESET << std::endl;
    
    try {
        // 创建话题管理器
        g_topic_manager = std::make_unique<multibotnet::TopicManager>();
        
        // 初始化
        if (!g_topic_manager->init(config_file)) {
            ROS_ERROR("Failed to initialize TopicManager");
            return 1;
        }
        
        // 启动
        g_topic_manager->start();
        
        // 设置统计信息打印定时器（如果需要）
        bool print_statistics = false;
        nh.param("print_statistics", print_statistics, false);
        
        if (print_statistics) {
            double stats_interval = 5.0;  // 默认5秒
            nh.param("statistics_interval", stats_interval, 5.0);
            
            // 修改：首次打印延迟5秒，之后每隔stats_interval秒打印一次
            g_stats_timer = nh.createTimer(
                ros::Duration(stats_interval),
                [](const ros::TimerEvent&) {
                    if (g_topic_manager) {
                        g_topic_manager->printStatistics();
                    }
                },
                false,  // oneshot = false
                true    // autostart = true
            );
        }
        
        ROS_INFO("Multibotnet topic node is running...");
        
        // 进入ROS事件循环
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    // 清理
    if (g_topic_manager) {
        g_topic_manager->stop();
    }
    
    return 0;
}