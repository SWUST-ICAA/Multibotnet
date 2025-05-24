#include <ros/ros.h>
#include <signal.h>
#include "multibotnet/managers/service_manager.hpp"
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
    std::unique_ptr<multibotnet::ServiceManager> g_service_manager;
    ros::Timer g_stats_timer;
    ros::Timer g_initial_delay_timer;
}

// 信号处理函数
void signalHandler(int sig) {
    if (sig == SIGINT) {
        ROS_INFO("Shutting down multibotnet_service_node...");
        if (g_service_manager) {
            g_service_manager->stop();
        }
        ros::shutdown();
    }
}

// 打印统计信息
void printServiceStatistics() {
    if (!g_service_manager) return;
    
    // 使用新的 printStatistics 方法
    g_service_manager->printStatistics();
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "multibotnet_service_node", 
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
    
    // 延时启动（给话题节点时间启动）
    ros::Duration(2).sleep();
    
    // 打印启动横幅
    std::cout << CYAN
              << " __  __       _ _   _ _           _   _   _      _   \n"
              << "|  \\/  |_   _| | |_(_) |__   ___ | |_| \\ | | ___| |_ \n"
              << "| |\\/| | | | | | __| | '_ \\ / _ \\| __|  \\| |/ _ \\ __|\n"
              << "| |  | | |_| | | |_| | |_) | (_) | |_| |\\  |  __/ |_ \n"
              << "|_|  |_|\\__,_|_|\\__|_|_.__/ \\___/ \\__|_| \\_|\\___|\\__|\n"
              << "                                                      \n"
              << "           Service Communication Node v4.0.0          \n"
              << RESET << std::endl;
    
    try {
        // 创建服务管理器
        g_service_manager = std::make_unique<multibotnet::ServiceManager>();
        
        // 初始化
        if (!g_service_manager->init(config_file)) {
            ROS_ERROR("Failed to initialize ServiceManager");
            return 1;
        }
        
        // 启动
        g_service_manager->start();
        
        // 设置统计信息打印定时器（如果需要）
        bool print_statistics = false;
        nh.param("print_statistics", print_statistics, false);
        
        if (print_statistics) {
            double stats_interval = 5.0;  // 默认5秒
            nh.param("statistics_interval", stats_interval, 5.0);
            
            // 修改：将服务统计的延迟改为5.5秒，确保在话题统计之后
            g_initial_delay_timer = nh.createTimer(
                ros::Duration(5.5),  // 延迟5.5秒开始，确保在话题统计（5秒）之后
                [&nh, stats_interval](const ros::TimerEvent&) {
                    // 停止初始延迟定时器
                    g_initial_delay_timer.stop();
                    
                    // 创建周期性统计定时器
                    g_stats_timer = nh.createTimer(
                        ros::Duration(stats_interval),
                        [](const ros::TimerEvent&) {
                            printServiceStatistics();
                        }
                    );
                    
                    // 立即打印一次
                    printServiceStatistics();
                },
                true  // oneshot = true
            );
        }
        
        ROS_INFO("Multibotnet service node is running...");
        
        // 进入ROS事件循环
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    
    // 清理
    if (g_service_manager) {
        g_service_manager->stop();
    }
    
    return 0;
}