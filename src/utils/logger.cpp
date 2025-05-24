#include "multibotnet/utils/logger.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace multibotnet {

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

void Logger::log(LogLevel level, const std::string& msg, 
                const std::string& file, int line) {
    if (level < level_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    // 构建日志消息
    std::stringstream ss;
    
    // 时间戳
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    
    // 日志级别
    if (color_enabled_) {
        ss << " " << levelToColor(level) << "[" << levelToString(level) << "]" << RESET;
    } else {
        ss << " [" << levelToString(level) << "]";
    }
    
    // 文件和行号（如果提供）- 注释掉这部分，不再显示文件名和行号
    /*
    if (!file.empty() && line > 0) {
        // 只显示文件名，不显示完整路径
        size_t pos = file.find_last_of("/\\");
        std::string filename = (pos != std::string::npos) ? 
                              file.substr(pos + 1) : file;
        ss << " [" << filename << ":" << line << "]";
    }
    */
    
    // 消息内容
    ss << " " << msg;
    
    // 直接输出到控制台，不使用ROS日志系统以避免额外的时间戳
    switch (level) {
        case LogLevel::DEBUG:
            if (color_enabled_) {
                std::cout << ss.str() << std::endl;
            } else {
                std::cout << ss.str() << std::endl;
            }
            break;
        case LogLevel::INFO:
            std::cout << ss.str() << std::endl;
            break;
        case LogLevel::WARN:
            std::cout << ss.str() << std::endl;
            break;
        case LogLevel::ERROR:
            std::cerr << ss.str() << std::endl;
            break;
        case LogLevel::FATAL:
            std::cerr << ss.str() << std::endl;
            break;
    }
}

std::string Logger::levelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

std::string Logger::levelToColor(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG: return CYAN;
        case LogLevel::INFO:  return GREEN;
        case LogLevel::WARN:  return YELLOW;
        case LogLevel::ERROR: return RED;
        case LogLevel::FATAL: return PURPLE;
        default: return RESET;
    }
}

} // namespace multibotnet