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
    
    // 文件和行号（如果提供）
    if (!file.empty() && line > 0) {
        // 只显示文件名，不显示完整路径
        size_t pos = file.find_last_of("/\\");
        std::string filename = (pos != std::string::npos) ? 
                              file.substr(pos + 1) : file;
        ss << " [" << filename << ":" << line << "]";
    }
    
    // 消息内容
    ss << " " << msg;
    
    // 输出到控制台或ROS日志系统
    switch (level) {
        case LogLevel::DEBUG:
            ROS_DEBUG_STREAM(ss.str());
            break;
        case LogLevel::INFO:
            ROS_INFO_STREAM(ss.str());
            break;
        case LogLevel::WARN:
            ROS_WARN_STREAM(ss.str());
            break;
        case LogLevel::ERROR:
            ROS_ERROR_STREAM(ss.str());
            break;
        case LogLevel::FATAL:
            ROS_FATAL_STREAM(ss.str());
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