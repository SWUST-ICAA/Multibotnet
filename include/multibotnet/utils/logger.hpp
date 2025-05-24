#ifndef MULTIBOTNET_UTILS_LOGGER_HPP
#define MULTIBOTNET_UTILS_LOGGER_HPP

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <mutex>

namespace multibotnet {

// ANSI颜色代码
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

/**
 * @brief 日志级别
 */
enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3,
    FATAL = 4
};

/**
 * @brief 日志管理器
 */
class Logger {
public:
    static Logger& getInstance();
    
    /**
     * @brief 设置日志级别
     * @param level 日志级别
     */
    void setLevel(LogLevel level) { level_ = level; }
    
    /**
     * @brief 获取当前日志级别
     */
    LogLevel getLevel() const { return level_; }
    
    /**
     * @brief 启用/禁用彩色输出
     * @param enable 是否启用
     */
    void setColorEnabled(bool enable) { color_enabled_ = enable; }
    
    /**
     * @brief 日志输出
     */
    void log(LogLevel level, const std::string& msg, 
             const std::string& file = "", int line = 0);
    
    // 便捷方法
    void debug(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::DEBUG, msg, file, line);
    }
    
    void info(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::INFO, msg, file, line);
    }
    
    void warn(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::WARN, msg, file, line);
    }
    
    void error(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::ERROR, msg, file, line);
    }
    
    void fatal(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::FATAL, msg, file, line);
    }
    
    /**
     * @brief 格式化输出
     */
    template<typename... Args>
    void logf(LogLevel level, const std::string& format, Args... args) {
        char buffer[1024];
        snprintf(buffer, sizeof(buffer), format.c_str(), args...);
        log(level, std::string(buffer));
    }
    
private:
    Logger() : level_(LogLevel::INFO), color_enabled_(true) {}  // 默认设置为INFO级别，不显示DEBUG
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    LogLevel level_;
    bool color_enabled_;
    std::mutex mutex_;
    
    std::string levelToString(LogLevel level) const;
    std::string levelToColor(LogLevel level) const;
};

// 宏定义，自动添加文件名和行号 - 修改为不传递文件名和行号
#define LOG_DEBUG(msg) multibotnet::Logger::getInstance().debug(msg)
#define LOG_INFO(msg) multibotnet::Logger::getInstance().info(msg)
#define LOG_WARN(msg) multibotnet::Logger::getInstance().warn(msg)
#define LOG_ERROR(msg) multibotnet::Logger::getInstance().error(msg)
#define LOG_FATAL(msg) multibotnet::Logger::getInstance().fatal(msg)

#define LOG_DEBUGF(fmt, ...) multibotnet::Logger::getInstance().logf(multibotnet::LogLevel::DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFOF(fmt, ...) multibotnet::Logger::getInstance().logf(multibotnet::LogLevel::INFO, fmt, ##__VA_ARGS__)
#define LOG_WARNF(fmt, ...) multibotnet::Logger::getInstance().logf(multibotnet::LogLevel::WARN, fmt, ##__VA_ARGS__)
#define LOG_ERRORF(fmt, ...) multibotnet::Logger::getInstance().logf(multibotnet::LogLevel::ERROR, fmt, ##__VA_ARGS__)

/**
 * @brief 带颜色的输出流
 */
class ColorStream {
public:
    ColorStream(const std::string& color) : color_(color) {}
    
    template<typename T>
    ColorStream& operator<<(const T& value) {
        if (Logger::getInstance().getLevel() <= LogLevel::INFO) {
            std::cout << color_ << value << RESET;
        }
        return *this;
    }
    
    ColorStream& operator<<(std::ostream& (*pf)(std::ostream&)) {
        if (Logger::getInstance().getLevel() <= LogLevel::INFO) {
            std::cout << pf;
        }
        return *this;
    }
    
private:
    std::string color_;
};

// 全局颜色流函数（修复语法）
inline ColorStream blue() { return ColorStream(BLUE); }
inline ColorStream green() { return ColorStream(GREEN); }
inline ColorStream yellow() { return ColorStream(YELLOW); }
inline ColorStream red() { return ColorStream(RED); }
inline ColorStream purple() { return ColorStream(PURPLE); }

// 为了兼容性，保留对象式的接口
namespace color {
    inline ColorStream blue() { return ColorStream(BLUE); }
    inline ColorStream green() { return ColorStream(GREEN); }
    inline ColorStream yellow() { return ColorStream(YELLOW); }
    inline ColorStream red() { return ColorStream(RED); }
    inline ColorStream purple() { return ColorStream(PURPLE); }
}

} // namespace multibotnet

#endif // MULTIBOTNET_UTILS_LOGGER_HPP