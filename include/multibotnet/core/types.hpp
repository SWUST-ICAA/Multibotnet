#ifndef MULTIBOTNET_CORE_TYPES_HPP
#define MULTIBOTNET_CORE_TYPES_HPP

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

namespace multibotnet {

// 消息批处理结构
struct MessageBatch {
    std::vector<std::vector<uint8_t>> messages;
    std::chrono::steady_clock::time_point timestamp;
    size_t total_size = 0;
};

// 话题配置
struct TopicConfig {
    std::string topic;
    std::string message_type;
    int max_frequency;
    std::string address;  // bind_address 或 connect_address
    int port;
    bool enable_compression = true;
    bool enable_batch = true;
    int batch_size = 10;
    int batch_timeout_ms = 100;
};

// 服务配置
struct ServiceConfig {
    std::string service_name;
    std::string service_type;
    std::string address;
    int port;
    int timeout_ms = 5000;
    int max_retries = 3;
};

// 连接状态
enum class ConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
};

// 统计信息
struct Statistics {
    uint64_t messages_sent = 0;
    uint64_t messages_received = 0;
    uint64_t bytes_sent = 0;
    uint64_t bytes_received = 0;
    uint64_t compression_ratio = 0;
    uint64_t errors = 0;
    std::chrono::steady_clock::time_point start_time;
};

// 回调函数类型
using MessageCallback = std::function<void(const std::vector<uint8_t>&)>;
using ServiceCallback = std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>;

} // namespace multibotnet

#endif // MULTIBOTNET_CORE_TYPES_HPP