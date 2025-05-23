#ifndef MULTIBOTNET_TRANSPORT_ZMQ_TRANSPORT_HPP
#define MULTIBOTNET_TRANSPORT_ZMQ_TRANSPORT_HPP

#include <zmq.hpp>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include "multibotnet/core/types.hpp"

namespace multibotnet {

/**
 * @brief ZMQ传输层封装类
 * 
 * 提供可靠的消息传输、自动重连、健康检查等功能
 */
class ZmqTransport {
public:
    enum class SocketType {
        PUB = 1,      // ZMQ_PUB
        SUB = 2,      // ZMQ_SUB
        REQ = 3,      // ZMQ_REQ
        REP = 4,      // ZMQ_REP
        DEALER = 5,   // ZMQ_DEALER
        ROUTER = 6    // ZMQ_ROUTER
    };
    
    /**
     * @brief 构造函数
     * @param context ZMQ上下文
     * @param socket_type 套接字类型
     * @param identity 套接字标识（可选）
     */
    ZmqTransport(zmq::context_t& context, 
                 SocketType socket_type,
                 const std::string& identity = "");
    
    ~ZmqTransport();
    
    /**
     * @brief 绑定到地址
     * @param address 绑定地址（如 "tcp://*:5555"）
     * @return 是否成功
     */
    bool bind(const std::string& address);
    
    /**
     * @brief 连接到地址
     * @param address 连接地址（如 "tcp://localhost:5555"）
     * @return 是否成功
     */
    bool connect(const std::string& address);
    
    /**
     * @brief 发送消息
     * @param data 消息数据
     * @param flags 发送标志
     * @return 是否成功
     */
    bool send(const std::vector<uint8_t>& data, int flags = 0);
    
    /**
     * @brief 批量发送消息
     * @param batch 消息批次
     * @return 是否成功
     */
    bool sendBatch(const MessageBatch& batch);
    
    /**
     * @brief 接收消息
     * @param data 接收缓冲区
     * @param timeout_ms 超时时间（毫秒）
     * @return 是否成功
     */
    bool receive(std::vector<uint8_t>& data, int timeout_ms = -1);
    
    /**
     * @brief 批量接收消息
     * @param batch 消息批次
     * @param max_messages 最大消息数
     * @param timeout_ms 超时时间
     * @return 接收到的消息数
     */
    size_t receiveBatch(MessageBatch& batch, size_t max_messages, int timeout_ms = -1);
    
    /**
     * @brief 设置套接字选项
     * @param option 选项名
     * @param value 选项值
     */
    template<typename T>
    void setOption(int option, const T& value);
    
    /**
     * @brief 订阅消息（用于SUB套接字）
     * @param filter 过滤器字符串
     */
    void subscribe(const std::string& filter = "");
    
    /**
     * @brief 获取连接状态
     * @return 当前连接状态
     */
    ConnectionState getState() const { return state_; }
    
    /**
     * @brief 获取统计信息
     * @return 统计数据
     */
    const Statistics& getStatistics() const { return stats_; }
    
    /**
     * @brief 重置统计信息
     */
    void resetStatistics();
    
    /**
     * @brief 启用/禁用健康检查
     * @param enable 是否启用
     * @param interval_ms 检查间隔（毫秒）
     */
    void enableHealthCheck(bool enable, int interval_ms = 1000);
    
    /**
     * @brief 设置重连策略
     * @param max_retries 最大重试次数
     * @param retry_interval_ms 重试间隔（毫秒）
     */
    void setReconnectPolicy(int max_retries, int retry_interval_ms);
    
private:
    zmq::context_t& context_;  // 保存context引用
    zmq::socket_t socket_;
    SocketType socket_type_;
    std::string identity_;
    std::atomic<ConnectionState> state_;
    Statistics stats_;
    
    // 健康检查
    std::atomic<bool> health_check_enabled_;
    std::thread health_check_thread_;
    std::atomic<bool> running_;
    int health_check_interval_ms_;
    
    // 重连策略
    int max_retries_;
    int retry_interval_ms_;
    std::string last_address_;
    bool is_bind_;
    
    // 内部方法
    void healthCheckLoop();
    bool checkConnection();
    void updateStatistics(size_t bytes, bool is_send);
    bool reconnect();
};

// 模板实现
template<typename T>
void ZmqTransport::setOption(int option, const T& value) {
    socket_.setsockopt(option, &value, sizeof(T));
}

} // namespace multibotnet

#endif // MULTIBOTNET_TRANSPORT_ZMQ_TRANSPORT_HPP