#ifndef MULTIBOTNET_TRANSPORT_CONNECTION_POOL_HPP
#define MULTIBOTNET_TRANSPORT_CONNECTION_POOL_HPP

#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include "multibotnet/transport/zmq_transport.hpp"

namespace multibotnet {

/**
 * @brief 连接池配置
 */
struct ConnectionPoolConfig {
    size_t min_connections = 1;        // 最小连接数
    size_t max_connections = 10;       // 最大连接数
    int idle_timeout_ms = 60000;       // 空闲超时（毫秒）
    int connection_timeout_ms = 5000;  // 连接超时（毫秒）
    bool enable_health_check = true;   // 启用健康检查
    int health_check_interval_ms = 5000; // 健康检查间隔
};

/**
 * @brief 连接包装器
 */
class PooledConnection {
public:
    PooledConnection(std::shared_ptr<ZmqTransport> transport,
                    const std::string& address,
                    std::chrono::steady_clock::time_point created_time);
    
    /**
     * @brief 获取传输对象
     */
    std::shared_ptr<ZmqTransport> getTransport() { return transport_; }
    
    /**
     * @brief 检查连接是否空闲超时
     */
    bool isIdleTimeout(int timeout_ms) const;
    
    /**
     * @brief 更新最后使用时间
     */
    void updateLastUsed();
    
    /**
     * @brief 获取连接地址
     */
    const std::string& getAddress() const { return address_; }
    
    /**
     * @brief 标记为正在使用
     */
    void setInUse(bool in_use) { in_use_ = in_use; }
    
    /**
     * @brief 是否正在使用
     */
    bool isInUse() const { return in_use_; }
    
private:
    std::shared_ptr<ZmqTransport> transport_;
    std::string address_;
    std::chrono::steady_clock::time_point created_time_;
    std::chrono::steady_clock::time_point last_used_time_;
    std::atomic<bool> in_use_;
};

/**
 * @brief 连接池管理器
 */
class ConnectionPool {
public:
    /**
     * @brief 构造函数
     * @param context ZMQ上下文
     * @param socket_type 套接字类型
     * @param config 连接池配置
     */
    ConnectionPool(zmq::context_t& context,
                  ZmqTransport::SocketType socket_type,
                  const ConnectionPoolConfig& config = ConnectionPoolConfig());
    
    ~ConnectionPool();
    
    /**
     * @brief 获取连接
     * @param address 连接地址
     * @param timeout_ms 获取超时（毫秒）
     * @return 连接对象（使用完后自动归还）
     */
    class ConnectionGuard {
    public:
        ConnectionGuard(std::shared_ptr<PooledConnection> conn, ConnectionPool* pool);
        ~ConnectionGuard();
        
        ConnectionGuard(const ConnectionGuard&) = delete;
        ConnectionGuard& operator=(const ConnectionGuard&) = delete;
        
        ConnectionGuard(ConnectionGuard&& other) noexcept;
        ConnectionGuard& operator=(ConnectionGuard&& other) noexcept;
        
        std::shared_ptr<ZmqTransport> operator->() { return conn_->getTransport(); }
        std::shared_ptr<ZmqTransport> get() { return conn_->getTransport(); }
        
    private:
        std::shared_ptr<PooledConnection> conn_;
        ConnectionPool* pool_;
    };
    
    ConnectionGuard getConnection(const std::string& address, int timeout_ms = -1);
    
    /**
     * @brief 预创建连接
     * @param address 连接地址
     * @param count 创建数量
     */
    void preCreateConnections(const std::string& address, size_t count);
    
    /**
     * @brief 清理空闲连接
     */
    void cleanupIdleConnections();
    
    /**
     * @brief 获取连接池状态
     * @return 各地址的连接数
     */
    std::unordered_map<std::string, size_t> getPoolStatus() const;
    
    /**
     * @brief 关闭所有连接
     */
    void shutdown();
    
private:
    zmq::context_t& context_;
    ZmqTransport::SocketType socket_type_;
    ConnectionPoolConfig config_;
    
    // 连接池（按地址分组）
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::queue<std::shared_ptr<PooledConnection>>> idle_connections_;
    std::unordered_map<std::string, std::vector<std::shared_ptr<PooledConnection>>> all_connections_;
    std::condition_variable cv_available_;
    
    // 后台清理线程
    std::thread cleanup_thread_;
    std::atomic<bool> running_;
    
    // 内部方法
    std::shared_ptr<PooledConnection> createConnection(const std::string& address);
    void returnConnection(std::shared_ptr<PooledConnection> conn);
    void cleanupLoop();
    void removeConnection(std::shared_ptr<PooledConnection> conn);
};

/**
 * @brief 负载均衡器
 */
class LoadBalancer {
public:
    enum class Strategy {
        ROUND_ROBIN,    // 轮询
        LEAST_LOADED,   // 最少负载
        RANDOM,         // 随机
        WEIGHTED        // 加权
    };
    
    LoadBalancer(Strategy strategy = Strategy::ROUND_ROBIN);
    
    /**
     * @brief 添加目标地址
     * @param address 地址
     * @param weight 权重（用于加权策略）
     */
    void addTarget(const std::string& address, int weight = 1);
    
    /**
     * @brief 移除目标地址
     * @param address 地址
     */
    void removeTarget(const std::string& address);
    
    /**
     * @brief 选择下一个目标
     * @param load_info 负载信息（用于最少负载策略）
     * @return 选中的地址
     */
    std::string selectTarget(const std::unordered_map<std::string, int>& load_info = {});
    
    /**
     * @brief 更新目标健康状态
     * @param address 地址
     * @param healthy 是否健康
     */
    void updateHealth(const std::string& address, bool healthy);
    
private:
    Strategy strategy_;
    std::vector<std::pair<std::string, int>> targets_;  // 地址和权重
    std::unordered_map<std::string, bool> health_status_;
    std::atomic<size_t> round_robin_index_;
    mutable std::mutex mutex_;
};

} // namespace multibotnet

#endif // MULTIBOTNET_TRANSPORT_CONNECTION_POOL_HPP