#include "multibotnet/transport/connection_pool.hpp"
#include "multibotnet/utils/logger.hpp"
#include <random>

namespace multibotnet {

// PooledConnection 实现
PooledConnection::PooledConnection(std::shared_ptr<ZmqTransport> transport,
                                 const std::string& address,
                                 std::chrono::steady_clock::time_point created_time)
    : transport_(transport),
      address_(address),
      created_time_(created_time),
      last_used_time_(created_time),
      in_use_(false) {
}

bool PooledConnection::isIdleTimeout(int timeout_ms) const {
    auto now = std::chrono::steady_clock::now();
    auto idle_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_used_time_).count();
    return idle_time > timeout_ms;
}

void PooledConnection::updateLastUsed() {
    last_used_time_ = std::chrono::steady_clock::now();
}

// ConnectionPool::ConnectionGuard 实现
ConnectionPool::ConnectionGuard::ConnectionGuard(
    std::shared_ptr<PooledConnection> conn, ConnectionPool* pool)
    : conn_(conn), pool_(pool) {
}

ConnectionPool::ConnectionGuard::~ConnectionGuard() {
    if (conn_ && pool_) {
        pool_->returnConnection(conn_);
    }
}

ConnectionPool::ConnectionGuard::ConnectionGuard(ConnectionGuard&& other) noexcept
    : conn_(std::move(other.conn_)), pool_(other.pool_) {
    other.pool_ = nullptr;
}

ConnectionPool::ConnectionGuard& ConnectionPool::ConnectionGuard::operator=(
    ConnectionGuard&& other) noexcept {
    if (this != &other) {
        if (conn_ && pool_) {
            pool_->returnConnection(conn_);
        }
        conn_ = std::move(other.conn_);
        pool_ = other.pool_;
        other.pool_ = nullptr;
    }
    return *this;
}

// ConnectionPool 实现
ConnectionPool::ConnectionPool(zmq::context_t& context,
                             ZmqTransport::SocketType socket_type,
                             const ConnectionPoolConfig& config)
    : context_(context),
      socket_type_(socket_type),
      config_(config),
      running_(true) {
    
    // 启动清理线程
    cleanup_thread_ = std::thread(&ConnectionPool::cleanupLoop, this);
}

ConnectionPool::~ConnectionPool() {
    shutdown();
}

ConnectionPool::ConnectionGuard ConnectionPool::getConnection(
    const std::string& address, int timeout_ms) {
    
    auto deadline = std::chrono::steady_clock::now() + 
                   std::chrono::milliseconds(timeout_ms);
    
    std::unique_lock<std::mutex> lock(mutex_);
    
    while (true) {
        // 检查是否有空闲连接
        auto& idle_queue = idle_connections_[address];
        if (!idle_queue.empty()) {
            auto conn = idle_queue.front();
            idle_queue.pop();
            
            // 检查连接是否健康
            if (conn->getTransport()->getState() == ConnectionState::CONNECTED) {
                conn->setInUse(true);
                conn->updateLastUsed();
                return ConnectionGuard(conn, this);
            } else {
                // 连接不健康，移除并继续
                removeConnection(conn);
                continue;
            }
        }
        
        // 检查是否可以创建新连接
        auto& all_conns = all_connections_[address];
        if (all_conns.size() < config_.max_connections) {
            lock.unlock();
            auto conn = createConnection(address);
            lock.lock();
            
            if (conn) {
                conn->setInUse(true);
                return ConnectionGuard(conn, this);
            }
        }
        
        // 等待连接可用
        if (timeout_ms < 0) {
            cv_available_.wait(lock);
        } else {
            if (cv_available_.wait_until(lock, deadline) == std::cv_status::timeout) {
                LOG_WARN("Connection pool timeout");
                return ConnectionGuard(nullptr, nullptr);
            }
        }
    }
}

void ConnectionPool::preCreateConnections(const std::string& address, size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto& all_conns = all_connections_[address];
    size_t current_count = all_conns.size();
    size_t target_count = std::min(current_count + count, config_.max_connections);
    
    LOG_INFOF("Pre-creating %zu connections for %s", 
             target_count - current_count, address.c_str());
    
    for (size_t i = current_count; i < target_count; i++) {
        auto conn = createConnection(address);
        if (!conn) {
            LOG_ERRORF("Failed to pre-create connection %zu", i);
            break;
        }
    }
}

void ConnectionPool::cleanupIdleConnections() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (auto& pair : idle_connections_) {
        auto& queue = pair.second;
        std::queue<std::shared_ptr<PooledConnection>> new_queue;
        
        while (!queue.empty()) {
            auto conn = queue.front();
            queue.pop();
            
            if (!conn->isIdleTimeout(config_.idle_timeout_ms) &&
                conn->getTransport()->getState() == ConnectionState::CONNECTED) {
                new_queue.push(conn);
            } else {
                LOG_DEBUGF("Removing idle connection to %s", conn->getAddress().c_str());
                removeConnection(conn);
            }
        }
        
        queue = std::move(new_queue);
    }
}

std::unordered_map<std::string, size_t> ConnectionPool::getPoolStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::unordered_map<std::string, size_t> status;
    for (const auto& pair : all_connections_) {
        status[pair.first] = pair.second.size();
    }
    
    return status;
}

void ConnectionPool::shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        cv_available_.notify_all();
    }
    
    if (cleanup_thread_.joinable()) {
        cleanup_thread_.join();
    }
    
    // 关闭所有连接
    std::lock_guard<std::mutex> lock(mutex_);
    idle_connections_.clear();
    all_connections_.clear();
}

std::shared_ptr<PooledConnection> ConnectionPool::createConnection(
    const std::string& address) {
    
    try {
        auto transport = std::make_shared<ZmqTransport>(
            context_, socket_type_);
        
        // 设置健康检查
        if (config_.enable_health_check) {
            transport->enableHealthCheck(true, config_.health_check_interval_ms);
        }
        
        // 连接
        if (!transport->connect(address)) {
            LOG_ERRORF("Failed to connect to %s", address.c_str());
            return nullptr;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto conn = std::make_shared<PooledConnection>(transport, address, now);
        
        // 添加到管理列表
        all_connections_[address].push_back(conn);
        
        LOG_DEBUGF("Created new connection to %s", address.c_str());
        return conn;
    } catch (const std::exception& e) {
        LOG_ERRORF("Exception creating connection: %s", e.what());
        return nullptr;
    }
}

void ConnectionPool::returnConnection(std::shared_ptr<PooledConnection> conn) {
    if (!conn) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    conn->setInUse(false);
    conn->updateLastUsed();
    
    // 检查连接是否仍然健康
    if (conn->getTransport()->getState() == ConnectionState::CONNECTED) {
        idle_connections_[conn->getAddress()].push(conn);
        cv_available_.notify_one();
    } else {
        // 连接不健康，移除
        removeConnection(conn);
    }
}

void ConnectionPool::cleanupLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        if (!running_) break;
        
        cleanupIdleConnections();
    }
}

void ConnectionPool::removeConnection(std::shared_ptr<PooledConnection> conn) {
    if (!conn) return;
    
    auto& all_conns = all_connections_[conn->getAddress()];
    all_conns.erase(
        std::remove(all_conns.begin(), all_conns.end(), conn),
        all_conns.end());
    
    // 如果没有连接了，移除地址条目
    if (all_conns.empty()) {
        all_connections_.erase(conn->getAddress());
        idle_connections_.erase(conn->getAddress());
    }
}

// LoadBalancer 实现
LoadBalancer::LoadBalancer(Strategy strategy)
    : strategy_(strategy), round_robin_index_(0) {
}

void LoadBalancer::addTarget(const std::string& address, int weight) {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.push_back({address, weight});
    health_status_[address] = true;
}

void LoadBalancer::removeTarget(const std::string& address) {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.erase(
        std::remove_if(targets_.begin(), targets_.end(),
                      [&address](const auto& pair) { return pair.first == address; }),
        targets_.end());
    health_status_.erase(address);
}

std::string LoadBalancer::selectTarget(
    const std::unordered_map<std::string, int>& load_info) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 过滤健康的目标
    std::vector<std::pair<std::string, int>> healthy_targets;
    for (const auto& target : targets_) {
        if (health_status_[target.first]) {
            healthy_targets.push_back(target);
        }
    }
    
    if (healthy_targets.empty()) {
        LOG_WARN("No healthy targets available");
        return "";
    }
    
    switch (strategy_) {
        case Strategy::ROUND_ROBIN: {
            size_t index = round_robin_index_.fetch_add(1) % healthy_targets.size();
            return healthy_targets[index].first;
        }
        
        case Strategy::LEAST_LOADED: {
            if (load_info.empty()) {
                // 没有负载信息，退化为轮询
                size_t index = round_robin_index_.fetch_add(1) % healthy_targets.size();
                return healthy_targets[index].first;
            }
            
            std::string least_loaded;
            int min_load = INT_MAX;
            
            for (const auto& target : healthy_targets) {
                auto it = load_info.find(target.first);
                int load = (it != load_info.end()) ? it->second : 0;
                
                if (load < min_load) {
                    min_load = load;
                    least_loaded = target.first;
                }
            }
            
            return least_loaded;
        }
        
        case Strategy::RANDOM: {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, healthy_targets.size() - 1);
            return healthy_targets[dis(gen)].first;
        }
        
        case Strategy::WEIGHTED: {
            // 计算总权重
            int total_weight = 0;
            for (const auto& target : healthy_targets) {
                total_weight += target.second;
            }
            
            // 随机选择
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, total_weight - 1);
            int random_weight = dis(gen);
            
            // 根据权重选择
            int accumulated_weight = 0;
            for (const auto& target : healthy_targets) {
                accumulated_weight += target.second;
                if (random_weight < accumulated_weight) {
                    return target.first;
                }
            }
            
            return healthy_targets.back().first;
        }
        
        default:
            return healthy_targets[0].first;
    }
}

void LoadBalancer::updateHealth(const std::string& address, bool healthy) {
    std::lock_guard<std::mutex> lock(mutex_);
    health_status_[address] = healthy;
}

} // namespace multibotnet