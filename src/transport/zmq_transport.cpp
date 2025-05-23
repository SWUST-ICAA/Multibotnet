#include "multibotnet/transport/zmq_transport.hpp"
#include "multibotnet/utils/logger.hpp"
#include <zmq.hpp>

namespace multibotnet {

ZmqTransport::ZmqTransport(zmq::context_t& context, 
                         SocketType socket_type,
                         const std::string& identity)
    : context_(context),
      socket_(context, static_cast<int>(socket_type)),
      socket_type_(socket_type),
      identity_(identity),
      state_(ConnectionState::DISCONNECTED),
      health_check_enabled_(false),
      running_(false),
      health_check_interval_ms_(1000),
      max_retries_(3),
      retry_interval_ms_(1000),
      is_bind_(false) {
    
    // 设置套接字身份（如果提供）
    if (!identity.empty()) {
        socket_.setsockopt(ZMQ_IDENTITY, identity.c_str(), identity.size());
    }
    
    // 设置默认选项
    int linger = 0;
    socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    
    // 初始化统计信息
    stats_.start_time = std::chrono::steady_clock::now();
}

ZmqTransport::~ZmqTransport() {
    running_ = false;
    if (health_check_thread_.joinable()) {
        health_check_thread_.join();
    }
}

bool ZmqTransport::bind(const std::string& address) {
    try {
        socket_.bind(address);
        last_address_ = address;
        is_bind_ = true;
        state_ = ConnectionState::CONNECTED;
        LOG_INFOF("Successfully bound to %s", address.c_str());
        return true;
    } catch (const zmq::error_t& e) {
        state_ = ConnectionState::ERROR;
        LOG_ERRORF("Failed to bind to %s: %s", address.c_str(), e.what());
        return false;
    }
}

bool ZmqTransport::connect(const std::string& address) {
    try {
        socket_.connect(address);
        last_address_ = address;
        is_bind_ = false;
        state_ = ConnectionState::CONNECTED;
        LOG_INFOF("Successfully connected to %s", address.c_str());
        return true;
    } catch (const zmq::error_t& e) {
        state_ = ConnectionState::ERROR;
        LOG_ERRORF("Failed to connect to %s: %s", address.c_str(), e.what());
        return false;
    }
}

bool ZmqTransport::send(const std::vector<uint8_t>& data, int flags) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_WARN("Attempting to send on disconnected socket");
        if (!reconnect()) {
            return false;
        }
    }
    
    try {
        zmq::message_t msg(data.size());
        memcpy(msg.data(), data.data(), data.size());
        
        bool result = socket_.send(msg, flags);
        if (result) {
            updateStatistics(data.size(), true);
        }
        return result;
    } catch (const zmq::error_t& e) {
        LOG_ERRORF("Send error: %s", e.what());
        state_ = ConnectionState::ERROR;
        stats_.errors++;
        return false;
    }
}

bool ZmqTransport::sendBatch(const MessageBatch& batch) {
    if (batch.messages.empty()) {
        return true;
    }
    
    try {
        // 创建批处理消息格式
        // [4字节: 消息数量][消息1长度][消息1数据][消息2长度][消息2数据]...
        std::vector<uint8_t> batch_data;
        batch_data.reserve(batch.total_size + 4 + batch.messages.size() * 4);
        
        // 写入消息数量
        uint32_t msg_count = batch.messages.size();
        batch_data.insert(batch_data.end(), 
                         reinterpret_cast<uint8_t*>(&msg_count),
                         reinterpret_cast<uint8_t*>(&msg_count) + 4);
        
        // 写入每个消息
        for (const auto& msg : batch.messages) {
            uint32_t msg_size = msg.size();
            batch_data.insert(batch_data.end(),
                            reinterpret_cast<uint8_t*>(&msg_size),
                            reinterpret_cast<uint8_t*>(&msg_size) + 4);
            batch_data.insert(batch_data.end(), msg.begin(), msg.end());
        }
        
        // 发送批处理数据
        return send(batch_data);
    } catch (const std::exception& e) {
        LOG_ERRORF("Batch send error: %s", e.what());
        return false;
    }
}

bool ZmqTransport::receive(std::vector<uint8_t>& data, int timeout_ms) {
    try {
        zmq::pollitem_t items[] = {{static_cast<void*>(socket_), 0, ZMQ_POLLIN, 0}};
        int rc = zmq::poll(items, 1, timeout_ms);
        
        if (rc < 0) {
            LOG_ERROR("Poll error");
            return false;
        }
        
        if (rc == 0) {
            // 超时
            return false;
        }
        
        if (items[0].revents & ZMQ_POLLIN) {
            zmq::message_t msg;
            if (!socket_.recv(&msg)) {
                return false;
            }
            
            data.resize(msg.size());
            memcpy(data.data(), msg.data(), msg.size());
            
            updateStatistics(data.size(), false);
            return true;
        }
        
        return false;
    } catch (const zmq::error_t& e) {
        LOG_ERRORF("Receive error: %s", e.what());
        state_ = ConnectionState::ERROR;
        stats_.errors++;
        return false;
    }
}

size_t ZmqTransport::receiveBatch(MessageBatch& batch, size_t max_messages, int timeout_ms) {
    batch.messages.clear();
    batch.total_size = 0;
    batch.timestamp = std::chrono::steady_clock::now();
    
    std::vector<uint8_t> batch_data;
    if (!receive(batch_data, timeout_ms)) {
        return 0;
    }
    
    if (batch_data.size() < 4) {
        LOG_ERROR("Invalid batch data size");
        return 0;
    }
    
    try {
        // 读取消息数量
        uint32_t msg_count;
        memcpy(&msg_count, batch_data.data(), 4);
        
        size_t offset = 4;
        size_t received = 0;
        
        // 解析每个消息
        while (offset < batch_data.size() && received < msg_count && received < max_messages) {
            if (offset + 4 > batch_data.size()) {
                LOG_ERROR("Invalid batch format");
                break;
            }
            
            uint32_t msg_size;
            memcpy(&msg_size, batch_data.data() + offset, 4);
            offset += 4;
            
            if (offset + msg_size > batch_data.size()) {
                LOG_ERROR("Invalid message size in batch");
                break;
            }
            
            std::vector<uint8_t> msg(batch_data.begin() + offset, 
                                    batch_data.begin() + offset + msg_size);
            batch.messages.push_back(std::move(msg));
            batch.total_size += msg_size;
            
            offset += msg_size;
            received++;
        }
        
        return received;
    } catch (const std::exception& e) {
        LOG_ERRORF("Batch receive error: %s", e.what());
        return 0;
    }
}

void ZmqTransport::resetStatistics() {
    stats_ = Statistics();
    stats_.start_time = std::chrono::steady_clock::now();
}

void ZmqTransport::subscribe(const std::string& filter) {
    if (socket_type_ != SocketType::SUB) {
        LOG_WARN("subscribe() called on non-SUB socket");
        return;
    }
    
    socket_.setsockopt(ZMQ_SUBSCRIBE, filter.c_str(), filter.size());
}

void ZmqTransport::enableHealthCheck(bool enable, int interval_ms) {
    health_check_enabled_ = enable;
    health_check_interval_ms_ = interval_ms;
    
    if (enable && !health_check_thread_.joinable()) {
        running_ = true;
        health_check_thread_ = std::thread(&ZmqTransport::healthCheckLoop, this);
    } else if (!enable && health_check_thread_.joinable()) {
        running_ = false;
        health_check_thread_.join();
    }
}

void ZmqTransport::setReconnectPolicy(int max_retries, int retry_interval_ms) {
    max_retries_ = max_retries;
    retry_interval_ms_ = retry_interval_ms;
}

void ZmqTransport::healthCheckLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(health_check_interval_ms_));
        
        if (!running_) break;
        
        if (!checkConnection()) {
            LOG_WARN("Health check failed, attempting reconnection");
            reconnect();
        }
    }
}

bool ZmqTransport::checkConnection() {
    // 对于PUB/SUB套接字，没有简单的方法检查连接
    // 这里只检查状态
    return state_ == ConnectionState::CONNECTED;
}

void ZmqTransport::updateStatistics(size_t bytes, bool is_send) {
    if (is_send) {
        stats_.messages_sent++;
        stats_.bytes_sent += bytes;
    } else {
        stats_.messages_received++;
        stats_.bytes_received += bytes;
    }
}

bool ZmqTransport::reconnect() {
    if (last_address_.empty()) {
        return false;
    }
    
    LOG_INFO("Attempting to reconnect...");
    
    for (int i = 0; i < max_retries_; i++) {
        try {
            // 关闭现有套接字
            socket_.close();
            
            // 等待一小段时间确保资源释放
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // 使用保存的context重新创建套接字
            socket_ = zmq::socket_t(context_, static_cast<int>(socket_type_));
            
            // 重新设置选项
            if (!identity_.empty()) {
                socket_.setsockopt(ZMQ_IDENTITY, identity_.c_str(), identity_.size());
            }
            int linger = 0;
            socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
            
            // 重新连接/绑定
            bool success = is_bind_ ? bind(last_address_) : connect(last_address_);
            
            if (success) {
                LOG_INFO("Reconnection successful");
                return true;
            }
        } catch (const zmq::error_t& e) {
            LOG_ERRORF("Reconnection attempt %d failed: %s", i + 1, e.what());
        }
        
        if (i < max_retries_ - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms_));
        }
    }
    
    LOG_ERROR("Failed to reconnect after all retries");
    return false;
}

} // namespace multibotnet