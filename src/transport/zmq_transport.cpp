#include "multibotnet/transport/zmq_transport.hpp"
#include "multibotnet/utils/logger.hpp"
#include <zmq.hpp>

namespace multibotnet {

ZmqTransport::ZmqTransport(zmq::context_t& context, 
                         SocketType socket_type)
    : context_(context),
      socket_(context, static_cast<int>(socket_type)),
      socket_type_(socket_type),
      state_(ConnectionState::DISCONNECTED),
      auto_reconnect_(true),
      max_retries_(3),
      retry_interval_ms_(1000),
      is_bind_(false) {

    // 设置默认选项
    int linger = 0;
    socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    // 初始化统计信息
    stats_.start_time = std::chrono::steady_clock::now();
}

ZmqTransport::~ZmqTransport() {
    try {
        socket_.close();
    } catch (...) {
        // 忽略关闭时的异常
    }
}

bool ZmqTransport::bind(const std::string& address) {
    try {
        socket_.bind(address);
        last_address_ = address;
        is_bind_ = true;
        state_ = ConnectionState::CONNECTED;
        LOG_DEBUGF("Successfully bound to %s", address.c_str());
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
        LOG_DEBUGF("Successfully connected to %s", address.c_str());
        return true;
    } catch (const zmq::error_t& e) {
        state_ = ConnectionState::ERROR;
        LOG_ERRORF("Failed to connect to %s: %s", address.c_str(), e.what());

        // 如果可用自动重连，尝试重连
        if (auto_reconnect_) {
            return tryReconnect();
        }
        return false;
    }
}

bool ZmqTransport::send(const std::vector<uint8_t>& data, int flags) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_WARN("Attempting to send on disconnected socket");
        if (auto_reconnect_ && tryReconnect()) {
            // 重连成功，继续发送
        } else {
            return false;
        }
    }

    try {
        zmq::message_t msg(data.size());
        memcpy(msg.data(), data.data(), data.size());

        // 使用新的API（ZMQ 4.3.1+）
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
        zmq::send_result_t result = socket_.send(msg, zmq::send_flags(flags));
        if (result.has_value()) {
            updateStatistics(data.size(), true);
            return true;
        }
        // 发送失败，尝试重连
        if (auto_reconnect_ && tryReconnect()) {
            // 重连成功，重试发送
            zmq::message_t retry_msg(data.size());
            memcpy(retry_msg.data(), data.data(), data.size());
            result = socket_.send(retry_msg, zmq::send_flags(flags));
            if (result.has_value()) {
                updateStatistics(data.size(), true);
                return true;
            }
        }
        return false;
#else
        // 旧版本API
        bool result = socket_.send(msg, flags);
        if (result) {
            updateStatistics(data.size(), true);
            return true;
        }
        // 发送失败，尝试重连
        if (auto_reconnect_ && tryReconnect()) {
            // 重连成功，重试发送
            zmq::message_t retry_msg(data.size());
            memcpy(retry_msg.data(), data.data(), data.size());
            if (socket_.send(retry_msg, flags)) {
                updateStatistics(data.size(), true);
                return true;
            }
        }
        return false;
#endif
    } catch (const zmq::error_t& e) {
        LOG_ERRORF("Send error: %s", e.what());
        state_ = ConnectionState::ERROR;
        stats_.errors++;

        // 尝试重连
        if (auto_reconnect_ && tryReconnect()) {
            // 重连成功，重试发送一次
            try {
                zmq::message_t retry_msg(data.size());
                memcpy(retry_msg.data(), data.data(), data.size());
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
                zmq::send_result_t result = socket_.send(retry_msg, zmq::send_flags(flags));
                if (result.has_value()) {
                    updateStatistics(data.size(), true);
                    return true;
                }
#else
                if (socket_.send(retry_msg, flags)) {
                    updateStatistics(data.size(), true);
                    return true;
                }
#endif
            } catch (...) {
                // 重试失败
            }
        }
        return false;
    }
}

bool ZmqTransport::receive(std::vector<uint8_t>& data, int timeout_ms) {
    try {
        zmq::pollitem_t items[] = {{static_cast<void*>(socket_), 0, ZMQ_POLLIN, 0}};
        int rc = zmq::poll(items, 1, timeout_ms);

        if (rc < 0) {
            if (errno == EINTR) {
                return false;
            }
            LOG_ERROR("Poll error");
            state_ = ConnectionState::ERROR;

            // 尝试重连
            if (auto_reconnect_) {
                tryReconnect();
            }
            return false;
        }

        if (rc == 0) {
            // 超时
            return false;
        }

        if (items[0].revents & ZMQ_POLLIN) {
            zmq::message_t msg;

#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
            zmq::recv_result_t result = socket_.recv(msg, zmq::recv_flags::none);
            if (!result.has_value()) {
                return false;
            }
#else
            if (!socket_.recv(&msg)) {
                return false;
            }
#endif

            data.resize(msg.size());
            memcpy(data.data(), msg.data(), msg.size());

            updateStatistics(data.size(), false);
            return true;
        }

        return false;
    } catch (const zmq::error_t& e) {
        if (e.num() != EINTR) {
            LOG_ERRORF("Receive error: %s", e.what());
            stats_.errors++;
            state_ = ConnectionState::ERROR;

            // 尝试重连
            if (auto_reconnect_) {
                tryReconnect();
            }
        }
        return false;
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

void ZmqTransport::setReconnectPolicy(int max_retries, int retry_interval_ms, bool silent) {
    max_retries_ = max_retries;
    retry_interval_ms_ = retry_interval_ms;

    // 只在非静默模式下打印日志
    if (!silent) {
        LOG_INFOF("Reconnect policy set: max_retries=%d, interval=%dms",
                 max_retries, retry_interval_ms);
    }
}

bool ZmqTransport::reconnect() {
    if (last_address_.empty()) {
        LOG_ERROR("Cannot reconnect: no address saved");
        return false;
    }

    LOG_INFO("Manual reconnection triggered...");
    return tryReconnect();
}

bool ZmqTransport::tryReconnect() {
    if (last_address_.empty()) {
        return false;
    }

    if (state_ == ConnectionState::CONNECTED) {
        return true;  // 已经连接
    }

    LOG_INFOF("Attempting to reconnect to %s (max %d attempts)...",
             last_address_.c_str(), max_retries_);

    for (int i = 0; i < max_retries_; i++) {
        try {
            // 关闭现有套接字
            socket_.close();

            // 等待一小段时间确保资源释放
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 重新创建套接字
            socket_ = zmq::socket_t(context_, static_cast<int>(socket_type_));

            // 重新设置选项
            int linger = 0;
            socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

            // 如果是SUB套接字，需要重新订阅
            if (socket_type_ == SocketType::SUB) {
                socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
            }

            // 重新连接/绑定
            bool success = false;
            if (is_bind_) {
                socket_.bind(last_address_);
                success = true;
            } else {
                socket_.connect(last_address_);
                success = true;
            }

            if (success) {
                state_ = ConnectionState::CONNECTED;
                LOG_INFOF("Reconnection successful on attempt %d", i + 1);
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
    state_ = ConnectionState::ERROR;
    return false;
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

} // namespace multibotnet