#include "multibotnet/managers/topic_manager.hpp"
#include "multibotnet/utils/logger.hpp"
#include "multibotnet/utils/config_parser.hpp"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unordered_set>
#include <sstream>

namespace multibotnet {

TopicManager::TopicManager() 
    : context_(1),
      running_(false),
      statistics_running_(false),
      thread_pool_size_(0),
      max_retries_(3),
      retry_interval_ms_(1000),
      enable_statistics_(false),
      statistics_interval_ms_(5000) {
    message_factory_ = std::make_unique<MessageFactory>();
    compression_manager_ = &CompressionManager::getInstance();
}

TopicManager::~TopicManager() {
    stop();
}

bool TopicManager::init(const std::string& config_file) {
    if (!loadConfig(config_file)) {
        return false;
    }

    // 创建线程池
    if (thread_pool_size_ == 0) {
        thread_pool_size_ = std::thread::hardware_concurrency();
    }
    thread_pool_ = std::make_unique<ThreadPool>(thread_pool_size_);

    displayConfig();
    return true;
}

void TopicManager::start() {
    if (running_) {
        LOG_WARN("TopicManager already running");
        return;
    }

    running_ = true;

    // 如果有接收话题，等待一小段时间让 ZMQ 订阅生效
    if (!recv_topics_.empty()) {
        LOG_INFO("Waiting for ZMQ subscriptions to take effect...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 启动所有接收线程
    for (auto& recv_topic : recv_topics_) {
        if (!recv_topic->recv_thread.joinable()) {
            recv_topic->active = true;
            recv_topic->recv_thread = std::thread(
                &TopicManager::recvTopicLoop, this, recv_topic.get());
        }
    }

    // 给接收线程一些时间来启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启动统计线程（如果启用）
    if (enable_statistics_ && statistics_interval_ms_ > 0) {
        statistics_running_ = true;
        statistics_thread_ = std::thread(&TopicManager::statisticsLoop, this);
        LOG_INFOF("Statistics enabled with interval %d ms", statistics_interval_ms_);
    }

    LOG_INFO("TopicManager started successfully");
}

void TopicManager::stop() {
    if (!running_) {
        return;
    }

    running_ = false;

    // 停止统计线程
    if (statistics_running_) {
        statistics_running_ = false;
        if (statistics_thread_.joinable()) {
            statistics_thread_.join();
        }
    }

    // 停止线程池
    if (thread_pool_) {
        thread_pool_->stop();
    }

    // 停止所有接收线程
    for (auto& recv_topic : recv_topics_) {
        recv_topic->active = false;
    }

    // 等待所有接收线程结束
    for (auto& recv_topic : recv_topics_) {
        if (recv_topic->recv_thread.joinable()) {
            recv_topic->recv_thread.join();
        }
    }

    LOG_INFO("TopicManager stopped");
}

void TopicManager::statisticsLoop() {
    LOG_INFO("Statistics thread started");

    while (statistics_running_ && running_) {
        // 等待指定的间隔时间
        std::this_thread::sleep_for(std::chrono::milliseconds(statistics_interval_ms_));

        // 检查是否仍在运行
        if (!statistics_running_ || !running_) {
            break;
        }

        // 打印统计信息
        printStatistics();
    }

    LOG_INFO("Statistics thread stopped");
}

std::unordered_map<std::string, Statistics> TopicManager::getStatistics() const {
    std::unordered_map<std::string, Statistics> stats;

    // 收集发送话题统计
    for (const auto& send_topic : send_topics_) {
        stats["send:" + send_topic->config.topic] =
            send_topic->transport->getStatistics();
    }

    // 收集接收话题统计
    for (const auto& recv_topic : recv_topics_) {
        stats["recv:" + recv_topic->config.topic] =
            recv_topic->transport->getStatistics();
    }

    return stats;
}

void TopicManager::printStatistics() const {
    auto stats = getStatistics();

    std::cout << BLUE << "========== Topic Statistics ==========" << RESET << std::endl;

    // 显示线程池状态
    if (thread_pool_) {
        std::cout << YELLOW << "Thread Pool: " << thread_pool_->size()
                  << " threads, " << thread_pool_->pending()
                  << " tasks pending" << RESET << std::endl;
    }

    // 分别收集发送和接收的统计信息
    std::vector<std::pair<std::string, Statistics>> send_stats;
    std::vector<std::pair<std::string, Statistics>> recv_stats;

    for (const auto& pair : stats) {
        if (pair.first.find("send:") == 0) {
            send_stats.push_back(pair);
        } else if (pair.first.find("recv:") == 0) {
            recv_stats.push_back(pair);
        }
    }

    // 打印接收统计
    for (const auto& pair : recv_stats) {
        const auto& name = pair.first;
        const auto& stat = pair.second;

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - stat.start_time).count();

        double msgs_per_sec = elapsed > 0 ?
            static_cast<double>(stat.messages_received) / elapsed : 0;

        double mb_received = static_cast<double>(stat.bytes_received) / (1024 * 1024);

        std::cout << GREEN << name << ":" << RESET << std::endl;
        std::cout << "  Messages: " << YELLOW << "recv=" << stat.messages_received
                  << " (" << msgs_per_sec << " msg/s)" << RESET << std::endl;
        std::cout << "  Data: " << YELLOW << "recv=" << mb_received << "MB" << RESET << std::endl;

        if (stat.errors > 0) {
            std::cout << RED << "  Errors: " << stat.errors << RESET << std::endl;
        }
    }

    // 打印发送统计
    for (const auto& pair : send_stats) {
        const auto& name = pair.first;
        const auto& stat = pair.second;

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - stat.start_time).count();

        double msgs_per_sec = elapsed > 0 ?
            static_cast<double>(stat.messages_sent) / elapsed : 0;

        double mb_sent = static_cast<double>(stat.bytes_sent) / (1024 * 1024);

        std::cout << GREEN << name << ":" << RESET << std::endl;
        std::cout << "  Messages: " << YELLOW << "sent=" << stat.messages_sent
                  << " (" << msgs_per_sec << " msg/s)" << RESET << std::endl;
        std::cout << "  Data: " << YELLOW << "sent=" << mb_sent << "MB" << RESET << std::endl;

        if (stat.errors > 0) {
            std::cout << RED << "  Errors: " << stat.errors << RESET << std::endl;
        }
    }

    std::cout << BLUE << "=====================================" << RESET << std::endl;
}

bool TopicManager::loadConfig(const std::string& config_file) {
    try {
        ConfigParser parser;
        if (!parser.parse(config_file)) {
            LOG_ERROR("Failed to parse config file: " + parser.getError());
            return false;
        }

        if (!parser.validate()) {
            LOG_ERROR("Invalid configuration: " + parser.getError());
            return false;
        }

        // 加载IP映射
        ip_map_ = parser.getIpMap();

        // 加载高级配置
        const auto& advanced = parser.getAdvancedConfig();
        thread_pool_size_ = advanced.thread_pool_size;
        max_retries_ = advanced.max_retries;
        retry_interval_ms_ = advanced.retry_interval_ms;

        // 保存高级配置用于显示和统计
        enable_compression_ = advanced.enable_compression;
        compression_type_ = advanced.compression_type;
        compression_level_ = advanced.compression_level;
        enable_statistics_ = advanced.enable_statistics;
        statistics_interval_ms_ = advanced.statistics_interval_ms;

        // 设置发送话题
        for (const auto& config : parser.getSendTopics()) {
            setupSendTopic(config);
        }

        // 设置接收话题
        for (const auto& config : parser.getRecvTopics()) {
            setupRecvTopic(config);
        }

        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to load config: " + std::string(e.what()));
        return false;
    }
}

void TopicManager::displayConfig() {
    std::stringstream ss;

    ss << std::endl;
    ss << CYAN << "============ Topic Node Configuration ============" << RESET << std::endl;

    // 显示完整的高级设置
    ss << std::endl << BLUE << "Advanced Settings:" << RESET << std::endl;

    // 压缩配置
    ss << YELLOW << "  Compression:" << RESET << std::endl;
    ss << "    Enabled: " << (enable_compression_ ? "true" : "false") << std::endl;
    ss << "    Type: " << compression_type_ << std::endl;
    ss << "    Level: " << compression_level_ << " (zlib only)" << std::endl;

    // 线程池配置
    ss << YELLOW << "  Thread Pool:" << RESET << std::endl;
    ss << "    Size: " << thread_pool_size_ << " threads" << std::endl;

    // 性能监控 - 显示来自配置文件的设置
    ss << YELLOW << "  Performance:" << RESET << std::endl;
    ss << "    Statistics: " << (enable_statistics_ ? GREEN "enabled" RESET : "disabled") << std::endl;
    if (enable_statistics_) {
        ss << "    Interval: " << statistics_interval_ms_ << " ms" << std::endl;
    }

    // 重试策略
    ss << YELLOW << "  Retry Policy:" << RESET << std::endl;
    ss << "    Max Retries: " << max_retries_ << std::endl;
    ss << "    Retry Interval: " << retry_interval_ms_ << " ms" << std::endl;

    // IP映射
    ss << std::endl << BLUE << "IP Mappings:" << RESET << std::endl;
    for (const auto& pair : ip_map_) {
        ss << "  " << YELLOW << pair.first << " -> " << pair.second << RESET << std::endl;
    }

    // 发送话题
    ss << std::endl << BLUE << "Send Topics (" << send_topics_.size() << "):" << RESET << std::endl;
    for (const auto& send_topic : send_topics_) {
        const auto& config = send_topic->config;
        std::string display_ip = resolveAddress(config.address);
        if (config.address == "self" || config.address == "*") {
            display_ip = getLocalIP();
        }

        ss << "  " << GREEN << config.topic << RESET
           << " [" << config.max_frequency << "Hz] "
           << "-> " << display_ip << ":" << config.port
           << (config.enable_compression ? " (compressed)" : "") << std::endl;
    }

    // 接收话题
    ss << std::endl << BLUE << "Receive Topics (" << recv_topics_.size() << "):" << RESET << std::endl;
    for (const auto& recv_topic : recv_topics_) {
        const auto& config = recv_topic->config;
        ss << "  " << GREEN << config.topic << RESET
           << " <- " << config.address << ":" << config.port << std::endl;
    }

    ss << CYAN << "=================================================" << RESET << std::endl;

    std::cout << ss.str() << std::flush;

    // 只打印一次通用的重连策略信息
    LOG_INFOF("All topics configured with reconnect policy: max_retries=%d, interval=%dms",
             max_retries_, retry_interval_ms_);
}

std::string TopicManager::resolveAddress(const std::string& address_key) {
    if (address_key == "self" || address_key == "*") {
        return "*";
    }

    auto it = ip_map_.find(address_key);
    if (it != ip_map_.end()) {
        return it->second;
    }

    return address_key;
}

std::string TopicManager::getLocalIP() {
    struct ifaddrs *ifaddr;
    std::string local_ip = "127.0.0.1";

    if (getifaddrs(&ifaddr) == -1) {
        LOG_ERROR("Failed to get local IP address");
        return local_ip;
    }

    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
            char *ip = inet_ntoa(addr->sin_addr);
            if (strcmp(ip, "127.0.0.1") != 0) {
                local_ip = ip;
                break;
            }
        }
    }

    freeifaddrs(ifaddr);
    return local_ip;
}

void TopicManager::setupSendTopic(const TopicConfig& config) {
    try {
        auto info = std::make_unique<SendTopicInfo>();
        info->config = config;
        info->config.address = resolveAddress(config.address);
        info->last_reset_time = ros::Time::now();
        info->message_count = 0;
        info->active = true;

        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::PUB);

        // 设置重连策略（静默设置，不打印日志）
        info->transport->setReconnectPolicy(max_retries_, retry_interval_ms_, true);
        info->transport->enableAutoReconnect(true);

        // 设置套接字选项
        int sndhwm = 1000;
        info->transport->setOption(ZMQ_SNDHWM, sndhwm);

        // 绑定地址
        std::string bind_address = "tcp://" + info->config.address + ":" +
                                  std::to_string(config.port);
        if (!info->transport->bind(bind_address)) {
            LOG_ERROR("Failed to bind send topic " + config.topic);
            return;
        }

        // 等待一下让绑定生效
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 创建ROS订阅者
        int topic_index = send_topics_.size();
        info->subscriber = message_factory_->createSubscriber(
            config.topic,
            [this, topic_index](const MessageFactory::ShapeShifterPtr& msg) {
                if (topic_index < send_topics_.size()) {
                    handleTopicMessage(send_topics_[topic_index].get(), msg);
                }
            }
        );

        send_topics_.push_back(std::move(info));
        LOG_DEBUGF("Setup send topic: %s on %s", config.topic.c_str(), bind_address.c_str());

    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup send topic " + config.topic + ": " + e.what());
    }
}

void TopicManager::handleTopicMessage(SendTopicInfo* info,
                                     const MessageFactory::ShapeShifterPtr& msg) {
    if (!info || !info->active || !running_) {
        return;
    }

    // 频率控制
    if (!checkFrequencyLimit(info)) {
        return;
    }

    // 使用线程池异步处理消息
    thread_pool_->enqueue([this, info, msg]() {
        processMessageAsync(info, msg);
    });
}

void TopicManager::processMessageAsync(SendTopicInfo* info,
                                      const MessageFactory::ShapeShifterPtr& msg) {
    try {
        // 创建带元数据的消息格式
        std::vector<uint8_t> full_message;

        // 获取消息元数据
        std::string msg_type = msg->getDataType();
        std::string msg_md5 = msg->getMD5Sum();
        std::string msg_def = msg->getMessageDefinition();

        // 写入类型长度和类型
        uint32_t type_len = msg_type.size();
        full_message.insert(full_message.end(),
                           reinterpret_cast<uint8_t*>(&type_len),
                           reinterpret_cast<uint8_t*>(&type_len) + 4);
        full_message.insert(full_message.end(), msg_type.begin(), msg_type.end());

        // 写入MD5长度和MD5
        uint32_t md5_len = msg_md5.size();
        full_message.insert(full_message.end(),
                           reinterpret_cast<uint8_t*>(&md5_len),
                           reinterpret_cast<uint8_t*>(&md5_len) + 4);
        full_message.insert(full_message.end(), msg_md5.begin(), msg_md5.end());

        // 写入定义长度和定义
        uint32_t def_len = msg_def.size();
        full_message.insert(full_message.end(),
                           reinterpret_cast<uint8_t*>(&def_len),
                           reinterpret_cast<uint8_t*>(&def_len) + 4);
        full_message.insert(full_message.end(), msg_def.begin(), msg_def.end());

        // 序列化消息数据
        auto serialized = message_factory_->serialize(msg);
        if (serialized.empty()) {
            LOG_ERROR("Failed to serialize message");
            return;
        }

        // 添加消息数据
        full_message.insert(full_message.end(), serialized.begin(), serialized.end());

        // 压缩消息（如果启用）
        std::vector<uint8_t> data_to_send;
        if (info->config.enable_compression) {
            CompressionType comp_type = compression_manager_->recommendCompression(
                full_message.size(), true);
            data_to_send = compression_manager_->compressWithHeader(full_message, comp_type);
        } else {
            data_to_send = full_message;
        }

        // 发送消息（加锁保护）
        {
            std::lock_guard<std::mutex> lock(info->send_mutex);
            if (!info->transport->send(data_to_send)) {
                LOG_ERROR("Failed to send message on topic " + info->config.topic);
            }
        }

    } catch (const std::exception& e) {
        LOG_ERROR("Error processing topic message: " + std::string(e.what()));
    }
}

bool TopicManager::checkFrequencyLimit(SendTopicInfo* info) {
    ros::Time now = ros::Time::now();
    double elapsed = (now - info->last_reset_time).toSec();

    if (elapsed >= 1.0) {
        // 重置计数器
        info->last_reset_time = now;
        info->message_count = 0;
    }

    if (info->message_count >= info->config.max_frequency) {
        return false;  // 超过频率限制
    }

    info->message_count++;
    return true;
}

void TopicManager::setupRecvTopic(const TopicConfig& config) {
    try {
        auto info = std::make_unique<RecvTopicInfo>();
        info->config = config;
        info->config.address = resolveAddress(config.address);
        info->active = false;
        info->first_message = true;
        info->has_advertised = false;

        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::SUB);

        // 设置重连策略（静默设置，不打印日志）
        info->transport->setReconnectPolicy(max_retries_, retry_interval_ms_, true);
        info->transport->enableAutoReconnect(true);

        // 设置套接字选项
        int rcvhwm = 1000;
        info->transport->setOption(ZMQ_RCVHWM, rcvhwm);

        // 设置接收超时
        int rcvtimeo = 100;  // 100ms
        info->transport->setOption(ZMQ_RCVTIMEO, rcvtimeo);

        // 特殊处理 localhost 连接
        std::string connect_ip = info->config.address;
        if (connect_ip == "127.0.0.1" || connect_ip == "localhost") {
            connect_ip = "127.0.0.1";
        }

        // 连接地址
        std::string connect_address = "tcp://" + connect_ip + ":" +
                                     std::to_string(config.port);

        if (!info->transport->connect(connect_address)) {
            LOG_ERROR("Failed to connect recv topic " + config.topic +
                     " to " + connect_address);
            return;
        }

        // 订阅所有消息
        info->transport->subscribe("");

        recv_topics_.push_back(std::move(info));
        LOG_DEBUGF("Setup recv topic: %s [%s] from %s",
                  config.topic.c_str(),
                  config.message_type.c_str(),
                  connect_address.c_str());

    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup recv topic " + config.topic + ": " + e.what());
    }
}

void TopicManager::recvTopicLoop(RecvTopicInfo* info) {
    static std::unordered_set<std::string> logged_topics;
    static std::mutex log_mutex;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (info->active && running_ && ros::ok()) {
        try {
            // 接收数据
            std::vector<uint8_t> data;
            if (!info->transport->receive(data, 100)) {  // 100ms超时
                continue;
            }

            // 首次接收时打印日志（合并打印）- 修改这里添加消息类型
            if (info->first_message) {
                std::lock_guard<std::mutex> lock(log_mutex);
                if (logged_topics.find(info->config.topic) == logged_topics.end()) {
                    LOG_INFOF("Topic '%s' [%s] receiving data from network",
                             info->config.topic.c_str(),
                             info->config.message_type.c_str());  // 添加消息类型显示
                    logged_topics.insert(info->config.topic);
                }
                info->first_message = false;
            }

            // 使用线程池异步处理接收到的消息
            thread_pool_->enqueue([this, info, data]() {
                processReceivedMessageAsync(info, data);
            });

        } catch (const std::exception& e) {
            LOG_ERROR("Error in recv loop for " + info->config.topic + ": " + e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void TopicManager::processReceivedMessageAsync(RecvTopicInfo* info,
                                              std::vector<uint8_t> data) {
    try {
        // 解压消息（如果需要）
        std::vector<uint8_t> decompressed;
        if (CompressionManager::hasCompressionHeader(data)) {
            decompressed = compression_manager_->decompressWithHeader(data);
            if (decompressed.empty()) {
                LOG_ERROR("Failed to decompress message");
                return;
            }
        } else {
            decompressed = std::move(data);
        }

        // 解析消息格式
        size_t offset = 0;

        // 读取类型长度
        if (decompressed.size() < offset + 4) {
            LOG_ERROR("Invalid message format: too small for type length");
            return;
        }
        uint32_t type_len;
        memcpy(&type_len, decompressed.data() + offset, 4);
        offset += 4;

        // 读取类型
        if (decompressed.size() < offset + type_len) {
            LOG_ERROR("Invalid message format: too small for type");
            return;
        }
        std::string msg_type(decompressed.begin() + offset,
                            decompressed.begin() + offset + type_len);
        offset += type_len;

        // 读取MD5长度
        if (decompressed.size() < offset + 4) {
            LOG_ERROR("Invalid message format: too small for MD5 length");
            return;
        }
        uint32_t md5_len;
        memcpy(&md5_len, decompressed.data() + offset, 4);
        offset += 4;

        // 读取MD5
        if (decompressed.size() < offset + md5_len) {
            LOG_ERROR("Invalid message format: too small for MD5");
            return;
        }
        std::string md5sum(decompressed.begin() + offset,
                          decompressed.begin() + offset + md5_len);
        offset += md5_len;

        // 读取定义长度
        if (decompressed.size() < offset + 4) {
            LOG_ERROR("Invalid message format: too small for definition length");
            return;
        }
        uint32_t def_len;
        memcpy(&def_len, decompressed.data() + offset, 4);
        offset += 4;

        // 读取定义
        if (decompressed.size() < offset + def_len) {
            LOG_ERROR("Invalid message format: too small for definition");
            return;
        }
        std::string msg_def(decompressed.begin() + offset,
                           decompressed.begin() + offset + def_len);
        offset += def_len;

        // 提取消息数据
        std::vector<uint8_t> msg_data(decompressed.begin() + offset, decompressed.end());

        // 反序列化消息
        auto shape_shifter_msg = message_factory_->deserialize(
            msg_data,
            msg_type,
            md5sum,
            msg_def
        );

        if (!shape_shifter_msg) {
            LOG_ERROR("Failed to deserialize message");
            return;
        }

        // 发布消息（加锁保护）
        {
            std::lock_guard<std::mutex> lock(info->publish_mutex);

            // 如果还没有创建发布者，使用 ShapeShifter 实例创建
            if (!info->has_advertised || !info->publisher) {
                info->publisher = message_factory_->createPublisherFromShapeShifter(
                    info->config.topic,
                    shape_shifter_msg,
                    10);

                if (info->publisher) {
                    info->has_advertised = true;
                } else {
                    LOG_ERROR("Failed to create publisher for " + info->config.topic);
                    return;
                }
            }

            // 发布消息
            if (info->publisher) {
                info->publisher.publish(shape_shifter_msg);
            }
        }

    } catch (const std::exception& e) {
        LOG_ERROR("Error processing received message: " + std::string(e.what()));
    }
}

void TopicManager::processReceivedMessage(RecvTopicInfo* info,
                                        const std::vector<uint8_t>& data) {
    // 这个函数保留以备兼容，但实际使用异步版本
    processReceivedMessageAsync(info, data);
}

} // namespace multibotnet