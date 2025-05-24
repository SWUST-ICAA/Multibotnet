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
    : context_(1), running_(false) {
    message_factory_ = std::make_unique<MessageFactory>();
    thread_pool_ = std::make_unique<ThreadPool>();
    compression_manager_ = &CompressionManager::getInstance();
}

TopicManager::~TopicManager() {
    stop();
}

bool TopicManager::init(const std::string& config_file) {
    if (!loadConfig(config_file)) {
        return false;
    }
    
    displayConfig();
    return true;
}

void TopicManager::start() {
    if (running_) {
        LOG_WARN("TopicManager already running");
        return;
    }
    
    running_ = true;
    
    // 启动所有接收线程（必须在 running_ = true 之后）
    for (auto& recv_topic : recv_topics_) {
        if (!recv_topic->recv_thread.joinable()) {
            recv_topic->active = true;
            recv_topic->recv_thread = std::thread(
                &TopicManager::recvTopicLoop, this, recv_topic.get());
        }
    }
    
    // 给接收线程一些时间来启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 启动批处理定时器
    ros::NodeHandle nh;
    batch_timer_ = nh.createTimer(ros::Duration(0.1), 
        [this](const ros::TimerEvent&) { processBatches(); });
    
    LOG_INFO("TopicManager started");
}

void TopicManager::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // 停止批处理定时器
    if (batch_timer_) {
        batch_timer_.stop();
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
    
    // 打印接收统计（只显示接收的数据）
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
    }
    
    // 打印发送统计（只显示发送的数据）
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
    // 显示 IP 映射
    std::cout << BLUE << "-------------IP------------" << RESET << std::endl;
    for (const auto& pair : ip_map_) {
        std::cout << YELLOW << pair.first << " : " << pair.second << RESET << std::endl;
    }
    
    // 显示发送话题
    std::cout << BLUE << "--------send topics--------" << RESET << std::endl;
    for (const auto& send_topic : send_topics_) {
        const auto& config = send_topic->config;
        std::string display_ip = resolveAddress(config.address);
        if (config.address == "self" || config.address == "*") {
            display_ip = getLocalIP();
        }
        
        std::cout << GREEN << config.topic << "  " << config.max_frequency 
                  << "Hz(max_frequency)  bind_port: " 
                  << display_ip << ":" << config.port << RESET << std::endl;
    }
    
    // 显示接收话题
    std::cout << BLUE << "-------receive topics------" << RESET << std::endl;
    for (const auto& recv_topic : recv_topics_) {
        const auto& config = recv_topic->config;
        std::cout << GREEN << config.topic << "  (IP from " 
                  << config.address << ":" << config.port << ")" << RESET << std::endl;
    }
}

std::string TopicManager::resolveAddress(const std::string& address_key) {
    if (address_key == "self" || address_key == "*") {
        return "*";
    }
    
    auto it = ip_map_.find(address_key);
    if (it != ip_map_.end()) {
        return it->second;
    }
    
    // 如果不在映射中，假设是直接的IP地址
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
        info->last_batch_time = std::chrono::steady_clock::now();
        
        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::PUB);
        
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
        LOG_INFO("Setup send topic: " + config.topic + " on " + bind_address);
        
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
    
    try {
        // 创建带元数据的消息格式
        // [4字节: 类型长度][类型字符串][4字节: MD5长度][MD5字符串][4字节: 定义长度][定义字符串][消息数据]
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
                full_message.size(), true);  // 速度优先
            data_to_send = compression_manager_->compressWithHeader(full_message, comp_type);
        } else {
            data_to_send = full_message;
        }
        
        // 批处理（如果启用）
        if (info->config.enable_batch) {
            std::lock_guard<std::mutex> lock(info->batch_mutex);
            info->batch.messages.push_back(data_to_send);
            info->batch.total_size += data_to_send.size();
            
            // 检查是否需要发送批次
            if (info->batch.messages.size() >= static_cast<size_t>(info->config.batch_size)) {
                sendBatchedMessages(info);
            }
        } else {
            // 直接发送
            if (!info->transport->send(data_to_send)) {
                LOG_ERROR("Failed to send message on topic " + info->config.topic);
            }
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error handling topic message: " + std::string(e.what()));
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

void TopicManager::sendBatchedMessages(SendTopicInfo* info) {
    if (info->batch.messages.empty()) {
        return;
    }
    
    // 发送批次
    if (!info->transport->sendBatch(info->batch)) {
        LOG_ERROR("Failed to send batch on topic " + info->config.topic);
    }
    
    // 清空批次
    info->batch.messages.clear();
    info->batch.total_size = 0;
    info->last_batch_time = std::chrono::steady_clock::now();
}

void TopicManager::processBatches() {
    auto now = std::chrono::steady_clock::now();
    
    for (auto& send_topic : send_topics_) {
        if (!send_topic->config.enable_batch) {
            continue;
        }
        
        std::lock_guard<std::mutex> lock(send_topic->batch_mutex);
        
        // 检查批次超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - send_topic->last_batch_time).count();
        
        if (elapsed >= send_topic->config.batch_timeout_ms && 
            !send_topic->batch.messages.empty()) {
            sendBatchedMessages(send_topic.get());
        }
    }
}

void TopicManager::setupRecvTopic(const TopicConfig& config) {
    try {
        auto info = std::make_unique<RecvTopicInfo>();
        info->config = config;
        info->config.address = resolveAddress(config.address);
        info->active = false;  // 先设置为false，在start()中启动
        info->first_message = true;
        info->has_advertised = false;
        
        // 不要提前创建发布者，等收到第一条消息再创建
        // info->publisher 保持为空
        
        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::SUB);
        
        // 设置套接字选项
        int rcvhwm = 1000;
        info->transport->setOption(ZMQ_RCVHWM, rcvhwm);
        
        // 设置接收超时，避免阻塞
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
        
        // 增加延迟时间，确保订阅生效（这是ZMQ PUB-SUB模式的特性）
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        
        // 不在这里启动接收线程，而是在start()函数中启动
        
        recv_topics_.push_back(std::move(info));
        LOG_INFO("Setup recv topic: " + config.topic + 
                " connected to " + connect_address);
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to setup recv topic " + config.topic + ": " + e.what());
    }
}

void TopicManager::recvTopicLoop(RecvTopicInfo* info) {
    static std::unordered_set<std::string> logged_topics;
    static std::mutex log_mutex;
    
    // 等待一下确保系统准备好
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    while (info->active && running_ && ros::ok()) {
        try {
            // 接收数据
            std::vector<uint8_t> data;
            if (!info->transport->receive(data, 100)) {  // 100ms超时
                continue;
            }
            
            // 首次接收时打印日志
            if (info->first_message) {
                std::lock_guard<std::mutex> lock(log_mutex);
                if (logged_topics.find(info->config.topic) == logged_topics.end()) {
                    LOG_INFO("--->[multibotnet_topic_node] \"" + 
                            info->config.topic + "\" received!<---");
                    logged_topics.insert(info->config.topic);
                }
                info->first_message = false;
            }
            
            // 处理接收到的消息
            processReceivedMessage(info, data);
            
        } catch (const std::exception& e) {
            LOG_ERROR("Error in recv loop for " + info->config.topic + ": " + e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void TopicManager::processReceivedMessage(RecvTopicInfo* info, 
                                        const std::vector<uint8_t>& data) {
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
            decompressed = data;
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
        
        // 如果还没有创建发布者，使用 ShapeShifter 实例创建
        if (!info->has_advertised || !info->publisher) {
            info->publisher = message_factory_->createPublisherFromShapeShifter(
                info->config.topic,
                shape_shifter_msg,
                10);
            
            if (info->publisher) {
                info->has_advertised = true;
                LOG_INFOF("Created publisher for %s (type: %s)", 
                    info->config.topic.c_str(),
                    shape_shifter_msg->getDataType().c_str());
            } else {
                LOG_ERROR("Failed to create publisher for " + info->config.topic);
                return;
            }
        }
        
        // 发布消息
        if (info->publisher) {
            info->publisher.publish(shape_shifter_msg);
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error processing received message: " + std::string(e.what()));
    }
}

} // namespace multibotnet