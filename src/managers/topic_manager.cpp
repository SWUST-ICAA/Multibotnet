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
    
    for (const auto& pair : stats) {
        const auto& name = pair.first;
        const auto& stat = pair.second;
        
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - stat.start_time).count();
        
        double msgs_per_sec = elapsed > 0 ? 
            static_cast<double>(stat.messages_sent + stat.messages_received) / elapsed : 0;
        
        double mb_sent = static_cast<double>(stat.bytes_sent) / (1024 * 1024);
        double mb_received = static_cast<double>(stat.bytes_received) / (1024 * 1024);
        
        std::cout << GREEN << name << ":" << RESET << std::endl;
        std::cout << "  Messages: " << YELLOW << "sent=" << stat.messages_sent 
                  << ", recv=" << stat.messages_received 
                  << " (" << msgs_per_sec << " msg/s)" << RESET << std::endl;
        std::cout << "  Data: " << YELLOW << "sent=" << mb_sent << "MB" 
                  << ", recv=" << mb_received << "MB" << RESET << std::endl;
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
        LOG_INFO("Setup send topic: " + config.topic);
        
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
        // 序列化消息
        auto serialized = message_factory_->serialize(msg);
        if (serialized.empty()) {
            LOG_ERROR("Failed to serialize message");
            return;
        }
        
        // 压缩消息（如果启用）
        std::vector<uint8_t> data_to_send;
        if (info->config.enable_compression) {
            CompressionType comp_type = compression_manager_->recommendCompression(
                serialized.size(), true);  // 速度优先
            data_to_send = compression_manager_->compressWithHeader(serialized, comp_type);
        } else {
            data_to_send = serialized;
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
        info->active = true;
        info->first_message = true;
        
        // 创建ZMQ传输
        info->transport = std::make_shared<ZmqTransport>(
            context_, ZmqTransport::SocketType::SUB);
        
        // 设置套接字选项
        int rcvhwm = 1000;
        info->transport->setOption(ZMQ_RCVHWM, rcvhwm);
        info->transport->subscribe("");  // 订阅所有消息
        
        // 特殊处理 localhost 连接
        std::string connect_ip = info->config.address;
        if (connect_ip == "127.0.0.1" || connect_ip == "localhost") {
            // 如果是本地连接，使用实际的本地IP地址
            connect_ip = "127.0.0.1";  // 确保使用标准的本地回环地址
        }
        
        // 连接地址
        std::string connect_address = "tcp://" + connect_ip + ":" + 
                                     std::to_string(config.port);
        
        LOG_DEBUG("Attempting to connect to: " + connect_address);
        
        if (!info->transport->connect(connect_address)) {
            LOG_ERROR("Failed to connect recv topic " + config.topic + 
                     " to " + connect_address);
            return;
        }
        
        // 创建ROS发布者
        info->publisher = message_factory_->createPublisher(
            config.topic, config.message_type);
        
        // 启动接收线程
        info->recv_thread = std::thread(
            &TopicManager::recvTopicLoop, this, info.get());
        
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
        // 检查是否是批处理消息
        if (data.size() >= 4) {
            uint32_t msg_count;
            memcpy(&msg_count, data.data(), 4);
            
            // 简单的批处理检测：如果前4字节表示的数量合理
            if (msg_count > 0 && msg_count < 1000) {
                // 可能是批处理消息
                MessageBatch batch;
                size_t received = info->transport->receiveBatch(
                    batch, msg_count, 0);  // 不等待，已经有数据了
                
                if (received > 0) {
                    // 处理批次中的每个消息
                    for (const auto& msg_data : batch.messages) {
                        processReceivedMessage(info, msg_data);
                    }
                    return;
                }
            }
        }
        
        // 解压消息（如果需要）
        std::vector<uint8_t> decompressed;
        if (CompressionManager::hasCompressionHeader(data)) {
            // 是压缩的消息
            decompressed = compression_manager_->decompressWithHeader(data);
            if (decompressed.empty()) {
                LOG_ERROR("Failed to decompress message");
                return;
            }
        } else {
            decompressed = data;
        }
        
        // 反序列化消息
        if (!info->template_msg) {
            // 第一次需要获取类型信息
            auto topic_info = message_factory_->getTopicInfo(info->config.topic);
            info->template_msg = message_factory_->deserialize(
                decompressed, 
                std::get<0>(topic_info),  // type
                std::get<1>(topic_info),  // md5sum
                std::get<2>(topic_info)   // definition
            );
        } else {
            // 使用已有的模板消息类型信息
            info->template_msg = message_factory_->deserialize(
                decompressed,
                info->template_msg->getDataType(),
                info->template_msg->getMD5Sum(),
                info->template_msg->getMessageDefinition()
            );
        }
        
        if (!info->template_msg) {
            LOG_ERROR("Failed to deserialize message");
            return;
        }
        
        // 发布消息
        info->publisher.publish(info->template_msg);
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error processing received message: " + std::string(e.what()));
    }
}

} // namespace multibotnet