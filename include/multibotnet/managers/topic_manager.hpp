#ifndef MULTIBOTNET_MANAGERS_TOPIC_MANAGER_HPP
#define MULTIBOTNET_MANAGERS_TOPIC_MANAGER_HPP

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include "multibotnet/core/types.hpp"
#include "multibotnet/core/message_factory.hpp"
#include "multibotnet/transport/zmq_transport.hpp"
#include "multibotnet/transport/compression.hpp"
#include "multibotnet/utils/thread_pool.hpp"

namespace multibotnet {

/**
 * @brief 话题管理器，负责管理所有的话题发送和接收
 */
class TopicManager {
public:
    TopicManager();
    ~TopicManager();

    /**
     * @brief 初始化话题管理器
     * @param config_file 配置文件路径
     * @return 是否成功
     */
    bool init(const std::string& config_file);

    /**
     * @brief 启动所有话题处理
     */
    void start();

    /**
     * @brief 停止所有话题处理
     */
    void stop();

    /**
     * @brief 获取统计信息
     * @return 统计数据映射
     */
    std::unordered_map<std::string, Statistics> getStatistics() const;

    /**
     * @brief 打印统计信息
     */
    void printStatistics() const;

private:
    // 发送话题信息
    struct SendTopicInfo {
        TopicConfig config;
        std::shared_ptr<ZmqTransport> transport;
        ros::Subscriber subscriber;
        ros::Time last_reset_time;
        std::atomic<int> message_count;
        std::atomic<bool> active;
        std::mutex send_mutex;  // 保护发送操作
    };

    // 接收话题信息
    struct RecvTopicInfo {
        TopicConfig config;
        std::shared_ptr<ZmqTransport> transport;
        ros::Publisher publisher;
        std::thread recv_thread;
        std::atomic<bool> active;
        bool first_message;
        bool has_advertised;
        std::mutex publish_mutex;  // 保护发布操作
    };

    // 成员变量
    zmq::context_t context_;
    std::unique_ptr<MessageFactory> message_factory_;
    std::unique_ptr<ThreadPool> thread_pool_;
    CompressionManager* compression_manager_;

    std::vector<std::unique_ptr<SendTopicInfo>> send_topics_;
    std::vector<std::unique_ptr<RecvTopicInfo>> recv_topics_;

    std::unordered_map<std::string, std::string> ip_map_;
    std::atomic<bool> running_;

    // 配置参数
    int thread_pool_size_;
    int max_retries_;
    int retry_interval_ms_;

    // 内部方法
    bool loadConfig(const std::string& config_file);
    void displayConfig();
    std::string resolveAddress(const std::string& address_key);
    std::string getLocalIP();

    // 发送相关
    void setupSendTopic(const TopicConfig& config);
    void handleTopicMessage(SendTopicInfo* info,
                           const MessageFactory::ShapeShifterPtr& msg);
    bool checkFrequencyLimit(SendTopicInfo* info);
    void processMessageAsync(SendTopicInfo* info,
                            const MessageFactory::ShapeShifterPtr& msg);

    // 接收相关
    void setupRecvTopic(const TopicConfig& config);
    void recvTopicLoop(RecvTopicInfo* info);
    void processReceivedMessage(RecvTopicInfo* info, const std::vector<uint8_t>& data);
    void processReceivedMessageAsync(RecvTopicInfo* info, std::vector<uint8_t> data);

    // 压缩相关
    std::vector<uint8_t> compressMessage(const std::vector<uint8_t>& data,
                                        CompressionType type);
    std::vector<uint8_t> decompressMessage(const std::vector<uint8_t>& data);
};

} // namespace multibotnet

#endif // MULTIBOTNET_MANAGERS_TOPIC_MANAGER_HPP