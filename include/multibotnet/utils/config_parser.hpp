#ifndef MULTIBOTNET_UTILS_CONFIG_PARSER_HPP
#define MULTIBOTNET_UTILS_CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <unordered_map>
#include "multibotnet/core/types.hpp"

namespace multibotnet {

/**
 * @brief 配置解析器，负责解析YAML配置文件
 */
class ConfigParser {
public:
    /**
     * @brief 解析配置文件
     * @param config_file 配置文件路径
     * @return 是否成功
     */
    bool parse(const std::string& config_file);
    
    /**
     * @brief 获取IP映射
     * @return IP映射表
     */
    const std::unordered_map<std::string, std::string>& getIpMap() const {
        return ip_map_;
    }
    
    /**
     * @brief 获取发送话题配置
     * @return 发送话题配置列表
     */
    const std::vector<TopicConfig>& getSendTopics() const {
        return send_topics_;
    }
    
    /**
     * @brief 获取接收话题配置
     * @return 接收话题配置列表
     */
    const std::vector<TopicConfig>& getRecvTopics() const {
        return recv_topics_;
    }
    
    /**
     * @brief 获取提供服务配置
     * @return 提供服务配置列表
     */
    const std::vector<ServiceConfig>& getProvideServices() const {
        return provide_services_;
    }
    
    /**
     * @brief 获取请求服务配置
     * @return 请求服务配置列表
     */
    const std::vector<ServiceConfig>& getRequestServices() const {
        return request_services_;
    }
    
    /**
     * @brief 获取高级配置
     */
    struct AdvancedConfig {
        // 压缩配置
        bool enable_compression = true;
        std::string compression_type = "lz4";  // none, zlib, lz4, snappy
        int compression_level = 6;  // 仅对zlib有效
        
        // 批处理配置
        bool enable_batch = true;
        int default_batch_size = 10;
        int batch_timeout_ms = 100;
        
        // 线程池配置
        int thread_pool_size = 0;  // 0表示自动
        
        // 连接池配置
        int min_connections = 1;
        int max_connections = 10;
        int connection_timeout_ms = 5000;
        int idle_timeout_ms = 60000;
        
        // 性能配置
        bool enable_statistics = true;
        int statistics_interval_ms = 5000;
        
        // 健康检查
        bool enable_health_check = true;
        int health_check_interval_ms = 5000;
        
        // 重试策略
        int max_retries = 3;
        int retry_interval_ms = 1000;
    };
    
    const AdvancedConfig& getAdvancedConfig() const {
        return advanced_config_;
    }
    
    /**
     * @brief 验证配置有效性
     * @return 是否有效
     */
    bool validate() const;
    
    /**
     * @brief 获取错误信息
     * @return 错误信息
     */
    const std::string& getError() const { return error_; }
    
private:
    std::unordered_map<std::string, std::string> ip_map_;
    std::vector<TopicConfig> send_topics_;
    std::vector<TopicConfig> recv_topics_;
    std::vector<ServiceConfig> provide_services_;
    std::vector<ServiceConfig> request_services_;
    AdvancedConfig advanced_config_;
    std::string error_;
    
    // 解析方法
    bool parseIpMap(const YAML::Node& node);
    bool parseSendTopics(const YAML::Node& node);
    bool parseRecvTopics(const YAML::Node& node);
    bool parseProvideServices(const YAML::Node& node);
    bool parseRequestServices(const YAML::Node& node);
    bool parseAdvancedConfig(const YAML::Node& node);
    
    // 辅助方法
    TopicConfig parseTopicConfig(const YAML::Node& node, bool is_send);
    ServiceConfig parseServiceConfig(const YAML::Node& node, bool is_provide);
    CompressionType stringToCompressionType(const std::string& str);
};

} // namespace multibotnet

#endif // MULTIBOTNET_UTILS_CONFIG_PARSER_HPP