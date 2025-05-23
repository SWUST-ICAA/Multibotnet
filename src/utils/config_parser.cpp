#include "multibotnet/utils/config_parser.hpp"
#include "multibotnet/utils/logger.hpp"
#include <fstream>

namespace multibotnet {

bool ConfigParser::parse(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        // 解析各部分
        if (!parseIpMap(config["IP"])) {
            return false;
        }
        
        if (config["send_topics"]) {
            if (!parseSendTopics(config["send_topics"])) {
                return false;
            }
        }
        
        if (config["recv_topics"]) {
            if (!parseRecvTopics(config["recv_topics"])) {
                return false;
            }
        }
        
        if (config["provide_services"]) {
            if (!parseProvideServices(config["provide_services"])) {
                return false;
            }
        }
        
        if (config["request_services"]) {
            if (!parseRequestServices(config["request_services"])) {
                return false;
            }
        }
        
        if (config["advanced"]) {
            if (!parseAdvancedConfig(config["advanced"])) {
                return false;
            }
        }
        
        return true;
        
    } catch (const YAML::Exception& e) {
        error_ = "YAML parsing error: " + std::string(e.what());
        return false;
    } catch (const std::exception& e) {
        error_ = "Unexpected error: " + std::string(e.what());
        return false;
    }
}

bool ConfigParser::validate() const {
    // 验证IP映射
    if (ip_map_.empty()) {
        const_cast<std::string&>(error_) = "IP map is empty";
        return false;
    }
    
    // 验证话题配置
    for (const auto& topic : send_topics_) {
        if (topic.topic.empty()) {
            const_cast<std::string&>(error_) = "Empty topic name in send_topics";
            return false;
        }
        if (topic.message_type.empty()) {
            const_cast<std::string&>(error_) = "Empty message type for topic " + topic.topic;
            return false;
        }
        if (topic.port <= 0 || topic.port > 65535) {
            const_cast<std::string&>(error_) = "Invalid port for topic " + topic.topic;
            return false;
        }
        if (topic.max_frequency <= 0) {
            const_cast<std::string&>(error_) = "Invalid max_frequency for topic " + topic.topic;
            return false;
        }
    }
    
    for (const auto& topic : recv_topics_) {
        if (topic.topic.empty()) {
            const_cast<std::string&>(error_) = "Empty topic name in recv_topics";
            return false;
        }
        if (topic.message_type.empty()) {
            const_cast<std::string&>(error_) = "Empty message type for topic " + topic.topic;
            return false;
        }
        if (topic.port <= 0 || topic.port > 65535) {
            const_cast<std::string&>(error_) = "Invalid port for topic " + topic.topic;
            return false;
        }
    }
    
    // 验证服务配置
    for (const auto& service : provide_services_) {
        if (service.service_name.empty()) {
            const_cast<std::string&>(error_) = "Empty service name in provide_services";
            return false;
        }
        if (service.service_type.empty()) {
            const_cast<std::string&>(error_) = "Empty service type for service " + service.service_name;
            return false;
        }
        if (service.port <= 0 || service.port > 65535) {
            const_cast<std::string&>(error_) = "Invalid port for service " + service.service_name;
            return false;
        }
    }
    
    for (const auto& service : request_services_) {
        if (service.service_name.empty()) {
            const_cast<std::string&>(error_) = "Empty service name in request_services";
            return false;
        }
        if (service.service_type.empty()) {
            const_cast<std::string&>(error_) = "Empty service type for service " + service.service_name;
            return false;
        }
        if (service.port <= 0 || service.port > 65535) {
            const_cast<std::string&>(error_) = "Invalid port for service " + service.service_name;
            return false;
        }
    }
    
    return true;
}

bool ConfigParser::parseIpMap(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        error_ = "IP section is missing or not a map";
        return false;
    }
    
    for (const auto& item : node) {
        std::string key = item.first.as<std::string>();
        std::string value = item.second.as<std::string>();
        ip_map_[key] = value;
    }
    
    return true;
}

bool ConfigParser::parseSendTopics(const YAML::Node& node) {
    if (!node.IsSequence()) {
        error_ = "send_topics is not a sequence";
        return false;
    }
    
    for (const auto& item : node) {
        send_topics_.push_back(parseTopicConfig(item, true));
    }
    
    return true;
}

bool ConfigParser::parseRecvTopics(const YAML::Node& node) {
    if (!node.IsSequence()) {
        error_ = "recv_topics is not a sequence";
        return false;
    }
    
    for (const auto& item : node) {
        recv_topics_.push_back(parseTopicConfig(item, false));
    }
    
    return true;
}

bool ConfigParser::parseProvideServices(const YAML::Node& node) {
    if (!node.IsSequence()) {
        error_ = "provide_services is not a sequence";
        return false;
    }
    
    for (const auto& item : node) {
        provide_services_.push_back(parseServiceConfig(item, true));
    }
    
    return true;
}

bool ConfigParser::parseRequestServices(const YAML::Node& node) {
    if (!node.IsSequence()) {
        error_ = "request_services is not a sequence";
        return false;
    }
    
    for (const auto& item : node) {
        request_services_.push_back(parseServiceConfig(item, false));
    }
    
    return true;
}

bool ConfigParser::parseAdvancedConfig(const YAML::Node& node) {
    if (!node.IsMap()) {
        return true;  // 高级配置是可选的
    }
    
    // 压缩配置
    if (node["compression"]) {
        const auto& comp = node["compression"];
        if (comp["enable"]) {
            advanced_config_.enable_compression = comp["enable"].as<bool>();
        }
        if (comp["type"]) {
            advanced_config_.compression_type = comp["type"].as<std::string>();
        }
        if (comp["level"]) {
            advanced_config_.compression_level = comp["level"].as<int>();
        }
    }
    
    // 批处理配置
    if (node["batch"]) {
        const auto& batch = node["batch"];
        if (batch["enable"]) {
            advanced_config_.enable_batch = batch["enable"].as<bool>();
        }
        if (batch["size"]) {
            advanced_config_.default_batch_size = batch["size"].as<int>();
        }
        if (batch["timeout_ms"]) {
            advanced_config_.batch_timeout_ms = batch["timeout_ms"].as<int>();
        }
    }
    
    // 线程池配置
    if (node["thread_pool"]) {
        const auto& tp = node["thread_pool"];
        if (tp["size"]) {
            advanced_config_.thread_pool_size = tp["size"].as<int>();
        }
    }
    
    // 连接池配置
    if (node["connection_pool"]) {
        const auto& cp = node["connection_pool"];
        if (cp["min_connections"]) {
            advanced_config_.min_connections = cp["min_connections"].as<int>();
        }
        if (cp["max_connections"]) {
            advanced_config_.max_connections = cp["max_connections"].as<int>();
        }
        if (cp["connection_timeout_ms"]) {
            advanced_config_.connection_timeout_ms = cp["connection_timeout_ms"].as<int>();
        }
        if (cp["idle_timeout_ms"]) {
            advanced_config_.idle_timeout_ms = cp["idle_timeout_ms"].as<int>();
        }
    }
    
    // 性能配置
    if (node["performance"]) {
        const auto& perf = node["performance"];
        if (perf["enable_statistics"]) {
            advanced_config_.enable_statistics = perf["enable_statistics"].as<bool>();
        }
        if (perf["statistics_interval_ms"]) {
            advanced_config_.statistics_interval_ms = perf["statistics_interval_ms"].as<int>();
        }
    }
    
    // 健康检查
    if (node["health_check"]) {
        const auto& hc = node["health_check"];
        if (hc["enable"]) {
            advanced_config_.enable_health_check = hc["enable"].as<bool>();
        }
        if (hc["interval_ms"]) {
            advanced_config_.health_check_interval_ms = hc["interval_ms"].as<int>();
        }
    }
    
    // 重试策略
    if (node["retry"]) {
        const auto& retry = node["retry"];
        if (retry["max_retries"]) {
            advanced_config_.max_retries = retry["max_retries"].as<int>();
        }
        if (retry["interval_ms"]) {
            advanced_config_.retry_interval_ms = retry["interval_ms"].as<int>();
        }
    }
    
    return true;
}

TopicConfig ConfigParser::parseTopicConfig(const YAML::Node& node, bool is_send) {
    TopicConfig config;
    
    config.topic = node["topic"].as<std::string>();
    config.message_type = node["message_type"].as<std::string>();
    config.port = node["port"].as<int>();
    
    if (is_send) {
        config.max_frequency = node["max_frequency"].as<int>();
        config.address = node["bind_address"].as<std::string>();
    } else {
        config.max_frequency = 0;  // 接收话题不需要频率限制
        config.address = node["connect_address"].as<std::string>();
    }
    
    // 可选的高级配置
    if (node["compression"]) {
        config.enable_compression = node["compression"].as<bool>();
    }
    if (node["batch"]) {
        config.enable_batch = node["batch"].as<bool>();
    }
    if (node["batch_size"]) {
        config.batch_size = node["batch_size"].as<int>();
    }
    if (node["batch_timeout_ms"]) {
        config.batch_timeout_ms = node["batch_timeout_ms"].as<int>();
    }
    
    // 使用全局高级配置作为默认值
    if (!node["compression"]) {
        config.enable_compression = advanced_config_.enable_compression;
    }
    if (!node["batch"]) {
        config.enable_batch = advanced_config_.enable_batch;
    }
    if (!node["batch_size"]) {
        config.batch_size = advanced_config_.default_batch_size;
    }
    if (!node["batch_timeout_ms"]) {
        config.batch_timeout_ms = advanced_config_.batch_timeout_ms;
    }
    
    return config;
}

ServiceConfig ConfigParser::parseServiceConfig(const YAML::Node& node, bool is_provide) {
    ServiceConfig config;
    
    config.service_name = node["service_name"].as<std::string>();
    config.service_type = node["service_type"].as<std::string>();
    config.port = node["port"].as<int>();
    
    if (is_provide) {
        config.address = node["bind_address"].as<std::string>();
    } else {
        config.address = node["connect_address"].as<std::string>();
    }
    
    // 可选配置
    if (node["timeout_ms"]) {
        config.timeout_ms = node["timeout_ms"].as<int>();
    }
    if (node["max_retries"]) {
        config.max_retries = node["max_retries"].as<int>();
    } else {
        config.max_retries = advanced_config_.max_retries;
    }
    
    return config;
}

CompressionType ConfigParser::stringToCompressionType(const std::string& str) {
    if (str == "none") return CompressionType::NONE;
    if (str == "zlib") return CompressionType::ZLIB;
    if (str == "lz4") return CompressionType::LZ4;
    if (str == "snappy") return CompressionType::SNAPPY;
    
    LOG_WARN("Unknown compression type: " + str + ", using LZ4");
    return CompressionType::LZ4;
}

} // namespace multibotnet