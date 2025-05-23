#include "multibotnet/core/message_factory.hpp"
#include "multibotnet/utils/logger.hpp"
#include <ros/master.h>

namespace multibotnet {

MessageFactory::MessageFactory() {
}

MessageFactory::~MessageFactory() {
}

ros::Subscriber MessageFactory::createSubscriber(const std::string& topic, 
                                               const SubscriberCallback& callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 创建通用订阅者，使用ShapeShifter处理任意消息类型
    auto sub = nh_.subscribe<topic_tools::ShapeShifter>(
        topic, 1,
        [this, callback, topic](const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
            try {
                // 缓存话题信息
                if (topic_info_cache_.find(topic) == topic_info_cache_.end()) {
                    TopicInfo info;
                    info.type = msg->getDataType();
                    info.md5sum = msg->getMD5Sum();
                    info.definition = msg->getMessageDefinition();
                    topic_info_cache_[topic] = info;
                    
                    LOG_INFOF("Cached topic info for %s: type=%s, md5=%s", 
                             topic.c_str(), info.type.c_str(), info.md5sum.c_str());
                }
                
                // 调用回调，传递非const版本
                auto non_const_msg = boost::const_pointer_cast<topic_tools::ShapeShifter>(msg);
                callback(non_const_msg);
            } catch (const std::exception& e) {
                LOG_ERRORF("Error in subscriber callback for topic %s: %s", 
                          topic.c_str(), e.what());
            }
        }
    );
    
    subscribers_[topic] = sub;
    return sub;
}

ros::Publisher MessageFactory::createPublisher(const std::string& topic,
                                             const std::string& message_type,
                                             uint32_t queue_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查是否已存在
    auto it = publishers_.find(topic);
    if (it != publishers_.end()) {
        return it->second;
    }
    
    // 创建ShapeShifter发布者
    // 注意：需要设置正确的参数以避免模板错误
    ros::NodeHandle nh;
    ros::AdvertiseOptions ops;
    ops.topic = topic;
    ops.queue_size = queue_size;
    ops.latch = false;
    ops.datatype = message_type;
    
    // 使用内部API创建发布者
    ros::Publisher pub = nh.advertise(ops);
    publishers_[topic] = pub;
    
    LOG_INFOF("Created publisher for topic %s with type %s", 
             topic.c_str(), message_type.c_str());
    
    return pub;
}

std::vector<uint8_t> MessageFactory::serialize(const ShapeShifterPtr& msg) {
    if (!msg) {
        LOG_ERROR("Cannot serialize null message");
        return {};
    }
    
    try {
        // 获取序列化大小
        uint32_t size = ros::serialization::serializationLength(*msg);
        
        // 创建缓冲区，包含元数据
        std::vector<uint8_t> buffer;
        buffer.reserve(size + 1024); // 预留额外空间给元数据
        
        // 写入元数据长度占位符
        buffer.resize(4);
        
        // 写入元数据
        std::string metadata = msg->getDataType() + "|" + 
                              msg->getMD5Sum() + "|" + 
                              msg->getMessageDefinition();
        
        uint32_t metadata_len = metadata.length();
        memcpy(buffer.data(), &metadata_len, 4);
        
        // 写入元数据
        buffer.insert(buffer.end(), metadata.begin(), metadata.end());
        
        // 写入消息数据
        size_t data_start = buffer.size();
        buffer.resize(data_start + size);
        
        ros::serialization::OStream stream(buffer.data() + data_start, size);
        ros::serialization::serialize(stream, *msg);
        
        return buffer;
    } catch (const std::exception& e) {
        LOG_ERRORF("Failed to serialize message: %s", e.what());
        return {};
    }
}

MessageFactory::ShapeShifterPtr MessageFactory::deserialize(
    const std::vector<uint8_t>& data,
    const std::string& message_type,
    const std::string& md5sum,
    const std::string& message_definition) {
    
    if (data.size() < 4) {
        LOG_ERROR("Invalid data size for deserialization");
        return nullptr;
    }
    
    try {
        // 读取元数据长度
        uint32_t metadata_len;
        memcpy(&metadata_len, data.data(), 4);
        
        if (data.size() < 4 + metadata_len) {
            LOG_ERROR("Invalid metadata length");
            return nullptr;
        }
        
        // 读取元数据
        std::string metadata(data.begin() + 4, data.begin() + 4 + metadata_len);
        
        // 解析元数据
        size_t pos1 = metadata.find('|');
        size_t pos2 = metadata.find('|', pos1 + 1);
        
        std::string type = metadata.substr(0, pos1);
        std::string md5 = metadata.substr(pos1 + 1, pos2 - pos1 - 1);
        std::string def = metadata.substr(pos2 + 1);
        
        // 创建ShapeShifter并设置类型信息
        auto msg = boost::make_shared<topic_tools::ShapeShifter>();
        msg->morph(md5, type, def, "");
        
        // 反序列化消息数据
        size_t data_start = 4 + metadata_len;
        size_t data_size = data.size() - data_start;
        
        ros::serialization::IStream stream(
            const_cast<uint8_t*>(data.data() + data_start), data_size);
        ros::serialization::deserialize(stream, *msg);
        
        return msg;
    } catch (const std::exception& e) {
        LOG_ERRORF("Failed to deserialize message: %s", e.what());
        return nullptr;
    }
}

std::tuple<std::string, std::string, std::string> 
MessageFactory::getTopicInfo(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查缓存
    auto it = topic_info_cache_.find(topic);
    if (it != topic_info_cache_.end()) {
        return std::make_tuple(it->second.type, it->second.md5sum, it->second.definition);
    }
    
    // 从ROS主节点获取话题信息
    ros::master::V_TopicInfo topic_infos;
    if (!ros::master::getTopics(topic_infos)) {
        LOG_ERROR("Failed to get topic info from ROS master");
        return std::make_tuple("", "", "");
    }
    
    for (const auto& info : topic_infos) {
        if (info.name == topic) {
            // 缓存信息
            TopicInfo cache_info;
            cache_info.type = info.datatype;
            cache_info.md5sum = "";  // 需要通过其他方式获取
            cache_info.definition = "";  // 需要通过其他方式获取
            topic_info_cache_[topic] = cache_info;
            
            return std::make_tuple(info.datatype, "", "");
        }
    }
    
    LOG_WARNF("Topic %s not found in ROS master", topic.c_str());
    return std::make_tuple("", "", "");
}

} // namespace multibotnet