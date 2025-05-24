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
        topic, 10,  // 增加队列大小
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
    LOG_INFOF("Created subscriber for topic %s", topic.c_str());
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
    
    // 返回一个空的发布者
    // 实际的发布者将在第一次收到消息时创建
    LOG_INFOF("Publisher for topic %s will be created on first message (type: %s)", 
             topic.c_str(), message_type.c_str());
    
    return ros::Publisher();  // 返回空发布者
}

ros::Publisher MessageFactory::createPublisherFromShapeShifter(
    const std::string& topic,
    const ShapeShifterPtr& msg,
    uint32_t queue_size) {
    
    if (!msg) {
        LOG_ERROR("Cannot create publisher from null ShapeShifter");
        return ros::Publisher();
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查是否已存在
    auto it = publishers_.find(topic);
    if (it != publishers_.end()) {
        return it->second;
    }
    
    try {
        // 使用 ShapeShifter 的 advertise 方法创建发布者
        ros::Publisher pub = msg->advertise(nh_, topic, queue_size, true);
        
        publishers_[topic] = pub;
        
        LOG_INFOF("Created publisher for topic %s (type: %s, md5: %s)", 
                 topic.c_str(), 
                 msg->getDataType().c_str(),
                 msg->getMD5Sum().c_str());
        
        return pub;
    } catch (const std::exception& e) {
        LOG_ERRORF("Failed to create publisher for topic %s: %s", 
                  topic.c_str(), e.what());
        return ros::Publisher();
    }
}

std::vector<uint8_t> MessageFactory::serialize(const ShapeShifterPtr& msg) {
    if (!msg) {
        LOG_ERROR("Cannot serialize null message");
        return {};
    }
    
    try {
        // 获取序列化大小
        uint32_t serial_size = ros::serialization::serializationLength(*msg);
        
        LOG_DEBUGF("Serializing message of type %s, size: %u bytes", 
                  msg->getDataType().c_str(), serial_size);
        
        // 分配缓冲区
        std::vector<uint8_t> buffer(serial_size);
        
        // 序列化
        ros::serialization::OStream stream(buffer.data(), serial_size);
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
    
    if (data.empty()) {
        LOG_ERROR("Cannot deserialize empty data");
        return nullptr;
    }
    
    try {
        LOG_DEBUGF("Deserializing %zu bytes as type %s", 
                  data.size(), message_type.c_str());
        
        // 创建 ShapeShifter
        auto shape_shifter = boost::make_shared<topic_tools::ShapeShifter>();
        
        // 设置消息类型信息
        shape_shifter->morph(md5sum, message_type, message_definition, "");
        
        // 反序列化消息数据
        ros::serialization::IStream stream(const_cast<uint8_t*>(data.data()), data.size());
        ros::serialization::deserialize(stream, *shape_shifter);
        
        LOG_DEBUGF("Successfully deserialized message of type %s", 
                  message_type.c_str());
        
        return shape_shifter;
    } catch (const std::exception& e) {
        LOG_ERRORF("Failed to deserialize message of type %s: %s", 
                  message_type.c_str(), e.what());
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
            // 缓存基本信息
            TopicInfo cache_info;
            cache_info.type = info.datatype;
            cache_info.md5sum = "";
            cache_info.definition = "";
            topic_info_cache_[topic] = cache_info;
            
            return std::make_tuple(info.datatype, "", "");
        }
    }
    
    LOG_WARNF("Topic %s not found in ROS master", topic.c_str());
    return std::make_tuple("", "", "");
}

} // namespace multibotnet