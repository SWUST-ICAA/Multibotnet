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
                    
                    LOG_INFOF("Cached topic info for %s: type=%s", 
                             topic.c_str(), info.type.c_str());
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
        // 使用 ROS 序列化直接序列化 ShapeShifter
        // 首先获取序列化后的大小
        uint32_t serial_size = ros::serialization::serializationLength(*msg);
        
        // 创建缓冲区，包含元数据和消息数据
        std::vector<uint8_t> buffer;
        
        // 获取消息元数据
        std::string datatype = msg->getDataType();
        std::string md5sum = msg->getMD5Sum();
        std::string msg_def = msg->getMessageDefinition();
        
        // 计算总大小
        uint32_t total_size = 4 + datatype.size() + 
                             4 + md5sum.size() + 
                             4 + msg_def.size() + 
                             4 + serial_size;
        
        buffer.reserve(total_size);
        
        // 1. 写入数据类型
        uint32_t datatype_len = datatype.size();
        buffer.insert(buffer.end(), (uint8_t*)&datatype_len, (uint8_t*)&datatype_len + 4);
        buffer.insert(buffer.end(), datatype.begin(), datatype.end());
        
        // 2. 写入 MD5
        uint32_t md5_len = md5sum.size();
        buffer.insert(buffer.end(), (uint8_t*)&md5_len, (uint8_t*)&md5_len + 4);
        buffer.insert(buffer.end(), md5sum.begin(), md5sum.end());
        
        // 3. 写入消息定义
        uint32_t msg_def_len = msg_def.size();
        buffer.insert(buffer.end(), (uint8_t*)&msg_def_len, (uint8_t*)&msg_def_len + 4);
        buffer.insert(buffer.end(), msg_def.begin(), msg_def.end());
        
        // 4. 写入序列化的消息数据
        buffer.insert(buffer.end(), (uint8_t*)&serial_size, (uint8_t*)&serial_size + 4);
        
        // 预分配空间
        size_t old_size = buffer.size();
        buffer.resize(old_size + serial_size);
        
        // 序列化消息
        ros::serialization::OStream stream(buffer.data() + old_size, serial_size);
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
    
    if (data.size() < 16) {  // 至少需要4个长度字段
        LOG_ERROR("Invalid data size for deserialization");
        return nullptr;
    }
    
    try {
        size_t offset = 0;
        
        // 1. 读取数据类型
        uint32_t datatype_len = 0;
        memcpy(&datatype_len, data.data() + offset, 4);
        offset += 4;
        
        if (offset + datatype_len > data.size()) {
            LOG_ERROR("Invalid datatype length");
            return nullptr;
        }
        
        std::string datatype(data.begin() + offset, data.begin() + offset + datatype_len);
        offset += datatype_len;
        
        // 2. 读取 MD5
        uint32_t md5_len = 0;
        memcpy(&md5_len, data.data() + offset, 4);
        offset += 4;
        
        if (offset + md5_len > data.size()) {
            LOG_ERROR("Invalid MD5 length");
            return nullptr;
        }
        
        std::string md5(data.begin() + offset, data.begin() + offset + md5_len);
        offset += md5_len;
        
        // 3. 读取消息定义
        uint32_t msg_def_len = 0;
        memcpy(&msg_def_len, data.data() + offset, 4);
        offset += 4;
        
        if (offset + msg_def_len > data.size()) {
            LOG_ERROR("Invalid message definition length");
            return nullptr;
        }
        
        std::string msg_def(data.begin() + offset, data.begin() + offset + msg_def_len);
        offset += msg_def_len;
        
        // 4. 读取消息数据大小
        uint32_t msg_size = 0;
        memcpy(&msg_size, data.data() + offset, 4);
        offset += 4;
        
        if (offset + msg_size > data.size()) {
            LOG_ERROR("Invalid message size");
            return nullptr;
        }
        
        // 创建 ShapeShifter
        auto shape_shifter = boost::make_shared<topic_tools::ShapeShifter>();
        
        // 设置消息类型信息
        shape_shifter->morph(md5, datatype, msg_def, "");
        
        // 反序列化消息数据
        ros::serialization::IStream stream(const_cast<uint8_t*>(data.data() + offset), msg_size);
        ros::serialization::deserialize(stream, *shape_shifter);
        
        return shape_shifter;
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