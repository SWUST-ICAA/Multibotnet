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
    
    // 使用 AdvertiseOptions 来避免模板实例化问题
    ros::AdvertiseOptions ops;
    ops.topic = topic;
    ops.queue_size = queue_size;
    ops.latch = true;
    
    // 设置消息类型信息
    // 注意：对于 ShapeShifter，我们需要在发布时动态设置类型
    // 这里只是创建一个通用的发布者
    ops.datatype = message_type;
    ops.md5sum = "*";  // 通配符，表示接受任何 MD5
    ops.message_definition = "";
    ops.has_header = false;
    
    ros::Publisher pub = nh_.advertise(ops);
    
    publishers_[topic] = pub;
    
    LOG_INFOF("Created publisher for topic %s (declared type: %s)", 
             topic.c_str(), message_type.c_str());
    
    return pub;
}

std::vector<uint8_t> MessageFactory::serialize(const ShapeShifterPtr& msg) {
    if (!msg) {
        LOG_ERROR("Cannot serialize null message");
        return {};
    }
    
    try {
        // 使用简单的序列化格式
        std::vector<uint8_t> buffer;
        
        // 保存消息的关键信息
        std::string type = msg->getDataType();
        std::string md5 = msg->getMD5Sum();
        std::string def = msg->getMessageDefinition();
        
        // 简单的TLV格式：Type-Length-Value
        // 格式：[类型长度:4][类型][MD5长度:4][MD5][定义长度:4][定义][消息长度:4][消息数据]
        
        // 类型
        uint32_t type_len = type.size();
        buffer.insert(buffer.end(), (uint8_t*)&type_len, (uint8_t*)&type_len + 4);
        buffer.insert(buffer.end(), type.begin(), type.end());
        
        // MD5
        uint32_t md5_len = md5.size();
        buffer.insert(buffer.end(), (uint8_t*)&md5_len, (uint8_t*)&md5_len + 4);
        buffer.insert(buffer.end(), md5.begin(), md5.end());
        
        // 定义
        uint32_t def_len = def.size();
        buffer.insert(buffer.end(), (uint8_t*)&def_len, (uint8_t*)&def_len + 4);
        buffer.insert(buffer.end(), def.begin(), def.end());
        
        // 消息数据
        // 获取消息大小
        uint32_t msg_size = ros::serialization::serializationLength(*msg);
        buffer.insert(buffer.end(), (uint8_t*)&msg_size, (uint8_t*)&msg_size + 4);
        
        // 序列化消息
        size_t start_pos = buffer.size();
        buffer.resize(start_pos + msg_size);
        ros::serialization::OStream stream(buffer.data() + start_pos, msg_size);
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
    
    if (data.size() < 16) {
        LOG_ERROR("Invalid data size for deserialization");
        return nullptr;
    }
    
    try {
        size_t offset = 0;
        
        // 读取类型
        uint32_t type_len = 0;
        memcpy(&type_len, data.data() + offset, 4);
        offset += 4;
        
        std::string type(data.begin() + offset, data.begin() + offset + type_len);
        offset += type_len;
        
        // 读取MD5
        uint32_t md5_len = 0;
        memcpy(&md5_len, data.data() + offset, 4);
        offset += 4;
        
        std::string md5(data.begin() + offset, data.begin() + offset + md5_len);
        offset += md5_len;
        
        // 读取定义
        uint32_t def_len = 0;
        memcpy(&def_len, data.data() + offset, 4);
        offset += 4;
        
        std::string def(data.begin() + offset, data.begin() + offset + def_len);
        offset += def_len;
        
        // 读取消息大小
        uint32_t msg_size = 0;
        memcpy(&msg_size, data.data() + offset, 4);
        offset += 4;
        
        // 创建 ShapeShifter
        auto shape_shifter = boost::make_shared<topic_tools::ShapeShifter>();
        
        // 设置消息类型信息
        shape_shifter->morph(md5, type, def, "");
        
        // 反序列化消息内容
        // ShapeShifter 提供了从序列化数据创建的方法
        boost::shared_array<uint8_t> msg_buf(new uint8_t[msg_size]);
        memcpy(msg_buf.get(), data.data() + offset, msg_size);
        
        // 创建一个stream并读取数据
        ros::serialization::IStream stream(msg_buf.get(), msg_size);
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