#ifndef MULTIBOTNET_CORE_MESSAGE_FACTORY_HPP
#define MULTIBOTNET_CORE_MESSAGE_FACTORY_HPP

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <memory>
#include <functional>
#include <unordered_map>
#include <mutex>
#include "multibotnet/core/types.hpp"

namespace multibotnet {

/**
 * @brief 动态消息工厂类，用于处理任意类型的ROS消息
 * 
 * 使用ShapeShifter实现动态消息类型处理，无需预先定义消息类型
 */
class MessageFactory {
public:
    using ShapeShifterPtr = boost::shared_ptr<topic_tools::ShapeShifter>;
    using SubscriberCallback = std::function<void(const ShapeShifterPtr&)>;
    
    MessageFactory();
    ~MessageFactory();
    
    /**
     * @brief 创建订阅者，自动处理任意消息类型
     * @param topic 话题名称
     * @param callback 接收消息的回调函数
     * @return ROS订阅者
     */
    ros::Subscriber createSubscriber(const std::string& topic, 
                                   const SubscriberCallback& callback);
    
    /**
     * @brief 创建发布者占位符（不会真正创建）
     * @param topic 话题名称
     * @param message_type 消息类型字符串（如 "sensor_msgs/Imu"）
     * @param queue_size 队列大小
     * @return 空的ROS发布者
     */
    ros::Publisher createPublisher(const std::string& topic,
                                 const std::string& message_type,
                                 uint32_t queue_size = 1);
    
    /**
     * @brief 从ShapeShifter实例创建发布者
     * @param topic 话题名称
     * @param msg ShapeShifter消息实例
     * @param queue_size 队列大小
     * @return ROS发布者
     */
    ros::Publisher createPublisherFromShapeShifter(const std::string& topic,
                                                  const ShapeShifterPtr& msg,
                                                  uint32_t queue_size = 1);
    
    /**
     * @brief 序列化ShapeShifter消息
     * @param msg ShapeShifter消息指针
     * @return 序列化后的字节数组
     */
    std::vector<uint8_t> serialize(const ShapeShifterPtr& msg);
    
    /**
     * @brief 反序列化为ShapeShifter消息
     * @param data 字节数组
     * @param message_type 消息类型
     * @param md5sum 消息MD5校验和
     * @param message_definition 消息定义
     * @return ShapeShifter消息指针
     */
    ShapeShifterPtr deserialize(const std::vector<uint8_t>& data,
                               const std::string& message_type,
                               const std::string& md5sum,
                               const std::string& message_definition);
    
    /**
     * @brief 获取消息类型信息
     * @param topic 话题名称
     * @return 包含类型、MD5和定义的元组
     */
    std::tuple<std::string, std::string, std::string> 
    getTopicInfo(const std::string& topic);
    
private:
    ros::NodeHandle nh_;
    std::unordered_map<std::string, ros::Publisher> publishers_;
    std::unordered_map<std::string, ros::Subscriber> subscribers_;
    std::mutex mutex_;
    
    // 缓存话题信息
    struct TopicInfo {
        std::string type;
        std::string md5sum;
        std::string definition;
    };
    std::unordered_map<std::string, TopicInfo> topic_info_cache_;
};

} // namespace multibotnet

#endif // MULTIBOTNET_CORE_MESSAGE_FACTORY_HPP