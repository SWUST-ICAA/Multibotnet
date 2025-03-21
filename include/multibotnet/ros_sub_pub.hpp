#ifndef ROS_SUB_PUB_HPP
#define ROS_SUB_PUB_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/GetPlan.h>

namespace multibotnet {

inline std::string getMsgType(const std::string& type) {
    if (type == "sensor_msgs/Imu") return "sensor_msgs::Imu";
    if (type == "geometry_msgs/Twist") return "geometry_msgs::Twist";
    if (type == "std_msgs/String") return "std_msgs::String";
    if (type == "nav_msgs/Odometry") return "nav_msgs::Odometry";
    if (type == "sensor_msgs/LaserScan") return "sensor_msgs::LaserScan";
    if (type == "sensor_msgs/Image") return "sensor_msgs::Image";
    if (type == "geometry_msgs/Pose") return "geometry_msgs::Pose";
    if (type == "geometry_msgs/Point") return "geometry_msgs::Point";
    if (type == "std_msgs/Float32") return "std_msgs::Float32";
    if (type == "std_msgs/Int32") return "std_msgs::Int32";
    if (type == "std_srvs/SetBool") return "std_srvs::SetBool";
    if (type == "nav_msgs/GetPlan") return "nav_msgs::GetPlan";
    return type;
}

template<typename T>
std::vector<uint8_t> serializeMsg(const T& msg) {
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    std::vector<uint8_t> buffer(serial_size);
    ros::serialization::OStream stream(buffer.data(), serial_size);
    ros::serialization::serialize(stream, msg);
    return buffer;
}

template<typename T>
T deserializeMsg(const uint8_t* data, size_t size) {
    T msg;
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), size);
    ros::serialization::deserialize(stream, msg);
    return msg;
}

} // namespace multibotnet

#endif // ROS_SUB_PUB_HPP