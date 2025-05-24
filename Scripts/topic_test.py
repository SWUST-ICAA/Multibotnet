#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Multibotnet v4.0.0 话题测试脚本
用于测试话题通信功能
"""

import rospy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Header
import math
import time

# ANSI颜色代码
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
PURPLE = "\033[35m"
CYAN = "\033[36m"

class TopicTester:
    def __init__(self):
        rospy.init_node('multibotnet_topic_tester')
        
        # 发布者 - 发布原始话题
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # 订阅者 - 订阅通过multibotnet转发的话题
        self.imu_sub = rospy.Subscriber('/topic_test/imu', Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber('/topic_test/cmd_vel', Twist, self.cmd_vel_callback)
        self.scan_sub = rospy.Subscriber('/topic_test/scan', LaserScan, self.scan_callback)
        
        # 统计信息
        self.sent_count = {'imu': 0, 'cmd_vel': 0, 'scan': 0}
        self.recv_count = {'imu': 0, 'cmd_vel': 0, 'scan': 0}
        self.last_recv_time = {'imu': None, 'cmd_vel': None, 'scan': None}
        
        # 测试数据验证
        self.test_seq = 0
        
        print(CYAN + "\n========== Multibotnet Topic Tester ==========" + RESET)
        print("Testing topic communication through Multibotnet")
        print("Publishing to: /imu, /cmd_vel, /scan")
        print("Subscribing to: /topic_test/imu, /topic_test/cmd_vel, /topic_test/scan")
        print(CYAN + "=============================================" + RESET + "\n")
        
    def imu_callback(self, msg):
        self.recv_count['imu'] += 1
        self.last_recv_time['imu'] = rospy.Time.now()
        
        # 验证数据
        expected_seq = int(msg.angular_velocity.x)  # 我们把序列号存在这里
        if expected_seq == self.sent_count['imu'] - 1:
            rospy.loginfo_once(GREEN + "✓ IMU data received correctly through Multibotnet!" + RESET)
        
    def cmd_vel_callback(self, msg):
        self.recv_count['cmd_vel'] += 1
        self.last_recv_time['cmd_vel'] = rospy.Time.now()
        
        # 验证数据
        if abs(msg.linear.x - math.sin(self.recv_count['cmd_vel'] * 0.1)) < 0.001:
            rospy.loginfo_once(GREEN + "✓ Twist data received correctly through Multibotnet!" + RESET)
        
    def scan_callback(self, msg):
        self.recv_count['scan'] += 1
        self.last_recv_time['scan'] = rospy.Time.now()
        
        # 验证数据
        if len(msg.ranges) == 360:
            rospy.loginfo_once(GREEN + "✓ LaserScan data received correctly through Multibotnet!" + RESET)
    
    def publish_test_data(self):
        """发布测试数据"""
        rate = rospy.Rate(10)  # 10Hz for all topics
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 发布IMU数据 (每次循环都发)
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "imu_link"
            imu_msg.header.seq = self.sent_count['imu']
            
            # 存储序列号用于验证
            imu_msg.angular_velocity.x = float(self.sent_count['imu'])
            imu_msg.angular_velocity.y = 0.1
            imu_msg.angular_velocity.z = 0.2
            
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 9.81
            
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            
            self.imu_pub.publish(imu_msg)
            self.sent_count['imu'] += 1
            
            # 发布Twist数据 (每次循环都发)
            twist_msg = Twist()
            twist_msg.linear.x = math.sin(self.sent_count['cmd_vel'] * 0.1)
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = math.cos(self.sent_count['cmd_vel'] * 0.1) * 0.5
            
            self.cmd_vel_pub.publish(twist_msg)
            self.sent_count['cmd_vel'] += 1
            
            # 发布LaserScan数据 (每秒1次，即每10次循环发1次)
            if self.sent_count['scan'] < self.sent_count['imu'] // 10:
                scan_msg = LaserScan()
                scan_msg.header.stamp = current_time
                scan_msg.header.frame_id = "laser_frame"
                scan_msg.header.seq = self.sent_count['scan']
                
                scan_msg.angle_min = -math.pi
                scan_msg.angle_max = math.pi
                scan_msg.angle_increment = math.pi / 180.0
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = 0.1
                scan_msg.range_min = 0.1
                scan_msg.range_max = 30.0
                
                # 生成测试数据
                scan_msg.ranges = []
                for i in range(360):
                    # 创建一个简单的模式
                    distance = 5.0 + 2.0 * math.sin(i * math.pi / 180.0 * 4)
                    scan_msg.ranges.append(distance)
                
                self.scan_pub.publish(scan_msg)
                self.sent_count['scan'] += 1
            
            rate.sleep()
    
    def print_statistics(self):
        """定期打印统计信息"""
        rate = rospy.Rate(0.2)  # 每5秒打印一次
        
        while not rospy.is_shutdown():
            rate.sleep()
            
            print("\n" + BLUE + "========== Test Statistics ==========" + RESET)
            
            for topic in ['imu', 'cmd_vel', 'scan']:
                sent = self.sent_count[topic]
                recv = self.recv_count[topic]
                
                if sent > 0:
                    success_rate = (recv / float(sent)) * 100
                    color = GREEN if success_rate > 95 else YELLOW if success_rate > 80 else RED
                    
                    print("{}:".format(topic.upper()))
                    print("  Sent: {}, Received: {}, Success: {}{}%{}".format(
                        sent, recv, color, int(success_rate), RESET))
                    
                    if self.last_recv_time[topic]:
                        latency = (rospy.Time.now() - self.last_recv_time[topic]).to_sec()
                        if latency < 5.0:
                            print("  Status: {}✓ Active{} (last received {:.1f}s ago)".format(
                                GREEN, RESET, latency))
                        else:
                            print("  Status: {}✗ Inactive{} (no data for {:.1f}s)".format(
                                RED, RESET, latency))
            
            print(BLUE + "=====================================" + RESET)

def main():
    try:
        tester = TopicTester()
        
        # 等待一下让订阅者准备好
        rospy.sleep(1.0)
        
        # 启动统计打印线程
        import threading
        stats_thread = threading.Thread(target=tester.print_statistics)
        stats_thread.daemon = True
        stats_thread.start()
        
        # 开始发布测试数据
        print(YELLOW + "\nStarting to publish test data..." + RESET)
        print("Press Ctrl+C to stop\n")
        
        tester.publish_test_data()
        
    except rospy.ROSInterruptException:
        print("\n" + YELLOW + "Test stopped by user" + RESET)
    except Exception as e:
        print(RED + "Error: {}".format(e) + RESET)

if __name__ == '__main__':
    main()