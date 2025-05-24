#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Multibotnet v4.0.0 话题测试脚本
测试话题的发送和接收功能
"""

import rospy
import time
import sys
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist, Vector3

# ANSI颜色代码
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
PURPLE = "\033[35m"
CYAN = "\033[36m"

class TopicTest:
    def __init__(self):
        rospy.init_node('multibotnet_topic_test', anonymous=True)
        
        # 统计信息
        self.sent = {
            '/imu': 0,
            '/cmd_vel': 0,
            '/scan': 0
        }
        self.received = {
            '/topic_test/imu': 0,
            '/topic_test/cmd_vel': 0,
            '/topic_test/scan': 0
        }
        
        # 创建发布者
        self.pub_imu = rospy.Publisher('/imu', Imu, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # 创建订阅者
        self.sub_imu = rospy.Subscriber('/topic_test/imu', Imu, self.imu_callback)
        self.sub_cmd_vel = rospy.Subscriber('/topic_test/cmd_vel', Twist, self.cmd_vel_callback)
        self.sub_scan = rospy.Subscriber('/topic_test/scan', LaserScan, self.scan_callback)
        
        print(CYAN + "\n========== Multibotnet Topic Test ==========" + RESET)
        print(GREEN + "Testing topic forwarding through Multibotnet..." + RESET)
        
        # 增加等待时间，让ZMQ订阅充分建立
        print(YELLOW + "\nWaiting for connections to establish (3 seconds)..." + RESET)
        rospy.sleep(3.0)  # 增加到3秒
        
        # 发送几条"预热"消息（这些可能会丢失，但能帮助建立连接）
        print(YELLOW + "Sending warm-up messages..." + RESET)
        self.send_warmup_messages()
        rospy.sleep(1.0)
        
    def send_warmup_messages(self):
        """发送预热消息，帮助建立ZMQ连接"""
        for i in range(5):
            # IMU消息
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.z = 9.8
            self.pub_imu.publish(imu_msg)
            
            # Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            self.pub_cmd_vel.publish(twist_msg)
            
            # LaserScan消息
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = "laser_frame"
            scan_msg.ranges = [1.0] * 10  # 简化的数据
            self.pub_scan.publish(scan_msg)
            
            rospy.sleep(0.1)
        
        print(GREEN + "✓ Warm-up messages sent" + RESET)
    
    def imu_callback(self, msg):
        self.received['/topic_test/imu'] += 1
        if self.received['/topic_test/imu'] == 1:
            print(GREEN + f"✓ First IMU message received on /topic_test/imu" + RESET)
            print(f"  Linear acceleration: x={msg.linear_acceleration.x:.2f}, "
                  f"y={msg.linear_acceleration.y:.2f}, z={msg.linear_acceleration.z:.2f}")
    
    def cmd_vel_callback(self, msg):
        self.received['/topic_test/cmd_vel'] += 1
        if self.received['/topic_test/cmd_vel'] == 1:
            print(GREEN + f"✓ First Twist message received on /topic_test/cmd_vel" + RESET)
            print(f"  Linear: x={msg.linear.x:.2f}, Angular: z={msg.angular.z:.2f}")
    
    def scan_callback(self, msg):
        self.received['/topic_test/scan'] += 1
        if self.received['/topic_test/scan'] == 1:
            print(GREEN + f"✓ First LaserScan message received on /topic_test/scan" + RESET)
            print(f"  Range count: {len(msg.ranges)}, "
                  f"Min angle: {msg.angle_min:.2f}, Max angle: {msg.angle_max:.2f}")
    
    def publish_test_messages(self, duration=3.0, rate_hz=10):
        """发布测试消息"""
        print(YELLOW + f"\nPublishing messages at {rate_hz}Hz for {duration} seconds..." + RESET)
        
        rate = rospy.Rate(rate_hz)
        start_time = rospy.Time.now()
        msg_count = 0
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            # 发布IMU消息
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.x = 0.1 + msg_count * 0.01
            imu_msg.linear_acceleration.y = 0.2
            imu_msg.linear_acceleration.z = 9.8
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.1
            self.pub_imu.publish(imu_msg)
            self.sent['/imu'] += 1
            
            # 发布Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = 1.0 + msg_count * 0.1
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.5
            self.pub_cmd_vel.publish(twist_msg)
            self.sent['/cmd_vel'] += 1
            
            # 发布LaserScan消息
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = "laser_frame"
            scan_msg.angle_min = -1.57
            scan_msg.angle_max = 1.57
            scan_msg.angle_increment = 0.01
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0
            scan_msg.ranges = [5.0 + 0.1 * (i % 10) for i in range(314)]
            scan_msg.intensities = []
            self.pub_scan.publish(scan_msg)
            self.sent['/scan'] += 1
            
            msg_count += 1
            
            # 每10条消息打印一次进度
            if msg_count % 10 == 0:
                print(f"  Published {msg_count} messages...")
            
            rate.sleep()
        
        print(f"Published {msg_count} messages of each type")
    
    def wait_for_messages(self, timeout=2.0):
        """等待消息接收"""
        print(YELLOW + f"\nWaiting {timeout}s for messages to be received..." + RESET)
        
        # 实时显示接收进度
        start_time = time.time()
        last_counts = dict(self.received)
        
        while time.time() - start_time < timeout:
            rospy.sleep(0.5)
            
            # 检查是否有新消息
            for topic in self.received:
                if self.received[topic] > last_counts[topic]:
                    print(f"  {topic}: received {self.received[topic]} messages")
                    last_counts[topic] = self.received[topic]
    
    def print_results(self):
        """打印测试结果"""
        print(CYAN + "\n========== Test Results ==========" + RESET)
        
        # 发送统计
        print(BLUE + "\nMessages Sent:" + RESET)
        for topic, count in self.sent.items():
            print(f"  {topic}: {YELLOW}{count}{RESET} messages")
        
        # 接收统计
        print(BLUE + "\nMessages Received:" + RESET)
        total_success = 0
        total_topics = 0
        
        for topic, count in self.received.items():
            total_topics += 1
            if count > 0:
                print(f"  {topic}: {GREEN}{count}{RESET} messages ✓")
                total_success += 1
            else:
                print(f"  {topic}: {RED}{count}{RESET} messages ✗")
        
        # 总结
        print(CYAN + "\n========== Summary ==========" + RESET)
        success_rate = (total_success / total_topics) * 100 if total_topics > 0 else 0
        
        if success_rate == 100:
            print(GREEN + f"✓ All topics working! ({total_success}/{total_topics})" + RESET)
        elif success_rate > 0:
            print(YELLOW + f"⚠ Partial success: {total_success}/{total_topics} topics working" + RESET)
        else:
            print(RED + f"✗ No topics received messages" + RESET)
        
        # 性能指标
        if total_success > 0:
            print(BLUE + "\nPerformance:" + RESET)
            for topic in self.sent:
                recv_topic = '/topic_test' + topic
                if recv_topic in self.received and self.sent[topic] > 0:
                    loss_rate = (1 - self.received[recv_topic] / self.sent[topic]) * 100
                    print(f"  {topic} → {recv_topic}: {100-loss_rate:.1f}% delivery rate")
        
        print(CYAN + "=============================" + RESET)
        print(YELLOW + "\n💡 Check Multibotnet output for detailed statistics" + RESET)
    
    def run_continuous_test(self, duration=10.0):
        """运行持续测试"""
        print(CYAN + f"\n========== Continuous Test ({duration}s) ==========" + RESET)
        
        rate = rospy.Rate(10)  # 10Hz
        start_time = rospy.Time.now()
        last_print_time = start_time
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            # 发布一条消息
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.linear_acceleration.z = 9.8
            self.pub_imu.publish(imu_msg)
            self.sent['/imu'] += 1
            
            # 每2秒打印一次进度
            if (rospy.Time.now() - last_print_time).to_sec() >= 2.0:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                print(f"  Progress: {elapsed:.1f}/{duration}s - "
                      f"Sent: {self.sent['/imu']}, "
                      f"Received: {self.received['/topic_test/imu']}")
                last_print_time = rospy.Time.now()
            
            rate.sleep()

def main():
    try:
        tester = TopicTest()
        
        # 基础测试
        print(CYAN + "\n======== Basic Test ========" + RESET)
        tester.publish_test_messages(duration=3.0, rate_hz=10)
        tester.wait_for_messages(timeout=3.0)  # 增加等待时间
        
        # 高频测试
        print(CYAN + "\n======== High Frequency Test ========" + RESET)
        tester.publish_test_messages(duration=2.0, rate_hz=20)
        tester.wait_for_messages(timeout=3.0)  # 增加等待时间
        
        # 打印结果
        tester.print_results()
        
        # 询问是否进行持续测试
        if len(sys.argv) > 1 and sys.argv[1] == '--continuous':
            tester.run_continuous_test(duration=10.0)
            tester.print_results()
        
    except rospy.ROSInterruptException:
        print(RED + "\nTest interrupted!" + RESET)
    except Exception as e:
        print(RED + f"\nError: {e}" + RESET)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()