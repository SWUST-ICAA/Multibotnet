#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Multibotnet v4.0.0 话题测试脚本
测试话题的发送和接收功能
"""

import rospy
import time
import sys
import threading
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
        
        # 延迟统计
        self.latencies = {
            'imu': [],
            'cmd_vel': [],
            'scan': []
        }
        
        # 消息时间戳记录
        self.msg_timestamps = {}
        self.lock = threading.Lock()
        
        # 创建发布者
        self.pub_imu = rospy.Publisher('/imu', Imu, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # 创建订阅者
        self.sub_imu = rospy.Subscriber('/topic_test/imu', Imu, self.imu_callback)
        self.sub_cmd_vel = rospy.Subscriber('/topic_test/cmd_vel', Twist, self.cmd_vel_callback)
        self.sub_scan = rospy.Subscriber('/topic_test/scan', LaserScan, self.scan_callback)
        
        print(CYAN + "\n╔════════════════════════════════════════════╗" + RESET)
        print(CYAN + "║      Multibotnet Topic Test v4.0.0         ║" + RESET)
        print(CYAN + "╚════════════════════════════════════════════╝" + RESET)
        print(GREEN + "\n✓ Testing topic forwarding through Multibotnet" + RESET)
        
        # 等待连接建立
        print(YELLOW + "\n⏳ Waiting for connections to establish..." + RESET)
        rospy.sleep(3.0)
        
        # 发送预热消息
        print(YELLOW + "📤 Sending warm-up messages..." + RESET)
        self.send_warmup_messages()
        rospy.sleep(1.0)
        print(GREEN + "✓ System ready for testing\n" + RESET)
    
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
            scan_msg.ranges = [1.0] * 10
            self.pub_scan.publish(scan_msg)
            
            rospy.sleep(0.1)
    
    def imu_callback(self, msg):
        self.received['/topic_test/imu'] += 1
        
        # 计算延迟
        with self.lock:
            key = f"imu_{msg.header.seq}"
            if key in self.msg_timestamps:
                latency = (rospy.Time.now() - self.msg_timestamps[key]).to_sec() * 1000  # ms
                self.latencies['imu'].append(latency)
                del self.msg_timestamps[key]
        
        if self.received['/topic_test/imu'] == 1:
            print(GREEN + f"✓ First IMU message received" + RESET)
    
    def cmd_vel_callback(self, msg):
        self.received['/topic_test/cmd_vel'] += 1
        
        # 计算延迟（使用线速度作为标识）
        with self.lock:
            key = f"cmd_vel_{msg.linear.x:.6f}"
            if key in self.msg_timestamps:
                latency = (rospy.Time.now() - self.msg_timestamps[key]).to_sec() * 1000  # ms
                self.latencies['cmd_vel'].append(latency)
                del self.msg_timestamps[key]
        
        if self.received['/topic_test/cmd_vel'] == 1:
            print(GREEN + f"✓ First Twist message received" + RESET)
    
    def scan_callback(self, msg):
        self.received['/topic_test/scan'] += 1
        
        # 计算延迟
        with self.lock:
            key = f"scan_{msg.header.seq}"
            if key in self.msg_timestamps:
                latency = (rospy.Time.now() - self.msg_timestamps[key]).to_sec() * 1000  # ms
                self.latencies['scan'].append(latency)
                del self.msg_timestamps[key]
        
        if self.received['/topic_test/scan'] == 1:
            print(GREEN + f"✓ First LaserScan message received" + RESET)
    
    def run_basic_test(self):
        """基础功能测试"""
        print(BLUE + "\n▶ Basic Functionality Test" + RESET)
        print("─" * 40)
        
        # 发送测试消息
        print("Sending test messages...")
        
        # IMU消息
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.seq = 1000
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = 0.1
        imu_msg.linear_acceleration.y = 0.2
        imu_msg.linear_acceleration.z = 9.8
        
        with self.lock:
            self.msg_timestamps[f"imu_{imu_msg.header.seq}"] = rospy.Time.now()
        self.pub_imu.publish(imu_msg)
        self.sent['/imu'] += 1
        print(f"  • IMU message sent (seq={imu_msg.header.seq})")
        
        # Twist消息
        twist_msg = Twist()
        twist_msg.linear.x = 1.5
        twist_msg.angular.z = 0.5
        
        with self.lock:
            self.msg_timestamps[f"cmd_vel_{twist_msg.linear.x:.6f}"] = rospy.Time.now()
        self.pub_cmd_vel.publish(twist_msg)
        self.sent['/cmd_vel'] += 1
        print(f"  • Twist message sent (linear.x={twist_msg.linear.x})")
        
        # LaserScan消息
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.seq = 2000
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = -1.57
        scan_msg.angle_max = 1.57
        scan_msg.angle_increment = 0.01
        scan_msg.ranges = [5.0] * 314
        
        with self.lock:
            self.msg_timestamps[f"scan_{scan_msg.header.seq}"] = rospy.Time.now()
        self.pub_scan.publish(scan_msg)
        self.sent['/scan'] += 1
        print(f"  • LaserScan message sent (seq={scan_msg.header.seq})")
        
        # 等待接收
        print("\nWaiting for messages...")
        rospy.sleep(2.0)
        
        # 检查结果
        print("\nResults:")
        for topic in ['/imu', '/cmd_vel', '/scan']:
            recv_topic = '/topic_test' + topic
            if self.received[recv_topic] > 0:
                print(f"  {GREEN}✓{RESET} {topic} → {recv_topic}")
            else:
                print(f"  {RED}✗{RESET} {topic} → {recv_topic}")
    
    def run_performance_test(self, duration=5.0, rate_hz=20):
        """性能测试"""
        print(BLUE + f"\n▶ Performance Test ({rate_hz}Hz, {duration}s)" + RESET)
        print("─" * 40)
        
        rate = rospy.Rate(rate_hz)
        start_time = rospy.Time.now()
        seq = 3000
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            # IMU消息
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.seq = seq
            imu_msg.header.frame_id = "imu_link"
            imu_msg.linear_acceleration.z = 9.8
            
            with self.lock:
                self.msg_timestamps[f"imu_{seq}"] = rospy.Time.now()
            self.pub_imu.publish(imu_msg)
            self.sent['/imu'] += 1
            
            # Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = 0.001 * seq  # 使用唯一值
            
            with self.lock:
                self.msg_timestamps[f"cmd_vel_{twist_msg.linear.x:.6f}"] = rospy.Time.now()
            self.pub_cmd_vel.publish(twist_msg)
            self.sent['/cmd_vel'] += 1
            
            # LaserScan消息
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.seq = seq
            scan_msg.header.frame_id = "laser_frame"
            scan_msg.ranges = [5.0] * 100
            
            with self.lock:
                self.msg_timestamps[f"scan_{seq}"] = rospy.Time.now()
            self.pub_scan.publish(scan_msg)
            self.sent['/scan'] += 1
            
            seq += 1
            rate.sleep()
        
        # 等待最后的消息
        print(f"Sent {seq - 3000} messages of each type")
        print("Waiting for remaining messages...")
        rospy.sleep(2.0)
    
    def print_results(self):
        """打印详细测试结果"""
        print(CYAN + "\n╔════════════════════════════════════════════╗" + RESET)
        print(CYAN + "║              Test Results                  ║" + RESET)
        print(CYAN + "╚════════════════════════════════════════════╝" + RESET)
        
        # 消息统计
        print(BLUE + "\n📊 Message Statistics:" + RESET)
        print("─" * 40)
        print(f"{'Topic':<20} {'Sent':<10} {'Received':<10} {'Loss Rate':<10}")
        print("─" * 40)
        
        total_sent = 0
        total_received = 0
        
        for send_topic in self.sent:
            recv_topic = '/topic_test' + send_topic
            sent = self.sent[send_topic]
            received = self.received[recv_topic]
            loss_rate = ((sent - received) / sent * 100) if sent > 0 else 0
            
            total_sent += sent
            total_received += received
            
            color = GREEN if loss_rate < 1 else YELLOW if loss_rate < 5 else RED
            print(f"{send_topic:<20} {sent:<10} {received:<10} "
                  f"{color}{loss_rate:.1f}%{RESET}")
        
        print("─" * 40)
        total_loss_rate = ((total_sent - total_received) / total_sent * 100) if total_sent > 0 else 0
        color = GREEN if total_loss_rate < 1 else YELLOW if total_loss_rate < 5 else RED
        print(f"{'Total':<20} {total_sent:<10} {total_received:<10} "
              f"{color}{total_loss_rate:.1f}%{RESET}")
        
        # 延迟统计
        print(BLUE + "\n⏱️  Latency Statistics (ms):" + RESET)
        print("─" * 40)
        print(f"{'Topic':<15} {'Min':<8} {'Avg':<8} {'Max':<8} {'Samples':<10}")
        print("─" * 40)
        
        for topic_type, latencies in self.latencies.items():
            if latencies:
                min_lat = min(latencies)
                avg_lat = sum(latencies) / len(latencies)
                max_lat = max(latencies)
                
                color = GREEN if avg_lat < 5 else YELLOW if avg_lat < 10 else RED
                print(f"{topic_type:<15} {min_lat:<8.2f} "
                      f"{color}{avg_lat:<8.2f}{RESET} "
                      f"{max_lat:<8.2f} {len(latencies):<10}")
            else:
                print(f"{topic_type:<15} {'N/A':<8} {'N/A':<8} {'N/A':<8} {'0':<10}")
        
        # 总体评估
        print(BLUE + "\n🎯 Overall Assessment:" + RESET)
        print("─" * 40)
        
        if total_loss_rate == 0 and all(len(l) > 0 for l in self.latencies.values()):
            avg_latency = sum(sum(l)/len(l) for l in self.latencies.values() if l) / 3
            if avg_latency < 5:
                print(GREEN + "✓ EXCELLENT: No message loss, low latency (<5ms)" + RESET)
            elif avg_latency < 10:
                print(GREEN + "✓ GOOD: No message loss, acceptable latency (<10ms)" + RESET)
            else:
                print(YELLOW + "⚠ FAIR: No message loss, but high latency (>10ms)" + RESET)
        elif total_loss_rate < 1:
            print(YELLOW + "⚠ ACCEPTABLE: Minimal message loss (<1%)" + RESET)
        else:
            print(RED + "✗ POOR: Significant message loss (>1%)" + RESET)
        
        print("\n" + YELLOW + "💡 Run 'rostopic echo' to verify message content" + RESET)
        print(YELLOW + "💡 Check Multibotnet logs for detailed statistics" + RESET)

def main():
    try:
        tester = TopicTest()
        
        # 运行测试
        tester.run_basic_test()
        tester.run_performance_test(duration=5.0, rate_hz=20)
        
        # 打印结果
        tester.print_results()
        
    except rospy.ROSInterruptException:
        print(RED + "\n❌ Test interrupted!" + RESET)
    except Exception as e:
        print(RED + f"\n❌ Error: {e}" + RESET)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()