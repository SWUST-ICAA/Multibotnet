#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ZMQ连接诊断工具
用于测试ZMQ PUB-SUB连接是否正常工作
"""

import zmq
import time
import sys
import threading

# ANSI颜色代码
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
PURPLE = "\033[35m"
CYAN = "\033[36m"

class ZmqDiagnostic:
    def __init__(self):
        self.context = zmq.Context()
        self.received_count = 0
        self.sent_count = 0
        
    def test_pub_sub(self, port=3001):
        """测试PUB-SUB连接"""
        print(CYAN + f"\n========== Testing ZMQ PUB-SUB on port {port} ==========" + RESET)
        
        # 创建发布者
        pub_socket = self.context.socket(zmq.PUB)
        pub_socket.setsockopt(zmq.SNDHWM, 1000)
        pub_socket.bind(f"tcp://*:{port}")
        print(GREEN + f"✓ Publisher bound to tcp://*:{port}" + RESET)
        
        # 创建订阅者
        sub_socket = self.context.socket(zmq.SUB)
        sub_socket.setsockopt(zmq.RCVHWM, 1000)
        sub_socket.connect(f"tcp://127.0.0.1:{port}")
        sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息
        print(GREEN + f"✓ Subscriber connected to tcp://127.0.0.1:{port}" + RESET)
        
        # 等待订阅生效（ZMQ的特性）
        print(YELLOW + "\nWaiting for subscription to take effect (2 seconds)..." + RESET)
        time.sleep(2)
        
        # 启动接收线程
        receiver_thread = threading.Thread(target=self._receive_loop, args=(sub_socket,))
        receiver_thread.daemon = True
        receiver_thread.start()
        
        # 发送测试消息
        print(YELLOW + "\nSending test messages..." + RESET)
        for i in range(10):
            message = f"Test message {i}".encode('utf-8')
            pub_socket.send(message)
            self.sent_count += 1
            print(f"  Sent: {message.decode()}")
            time.sleep(0.1)
        
        # 等待接收
        print(YELLOW + "\nWaiting for messages to be received..." + RESET)
        time.sleep(1)
        
        # 打印结果
        print(CYAN + "\n========== Results ==========" + RESET)
        print(f"Messages sent: {YELLOW}{self.sent_count}{RESET}")
        print(f"Messages received: {YELLOW}{self.received_count}{RESET}")
        
        if self.received_count == self.sent_count:
            print(GREEN + f"✓ All messages received! ZMQ connection is working correctly." + RESET)
        elif self.received_count > 0:
            print(YELLOW + f"⚠ Partial success: {self.received_count}/{self.sent_count} messages received" + RESET)
            print(YELLOW + "  This is normal for ZMQ PUB-SUB due to the 'slow joiner' syndrome." + RESET)
        else:
            print(RED + f"✗ No messages received. Check your ZMQ installation and firewall settings." + RESET)
        
        # 清理
        pub_socket.close()
        sub_socket.close()
        
    def _receive_loop(self, socket):
        """接收消息循环"""
        while True:
            try:
                message = socket.recv(flags=zmq.NOBLOCK)
                self.received_count += 1
                print(f"  {GREEN}Received: {message.decode()}{RESET}")
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                print(RED + f"  Receive error: {e}" + RESET)
                break
    
    def test_multibotnet_ports(self):
        """测试Multibotnet使用的所有端口"""
        print(CYAN + "\n========== Testing Multibotnet Ports ==========" + RESET)
        
        # 话题端口
        topic_ports = [3001, 3002, 3003]
        print(BLUE + "\nTopic ports:" + RESET)
        for port in topic_ports:
            self._test_port_connectivity(port)
        
        # 服务端口
        service_ports = [5001, 5002]
        print(BLUE + "\nService ports:" + RESET)
        for port in service_ports:
            self._test_port_connectivity(port)
    
    def _test_port_connectivity(self, port):
        """测试单个端口的连接性"""
        try:
            # 尝试创建一个临时的SUB套接字连接
            test_socket = self.context.socket(zmq.SUB)
            test_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms超时
            test_socket.connect(f"tcp://127.0.0.1:{port}")
            
            # 尝试接收（应该超时，但不会出错）
            try:
                test_socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                pass  # 正常，没有消息
            
            print(f"  Port {port}: {GREEN}✓ Connectable{RESET}")
            test_socket.close()
        except Exception as e:
            print(f"  Port {port}: {RED}✗ Error: {e}{RESET}")

def main():
    print(CYAN + "ZMQ Diagnostic Tool for Multibotnet" + RESET)
    print("This tool tests ZMQ connectivity without ROS dependencies")
    
    diagnostic = ZmqDiagnostic()
    
    # 测试基本的PUB-SUB功能
    diagnostic.test_pub_sub(port=9999)  # 使用不同的端口避免冲突
    
    # 测试Multibotnet端口
    diagnostic.test_multibotnet_ports()
    
    print(CYAN + "\n=============================" + RESET)
    print(YELLOW + "If the basic PUB-SUB test works but Multibotnet doesn't," + RESET)
    print(YELLOW + "the issue might be in the message serialization format." + RESET)

if __name__ == '__main__':
    main()