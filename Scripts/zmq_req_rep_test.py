#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ZMQ REQ-REP模式测试
验证基本的请求-响应通信是否正常
"""

import zmq
import threading
import time
import sys

# ANSI颜色代码
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
PURPLE = "\033[35m"
CYAN = "\033[36m"

def rep_server(port=5555):
    """REP服务器"""
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://*:{port}")
    
    print(GREEN + f"REP server bound to tcp://*:{port}" + RESET)
    
    request_count = 0
    while True:
        try:
            # 等待请求
            print(YELLOW + "REP: Waiting for request..." + RESET)
            message = socket.recv()
            request_count += 1
            
            print(GREEN + f"REP: Received request #{request_count}: {message}" + RESET)
            
            # 发送响应
            response = f"Response to: {message.decode()}"
            socket.send(response.encode())
            print(BLUE + f"REP: Sent response: {response}" + RESET)
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(RED + f"REP Error: {e}" + RESET)
            break
    
    socket.close()
    context.term()
    print(YELLOW + "REP server stopped" + RESET)

def req_client(port=5555, num_requests=5):
    """REQ客户端"""
    # 等待服务器启动
    time.sleep(1)
    
    context = zmq.Context()
    
    for i in range(num_requests):
        # 为每个请求创建新的REQ套接字
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5秒超时
        socket.connect(f"tcp://localhost:{port}")
        
        print(CYAN + f"\n--- Request {i+1} ---" + RESET)
        print(GREEN + f"REQ: Connected to tcp://localhost:{port}" + RESET)
        
        try:
            # 发送请求
            request = f"Request #{i+1}"
            print(BLUE + f"REQ: Sending: {request}" + RESET)
            socket.send(request.encode())
            
            # 等待响应
            print(YELLOW + "REQ: Waiting for response..." + RESET)
            response = socket.recv()
            print(GREEN + f"REQ: Received: {response.decode()}" + RESET)
            
        except zmq.Again:
            print(RED + "REQ: Timeout waiting for response!" + RESET)
        except Exception as e:
            print(RED + f"REQ Error: {e}" + RESET)
        
        socket.close()
        time.sleep(0.5)
    
    context.term()
    print(CYAN + "\nREQ client finished" + RESET)

def test_multibotnet_pattern():
    """测试Multibotnet使用的模式"""
    print(CYAN + "\n========== Testing Multibotnet REQ-REP Pattern ==========" + RESET)
    
    # 启动REP服务器（模拟Multibotnet的服务提供端）
    server_thread = threading.Thread(target=rep_server, args=(5001,))
    server_thread.daemon = True
    server_thread.start()
    
    # 运行REQ客户端（模拟Multibotnet的服务请求端）
    req_client(5001, 3)
    
    print(CYAN + "\n========== Test Complete ==========" + RESET)

def test_basic_req_rep():
    """测试基本的REQ-REP通信"""
    print(CYAN + "\n========== Testing Basic ZMQ REQ-REP ==========" + RESET)
    
    # 启动服务器线程
    server_thread = threading.Thread(target=rep_server, args=(5555,))
    server_thread.daemon = True
    server_thread.start()
    
    # 运行客户端
    req_client(5555, 3)
    
    print(CYAN + "\n========== Test Complete ==========" + RESET)

def test_binary_data():
    """测试二进制数据传输"""
    print(CYAN + "\n========== Testing Binary Data Transfer ==========" + RESET)
    
    context = zmq.Context()
    
    # REP服务器
    def binary_server():
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5556")
        print(GREEN + "Binary REP server started" + RESET)
        
        message = socket.recv()
        print(GREEN + f"REP: Received {len(message)} bytes" + RESET)
        print(GREEN + f"REP: First 10 bytes: {list(message[:10])}" + RESET)
        
        # 发送二进制响应
        response = bytes([0x01, 0x02, 0x03, 0x04, 0x05])
        socket.send(response)
        print(BLUE + f"REP: Sent {len(response)} bytes" + RESET)
        
        socket.close()
    
    # 启动服务器
    server_thread = threading.Thread(target=binary_server)
    server_thread.daemon = True
    server_thread.start()
    
    time.sleep(0.5)
    
    # REQ客户端
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5556")
    
    # 发送二进制数据
    data = bytes([0x00, 0x01, 0x02, 0x03])
    print(BLUE + f"REQ: Sending {len(data)} bytes: {list(data)}" + RESET)
    socket.send(data)
    
    # 接收响应
    response = socket.recv()
    print(GREEN + f"REQ: Received {len(response)} bytes: {list(response)}" + RESET)
    
    socket.close()
    context.term()
    
    print(CYAN + "\n========== Binary Test Complete ==========" + RESET)

def main():
    print(CYAN + "ZMQ REQ-REP Test Tool" + RESET)
    print(CYAN + "====================" + RESET)
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--server":
            # 只运行服务器
            print(YELLOW + "Running REP server only (Ctrl+C to stop)..." + RESET)
            rep_server(5001)
        elif sys.argv[1] == "--client":
            # 只运行客户端
            print(YELLOW + "Running REQ client only..." + RESET)
            req_client(5001, 3)
        elif sys.argv[1] == "--binary":
            # 测试二进制数据
            test_binary_data()
    else:
        # 运行所有测试
        test_basic_req_rep()
        time.sleep(1)
        test_binary_data()
        time.sleep(1)
        test_multibotnet_pattern()

if __name__ == '__main__':
    main()