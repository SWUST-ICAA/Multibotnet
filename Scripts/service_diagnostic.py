#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
服务通信诊断工具
用于诊断Multibotnet服务通信问题
"""

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
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

def check_service(service_name, timeout=5.0):
    """检查服务是否可用"""
    print(YELLOW + f"Checking service {service_name}..." + RESET)
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        print(GREEN + f"  ✓ Service {service_name} is available" + RESET)
        return True
    except rospy.ROSException:
        print(RED + f"  ✗ Service {service_name} is NOT available" + RESET)
        return False

def list_services():
    """列出所有可用的服务"""
    print(BLUE + "\nAvailable ROS services:" + RESET)
    try:
        services = rospy.get_published_services()
        for service_name, service_type in services:
            print(f"  - {service_name} [{service_type}]")
    except Exception as e:
        print(RED + f"  Error listing services: {e}" + RESET)

def test_direct_service_call():
    """直接测试本地服务调用"""
    print(CYAN + "\n========== Testing Direct Service Calls ==========" + RESET)
    
    # 创建本地服务
    def handle_set_mode(req):
        resp = SetBoolResponse()
        resp.success = True
        resp.message = f"Direct: Mode set to {'ON' if req.data else 'OFF'}"
        print(BLUE + f"  Local service called with data={req.data}" + RESET)
        return resp
    
    srv = rospy.Service('/test/set_mode_direct', SetBool, handle_set_mode)
    print(GREEN + "✓ Created local service /test/set_mode_direct" + RESET)
    
    # 等待服务注册
    time.sleep(0.5)
    
    # 测试直接调用
    try:
        client = rospy.ServiceProxy('/test/set_mode_direct', SetBool)
        print(YELLOW + "Calling /test/set_mode_direct with data=True..." + RESET)
        resp = client(True)
        print(GREEN + f"  ✓ Response: success={resp.success}, message='{resp.message}'" + RESET)
    except Exception as e:
        print(RED + f"  ✗ Error: {e}" + RESET)
    
    srv.shutdown()

def test_multibotnet_service():
    """测试通过Multibotnet的服务调用"""
    print(CYAN + "\n========== Testing Multibotnet Service ==========" + RESET)
    
    # 检查必要的服务
    services_to_check = [
        '/set_mode',
        '/server_test/set_mode',
        '/test_trigger',
        '/server_test/test_trigger'
    ]
    
    all_available = True
    for service in services_to_check:
        if not check_service(service, timeout=2.0):
            all_available = False
    
    if not all_available:
        print(RED + "\nSome required services are not available!" + RESET)
        print(YELLOW + "Make sure both multibotnet nodes and server_test.py are running." + RESET)
        return
    
    # 测试SetBool服务
    print(YELLOW + "\nTesting SetBool service through Multibotnet..." + RESET)
    try:
        client = rospy.ServiceProxy('/server_test/set_mode', SetBool)
        
        # 发送请求
        print(YELLOW + "  Sending request with data=True..." + RESET)
        start_time = time.time()
        resp = client(True)
        elapsed = time.time() - start_time
        
        print(GREEN + f"  ✓ Response received in {elapsed*1000:.1f}ms" + RESET)
        print(GREEN + f"    success={resp.success}, message='{resp.message}'" + RESET)
        
    except rospy.ServiceException as e:
        print(RED + f"  ✗ Service call failed: {e}" + RESET)
    except Exception as e:
        print(RED + f"  ✗ Unexpected error: {e}" + RESET)
    
    # 测试Trigger服务
    print(YELLOW + "\nTesting Trigger service through Multibotnet..." + RESET)
    try:
        client = rospy.ServiceProxy('/server_test/test_trigger', Trigger)
        
        # 发送请求
        print(YELLOW + "  Sending trigger request..." + RESET)
        start_time = time.time()
        resp = client()
        elapsed = time.time() - start_time
        
        print(GREEN + f"  ✓ Response received in {elapsed*1000:.1f}ms" + RESET)
        print(GREEN + f"    success={resp.success}, message='{resp.message}'" + RESET)
        
    except rospy.ServiceException as e:
        print(RED + f"  ✗ Service call failed: {e}" + RESET)
    except Exception as e:
        print(RED + f"  ✗ Unexpected error: {e}" + RESET)

def test_service_performance():
    """测试服务调用性能"""
    print(CYAN + "\n========== Service Performance Test ==========" + RESET)
    
    if not check_service('/server_test/set_mode', timeout=2.0):
        print(RED + "Service not available for performance test" + RESET)
        return
    
    client = rospy.ServiceProxy('/server_test/set_mode', SetBool)
    
    # 预热
    print(YELLOW + "Warming up..." + RESET)
    for i in range(5):
        try:
            client(True)
        except:
            pass
    
    # 性能测试
    print(YELLOW + "Running performance test (10 calls)..." + RESET)
    
    latencies = []
    errors = 0
    
    for i in range(10):
        try:
            start_time = time.time()
            resp = client(i % 2 == 0)  # 交替发送True/False
            elapsed = (time.time() - start_time) * 1000  # 转换为毫秒
            latencies.append(elapsed)
            print(f"  Call {i+1}: {elapsed:.1f}ms")
        except Exception as e:
            errors += 1
            print(RED + f"  Call {i+1}: ERROR - {e}" + RESET)
    
    if latencies:
        avg_latency = sum(latencies) / len(latencies)
        min_latency = min(latencies)
        max_latency = max(latencies)
        
        print(BLUE + f"\nStatistics:" + RESET)
        print(f"  Success rate: {(10-errors)/10*100:.0f}%")
        print(f"  Average latency: {avg_latency:.1f}ms")
        print(f"  Min latency: {min_latency:.1f}ms")
        print(f"  Max latency: {max_latency:.1f}ms")

def main():
    rospy.init_node('service_diagnostic')
    
    print(CYAN + "Multibotnet Service Diagnostic Tool" + RESET)
    print(CYAN + "===================================" + RESET)
    
    # 列出所有服务
    list_services()
    
    # 测试直接服务调用（验证ROS服务机制正常）
    test_direct_service_call()
    
    # 测试Multibotnet服务
    test_multibotnet_service()
    
    # 性能测试
    if len(sys.argv) > 1 and sys.argv[1] == "--perf":
        test_service_performance()
    
    print(CYAN + "\n===================================" + RESET)
    print(YELLOW + "Diagnostic complete!" + RESET)

if __name__ == '__main__':
    main()