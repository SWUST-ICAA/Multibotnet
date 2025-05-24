#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Multibotnet v4.0.0 服务测试脚本
测试服务的请求和响应功能
"""

import rospy
import time
import threading
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# ANSI颜色代码
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
PURPLE = "\033[35m"
CYAN = "\033[36m"

class ServiceTest:
    def __init__(self):
        rospy.init_node('multibotnet_service_test', anonymous=True)
        
        # 统计信息
        self.local_calls = {
            '/set_mode': 0,
            '/test_trigger': 0
        }
        self.remote_calls = {
            '/server_test/set_mode': 0,
            '/server_test/test_trigger': 0
        }
        self.call_latencies = []
        
        # 创建本地服务（这些服务会被 Multibotnet 转发）
        self.srv_set_mode = rospy.Service('/set_mode', SetBool, self.handle_set_mode)
        self.srv_trigger = rospy.Service('/test_trigger', Trigger, self.handle_trigger)
        
        # 创建"远程"服务（模拟通过 Multibotnet 暴露的服务）
        self.srv_remote_set_mode = rospy.Service(
            '/server_test/set_mode', SetBool, self.handle_remote_set_mode)
        self.srv_remote_trigger = rospy.Service(
            '/server_test/test_trigger', Trigger, self.handle_remote_trigger)
        
        print(CYAN + "\n========== Multibotnet Service Test ==========" + RESET)
        print(GREEN + "Testing service forwarding through Multibotnet..." + RESET)
        print(BLUE + "\nLocal services created:" + RESET)
        print("  - /set_mode (SetBool)")
        print("  - /test_trigger (Trigger)")
        print(BLUE + "\nRemote services created:" + RESET)
        print("  - /server_test/set_mode (SetBool)")
        print("  - /server_test/test_trigger (Trigger)")
    
    def handle_set_mode(self, req):
        """处理本地 SetBool 服务请求"""
        self.local_calls['/set_mode'] += 1
        print(PURPLE + f"[Local] /set_mode called (#{self.local_calls['/set_mode']})" + RESET)
        
        res = SetBoolResponse()
        res.success = True
        res.message = f"Local: Mode set to {req.data}"
        return res
    
    def handle_trigger(self, req):
        """处理本地 Trigger 服务请求"""
        self.local_calls['/test_trigger'] += 1
        print(PURPLE + f"[Local] /test_trigger called (#{self.local_calls['/test_trigger']})" + RESET)
        
        res = TriggerResponse()
        res.success = True
        res.message = "Local: Trigger executed"
        return res
    
    def handle_remote_set_mode(self, req):
        """处理远程 SetBool 服务请求（通过 Multibotnet）"""
        self.remote_calls['/server_test/set_mode'] += 1
        print(BLUE + f"[Remote] /server_test/set_mode called with data: {req.data}" + RESET)
        
        res = SetBoolResponse()
        res.success = True
        res.message = f"Remote: Mode set to {req.data} via Multibotnet"
        return res
    
    def handle_remote_trigger(self, req):
        """处理远程 Trigger 服务请求（通过 Multibotnet）"""
        self.remote_calls['/server_test/test_trigger'] += 1
        print(BLUE + "[Remote] /server_test/test_trigger called" + RESET)
        
        res = TriggerResponse()
        res.success = True
        res.message = "Remote: Trigger executed via Multibotnet"
        return res
    
    def test_local_services(self):
        """测试本地服务调用"""
        print(CYAN + "\n======== Testing Local Services ========" + RESET)
        
        # 测试 SetBool
        try:
            print(YELLOW + "\n1. Testing /set_mode (SetBool)..." + RESET)
            set_mode_client = rospy.ServiceProxy('/set_mode', SetBool)
            
            for i in range(3):
                req = SetBoolRequest(data=(i % 2 == 0))
                start_time = time.time()
                resp = set_mode_client(req)
                latency = (time.time() - start_time) * 1000  # ms
                
                print(f"  Call {i+1}: data={req.data}")
                print(f"    Response: success={resp.success}, message='{resp.message}'")
                print(f"    Latency: {latency:.2f}ms")
                time.sleep(0.2)
                
        except Exception as e:
            print(RED + f"  Error: {e}" + RESET)
        
        # 测试 Trigger
        try:
            print(YELLOW + "\n2. Testing /test_trigger (Trigger)..." + RESET)
            trigger_client = rospy.ServiceProxy('/test_trigger', Trigger)
            
            for i in range(3):
                req = TriggerRequest()
                start_time = time.time()
                resp = trigger_client(req)
                latency = (time.time() - start_time) * 1000  # ms
                
                print(f"  Call {i+1}:")
                print(f"    Response: success={resp.success}, message='{resp.message}'")
                print(f"    Latency: {latency:.2f}ms")
                time.sleep(0.2)
                
        except Exception as e:
            print(RED + f"  Error: {e}" + RESET)
    
    def test_remote_services(self):
        """测试远程服务调用（通过 Multibotnet）"""
        print(CYAN + "\n======== Testing Remote Services (via Multibotnet) ========" + RESET)
        
        # 等待服务可用
        print(YELLOW + "Waiting for remote services..." + RESET)
        try:
            rospy.wait_for_service('/server_test/set_mode', timeout=3.0)
            rospy.wait_for_service('/server_test/test_trigger', timeout=3.0)
            print(GREEN + "✓ Remote services are available" + RESET)
        except:
            print(RED + "✗ Remote services not available (timeout)" + RESET)
            return
        
        # 测试远程 SetBool
        try:
            print(YELLOW + "\n1. Testing /server_test/set_mode (SetBool) via Multibotnet..." + RESET)
            remote_set_mode = rospy.ServiceProxy('/server_test/set_mode', SetBool)
            
            for i in range(3):
                req = SetBoolRequest(data=(i % 2 == 1))
                start_time = time.time()
                resp = remote_set_mode(req)
                latency = (time.time() - start_time) * 1000  # ms
                self.call_latencies.append(latency)
                
                print(f"  Call {i+1}: data={req.data}")
                print(f"    Response: success={resp.success}, message='{resp.message}'")
                print(f"    Latency: {latency:.2f}ms")
                time.sleep(0.3)
                
        except Exception as e:
            print(RED + f"  Error: {e}" + RESET)
        
        # 测试远程 Trigger
        try:
            print(YELLOW + "\n2. Testing /server_test/test_trigger (Trigger) via Multibotnet..." + RESET)
            remote_trigger = rospy.ServiceProxy('/server_test/test_trigger', Trigger)
            
            for i in range(3):
                req = TriggerRequest()
                start_time = time.time()
                resp = remote_trigger(req)
                latency = (time.time() - start_time) * 1000  # ms
                self.call_latencies.append(latency)
                
                print(f"  Call {i+1}:")
                print(f"    Response: success={resp.success}, message='{resp.message}'")
                print(f"    Latency: {latency:.2f}ms")
                time.sleep(0.3)
                
        except Exception as e:
            print(RED + f"  Error: {e}" + RESET)
    
    def stress_test(self, num_calls=20, delay=0.1):
        """压力测试"""
        print(CYAN + f"\n======== Stress Test ({num_calls} calls) ========" + RESET)
        
        success_count = 0
        error_count = 0
        
        try:
            set_mode_client = rospy.ServiceProxy('/server_test/set_mode', SetBool)
            
            print(YELLOW + f"Sending {num_calls} rapid service calls..." + RESET)
            start_time = time.time()
            
            for i in range(num_calls):
                try:
                    req = SetBoolRequest(data=(i % 2 == 0))
                    resp = set_mode_client(req)
                    if resp.success:
                        success_count += 1
                    else:
                        error_count += 1
                    
                    if (i + 1) % 5 == 0:
                        print(f"  Progress: {i+1}/{num_calls} calls")
                    
                    time.sleep(delay)
                    
                except Exception as e:
                    error_count += 1
                    print(RED + f"  Call {i+1} failed: {e}" + RESET)
            
            elapsed_time = time.time() - start_time
            calls_per_sec = num_calls / elapsed_time
            
            print(GREEN + f"\n✓ Completed in {elapsed_time:.2f}s" + RESET)
            print(f"  Success: {success_count}/{num_calls}")
            print(f"  Errors: {error_count}")
            print(f"  Rate: {calls_per_sec:.1f} calls/sec")
            
        except Exception as e:
            print(RED + f"Stress test failed: {e}" + RESET)
    
    def print_results(self):
        """打印测试结果"""
        print(CYAN + "\n========== Test Results ==========" + RESET)
        
        # 本地服务调用统计
        print(BLUE + "\nLocal Service Calls:" + RESET)
        for service, count in self.local_calls.items():
            if count > 0:
                print(f"  {service}: {GREEN}{count}{RESET} calls ✓")
            else:
                print(f"  {service}: {YELLOW}{count}{RESET} calls")
        
        # 远程服务调用统计
        print(BLUE + "\nRemote Service Calls (via Multibotnet):" + RESET)
        for service, count in self.remote_calls.items():
            if count > 0:
                print(f"  {service}: {GREEN}{count}{RESET} calls ✓")
            else:
                print(f"  {service}: {YELLOW}{count}{RESET} calls")
        
        # 性能统计
        if self.call_latencies:
            avg_latency = sum(self.call_latencies) / len(self.call_latencies)
            min_latency = min(self.call_latencies)
            max_latency = max(self.call_latencies)
            
            print(BLUE + "\nPerformance Metrics:" + RESET)
            print(f"  Average latency: {avg_latency:.2f}ms")
            print(f"  Min latency: {min_latency:.2f}ms")
            print(f"  Max latency: {max_latency:.2f}ms")
        
        # 总结
        total_success = sum(1 for v in self.remote_calls.values() if v > 0)
        total_services = len(self.remote_calls)
        
        print(CYAN + "\n========== Summary ==========" + RESET)
        if total_success == total_services:
            print(GREEN + f"✓ All services working! ({total_success}/{total_services})" + RESET)
        elif total_success > 0:
            print(YELLOW + f"⚠ Partial success: {total_success}/{total_services} services working" + RESET)
        else:
            print(RED + f"✗ No remote services working" + RESET)
        
        print(CYAN + "=============================" + RESET)
        print(YELLOW + "\n💡 Check Multibotnet output for detailed statistics" + RESET)

def main():
    try:
        tester = ServiceTest()
        
        # 等待系统初始化
        print(YELLOW + "\nWaiting for system initialization..." + RESET)
        rospy.sleep(1.0)
        
        # 测试本地服务
        tester.test_local_services()
        
        # 测试远程服务（通过 Multibotnet）
        tester.test_remote_services()
        
        # 压力测试
        print(YELLOW + "\nRun stress test? (y/n): " + RESET, end='')
        try:
            # 设置超时读取
            import select
            import sys
            ready, _, _ = select.select([sys.stdin], [], [], 5.0)
            if ready:
                response = sys.stdin.readline().strip().lower()
                if response == 'y':
                    tester.stress_test(num_calls=20, delay=0.1)
        except:
            print("Skipping stress test")
        
        # 打印结果
        tester.print_results()
        
        # 保持节点运行一段时间以接收可能的延迟调用
        print(YELLOW + "\nKeeping node alive for 2 seconds..." + RESET)
        rospy.sleep(2.0)
        
    except rospy.ROSInterruptException:
        print(RED + "\nTest interrupted!" + RESET)
    except Exception as e:
        print(RED + f"\nError: {e}" + RESET)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()