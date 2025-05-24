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
        self.stats = {
            'set_mode': {
                'provided': 0,
                'requested': 0,
                'success': 0,
                'failed': 0,
                'latencies': []
            },
            'test_trigger': {
                'provided': 0,
                'requested': 0,
                'success': 0,
                'failed': 0,
                'latencies': []
            }
        }
        
        self.lock = threading.Lock()
        
        print(CYAN + "\n╔════════════════════════════════════════════╗" + RESET)
        print(CYAN + "║      Multibotnet Service Test v4.0.0       ║" + RESET)
        print(CYAN + "╚════════════════════════════════════════════╝" + RESET)
        print(GREEN + "\n✓ Testing service forwarding through Multibotnet" + RESET)
        
        # 创建服务服务器（处理来自Multibotnet的请求）
        self.srv_set_mode = rospy.Service('/set_mode', SetBool, self.handle_set_mode)
        self.srv_trigger = rospy.Service('/test_trigger', Trigger, self.handle_trigger)
        print(GREEN + "✓ Service servers created:" + RESET)
        print(f"  • /set_mode (std_srvs/SetBool)")
        print(f"  • /test_trigger (std_srvs/Trigger)")
        
        # 等待服务可用
        print(YELLOW + "\n⏳ Waiting for Multibotnet services..." + RESET)
        services_ready = self.wait_for_services()
        
        if services_ready:
            # 创建服务客户端（调用通过Multibotnet转发的服务）
            self.cli_set_mode = rospy.ServiceProxy('/server_test/set_mode', SetBool)
            self.cli_trigger = rospy.ServiceProxy('/server_test/test_trigger', Trigger)
            print(GREEN + "✓ Service clients created:" + RESET)
            print(f"  • /server_test/set_mode")
            print(f"  • /server_test/test_trigger")
            print(GREEN + "\n✓ System ready for testing\n" + RESET)
        else:
            print(RED + "✗ Timeout waiting for services!" + RESET)
            print(YELLOW + "Make sure Multibotnet service node is running." + RESET)
    
    def wait_for_services(self, timeout=10.0):
        """等待服务可用"""
        services = ['/server_test/set_mode', '/server_test/test_trigger']
        start_time = time.time()
        
        for service in services:
            try:
                rospy.wait_for_service(service, timeout=timeout)
                print(f"  {GREEN}✓{RESET} {service} is available")
            except rospy.ROSException:
                print(f"  {RED}✗{RESET} {service} not available")
                return False
        
        return True
    
    def handle_set_mode(self, req):
        """处理SetBool服务请求"""
        with self.lock:
            self.stats['set_mode']['provided'] += 1
        
        # 模拟一些处理
        response = SetBoolResponse()
        response.success = req.data
        
        if req.data:
            response.message = f"Mode enabled at {rospy.Time.now()}"
        else:
            response.message = f"Mode disabled at {rospy.Time.now()}"
        
        if self.stats['set_mode']['provided'] == 1:
            print(GREEN + f"✓ First SetBool request received (data={req.data})" + RESET)
        
        return response
    
    def handle_trigger(self, req):
        """处理Trigger服务请求"""
        with self.lock:
            self.stats['test_trigger']['provided'] += 1
        
        # 模拟一些处理
        response = TriggerResponse()
        response.success = True
        response.message = f"Triggered at {rospy.Time.now()}, count={self.stats['test_trigger']['provided']}"
        
        if self.stats['test_trigger']['provided'] == 1:
            print(GREEN + f"✓ First Trigger request received" + RESET)
        
        return response
    
    def call_set_mode(self, data):
        """调用SetBool服务"""
        try:
            start_time = rospy.Time.now()
            
            request = SetBoolRequest()
            request.data = data
            
            response = self.cli_set_mode(request)
            
            # 计算延迟
            latency = (rospy.Time.now() - start_time).to_sec() * 1000  # ms
            
            with self.lock:
                self.stats['set_mode']['requested'] += 1
                self.stats['set_mode']['latencies'].append(latency)
                if response.success == data:  # 验证响应
                    self.stats['set_mode']['success'] += 1
                else:
                    self.stats['set_mode']['failed'] += 1
            
            return response, latency
            
        except rospy.ServiceException as e:
            with self.lock:
                self.stats['set_mode']['requested'] += 1
                self.stats['set_mode']['failed'] += 1
            print(RED + f"✗ Service call failed: {e}" + RESET)
            return None, None
    
    def call_trigger(self):
        """调用Trigger服务"""
        try:
            start_time = rospy.Time.now()
            
            request = TriggerRequest()
            response = self.cli_trigger(request)
            
            # 计算延迟
            latency = (rospy.Time.now() - start_time).to_sec() * 1000  # ms
            
            with self.lock:
                self.stats['test_trigger']['requested'] += 1
                self.stats['test_trigger']['latencies'].append(latency)
                if response.success:
                    self.stats['test_trigger']['success'] += 1
                else:
                    self.stats['test_trigger']['failed'] += 1
            
            return response, latency
            
        except rospy.ServiceException as e:
            with self.lock:
                self.stats['test_trigger']['requested'] += 1
                self.stats['test_trigger']['failed'] += 1
            print(RED + f"✗ Service call failed: {e}" + RESET)
            return None, None
    
    def run_basic_test(self):
        """基础功能测试"""
        print(BLUE + "\n▶ Basic Functionality Test" + RESET)
        print("─" * 40)
        
        # 测试SetBool服务
        print("\nTesting SetBool service:")
        
        # 测试True
        print("  • Calling with data=True...")
        response, latency = self.call_set_mode(True)
        if response:
            print(f"    {GREEN}✓{RESET} Success: {response.success}")
            print(f"    Message: {response.message}")
            print(f"    Latency: {latency:.2f}ms")
        
        rospy.sleep(0.5)
        
        # 测试False
        print("  • Calling with data=False...")
        response, latency = self.call_set_mode(False)
        if response:
            print(f"    {GREEN}✓{RESET} Success: {response.success}")
            print(f"    Message: {response.message}")
            print(f"    Latency: {latency:.2f}ms")
        
        rospy.sleep(0.5)
        
        # 测试Trigger服务
        print("\nTesting Trigger service:")
        print("  • Calling trigger...")
        response, latency = self.call_trigger()
        if response:
            print(f"    {GREEN}✓{RESET} Success: {response.success}")
            print(f"    Message: {response.message}")
            print(f"    Latency: {latency:.2f}ms")
    
    def run_performance_test(self, num_calls=50):
        """性能测试"""
        print(BLUE + f"\n▶ Performance Test ({num_calls} calls each)" + RESET)
        print("─" * 40)
        
        # 测试SetBool服务
        print(f"\nTesting SetBool service ({num_calls} calls)...")
        start_time = time.time()
        
        for i in range(num_calls):
            data = (i % 2 == 0)  # 交替True/False
            self.call_set_mode(data)
            
            # 显示进度
            if (i + 1) % 10 == 0:
                print(f"  Progress: {i + 1}/{num_calls}")
        
        set_bool_time = time.time() - start_time
        print(f"  Completed in {set_bool_time:.2f}s")
        
        rospy.sleep(1.0)
        
        # 测试Trigger服务
        print(f"\nTesting Trigger service ({num_calls} calls)...")
        start_time = time.time()
        
        for i in range(num_calls):
            self.call_trigger()
            
            # 显示进度
            if (i + 1) % 10 == 0:
                print(f"  Progress: {i + 1}/{num_calls}")
        
        trigger_time = time.time() - start_time
        print(f"  Completed in {trigger_time:.2f}s")
    
    def run_concurrent_test(self, duration=5.0):
        """并发测试"""
        print(BLUE + f"\n▶ Concurrent Test ({duration}s)" + RESET)
        print("─" * 40)
        
        stop_event = threading.Event()
        threads = []
        
        def set_bool_worker():
            count = 0
            while not stop_event.is_set():
                self.call_set_mode(count % 2 == 0)
                count += 1
                time.sleep(0.1)
        
        def trigger_worker():
            while not stop_event.is_set():
                self.call_trigger()
                time.sleep(0.15)
        
        # 启动工作线程
        for i in range(2):
            t1 = threading.Thread(target=set_bool_worker)
            t2 = threading.Thread(target=trigger_worker)
            threads.extend([t1, t2])
            t1.start()
            t2.start()
        
        print(f"Running {len(threads)} concurrent threads...")
        time.sleep(duration)
        
        # 停止线程
        stop_event.set()
        for t in threads:
            t.join()
        
        print("Concurrent test completed")
    
    def print_results(self):
        """打印详细测试结果"""
        print(CYAN + "\n╔════════════════════════════════════════════╗" + RESET)
        print(CYAN + "║              Test Results                  ║" + RESET)
        print(CYAN + "╚════════════════════════════════════════════╝" + RESET)
        
        # 服务调用统计
        print(BLUE + "\n📊 Service Call Statistics:" + RESET)
        print("─" * 50)
        print(f"{'Service':<20} {'Requested':<12} {'Successful':<12} {'Failed':<8}")
        print("─" * 50)
        
        total_requested = 0
        total_successful = 0
        total_failed = 0
        
        for service_name, stats in self.stats.items():
            requested = stats['requested']
            successful = stats['success']
            failed = stats['failed']
            
            total_requested += requested
            total_successful += successful
            total_failed += failed
            
            color = GREEN if failed == 0 else YELLOW if failed < requested * 0.05 else RED
            print(f"{service_name:<20} {requested:<12} "
                  f"{color}{successful:<12}{RESET} "
                  f"{RED if failed > 0 else ''}{failed:<8}{RESET}")
        
        print("─" * 50)
        color = GREEN if total_failed == 0 else YELLOW if total_failed < total_requested * 0.05 else RED
        print(f"{'Total':<20} {total_requested:<12} "
              f"{color}{total_successful:<12}{RESET} "
              f"{RED if total_failed > 0 else ''}{total_failed:<8}{RESET}")
        
        # 服务提供统计
        print(BLUE + "\n📥 Services Provided:" + RESET)
        print("─" * 30)
        for service_name, stats in self.stats.items():
            provided = stats['provided']
            print(f"{service_name:<20} {provided} requests handled")
        
        # 延迟统计
        print(BLUE + "\n⏱️  Latency Statistics (ms):" + RESET)
        print("─" * 50)
        print(f"{'Service':<20} {'Min':<10} {'Avg':<10} {'Max':<10} {'P95':<10}")
        print("─" * 50)
        
        for service_name, stats in self.stats.items():
            latencies = stats['latencies']
            if latencies:
                latencies_sorted = sorted(latencies)
                min_lat = latencies_sorted[0]
                avg_lat = sum(latencies) / len(latencies)
                max_lat = latencies_sorted[-1]
                p95_lat = latencies_sorted[int(len(latencies) * 0.95)]
                
                color = GREEN if avg_lat < 10 else YELLOW if avg_lat < 20 else RED
                print(f"{service_name:<20} {min_lat:<10.2f} "
                      f"{color}{avg_lat:<10.2f}{RESET} "
                      f"{max_lat:<10.2f} {p95_lat:<10.2f}")
            else:
                print(f"{service_name:<20} {'N/A':<10} {'N/A':<10} {'N/A':<10} {'N/A':<10}")
        
        # 总体评估
        print(BLUE + "\n🎯 Overall Assessment:" + RESET)
        print("─" * 50)
        
        if total_failed == 0:
            avg_latency = sum(sum(s['latencies'])/len(s['latencies']) 
                            for s in self.stats.values() 
                            if s['latencies']) / len(self.stats)
            if avg_latency < 10:
                print(GREEN + "✓ EXCELLENT: All calls successful, low latency (<10ms)" + RESET)
            elif avg_latency < 20:
                print(GREEN + "✓ GOOD: All calls successful, acceptable latency (<20ms)" + RESET)
            else:
                print(YELLOW + "⚠ FAIR: All calls successful, but high latency (>20ms)" + RESET)
        elif total_failed < total_requested * 0.01:
            print(YELLOW + "⚠ ACCEPTABLE: Less than 1% failure rate" + RESET)
        else:
            print(RED + "✗ POOR: High failure rate (>1%)" + RESET)
        
        # 验证消息路由
        print(BLUE + "\n🔄 Message Routing Verification:" + RESET)
        print("─" * 50)
        
        for service_name, stats in self.stats.items():
            if stats['requested'] > 0 and stats['provided'] > 0:
                print(f"{GREEN}✓{RESET} {service_name}: Local loopback working "
                      f"(sent {stats['requested']}, received {stats['provided']})")
            elif stats['requested'] > 0:
                print(f"{RED}✗{RESET} {service_name}: No requests received by local server")
            else:
                print(f"{YELLOW}⚠{RESET} {service_name}: No requests sent")
        
        print("\n" + YELLOW + "💡 Check Multibotnet logs for detailed statistics" + RESET)

def main():
    try:
        tester = ServiceTest()
        
        # 运行测试
        tester.run_basic_test()
        tester.run_performance_test(num_calls=50)
        tester.run_concurrent_test(duration=5.0)
        
        # 等待一下确保所有响应都被处理
        rospy.sleep(2.0)
        
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