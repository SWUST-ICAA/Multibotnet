#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Multibotnet v4.0.0 服务测试脚本
用于测试服务通信功能
"""

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import time
import threading

# ANSI颜色代码
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
PURPLE = "\033[35m"
CYAN = "\033[36m"

class ServiceTester:
    def __init__(self):
        rospy.init_node('multibotnet_service_tester')
        
        # 服务统计
        self.provided_count = {'set_mode': 0, 'test_trigger': 0}
        self.called_count = {'set_mode': 0, 'test_trigger': 0}
        self.success_count = {'set_mode': 0, 'test_trigger': 0}
        
        print(CYAN + "\n========== Multibotnet Service Tester ==========" + RESET)
        print("Testing service communication through Multibotnet")
        print(CYAN + "===============================================" + RESET + "\n")
        
        # 创建本地服务供multibotnet调用
        self.setup_service_providers()
        
        # 等待multibotnet服务代理准备好
        print(YELLOW + "Waiting for Multibotnet service proxies..." + RESET)
        self.wait_for_services()
        
        # 创建服务客户端来调用通过multibotnet的服务
        self.setup_service_clients()
        
    def setup_service_providers(self):
        """设置本地服务提供者"""
        self.set_mode_srv = rospy.Service('/set_mode', SetBool, self.handle_set_mode)
        self.test_trigger_srv = rospy.Service('/test_trigger', Trigger, self.handle_test_trigger)
        
        print(GREEN + "✓ Local services created:" + RESET)
        print("  - /set_mode (std_srvs/SetBool)")
        print("  - /test_trigger (std_srvs/Trigger)")
    
    def setup_service_clients(self):
        """设置服务客户端"""
        try:
            self.set_mode_client = rospy.ServiceProxy('/server_test/set_mode', SetBool)
            self.test_trigger_client = rospy.ServiceProxy('/server_test/test_trigger', Trigger)
            print(GREEN + "\n✓ Service clients created:" + RESET)
            print("  - /server_test/set_mode")
            print("  - /server_test/test_trigger")
        except Exception as e:
            print(RED + "Failed to create service clients: {}".format(e) + RESET)
    
    def wait_for_services(self):
        """等待服务可用"""
        services = ['/server_test/set_mode', '/server_test/test_trigger']
        timeout = 15.0  # 增加超时时间
        
        # 先等待一下让multibotnet服务节点完全启动
        print(YELLOW + "Giving Multibotnet time to initialize services..." + RESET)
        rospy.sleep(3.0)
        
        for service in services:
            try:
                rospy.wait_for_service(service, timeout=timeout)
                print(GREEN + "  ✓ {} is ready".format(service) + RESET)
            except rospy.ROSException:
                print(YELLOW + "  ⚠ {} not available (timeout)".format(service) + RESET)
    
    def handle_set_mode(self, req):
        """处理SetBool服务请求"""
        self.provided_count['set_mode'] += 1
        
        # 创建响应
        resp = SetBoolResponse()
        resp.success = True
        resp.message = "Mode set to: {}".format("ON" if req.data else "OFF")
        
        rospy.loginfo(BLUE + "Service /set_mode called with data={}".format(req.data) + RESET)
        return resp
    
    def handle_test_trigger(self, req):
        """处理Trigger服务请求"""
        self.provided_count['test_trigger'] += 1
        
        # 创建响应
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Trigger executed at {}".format(rospy.Time.now())
        
        rospy.loginfo(BLUE + "Service /test_trigger called" + RESET)
        return resp
    
    def test_service_calls(self):
        """测试服务调用"""
        rate = rospy.Rate(0.5)  # 每2秒调用一次
        toggle = True
        
        print(YELLOW + "\nStarting service call tests..." + RESET)
        print("Press Ctrl+C to stop\n")
        
        # 检查服务是否真的存在
        print(BLUE + "Available services:" + RESET)
        try:
            service_list = rospy.get_published_services()
            for srv in service_list:
                print("  - " + srv)
        except:
            pass
        print("")
        
        while not rospy.is_shutdown():
            # 测试SetBool服务
            try:
                self.called_count['set_mode'] += 1
                print(YELLOW + "Calling /server_test/set_mode with data={}...".format(toggle) + RESET)
                resp = self.set_mode_client(toggle)
                if resp.success:
                    self.success_count['set_mode'] += 1
                    print(GREEN + "✓ SetBool call successful: {}".format(resp.message) + RESET)
                else:
                    print(RED + "✗ SetBool call failed: {}".format(resp.message) + RESET)
                toggle = not toggle
            except rospy.ServiceException as e:
                print(RED + "✗ SetBool service call failed: {}".format(e) + RESET)
            
            # 稍微延迟一下
            rospy.sleep(0.5)
            
            # 测试Trigger服务
            try:
                self.called_count['test_trigger'] += 1
                resp = self.test_trigger_client()
                if resp.success:
                    self.success_count['test_trigger'] += 1
                    print(GREEN + "✓ Trigger call successful: {}".format(resp.message) + RESET)
                else:
                    print(RED + "✗ Trigger call failed: {}".format(resp.message) + RESET)
            except rospy.ServiceException as e:
                print(RED + "✗ Trigger service call failed: {}".format(e) + RESET)
            
            rate.sleep()
    
    def print_statistics(self):
        """定期打印统计信息"""
        rate = rospy.Rate(0.2)  # 每5秒打印一次
        
        # 等待一段时间让测试开始
        rospy.sleep(3.0)
        
        while not rospy.is_shutdown():
            print("\n" + BLUE + "========== Service Statistics ==========" + RESET)
            
            # 显示提供的服务统计
            print(PURPLE + "Services Provided (called by Multibotnet):" + RESET)
            for service in ['set_mode', 'test_trigger']:
                count = self.provided_count[service]
                print("  {}: {} calls received".format(service, count))
            
            # 显示调用的服务统计
            print(PURPLE + "\nServices Called (through Multibotnet):" + RESET)
            for service in ['set_mode', 'test_trigger']:
                called = self.called_count[service]
                success = self.success_count[service]
                if called > 0:
                    success_rate = (success / float(called)) * 100
                    color = GREEN if success_rate > 95 else YELLOW if success_rate > 80 else RED
                    print("  {}: {}/{} calls succeeded ({}{}%{})".format(
                        service, success, called, color, int(success_rate), RESET))
                else:
                    print("  {}: No calls made yet".format(service))
            
            print(BLUE + "========================================" + RESET)
            rate.sleep()

def main():
    try:
        tester = ServiceTester()
        
        # 启动统计打印线程
        stats_thread = threading.Thread(target=tester.print_statistics)
        stats_thread.daemon = True
        stats_thread.start()
        
        # 等待一下确保所有服务都准备好
        rospy.sleep(2.0)
        
        # 开始测试服务调用
        tester.test_service_calls()
        
    except rospy.ROSInterruptException:
        print("\n" + YELLOW + "Test stopped by user" + RESET)
    except Exception as e:
        print(RED + "Error: {}".format(e) + RESET)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()