#!/bin/bash
# test_v4_final.sh - 最终测试V4版本所有修复

echo "=== 最终测试 Multibotnet V4 ==="

# 1. 编译项目
echo "1. 编译项目..."
cd ~/catkin_ws
catkin_make
if [ $? -ne 0 ]; then
    echo "   ✗ 编译失败"
    exit 1
fi
echo "   ✓ 编译成功"

# 2. 回到项目目录
cd ~/catkin_ws/src/multibotnet
if [ ! -f "package.xml" ]; then
    cd ~/Develop_Workplace/Multibotnet_project
fi

# 3. 创建测试配置
echo -e "\n2. 创建测试配置..."
mkdir -p config
cat > config/test_final.yaml << 'EOF'
# 本地测试配置
IP:
  self: '*'
  localhost: '127.0.0.1'

send_topics:
  - topic: /test/imu
    message_type: sensor_msgs/Imu
    max_frequency: 10
    bind_address: self
    port: 4001
    compression: false
    batch: false

  - topic: /test/cmd_vel
    message_type: geometry_msgs/Twist
    max_frequency: 20
    bind_address: self
    port: 4002
    compression: false
    batch: false

  - topic: /test/string
    message_type: std_msgs/String
    max_frequency: 5
    bind_address: self
    port: 4003
    compression: false
    batch: false

recv_topics:
  - topic: /received/imu
    message_type: sensor_msgs/Imu
    connect_address: localhost
    port: 4001

  - topic: /received/cmd_vel
    message_type: geometry_msgs/Twist
    connect_address: localhost
    port: 4002

  - topic: /received/string
    message_type: std_msgs/String
    connect_address: localhost
    port: 4003

provide_services:
  - service_name: /test/set_bool
    service_type: std_srvs/SetBool
    bind_address: self
    port: 5001

request_services:
  - service_name: /test/set_bool
    service_type: std_srvs/SetBool
    connect_address: localhost
    port: 5001
    timeout_ms: 3000
    max_retries: 3

advanced:
  compression:
    enable: false
  batch:
    enable: false
EOF
echo "   ✓ 配置文件创建成功"

# 4. 启动 Multibotnet
echo -e "\n3. 启动 Multibotnet..."
roslaunch multibotnet multibotnet.launch config_file:=$(pwd)/config/test_final.yaml &
LAUNCH_PID=$!
sleep 5

# 检查进程是否运行
if ! ps -p $LAUNCH_PID > /dev/null; then
    echo "   ✗ Multibotnet 启动失败"
    exit 1
fi

# 5. 检查节点
echo -e "\n4. 检查节点状态..."
NODES_OK=true
if rosnode list | grep -q "multibotnet_topic_node"; then
    echo "   ✓ multibotnet_topic_node 运行正常"
    # 检查节点详情
    rosnode info /multibotnet_topic_node | grep -q "Publications:" && echo "     - 发布者正常"
    rosnode info /multibotnet_topic_node | grep -q "Subscriptions:" && echo "     - 订阅者正常"
else
    echo "   ✗ multibotnet_topic_node 未找到"
    NODES_OK=false
fi

if rosnode list | grep -q "multibotnet_service_node"; then
    echo "   ✓ multibotnet_service_node 运行正常"
else
    echo "   ✗ multibotnet_service_node 未找到"
    NODES_OK=false
fi

if [ "$NODES_OK" = false ]; then
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

# 6. 测试话题通信
echo -e "\n5. 测试话题通信..."

# 测试IMU
echo "   测试 IMU 话题..."
rostopic pub -r 5 /test/imu sensor_msgs/Imu '{header: {stamp: now, frame_id: "test"}, orientation: {w: 1.0}}' &
PUB_IMU=$!
sleep 2
if timeout 3 rostopic echo -n 1 /received/imu > /dev/null 2>&1; then
    echo "   ✓ IMU 话题通信成功"
else
    echo "   ✗ IMU 话题通信失败"
fi
kill $PUB_IMU 2>/dev/null

# 测试Twist
echo "   测试 Twist 话题..."
rostopic pub -r 10 /test/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}' &
PUB_TWIST=$!
sleep 2
if timeout 3 rostopic echo -n 1 /received/cmd_vel > /dev/null 2>&1; then
    echo "   ✓ Twist 话题通信成功"
else
    echo "   ✗ Twist 话题通信失败"
fi
kill $PUB_TWIST 2>/dev/null

# 测试String
echo "   测试 String 话题..."
rostopic pub -r 2 /test/string std_msgs/String "data: 'Hello Multibotnet V4'" &
PUB_STRING=$!
sleep 2
if timeout 3 rostopic echo -n 1 /received/string 2>&1 | grep -q "Hello Multibotnet"; then
    echo "   ✓ String 话题通信成功"
else
    echo "   ✗ String 话题通信失败"
fi
kill $PUB_STRING 2>/dev/null

# 7. 测试服务
echo -e "\n6. 测试服务通信..."
# 先创建一个真实的ROS服务
rosservice call /test/set_bool "data: true" > /tmp/service_result.txt 2>&1 &
SERVICE_PID=$!
sleep 2

if grep -q "success: true" /tmp/service_result.txt 2>/dev/null || \
   grep -q "success: false" /tmp/service_result.txt 2>/dev/null; then
    echo "   ✓ 服务通信成功"
else
    echo "   ✗ 服务通信失败"
    cat /tmp/service_result.txt 2>/dev/null
fi
rm -f /tmp/service_result.txt

# 8. 显示话题列表
echo -e "\n7. 当前活跃话题列表："
rostopic list | grep -E "(test|received)" | sort

# 9. 性能测试
echo -e "\n8. 简单性能测试..."
echo "   发送100条消息..."
for i in {1..100}; do
    rostopic pub -1 /test/string std_msgs/String "data: 'Message $i'" &
done
wait
sleep 2
echo "   ✓ 性能测试完成"

# 10. 清理
echo -e "\n9. 清理..."
kill $LAUNCH_PID 2>/dev/null
sleep 2

echo -e "\n=== 测试完成 ==="
echo "✓ 表示通过的测试"
echo "✗ 表示失败的测试"
echo ""
echo "如果大部分测试通过，说明 V4 版本修复成功！"