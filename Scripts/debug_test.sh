#!/bin/bash

# Multibotnet 调试测试脚本
# 启用DEBUG级别日志并运行测试

echo "====================================="
echo "Multibotnet Debug Test"
echo "====================================="

# 设置ROS日志级别为DEBUG
export ROSCONSOLE_CONFIG_FILE=""
export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${message}'

# 创建临时的rosconsole配置文件
ROSCONSOLE_CONFIG=$(mktemp)
cat > $ROSCONSOLE_CONFIG << EOF
log4j.logger.ros=INFO
log4j.logger.ros.multibotnet=DEBUG
EOF
export ROSCONSOLE_CONFIG_FILE=$ROSCONSOLE_CONFIG

# 启动roscore如果还没有运行
echo "Checking if roscore is running..."
if ! rostopic list &>/dev/null; then
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 2
fi

# 清理函数
cleanup() {
    echo -e "\n\nCleaning up..."
    
    # 终止所有ROS节点
    rosnode kill -a 2>/dev/null
    
    # 如果我们启动了roscore，关闭它
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null
    fi
    
    # 删除临时配置文件
    rm -f $ROSCONSOLE_CONFIG
    
    exit
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 启动Multibotnet节点（带DEBUG日志）
echo -e "\n1. Starting Multibotnet nodes with DEBUG logging..."
roslaunch multibotnet test_local.launch &
LAUNCH_PID=$!

# 等待节点启动
echo -e "\n2. Waiting for nodes to initialize (5 seconds)..."
sleep 5

# 检查节点是否运行
echo -e "\n3. Checking node status..."
rosnode list

# 运行话题测试
echo -e "\n4. Running topic test..."
rosrun multibotnet topic_test.py

# 等待一下查看最终统计
echo -e "\n5. Waiting for final statistics..."
sleep 3

# 显示话题列表
echo -e "\n6. Current topics:"
rostopic list | grep -E "(imu|cmd_vel|scan)"

# 检查话题连接
echo -e "\n7. Topic connections:"
for topic in /imu /cmd_vel /scan /topic_test/imu /topic_test/cmd_vel /topic_test/scan; do
    echo -n "  $topic: "
    info=$(rostopic info $topic 2>/dev/null | grep -E "(Publishers|Subscribers)" | wc -l)
    if [ $info -gt 0 ]; then
        echo "Active"
        rostopic info $topic | grep -E "(Publishers|Subscribers)" | head -4
    else
        echo "Not found"
    fi
done

# 清理
cleanup