# Multibotnet v4.1.0

[![ROS Version](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-4.1.0-brightgreen.svg)](https://github.com/SWUST-ICAA/Multibotnet/releases)

## 🎯 基本特性

- **支持任意ROS消息类型** - 无需修改代码，配置即用
- **高性能通信** - 基于ZeroMQ，低延迟高吞吐
- **数据压缩** - 支持LZ4/ZLIB压缩，节省带宽
- **自动重连** - 网络中断自动恢复
- **简单配置** - YAML配置文件，清晰易懂

## 🚀 快速开始

### 1. 安装依赖

```bash
# 必需依赖
sudo apt-get install libzmq3-dev libyaml-cpp-dev ros-$ROS_DISTRO-topic-tools

# 可选依赖（推荐安装，用于压缩）
sudo apt-get install liblz4-dev zlib1g-dev
```

### 2. 编译

```bash
cd ~/catkin_ws/src
git clone https://github.com/SWUST-ICAA/Multibotnet.git
cd ~/catkin_ws
catkin_make
```

### 3. 运行

```bash
roslaunch multibotnet multibotnet.launch config_file:=config/default.yaml
```

## 📝 配置文件详解

### IP映射

定义机器人的IP地址，方便在配置中引用：

```yaml
IP:
  self: '*'                    # '*' 表示绑定本机所有IP
  localhost: '127.0.0.1'       # 本地回环地址
  robot1: '192.168.1.101'      # 给机器人1的IP起个别名
  robot2: '192.168.1.102'      # 给机器人2的IP起个别名
```

### 新增一个发送话题

在`send_topics`下添加配置，将本地话题发送到网络：

```yaml
send_topics:
  - topic: /your_topic_name           # 本地ROS话题名
    message_type: sensor_msgs/Imu     # 消息类型（必须正确）
    max_frequency: 50                 # 最大发送频率(Hz)
    bind_address: self                # 使用IP映射中的键名
    port: 3001                        # 端口号（不要重复）
    compression: true                 # 是否压缩
```

### 新增一个接收话题

在`recv_topics`下添加配置，从网络接收话题：

```yaml
recv_topics:
  - topic: /received_topic_name       # 接收后本地发布的话题名
    message_type: sensor_msgs/Imu     # 消息类型（必须与发送端一致）
    connect_address: robot1           # 连接到哪个机器人（IP映射的键名）
    port: 3001                        # 端口号（必须与发送端一致）
```

### 高级配置详解

```yaml
advanced:
  # 压缩配置
  compression:
    enable: true              # 是否全局启用压缩
    type: lz4                # 压缩算法：
                             #   none - 不压缩
                             #   lz4  - 高速压缩（推荐）
                             #   zlib - 高压缩率
    level: 6                 # 压缩级别(1-9)，仅zlib有效
    
  # 线程池配置
  thread_pool:
    size: 4                  # 处理线程数
                            # 0 = 自动（使用CPU核心数）
                            # 建议设置为CPU核心数
    
  # 性能监控
  performance:
    enable_statistics: true           # 启用统计信息
    statistics_interval_ms: 5000     # 统计输出间隔（毫秒）
    
  # 重连策略
  retry:
    max_retries: 3           # 断线后最大重试次数
    interval_ms: 1000        # 每次重试间隔（毫秒）
```

## 🧪 本地测试（单机调试）

测试配置文件已提供，可在单机上测试功能：

```bash
# 终端1：启动通信节点
roslaunch multibotnet test_local.launch

# 终端2：运行测试脚本
rosrun multibotnet topic_test.py

# 终端3：查看接收到的话题
rostopic list
rostopic echo /topic_test/imu
```

测试脚本会发送IMU、Twist、LaserScan数据，并统计收发成功率。

## ❓ 常见问题

**Q: 为什么收不到数据？**
- 检查防火墙：`sudo ufw allow 3001` (开放端口)
- 检查IP配置：`ping 192.168.1.101` (测试连通性)
- 检查消息类型：发送端和接收端必须完全一致
- 检查端口号：发送端和接收端必须一致

**Q: 如何选择端口号？**
- 每个话题用不同端口
- 推荐3000-9000范围
- 避免常用端口（如80, 22, 3306等）

**Q: 压缩该开还是关？**
- 点云、图像等大数据：开启压缩
- IMU、cmd_vel等小数据：关闭压缩
- 网络带宽受限：开启压缩

**Q: 话题名冲突怎么办？**
- 接收话题加前缀：`/robot1/odom`
- 使用命名空间：`/robot1/sensors/imu`

## ⚡ 性能优化建议

1. **合理设置频率** - `max_frequency`不要设太高，够用即可
2. **选对压缩算法** - 实时数据用`lz4`，日志数据用`zlib`
3. **调整线程数** - `thread_pool.size`设为CPU核心数
4. **批量配置** - 相似的话题可以用相近的端口号，方便管理

## 📄 许可证

Apache License 2.0

## 🙋 需要帮助？

- 提交Issue: https://github.com/SWUST-ICAA/Multibotnet/issues
- 邮箱: nanwan2004@126.com

---
**让ROS多机器人通信像本地一样简单！**