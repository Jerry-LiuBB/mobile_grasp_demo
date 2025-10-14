# WooshRobot SDK

Python SDK for Woosh Robot Control System - 悟时机器人控制系统SDK

## 项目简介

WooshRobot SDK 是一个用于控制和管理悟时机器人的 Python 软件开发工具包。它提供了一套完整的接口和工具，使开发者能够轻松地实现机器人的运动控制、导航规划、任务管理等功能。

基于WebSocket的异步通信机制和Protocol Buffers消息序列化，本SDK支持高效、可靠的机器人控制和状态监控。

## 功能特点

- **机器人信息获取**：获取机器人基本信息、状态、模式、电池状态等
- **运动控制**：支持速度控制（线速度、角速度）、跟随模式、导航路径规划和执行
- **地图管理**：地图下载、上传、重命名、删除，场景数据获取和管理
- **任务管理**：预设任务执行、自定义任务执行、任务历史查询
- **设备控制**：LED控制、语音播报、远程控制器支持
- **系统设置**：身份设置、服务器设置、自动充电、自动停靠设置、电源和声音设置

## 安装说明

### 环境要求

- Python 3.7+
- 网络连接到悟时机器人

## 快速开始

### 基础示例

```python
import asyncio
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings 
from woosh.proto.robot.robot_pb2 import RobotInfo
from woosh.proto.robot.robot_pack_pb2 import Speak 

async def main():
    # 初始化机器人连接
    settings = CommuSettings(
        addr="172.20.128.2",  # 机器人IP地址
        port=5480,            # 机器人端口
        identity="woosdk-demo"  # 客户端标识
    )
    robot = WooshRobot(settings)
    await robot.run()
    
    # 获取机器人信息
    robot_info, ok, msg = await robot.robot_info_req(RobotInfo())
    if ok:
        print(f"机器人信息: {str(robot_info)}") 
    else:
        print(f"获取机器人信息失败: {msg}")
    
    # 语音播报
    speak = Speak()
    speak.text = "你好，我是悟时机器人"
    result, ok, msg = await robot.speak_req(speak)
    if ok:
        print(f"成功播报: {speak.text}")
    else:
        print(f"语音播报失败: {msg}")
    
    # 清理连接
    await robot.close()

if __name__ == "__main__":
    asyncio.run(main())
```

### 订阅机器人状态

```python
import asyncio
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings 
from woosh.proto.robot.robot_pb2 import Battery, PoseSpeed

async def main():
    # 初始化机器人连接
    settings = CommuSettings(
        addr="172.20.128.2",
        port=5480,
        identity="woosdk-demo"
    )
    robot = WooshRobot(settings)
    await robot.run()
    
    # 定义回调函数
    def battery_callback(battery: Battery):
        print(f"电池电量: {battery.power}%")
        print(f"充电状态: {battery.charge_state}")
    
    def pose_speed_callback(pose_speed: PoseSpeed):
        print(f"位置 - X: {pose_speed.pose.x}, Y: {pose_speed.pose.y}")
        print(f"速度 - 线速度: {pose_speed.twist.linear}, 角速度: {pose_speed.twist.angular}")
    
    # 订阅电池状态
    await robot.robot_battery_sub(battery_callback)
    
    # 订阅位姿速度
    await robot.robot_pose_speed_sub(pose_speed_callback)
    
    # 保持程序运行
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        # 取消订阅
        await robot.unsubscribe("woosh.robot.Battery")
        await robot.unsubscribe("woosh.robot.PoseSpeed")
        # 清理连接
        await robot.close()

if __name__ == "__main__":
    asyncio.run(main())
```

## 依赖说明

- protobuf==4.21：用于消息序列化
- websockets==15.0：用于WebSocket通信
