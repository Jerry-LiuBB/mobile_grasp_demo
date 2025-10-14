"""
WooshRobot SDK - Python SDK for Woosh Robot

这是悟时机器人控制系统的Python SDK，提供了一套完整的接口和工具，
使开发者能够轻松地实现机器人的运动控制、导航规划、任务管理等功能。

基本用法:
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings

    # 初始化连接
    settings = CommuSettings(addr="172.20.128.2", port=5480, identity="woosdk-demo")
    robot = WooshRobot(settings)
    await robot.run()

    # 控制机器人
    # ...

    # 清理连接
    await robot.stop()
"""

from .woosh_robot import WooshRobot
from .woosh_interface import (
    RobotInterface,
    PrintPackLevel,
    CommuSettings,
    NO_PRINT,
    HEAD_ONLY,
    FULL_PRINT,
)
from .woosh_base import (
    RobotCommunication,
    Common,
    RobotInfo,
    RobotSetting,
    MapInfo,
    MapEdit,
    DeviceInfo,
    RobotProxy,
)

# 版本信息
__version__ = "0.2.1"
__author__ = "woosh"
__email__ = "support@wooshrobot.com"
__license__ = "Proprietary"
__copyright__ = "Copyright 2025 Woosh Robotics"

__all__ = [
    # 主要类
    "WooshRobot",
    "RobotInterface",
    "CommuSettings",
    # 日志和打印级别
    "PrintPackLevel",
    "NO_PRINT",
    "HEAD_ONLY",
    "FULL_PRINT",
    # 基础组件
    "RobotCommunication",
    "Common",
    "RobotInfo",
    "RobotSetting",
    "MapInfo",
    "MapEdit",
    "DeviceInfo",
    "RobotProxy",
    # 版本信息
    "__version__",
    "__author__",
    "__email__",
]
