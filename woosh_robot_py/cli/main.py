#!/usr/bin/env python3
import sys
import asyncio
import argparse
from typing import Optional, Tuple

sys.path.append("..")  

# 导入子菜单模块
from robot_info_menu import RobotInfoMenu
from robot_request_menu import RobotRequestMenu
from robot_subscribe_menu import RobotSubscribeMenu
from robot_setting_menu import RobotSettingMenu
from map_info_menu import MapInfoMenu
from ros_action_menu import RosActionMenu
from base_menu import BaseMenu

from woosh_interface import CommuSettings
from woosh_robot import WooshRobot


# 创建一个独立的函数来实例化和连接WooshRobot
async def create_and_connect_robot(
    ip: str, port: int, loop: asyncio.AbstractEventLoop
) -> Tuple[Optional[WooshRobot], bool]:
    """
    创建WooshRobot实例并连接

    Args:
        ip: 机器人IP地址
        port: 机器人端口
        loop: 事件循环

    Returns:
        Tuple[Optional[WooshRobot], bool]: (机器人实例, 是否连接成功)
    """
    settings = CommuSettings(addr=ip, port=port, loop=loop)
    robot = WooshRobot(settings)

    # 启动SDK，建立连接
    success = await robot.run()

    if success:
        print("连接已建立")
    else:
        print("警告：SDK启动成功，但连接未建立")

    return robot, success


class MainMenu(BaseMenu):
    """悟时机器人CLI主菜单"""

    intro = "欢迎使用悟时机器人命令行工具。输入 'help' 或 '?' 查看帮助。"
    prompt = "(woosh) "

    def __init__(self, robot: WooshRobot, loop: asyncio.AbstractEventLoop):
        super().__init__(robot, loop)

    def preloop(self):
        """在命令循环开始前显示帮助信息"""
        self._print_help()

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "功能菜单": [
                ("info", "进入机器人信息菜单"),
                ("request", "进入机器人请求菜单"),
                ("subscribe", "进入机器人订阅菜单"),
                ("setting", "进入机器人设置菜单"),
                ("map", "进入地图信息菜单"),
                ("action", "进入ROS Action菜单"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/quit", "退出程序"),
            ],
        }
        self.format_menu_help("悟时机器人主菜单帮助", sections, 50)

    def run_async(self, coro):
        """运行异步协程并返回结果"""
        return self.loop.run_until_complete(coro)

    def do_info(self, arg):
        """进入机器人信息菜单"""
        RobotInfoMenu(self.robot, self.loop).cmdloop()
        self._print_help()  # 返回主菜单时显示帮助

    def do_request(self, arg):
        """进入机器人请求菜单"""
        RobotRequestMenu(self.robot, self.loop).cmdloop()
        self._print_help()  # 返回主菜单时显示帮助

    def do_subscribe(self, arg):
        """进入机器人订阅菜单"""
        RobotSubscribeMenu(self.robot, self.loop).cmdloop()
        self._print_help()  # 返回主菜单时显示帮助

    def do_setting(self, arg):
        """进入机器人设置菜单"""
        RobotSettingMenu(self.robot, self.loop).cmdloop()
        self._print_help()  # 返回主菜单时显示帮助

    def do_map(self, arg):
        """进入地图信息菜单"""
        MapInfoMenu(self.robot, self.loop).cmdloop()
        self._print_help()  # 返回主菜单时显示帮助

    def do_action(self, arg):
        """进入ROS Action菜单"""
        RosActionMenu(self.robot, self.loop).cmdloop()
        self._print_help()  # 返回主菜单时显示帮助

    def do_help(self, arg):
        """显示帮助信息"""
        if arg:
            # 显示特定命令的帮助信息
            super().do_help(arg)
        else:
            # 显示所有命令的帮助信息
            self._print_help()

    def do_exit(self, arg):
        """退出程序"""
        if self.robot:
            self.run_async(self.robot.stop())
        print("退出悟时机器人命令行工具")
        return True

    def do_quit(self, arg):
        """退出程序"""
        return self.do_exit(arg)


def main():
    """主程序入口"""
    # 添加命令行参数解析
    parser = argparse.ArgumentParser(description="悟时机器人命令行工具")
    parser.add_argument(
        "--ip", type=str, default="172.20.254.63", help="机器人IP地址 (默认: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=10003, help="机器人端口 (默认: 5480)"
    )
    args = parser.parse_args()

    # 创建事件循环
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        # 实例化并连接机器人
        print(f"设置连接参数: IP={args.ip}, 端口={args.port}")

        # 使用事件循环运行异步函数
        robot, success = loop.run_until_complete(
            create_and_connect_robot(args.ip, args.port, loop)
        )

        if success:
            print("成功连接到机器人")
            # 使用已连接的机器人实例初始化MainMenu
            MainMenu(robot, loop).cmdloop()
        else:
            print("连接机器人失败，程序退出")
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 清理资源
        if "robot" in locals() and robot:
            loop.run_until_complete(robot.stop())

        # 关闭事件循环
        loop.close()


if __name__ == "__main__":
    main()
