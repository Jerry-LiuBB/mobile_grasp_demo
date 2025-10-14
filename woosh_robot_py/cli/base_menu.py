import cmd
import asyncio
import sys
from typing import Optional

from woosh_robot import WooshRobot


class BaseMenu(cmd.Cmd):
    """基础菜单类，提供共享功能"""

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        super().__init__()
        self.robot = robot
        self.loop = loop
        self.subscriptions = {}
        self.callbacks = {}

        # 如果没有提供事件循环，尝试获取当前线程的事件循环
        if self.loop is None:
            try:
                self.loop = asyncio.get_running_loop()
            except RuntimeError:
                # 如果没有运行中的事件循环，创建一个新的
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)

    def run_async(self, coro):
        """运行异步协程并返回结果

        使用菜单实例化时提供的事件循环来执行异步协程，避免创建多个事件循环。

        Args:
            coro: 要执行的异步协程

        Returns:
            协程的执行结果
        """
        return self.loop.run_until_complete(coro)

    def ensure_connected(self, func):
        """装饰器：确保在执行命令前已连接

        如果未连接，会尝试重连并通知用户。

        Args:
            func: 要装饰的方法

        Returns:
            装饰后的方法
        """

        def wrapper(self, arg):
            if not self.robot.is_connected():
                print("未连接到机器人，尝试重连...")
                success = self.run_async(self.robot.ensure_connected())
                if not success:
                    print("重连失败，请检查网络连接")
                    return
                print("重连成功，继续执行命令")
            return func(self, arg)

        return wrapper

    def format_success(self, message, data):
        """格式化成功消息

        Args:
            message: 成功消息前缀
            data: 要显示的数据

        Returns:
            格式化后的成功消息
        """
        return f"{message}成功\n{str(data)}"

    def format_error(self, message, error):
        """格式化错误消息

        Args:
            message: 错误消息前缀
            error: 错误信息

        Returns:
            格式化后的错误消息
        """
        return f"{message}失败, msg: {error}"

    def do_back(self, arg):
        """返回上一级菜单"""
        # 在返回上一级菜单前清理资源
        self.cleanup()
        return True

    def do_exit(self, arg):
        """退出程序"""
        # 在退出前清理资源
        self.cleanup()
        print("退出程序")
        sys.exit(0)

    def cleanup(self):
        """清理资源

        这个方法会在菜单退出前被调用，用于清理订阅等资源。
        子类可以重写这个方法来添加特定的清理逻辑。
        """
        if hasattr(self, "subscriptions") and self.subscriptions:
            self.run_async(self._cleanup_subscriptions())

    async def _cleanup_subscriptions(self):
        """清理所有订阅

        这个方法会尝试取消所有活跃的订阅，并处理可能的异常。
        """
        if not hasattr(self, "subscriptions") or not self.subscriptions:
            return

        for topic, sub_id in list(self.subscriptions.items()):
            try:
                success = await self.robot.unsubscribe(sub_id)
                if success:
                    print(f"成功取消订阅: {topic}")
                    del self.subscriptions[topic]
                    if hasattr(self, "callbacks") and topic in self.callbacks:
                        del self.callbacks[topic]
                else:
                    print(f"取消订阅失败: {topic}")
            except Exception as e:
                print(f"取消订阅时发生错误: {topic}, 错误: {str(e)}")

    def check_connection(self) -> bool:
        """检查连接状态，如果未连接则尝试重连

        Returns:
            bool: 是否已连接
        """
        if not self.robot.is_connected():
            print("未连接到机器人，尝试重连...")
            success = self.run_async(self.robot.ensure_connected())
            if not success:
                print("重连失败，请检查网络连接")
                return False
            print("重连成功")
        return True

    def format_menu_help(self, title: str, sections: dict, width: int = 60):
        """统一格式化菜单帮助信息

        Args:
            title: 菜单标题
            sections: 包含各个分区及其命令的字典，格式为:
                     {
                         "分区名称": [("命令名", "命令描述"), ...],
                         ...
                     }
            width: 输出宽度，默认60个字符
        """
        # 打印标题
        print("\n" + "=" * width)
        padding = (width - len(title)) // 2
        print(" " * padding + title + " " * (width - padding - len(title)))
        print("=" * width)

        # 打印各个分区的命令
        for section, commands in sections.items():
            print(f"\n{section}:")
            for cmd, desc in commands:
                print(f"  {cmd.ljust(15)}- {desc}")

        # 打印提示信息
        print("\n提示: 使用 help <命令名> 可查看具体命令的详细信息")
        print("=" * width + "\n")

    def preloop(self):
        """在命令循环开始前显示帮助信息"""
        self._print_help()

    def _print_help(self):
        """打印帮助信息，子类应该重写这个方法"""
        pass

    def do_help(self, arg):
        """显示帮助信息"""
        if arg:
            # 显示特定命令的帮助信息
            super().do_help(arg)
        else:
            # 显示所有命令的帮助信息
            self._print_help()
