import asyncio
from typing import Optional

from woosh_robot import WooshRobot
from woosh_interface import FULL_PRINT
from woosh.proto.robot.robot_setting_pb2 import (
    Identity,
    Server,
    AutoCharge,
    AutoPark,
    GoodsCheck,
    Power,
    Sound,
)
from woosh.proto.robot.robot_pack_pb2 import (
    SwitchControlMode,
    SwitchWorkMode,
    SetMuteCall,
    SetProgramMute,
)

from base_menu import BaseMenu


class RobotSettingMenu(BaseMenu):
    """机器人设置菜单"""

    prompt = "(setting) "

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        super().__init__(robot, loop)

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "基本设置": [
                ("set_identity", "设置机器人名称 (名称)"),
                ("set_server", "设置服务器 (地址 端口)"),
            ],
            "自动化设置": [
                ("set_auto_charge", "设置自动充电 (开关)"),
                ("set_auto_park", "设置自动泊车 (开关)"),
                ("set_goods_check", "设置货物检查 (开关)"),
            ],
            "电源设置": [
                ("set_power", "设置电源管理 (警告电量 低电量 空闲电量 满电量)"),
            ],
            "音频设置": [
                ("set_sound", "设置音量 (0-100)"),
                ("set_mute_call", "设置屏蔽呼叫 (开关)"),
                ("set_program_mute", "设置程序静音 (开关)"),
            ],
            "模式设置": [
                ("switch_control_mode", "切换控制模式"),
                ("switch_work_mode", "切换工作模式"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/back", "返回上级菜单"),
            ],
        }
        self.format_menu_help("机器人设置菜单帮助", sections)

    def do_set_identity(self, arg):
        """设置机器人身份: set_identity <名称>"""
        args = arg.split()
        if len(args) < 1:
            print("用法: set_identity <名称>")
            return

        robot_name = args[0]
        self.run_async(self._set_identity(robot_name))

    async def _set_identity(self, robot_name):
        identity = Identity()
        identity.name = robot_name

        result, ok, msg = await self.robot.set_identity(
            identity, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success(f"设置机器人身份: 名称={robot_name}", result))
        else:
            print(self.format_error("设置机器人身份请求", msg))

    def do_set_server(self, arg):
        """设置服务器: set_server <地址> <端口>"""
        args = arg.split()
        if len(args) != 2:
            print("用法: set_server <地址> <端口>")
            return

        try:
            addr = args[0]
            port = int(args[1])
        except ValueError:
            print("端口必须是整数")
            return

        self.run_async(self._set_server(addr, port))

    async def _set_server(self, addr, port):
        server = Server()
        server.ip = addr
        server.port = port

        result, ok, msg = await self.robot.set_server(server, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success(f"设置服务器: 地址={addr}, 端口={port}", result))
        else:
            print(self.format_error("设置服务器请求", msg))

    def do_set_auto_charge(self, arg):
        """设置自动充电: set_auto_charge <开启/关闭>"""
        args = arg.split()
        if len(args) < 1:
            print("用法: set_auto_charge <on/off>")
            return

        enable = args[0].lower() in ["on", "1", "true", "开启", "true"]

        self.run_async(self._set_auto_charge(enable))

    async def _set_auto_charge(self, enable):
        auto_charge = AutoCharge()
        auto_charge.allow = enable

        result, ok, msg = await self.robot.auto_charge(
            auto_charge, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            status = "开启" if enable else "关闭"
            print(self.format_success(f"设置自动充电: {status}", result))
        else:
            print(self.format_error("设置自动充电请求", msg))

    def do_set_auto_park(self, arg):
        """设置自动泊车: set_auto_park <开启/关闭>"""
        if not arg:
            print("用法: set_auto_park <on/off>")
            return

        enable = arg.lower() in ["on", "1", "true", "开启", "true"]
        self.run_async(self._set_auto_park(enable))

    async def _set_auto_park(self, enable):
        auto_park = AutoPark()
        auto_park.allow = enable

        result, ok, msg = await self.robot.auto_park(auto_park, FULL_PRINT, FULL_PRINT)

        if ok and result:
            status = "开启" if enable else "关闭"
            print(self.format_success(f"设置自动泊车: {status}", result))
        else:
            print(self.format_error("设置自动泊车请求", msg))

    def do_set_goods_check(self, arg):
        """设置货物检测: set_goods_check <开启/关闭>"""
        if not arg:
            print("用法: set_goods_check <on/off>")
            return

        enable = arg.lower() in ["on", "1", "true", "开启", "true"]
        self.run_async(self._set_goods_check(enable))

    async def _set_goods_check(self, enable):
        goods_check = GoodsCheck()
        goods_check.allow = enable

        result, ok, msg = await self.robot.goods_check(
            goods_check, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            status = "开启" if enable else "关闭"
            print(self.format_success(f"设置货物检测: {status}", result))
        else:
            print(self.format_error("设置货物检测请求", msg))

    def do_set_power(self, arg):
        """设置电源管理: set_power <警告电量> <低电量> <空闲电量> <满电量>"""
        args = arg.split()
        if len(args) != 4:
            print("用法: set_power <警告电量> <低电量> <空闲电量> <满电量>")
            return

        try:
            alarm = int(args[0])
            low = int(args[1])
            idle = int(args[2])
            full = int(args[3])
        except ValueError:
            print("所有电量值必须是整数")
            return

        self.run_async(self._set_power(alarm, low, idle, full))

    async def _set_power(self, alarm, low, idle, full):
        power = Power()
        power.alarm = alarm
        power.low = low
        power.idle = idle
        power.full = full

        result, ok, msg = await self.robot.config_power(power, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(
                self.format_success(
                    f"设置电源管理: 警告电量={alarm}, 低电量={low}, 空闲电量={idle}, 满电量={full}",
                    result,
                )
            )
        else:
            print(self.format_error("设置电源管理请求", msg))

    def do_set_sound(self, arg):
        """设置声音: set_sound <音量>"""
        if not arg:
            print("用法: set_sound <音量>")
            return

        try:
            volume = int(arg)
            if volume < 0 or volume > 100:
                print("音量必须在0-100之间")
                return
        except ValueError:
            print("音量必须是整数")
            return

        self.run_async(self._set_sound(volume))

    async def _set_sound(self, volume):
        sound = Sound()
        sound.volume = volume

        result, ok, msg = await self.robot.set_sound(sound, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success(f"设置声音: 音量={volume}", result))
        else:
            print(self.format_error("设置声音请求", msg))

    def do_set_mute_call(self, arg):
        """设置通话静音: set_mute_call <开启/关闭>"""
        if not arg:
            print("用法: set_mute_call <on/off>")
            return

        enable = arg.lower() in ["on", "1", "true", "开启", "true"]
        self.run_async(self._set_mute_call(enable))

    async def _set_mute_call(self, enable):
        mute_call = SetMuteCall()
        mute_call.mute = enable

        result, ok, msg = await self.robot.set_mute_call_req(
            mute_call, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            status = "开启" if enable else "关闭"
            print(self.format_success(f"设置屏蔽呼叫: {status}", result))
        else:
            print(self.format_error("设置屏蔽呼叫请求", msg))

    def do_set_program_mute(self, arg):
        """设置程序静音: set_program_mute <开启/关闭>"""
        if not arg:
            print("用法: set_program_mute <on/off>")
            return

        enable = arg.lower() in ["on", "1", "true", "开启", "true"]
        self.run_async(self._set_program_mute(enable))

    async def _set_program_mute(self, enable):
        program_mute = SetProgramMute()
        program_mute.enable = enable

        result, ok, msg = await self.robot.set_program_mute_req(
            program_mute, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            status = "开启" if enable else "关闭"
            print(self.format_success(f"设置程序静音: {status}", result))
        else:
            print(self.format_error("设置程序静音请求", msg))

    def do_switch_control_mode(self, arg):
        """切换控制模式: switch_control_mode <模式>
        模式: 0:未定义, 1:自动, 2:手动, 3:维护"""
        if not arg:
            print("用法: switch_control_mode <模式>")
            print("模式: 0:未定义, 1:自动, 2:手动, 3:维护")
            return

        try:
            mode = int(arg)
        except ValueError:
            print("模式必须是整数")
            return

        self.run_async(self._switch_control_mode(mode))

    async def _switch_control_mode(self, mode):
        switch_mode = SwitchControlMode()
        switch_mode.mode = mode

        result, ok, msg = await self.robot.switch_control_mode_req(
            switch_mode, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            mode_str = {0: "未定义", 1: "自动", 2: "手动", 3: "维护"}.get(
                mode, str(mode)
            )
            print(self.format_success(f"切换控制模式: {mode_str}", result))
        else:
            print(self.format_error("切换控制模式请求", msg))

    def do_switch_work_mode(self, arg):
        """切换工作模式: switch_work_mode <模式>
        模式: 0:未定义, 1:部署模式, 2:任务模式, 3:调度模式"""
        if not arg:
            print("用法: switch_work_mode <模式>")
            print("模式: 0:未定义, 1:部署模式, 2:任务模式, 3:调度模式")
            return

        try:
            mode = int(arg)
        except ValueError:
            print("模式必须是整数")
            return

        self.run_async(self._switch_work_mode(mode))

    async def _switch_work_mode(self, mode):
        switch_mode = SwitchWorkMode()
        switch_mode.mode = mode

        result, ok, msg = await self.robot.switch_work_mode_req(
            switch_mode, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            mode_str = {
                0: "未定义",
                1: "部署模式",
                2: "任务模式",
                3: "调度模式",
            }.get(mode, str(mode))
            print(self.format_success(f"切换工作模式: {mode_str}", result))
        else:
            print(self.format_error("切换工作模式请求", msg))

    def do_help(self, arg):
        """显示帮助信息"""
        self._print_help()
