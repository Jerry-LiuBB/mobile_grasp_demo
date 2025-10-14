import asyncio
from typing import Optional

from woosh_robot import WooshRobot
from woosh_interface import FULL_PRINT
from woosh.proto.robot.robot_pb2 import (
    RobotState,
    Battery,
    PoseSpeed,
    General,
    Network,
    RobotInfo,
    HardwareState,
    Mode,
    Scene,
    TaskProc,
    DeviceState,
    OperationState,
    Model,
    NavPath,
    TaskHistory,
)
from woosh.proto.robot.robot_count_pb2 import StatusCodes, AbnormalCodes

from base_menu import BaseMenu


class RobotInfoMenu(BaseMenu):
    """机器人信息菜单"""

    prompt = "(info) "

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        super().__init__(robot, loop)

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "基础信息查询": [
                ("all", "获取所有机器人信息"),
                ("general", "获取常规信息"),
                ("setting", "获取配置信息"),
                ("model", "获取机器人模型"),
            ],
            "状态信息查询": [
                ("state", "获取机器人状态"),
                ("battery", "获取电量信息"),
                ("mode", "获取机器人模式"),
                ("pose_speed", "获取位姿速度"),
                ("network", "获取网络信息"),
            ],
            "任务相关信息": [
                ("scene", "获取场景信息"),
                ("task_exec", "获取任务进度信息"),
                ("task_history", "获取任务历史"),
                ("nav_path", "获取导航路径"),
            ],
            "设备状态信息": [
                ("hardware_state", "获取硬件状态信息"),
                ("device_state", "获取设备状态信息"),
                ("operation_state", "获取运行状态信息"),
            ],
            "诊断信息": [
                ("status_codes", "获取状态码"),
                ("abnormal_codes", "获取未恢复的异常码"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/back", "返回上级菜单"),
            ],
        }
        self.format_menu_help("机器人信息菜单帮助", sections)

    def preloop(self):
        """在命令循环开始前显示帮助信息"""
        self._print_help()

    def do_all(self, arg):
        """机器人信息请求"""
        self.run_async(self._get_robot_info())

    async def _get_robot_info(self):
        req_info = RobotInfo()
        info, ok, msg = await self.robot.robot_info_req(
            req_info, FULL_PRINT, FULL_PRINT
        )
        if ok and info:
            print(self.format_success("机器人信息请求", info))
        else:
            print(self.format_error("机器人信息请求", msg))

    def do_general(self, arg):
        """常规信息请求"""
        self.run_async(self._get_general())

    async def _get_general(self):
        req_general = General()
        general, ok, msg = await self.robot.robot_general_req(
            req_general, FULL_PRINT, FULL_PRINT
        )
        if ok and general:
            print(self.format_success("常规信息请求", general))
        else:
            print(self.format_error("常规信息请求", msg))

    def do_setting(self, arg):
        """配置信息请求"""
        self.run_async(self._get_setting())

    async def _get_setting(self):
        from woosh.proto.robot.robot_pb2 import Setting

        req_setting = Setting()
        setting, ok, msg = await self.robot.robot_setting_req(
            req_setting, FULL_PRINT, FULL_PRINT
        )
        if ok and setting:
            print(self.format_success("配置信息请求", setting))
        else:
            print(self.format_error("配置信息请求", msg))

    def do_state(self, arg):
        """机器人状态请求"""
        self.run_async(self._get_state())

    async def _get_state(self):
        req_state = RobotState()
        state, ok, msg = await self.robot.robot_state_req(
            req_state, FULL_PRINT, FULL_PRINT
        )
        if ok and state:
            print(self.format_success("机器人状态请求", state))
        else:
            print(self.format_error("机器人状态请求", msg))

    def do_battery(self, arg):
        """电量信息请求"""
        self.run_async(self._get_battery())

    async def _get_battery(self):
        req_battery = Battery()
        battery, ok, msg = await self.robot.robot_battery_req(
            req_battery, FULL_PRINT, FULL_PRINT
        )
        if ok and battery:
            print(self.format_success("电量信息请求", battery))
        else:
            print(self.format_error("电量信息请求", msg))

    def do_mode(self, arg):
        """机器人模式请求"""
        self.run_async(self._get_mode())

    async def _get_mode(self):
        req_mode = Mode()
        mode, ok, msg = await self.robot.robot_mode_req(
            req_mode, FULL_PRINT, FULL_PRINT
        )
        if ok and mode:
            print(self.format_success("机器人模式请求", mode))
        else:
            print(self.format_error("机器人模式请求", msg))

    def do_pose_speed(self, arg):
        """位姿速度请求"""
        self.run_async(self._get_pose())

    async def _get_pose(self):
        req_pose = PoseSpeed()
        pose, ok, msg = await self.robot.robot_pose_speed_req(
            req_pose, FULL_PRINT, FULL_PRINT
        )
        if ok and pose:
            print(self.format_success("位姿速度请求", pose))
        else:
            print(self.format_error("位姿速度请求", msg))

    def do_network(self, arg):
        """网络信息请求"""
        self.run_async(self._get_network())

    async def _get_network(self):
        req_network = Network()
        network, ok, msg = await self.robot.robot_network_req(
            req_network, FULL_PRINT, FULL_PRINT
        )
        if ok and network:
            print(self.format_success("网络信息请求", network))
        else:
            print(self.format_error("网络信息请求", msg))

    def do_scene(self, arg):
        """场景信息请求"""
        self.run_async(self._get_scene())

    async def _get_scene(self):
        req_scene = Scene()
        scene, ok, msg = await self.robot.robot_scene_req(
            req_scene, FULL_PRINT, FULL_PRINT
        )
        if ok and scene:
            print(self.format_success("场景信息请求", scene))
        else:
            print(self.format_error("场景信息请求", msg))

    def do_task_exec(self, arg):
        """任务进度信息请求"""
        self.run_async(self._get_task_exec())

    async def _get_task_exec(self):
        req_task = TaskProc()
        task, ok, msg = await self.robot.robot_task_process_req(
            req_task, FULL_PRINT, FULL_PRINT
        )
        if ok and task:
            print(self.format_success("任务进度信息请求", task))
        else:
            print(self.format_error("任务进度信息请求", msg))

    def do_hardware_state(self, arg):
        """硬件状态信息请求"""
        self.run_async(self._get_hardware())

    async def _get_hardware(self):
        req_hardware = HardwareState()
        hardware, ok, msg = await self.robot.robot_hardware_state_req(
            req_hardware, FULL_PRINT, FULL_PRINT
        )
        if ok and hardware:
            print(self.format_success("硬件状态信息请求", hardware))
        else:
            print(self.format_error("硬件状态信息请求", msg))

    def do_device_state(self, arg):
        """设备状态信息请求"""
        self.run_async(self._get_device_state())

    async def _get_device_state(self):
        req_device = DeviceState()
        device, ok, msg = await self.robot.robot_device_state_req(
            req_device, FULL_PRINT, FULL_PRINT
        )
        if ok and device:
            print(self.format_success("设备状态信息请求", device))
        else:
            print(self.format_error("设备状态信息请求", msg))

    def do_operation_state(self, arg):
        """运行状态信息请求"""
        self.run_async(self._get_operation_state())

    async def _get_operation_state(self):
        req_operation = OperationState()
        operation, ok, msg = await self.robot.robot_operation_state_req(
            req_operation, FULL_PRINT, FULL_PRINT
        )
        if ok and operation:
            print(self.format_success("运行状态信息请求", operation))
        else:
            print(self.format_error("运行状态信息请求", msg))

    def do_model(self, arg):
        """机器人模型请求"""
        self.run_async(self._get_model())

    async def _get_model(self):
        req_model = Model()
        model, ok, msg = await self.robot.robot_model_req(
            req_model, FULL_PRINT, FULL_PRINT
        )
        if ok and model:
            print(self.format_success("机器人模型请求", model))
        else:
            print(self.format_error("机器人模型请求", msg))

    def do_nav_path(self, arg):
        """机器人导航路径请求"""
        self.run_async(self._get_nav_path())

    async def _get_nav_path(self):
        req_path = NavPath()
        path, ok, msg = await self.robot.robot_nav_path_req(
            req_path, FULL_PRINT, FULL_PRINT
        )
        if ok and path:
            print(self.format_success("机器人导航路径请求", path))
        else:
            print(self.format_error("机器人导航路径请求", msg))

    def do_task_history(self, arg):
        """历史任务请求"""
        self.run_async(self._get_task_history())

    async def _get_task_history(self):
        req_history = TaskHistory()
        history, ok, msg = await self.robot.robot_task_history_req(
            req_history, FULL_PRINT, FULL_PRINT
        )
        if ok and history:
            print(self.format_success("历史任务请求", history))
        else:
            print(self.format_error("历史任务请求", msg))

    def do_status_codes(self, arg):
        """状态码请求"""
        self.run_async(self._get_status_codes())

    async def _get_status_codes(self):
        req_codes = StatusCodes()
        codes, ok, msg = await self.robot.robot_status_codes_req(
            req_codes, FULL_PRINT, FULL_PRINT
        )
        if ok and codes:
            print(self.format_success("状态码请求", codes))
        else:
            print(self.format_error("状态码请求", msg))

    def do_abnormal_codes(self, arg):
        """未恢复的异常码请求"""
        self.run_async(self._get_abnormal_codes())

    async def _get_abnormal_codes(self):
        req_codes = AbnormalCodes()
        codes, ok, msg = await self.robot.robot_abnormal_codes_req(
            req_codes, FULL_PRINT, FULL_PRINT
        )
        if ok and codes:
            print(self.format_success("未恢复的异常码请求", codes))
        else:
            print(self.format_error("未恢复的异常码请求", msg))

    def do_help(self, arg):
        """显示帮助信息"""
        if arg:
            # 显示特定命令的帮助信息
            super().do_help(arg)
        else:
            # 显示所有命令的帮助信息
            self._print_help()
