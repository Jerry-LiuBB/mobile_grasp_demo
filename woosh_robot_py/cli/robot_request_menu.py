import asyncio
from typing import Optional

from woosh_robot import WooshRobot
from woosh_interface import FULL_PRINT, PrintPackLevel
from woosh.proto.robot.robot_pack_pb2 import (
    Speak,
    LED,
    Twist,
    Follow,
    BuildMap,
    SwitchMap,
)
from woosh.proto.robot.robot_pack_pb2 import (
    InitRobot,
    SetRobotPose,
    SetOccupancy,
    ExecPreTask,
    ExecTask,
    ActionOrder,
    RobotWiFi,
)
from woosh.proto.util.action_pb2 import Order as ActionOrder
from google.protobuf.empty_pb2 import Empty

from base_menu import BaseMenu


class RobotRequestMenu(BaseMenu):
    """机器人请求菜单"""

    prompt = "(request) "

    # 定义打印级别常量
    PPLB = PrintPackLevel.BODY

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        super().__init__(robot, loop)

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "基础控制": [
                ("init", "初始化机器人"),
                ("set_pose", "设置机器人位姿 (x y theta)"),
                ("set_occupy", "设置占用状态 (0/1)"),
            ],
            "地图操作": [
                ("build_map", "构建地图"),
                ("switch_map", "切换地图"),
                ("deployment", "部署地图"),
            ],
            "任务控制": [
                ("exec_pre_task", "执行预设任务"),
                ("exec_task", "执行任务"),
                ("action_order", "执行动作指令"),
            ],
            "设备控制": [
                ("speak", "语音播报"),
                ("wifi", "WiFi设置"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/back", "返回上级菜单"),
            ],
        }
        self.format_menu_help("机器人请求菜单帮助", sections)

    def do_init(self, arg):
        """初始化机器人请求"""
        self.run_async(self._init())

    async def _init(self):
        init_robot = InitRobot()
        init_robot.is_record = True

        result, ok, msg = await self.robot.init_robot_req(
            init_robot, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("初始化机器人请求", result))
        else:
            print(self.format_error("初始化机器人请求", msg))

    def do_set_pose(self, arg):
        """设置机器人位姿: set_pose <x> <y> <theta>"""
        args = arg.split()
        if len(args) != 3:
            print("用法: set_pose <x> <y> <theta>")
            return

        try:
            x = float(args[0])
            y = float(args[1])
            theta = float(args[2])
        except ValueError:
            print("坐标必须是数字")
            return

        self.run_async(self._set_pose(x, y, theta))

    async def _set_pose(self, x, y, theta):
        pose = SetRobotPose()
        pose.pose.x = x
        pose.pose.y = y
        pose.pose.theta = theta

        result, ok, msg = await self.robot.set_robot_pose_req(
            pose, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(
                self.format_success(
                    f"设置机器人位姿: x={x}, y={y}, theta={theta}", result
                )
            )
        else:
            print(self.format_error("设置机器人位姿请求", msg))

    def do_set_occupy(self, arg):
        """设置机器人占用: set_occupy <true/false>"""
        if arg.lower() in ["on", "1", "true", "开启", "true"]:
            occupy = True
        elif arg.lower() in ["off", "0", "false", "关闭", "false"]:
            occupy = False
        else:
            print("用法: set_occupy <true/false>")
            return

        self.run_async(self._set_occupy(occupy))

    async def _set_occupy(self, occupy):
        req = SetOccupancy()
        req.occupy = occupy

        result, ok, msg = await self.robot.set_occupancy_req(
            req, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            status = "开启" if occupy else "关闭"
            print(self.format_success(f"设置机器人占用: {status}", result))
        else:
            print(self.format_error("设置机器人占用请求", msg))

    def do_build_map(self, arg):
        """构建地图: build_map <地图名称>"""
        if not arg:
            print("用法: build_map <地图名称>")
            return

        # TODO 不完整的

        self.run_async(self._build_map(arg))

    async def _build_map(self, map_name):
        build_map = BuildMap()
        build_map.map_name = map_name

        result, ok, msg = await self.robot.build_map_req(
            build_map, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success(f"构建地图: {map_name}", result))
        else:
            print(self.format_error("构建地图请求", msg))

    def do_switch_map(self, arg):
        """切换地图: switch_map <地图名称>"""
        if not arg:
            print("用法: switch_map <地图名称>")
            return

        self.run_async(self._switch_map(arg))

    async def _switch_map(self, map_name):
        switch_map = SwitchMap()
        switch_map.scene_name = map_name

        result, ok, msg = await self.robot.switch_map_req(
            switch_map, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success(f"切换地图: {map_name}", result))
        else:
            print(self.format_error("切换地图请求", msg))

    def do_deployment(self, arg):
        """部署请求"""
        print("部署功能未实现")

    def do_exec_pre_task(self, arg):
        """执行预设任务: exec_pre_task <任务ID>"""
        if not arg:
            print("用法: exec_pre_task <任务ID>")
            return

        try:
            task_id = int(arg)
        except ValueError:
            print("任务ID必须是整数")
            return

        self.run_async(self._exec_pre_task(task_id))

    async def _exec_pre_task(self, task_id):
        exec_task = ExecPreTask()
        exec_task.task_set_id = task_id

        result, ok, msg = await self.robot.exec_pre_task_req(
            exec_task, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success(f"执行预设任务: ID={task_id}", result))
        else:
            print(self.format_error("执行预设任务请求", msg))

    def do_exec_task(self, arg):
        """执行任务: exec_task <任务类型> <方向> <目标点编号>
        任务类型: 0:未定义, 1:拣选, 2:泊车, 3:充电, 4:搬运
        方向: 0:未定义, 1:上料, 2:下料
        目标点编号: 导航点编号"""
        args = arg.split()
        if len(args) < 3:
            print("用法: exec_task <任务类型> <方向> <目标点编号>")
            print("任务类型: 0:未定义, 1:拣选, 2:泊车, 3:充电, 4:搬运")
            print("方向: 0:未定义, 1:上料, 2:下料")
            print("目标点编号: 导航点编号")
            return

        try:
            task_type = int(args[0])
            direction = int(args[1])
        except ValueError:
            print("任务类型和方向必须是整数")
            return

        mark_no = args[3]
        self.run_async(self._exec_task(task_type, direction, mark_no))

    async def _exec_task(self, task_type, direction, mark_no):
        exec_task = ExecTask()
        exec_task.type = task_type
        exec_task.direction = direction
        exec_task.mark_no = mark_no

        result, ok, msg = await self.robot.exec_task_req(
            exec_task, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            type_str = {
                0: "未定义",
                1: "拣选",
                2: "泊车",
                3: "充电",
                4: "搬运",
            }.get(task_type, str(task_type))
            dir_str = {0: "未定义", 1: "上料", 2: "下料"}.get(direction, str(direction))
            print(
                self.format_success(
                    f"执行任务: 类型={type_str}, 方向={dir_str}, 目标点编号={mark_no}",
                    result,
                )
            )
        else:
            print(self.format_error("执行任务请求", msg))

    def do_action_order(self, arg):
        """动作指令: action_order <指令>
        指令: 0:未定义, 2:暂停, 3:继续, 4:取消, 5:恢复, 6:等待打断, 7:交通管制, 8:解除管制"""
        if not arg:
            print("用法: action_order <指令>")
            print(
                "指令: 0:未定义, 2:暂停, 3:继续, 4:取消, 5:恢复, 6:等待打断, 7:交通管制, 8:解除管制"
            )
            return

        try:
            order = int(arg)
        except ValueError:
            print("指令必须是整数")
            return

        self.run_async(self._action_order(order))

    async def _action_order(self, order):
        action_order = ActionOrder()
        action_order.order = order

        result, ok, msg = await self.robot.action_order_req(
            action_order, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            order_str = {
                0: "未定义",
                2: "暂停",
                3: "继续",
                4: "取消",
                5: "恢复",
                6: "等待打断",
                7: "交通管制",
                8: "解除管制",
            }.get(order, str(order))
            print(self.format_success(f"动作指令: {order_str}", result))
        else:
            print(self.format_error("动作指令请求", msg))

    def do_wifi(self, arg):
        """WiFi信息请求"""
        self.run_async(self._wifi())

    async def _wifi(self):
        wifi = RobotWiFi()
        wifi.order = RobotWiFi.kWiFiList
        result, ok, msg = await self.robot.robot_wifi_req(wifi, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success("WiFi信息请求", result))
        else:
            print(self.format_error("WiFi信息请求", msg))

    def do_speak(self, arg):
        """语音播报: speak <文本>"""
        if not arg:
            print("用法: speak <文本>")
            return

        self.run_async(self._speak(arg))

    async def _speak(self, text):
        speak = Speak()
        speak.text = text

        result, ok, msg = await self.robot.speak_req(speak, FULL_PRINT, FULL_PRINT)

        if ok and result:
            print(self.format_success(f"语音播报: {text}", result))
        else:
            print(self.format_error("语音播报请求", msg))

    def do_back(self, arg):
        """返回主菜单"""
        return True

    def do_help(self, arg):
        """显示帮助信息"""
        self._print_help()

    def do_exit(self, arg):
        """退出程序"""
        return True
