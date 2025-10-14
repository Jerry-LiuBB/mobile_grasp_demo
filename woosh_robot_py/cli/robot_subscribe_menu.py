import asyncio
from typing import Optional, Dict, Callable

from woosh_robot import WooshRobot
from woosh_interface import FULL_PRINT
from woosh.proto.robot.robot_pb2 import (
    RobotState,
    Battery,
    PoseSpeed,
    Network,
    Setting,
    Mode,
    Scene,
    TaskProc,
    DeviceState,
    OperationState,
    HardwareState,
    Model,
    NavPath,
)
from woosh.proto.robot.robot_count_pb2 import StatusCode, AbnormalCodes
from woosh.proto.robot.robot_pack_pb2 import BuildMapData
from woosh.proto.robot.robot_count_pb2 import (
    Operation as CountOperation,
    Task as CountTask,
    Status as CountStatus,
)

from base_menu import BaseMenu


class RobotSubscribeMenu(BaseMenu):
    """机器人订阅菜单"""

    prompt = "(subscribe) "

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        super().__init__(robot, loop)
        self.active_subscriptions = {}
        self.callbacks: Dict[str, Callable] = {}
        self.monitor_task = None

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "订阅管理": [
                ("list", "列出当前活跃的订阅"),
                ("unsub", "取消指定话题的订阅"),
                ("unsub_all", "取消所有订阅"),
                ("monitor", "监控订阅状态"),
            ],
            "基础信息订阅": [
                ("setting", "订阅配置信息"),
                ("state", "订阅机器人状态"),
                ("battery", "订阅电量信息"),
                ("mode", "订阅机器人模式"),
                ("model", "订阅机器人模型"),
            ],
            "运行状态订阅": [
                ("pose_speed", "订阅位姿速度"),
                ("network", "订阅网络信息"),
                ("scene", "订阅场景信息"),
                ("task_exec", "订阅任务执行状态"),
            ],
            "设备状态订阅": [
                ("device_state", "订阅设备状态"),
                ("operation_state", "订阅运行状态"),
                ("hardware_state", "订阅硬件状态"),
                ("nav_path", "订阅导航路径"),
            ],
            "诊断信息订阅": [
                ("status_code", "订阅状态码"),
                ("abnormal_codes", "订阅异常码"),
            ],
            "地图与计数订阅": [
                ("build_map", "订阅地图构建数据"),
                ("count_operation", "订阅操作计数"),
                ("count_task", "订阅任务计数"),
                ("count_status", "订阅状态计数"),
                ("scanner", "订阅扫描仪数据"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/back", "返回上级菜单"),
            ],
        }
        self.format_menu_help("机器人订阅菜单帮助", sections)

    def do_list(self, arg):
        """列出当前活跃的订阅"""
        if not self.active_subscriptions:
            print("当前没有活跃的订阅")
        else:
            print("活跃的订阅:")
            for topic in self.active_subscriptions:
                print(f"- {topic}")

    def do_setting(self, arg):
        """配置信息订阅"""
        self.run_async(self._sub_setting())

    async def _sub_setting(self):
        topic = "woosh.robot.Setting"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def setting_callback(setting: Setting):
            print("配置信息更新:\n" + str(setting))

        self.callbacks[topic] = setting_callback

        try:
            success = await self.robot.robot_setting_sub(setting_callback, FULL_PRINT)
            if success:
                print("成功订阅配置信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅配置信息失败")
        except Exception as e:
            print(f"订阅配置信息时发生错误: {str(e)}")

    def do_state(self, arg):
        """机器人状态订阅"""
        self.run_async(self._sub_state())

    async def _sub_state(self):
        topic = "woosh.robot.RobotState"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def state_callback(state: RobotState):
            print("机器人状态更新:\n" + str(state))

        self.callbacks[topic] = state_callback

        try:
            success = await self.robot.robot_state_sub(state_callback, FULL_PRINT)
            if success:
                print("成功订阅机器人状态")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅机器人状态失败")
        except Exception as e:
            print(f"订阅机器人状态时发生错误: {str(e)}")

    def do_battery(self, arg):
        """电量信息订阅"""
        self.run_async(self._sub_battery())

    async def _sub_battery(self):
        topic = "woosh.robot.Battery"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def battery_callback(battery: Battery):
            print(f"电量信息更新:\n{str(battery)}")

        self.callbacks[topic] = battery_callback

        try:
            success = await self.robot.robot_battery_sub(battery_callback, FULL_PRINT)
            if success:
                print("成功订阅电池状态")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅电池状态失败")
        except Exception as e:
            print(f"订阅电池状态时发生错误: {str(e)}")

    def do_mode(self, arg):
        """机器人模式订阅"""
        self.run_async(self._sub_mode())

    async def _sub_mode(self):
        topic = "woosh.robot.Mode"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def mode_callback(mode: Mode):
            print("机器人模式更新:\n" + str(mode))

        self.callbacks[topic] = mode_callback

        try:
            success = await self.robot.robot_mode_sub(mode_callback, FULL_PRINT)
            if success:
                print("成功订阅机器人模式")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅机器人模式失败")
        except Exception as e:
            print(f"订阅机器人模式时发生错误: {str(e)}")

    def do_network(self, arg):
        """网络信息订阅"""
        self.run_async(self._sub_network())

    async def _sub_network(self):
        topic = "woosh.robot.Network"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def network_callback(network: Network):
            print("网络信息更新:\n" + str(network))

        self.callbacks[topic] = network_callback

        try:
            success = await self.robot.robot_network_sub(network_callback, FULL_PRINT)
            if success:
                print("成功订阅网络信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅网络信息失败")
        except Exception as e:
            print(f"订阅网络信息时发生错误: {str(e)}")

    def do_scene(self, arg):
        """场景信息订阅"""
        self.run_async(self._sub_scene())

    async def _sub_scene(self):
        topic = "woosh.robot.Scene"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def scene_callback(scene: Scene):
            print("场景信息更新:\n" + str(scene))

        self.callbacks[topic] = scene_callback

        try:
            success = await self.robot.robot_scene_sub(scene_callback, FULL_PRINT)
            if success:
                print("成功订阅场景信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅场景信息失败")
        except Exception as e:
            print(f"订阅场景信息时发生错误: {str(e)}")

    def do_task_exec(self, arg):
        """任务进度信息订阅"""
        self.run_async(self._sub_task_exec())

    async def _sub_task_exec(self):
        topic = "woosh.robot.TaskProc"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def task_exec_callback(task_exec: TaskProc):
            print("任务进度信息更新:\n" + str(task_exec))

        self.callbacks[topic] = task_exec_callback

        try:
            success = await self.robot.robot_task_process_sub(
                task_exec_callback, FULL_PRINT
            )
            if success:
                print("成功订阅任务进度信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅任务进度信息失败")
        except Exception as e:
            print(f"订阅任务进度信息时发生错误: {str(e)}")

    def do_pose_speed(self, arg):
        """位姿速度订阅"""
        self.run_async(self._sub_pose_speed())

    async def _sub_pose_speed(self):
        topic = "woosh.robot.PoseSpeed"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def pose_speed_callback(pose_speed: PoseSpeed):
            print("位姿速度更新:\n" + str(pose_speed))

        self.callbacks[topic] = pose_speed_callback

        try:
            success = await self.robot.robot_pose_speed_sub(
                pose_speed_callback, FULL_PRINT
            )
            if success:
                print("成功订阅位姿速度")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅位姿速度失败")
        except Exception as e:
            print(f"订阅位姿速度时发生错误: {str(e)}")

    def do_device_state(self, arg):
        """设备状态信息订阅"""
        self.run_async(self._sub_device_state())

    async def _sub_device_state(self):
        topic = "woosh.robot.DeviceState"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def device_state_callback(device_state: DeviceState):
            print("设备状态信息更新:\n" + str(device_state))

        self.callbacks[topic] = device_state_callback

        try:
            success = await self.robot.robot_device_state_sub(
                device_state_callback, FULL_PRINT
            )
            if success:
                print("成功订阅设备状态信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅设备状态信息失败")
        except Exception as e:
            print(f"订阅设备状态信息时发生错误: {str(e)}")

    def do_operation_state(self, arg):
        """运行状态信息订阅"""
        self.run_async(self._sub_operation_state())

    async def _sub_operation_state(self):
        topic = "woosh.robot.OperationState"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def operation_state_callback(operation_state: OperationState):
            print("运行状态信息更新:\n" + str(operation_state))

        self.callbacks[topic] = operation_state_callback

        try:
            success = await self.robot.robot_operation_state_sub(
                operation_state_callback, FULL_PRINT
            )
            if success:
                print("成功订阅运行状态信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅运行状态信息失败")
        except Exception as e:
            print(f"订阅运行状态信息时发生错误: {str(e)}")

    def do_hardware_state(self, arg):
        """硬件状态信息订阅"""
        self.run_async(self._sub_hardware_state())

    async def _sub_hardware_state(self):
        topic = "woosh.robot.HardwareState"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def hardware_state_callback(hardware_state: HardwareState):
            print("硬件状态信息更新:\n" + str(hardware_state))

        self.callbacks[topic] = hardware_state_callback

        try:
            success = await self.robot.robot_hardware_state_sub(
                hardware_state_callback, FULL_PRINT
            )
            if success:
                print("成功订阅硬件状态信息")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅硬件状态信息失败")
        except Exception as e:
            print(f"订阅硬件状态信息时发生错误: {str(e)}")

    def do_model(self, arg):
        """机器人模型订阅"""
        self.run_async(self._sub_model())

    async def _sub_model(self):
        topic = "woosh.robot.Model"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def model_callback(model: Model):
            print("机器人模型更新:\n" + str(model))

        self.callbacks[topic] = model_callback

        try:
            success = await self.robot.robot_model_sub(model_callback, FULL_PRINT)
            if success:
                print("成功订阅机器人模型")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅机器人模型失败")
        except Exception as e:
            print(f"订阅机器人模型时发生错误: {str(e)}")

    def do_nav_path(self, arg):
        """机器人导航路径订阅"""
        self.run_async(self._sub_nav_path())

    async def _sub_nav_path(self):
        topic = "woosh.robot.NavPath"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def nav_path_callback(nav_path: NavPath):
            print("机器人导航路径更新:\n" + str(nav_path))

        self.callbacks[topic] = nav_path_callback

        try:
            success = await self.robot.robot_nav_path_sub(nav_path_callback, FULL_PRINT)
            if success:
                print("成功订阅机器人导航路径")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅机器人导航路径失败")
        except Exception as e:
            print(f"订阅机器人导航路径时发生错误: {str(e)}")

    def do_status_code(self, arg):
        """状态码订阅"""
        self.run_async(self._sub_status_code())

    async def _sub_status_code(self):
        topic = "woosh.robot.count.StatusCode"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def status_code_callback(status_code: StatusCode):
            print("状态码更新:\n" + str(status_code))

        self.callbacks[topic] = status_code_callback

        try:
            success = await self.robot.robot_status_code_sub(
                status_code_callback, FULL_PRINT
            )
            if success:
                print("成功订阅状态码")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅状态码失败")
        except Exception as e:
            print(f"订阅状态码时发生错误: {str(e)}")

    def do_abnormal_codes(self, arg):
        """未恢复的异常码订阅"""
        self.run_async(self._sub_abnormal_codes())

    async def _sub_abnormal_codes(self):
        topic = "woosh.robot.count.AbnormalCodes"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def abnormal_codes_callback(abnormal_codes: AbnormalCodes):
            print("未恢复的异常码更新:\n" + str(abnormal_codes))

        self.callbacks[topic] = abnormal_codes_callback

        try:
            success = await self.robot.robot_abnormal_codes_sub(
                abnormal_codes_callback, FULL_PRINT
            )
            if success:
                print("成功订阅未恢复的异常码")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅未恢复的异常码失败")
        except Exception as e:
            print(f"订阅未恢复的异常码时发生错误: {str(e)}")

    # 保留Python版本特有的功能
    def do_build_map(self, arg):
        """地图构建数据订阅"""
        self.run_async(self._sub_build_map())

    async def _sub_build_map(self):
        topic = "woosh.robot.BuildMapData"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def build_map_callback(data: BuildMapData):
            print("地图构建更新:\n" + str(data))

        self.callbacks[topic] = build_map_callback

        try:
            success = await self.robot.build_map_data_sub(
                build_map_callback, FULL_PRINT
            )
            if success:
                print("成功订阅地图构建数据")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅地图构建数据失败")
        except Exception as e:
            print(f"订阅地图构建数据时发生错误: {str(e)}")

    def do_count_operation(self, arg):
        """操作计数订阅"""
        self.run_async(self._sub_count_operation())

    async def _sub_count_operation(self):
        topic = "woosh.robot.count.Operation"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def count_operation_callback(data: CountOperation):
            print("操作计数更新:\n" + str(data))

        self.callbacks[topic] = count_operation_callback

        try:
            success = await self.robot.robot_count_operation_sub(
                count_operation_callback, FULL_PRINT
            )
            if success:
                print("成功订阅操作计数")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅操作计数失败")
        except Exception as e:
            print(f"订阅操作计数时发生错误: {str(e)}")

    def do_count_task(self, arg):
        """任务计数订阅"""
        self.run_async(self._sub_count_task())

    async def _sub_count_task(self):
        topic = "woosh.robot.count.Task"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def count_task_callback(data: CountTask):
            print("任务计数更新:\n" + str(data))

        self.callbacks[topic] = count_task_callback

        try:
            success = await self.robot.robot_count_task_sub(
                count_task_callback, FULL_PRINT
            )
            if success:
                print("成功订阅任务计数")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅任务计数失败")
        except Exception as e:
            print(f"订阅任务计数时发生错误: {str(e)}")

    def do_count_status(self, arg):
        """状态计数订阅"""
        self.run_async(self._sub_count_status())

    async def _sub_count_status(self):
        topic = "woosh.robot.count.Status"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def count_status_callback(data: CountStatus):
            print("状态计数更新:\n" + str(data))

        self.callbacks[topic] = count_status_callback

        try:
            success = await self.robot.robot_count_status_sub(
                count_status_callback, FULL_PRINT
            )
            if success:
                print("成功订阅状态计数")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅状态计数失败")
        except Exception as e:
            print(f"订阅状态计数时发生错误: {str(e)}")

    def do_scanner(self, arg):
        """扫描仪数据订阅"""
        self.run_async(self._sub_scanner())

    async def _sub_scanner(self):
        topic = "woosh.robot.ScannerData"

        # 如果已经订阅，先取消
        if topic in self.active_subscriptions:
            await self._unsub(topic)

        def scanner_callback(data):
            print(f"扫描仪数据更新:\n{str(data)}")

        self.callbacks[topic] = scanner_callback

        try:
            success = await self.robot.scanner_data_sub(scanner_callback, FULL_PRINT)
            if success:
                print("成功订阅扫描仪数据")
                self.active_subscriptions[topic] = topic
            else:
                print("订阅扫描仪数据失败")
        except Exception as e:
            print(f"订阅扫描仪数据时发生错误: {str(e)}")

    def do_unsub(self, arg):
        """取消订阅: unsub <订阅名称>"""
        if not arg:
            print("用法: unsub <订阅名称>")
            return

        self.run_async(self._unsub(arg))

    async def _unsub(self, topic):
        if topic in self.active_subscriptions:
            try:
                success = await self.robot.unsubscribe(self.active_subscriptions[topic])
                if success:
                    print(f"成功取消订阅: {topic}")
                    del self.active_subscriptions[topic]
                    if topic in self.callbacks:
                        del self.callbacks[topic]
                    return True
                else:
                    print(f"取消订阅失败: {topic}")
                    return False
            except Exception as e:
                print(f"取消订阅时发生错误: {topic}, 错误: {str(e)}")
                return False
        else:
            print(f"未找到订阅: {topic}")
            return False

    def do_unsub_all(self, arg):
        """取消所有订阅"""
        self.run_async(self._unsub_all())

    async def _unsub_all(self):
        for topic in list(self.active_subscriptions.keys()):
            await self._unsub(topic)

    def do_monitor(self, arg):
        """监控所有订阅 (按Ctrl+C停止)"""
        if not self.active_subscriptions:
            print("没有活跃的订阅，请先订阅一些话题")
            return

        try:
            print("开始监控订阅 (按Ctrl+C停止)...")
            self.run_async(self._monitor())
        except KeyboardInterrupt:
            print("\n停止监控")

    async def _monitor(self):
        try:
            # 保持运行直到用户中断
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n停止监控")
        finally:
            # 确保清理任何资源
            if self.monitor_task:
                self.monitor_task.cancel()
                self.monitor_task = None

    def cleanup(self):
        """清理资源"""
        # 取消所有订阅
        if self.active_subscriptions:
            self.run_async(self._unsub_all())

        # 取消监控任务
        if self.monitor_task:
            self.monitor_task.cancel()
            self.monitor_task = None

        # 调用父类的清理方法
        super().cleanup()

    def do_help(self, arg):
        """显示帮助信息"""
        self._print_help()
