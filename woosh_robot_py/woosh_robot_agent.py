import asyncio
import logging
from collections import deque
from typing import Optional, Callable, Dict, Deque

# 导入Woosh相关模块
from woosh_interface import (
    CommuSettings,
    NO_PRINT,
    HEAD_ONLY,
    FULL_PRINT,
)
from woosh_robot import WooshRobot
from woosh.logger import create_logger

# 导入task_pb2协议相关类型
from woosh.proto.util.task_pb2 import (
    kCompleted as kTaskCompleted,
    kFailed as kTaskFailed,
    kCanceled as kTaskCanceled,
)

# 导入robot_pb2协议相关类型
from woosh.proto.robot.robot_pb2 import (
    RobotInfo,
    General,
    RobotState,
    Mode,
    PoseSpeed,
    Battery,
    Network,
    Scene,
    TaskProc,
    DeviceState,
    HardwareState,
    OperationState,
    Model,
    NavPath,
)

# 导入robot_count_pb2协议相关类型
from woosh.proto.robot.robot_count_pb2 import (
    StatusCode,
    StatusCodes,
    AbnormalCodes,
    Operation as CountOperation,
    Task as CountTask,
    Status as CountStatus,
)


class WooshRobotAgent:
    """woosh机器人代理类，用于管理机器人连接和缓存机器人状态信息"""

    # 类级别的常量设置
    DEFAULT_PORT = 5480

    def __init__(
        self,
        robot_ip: str,
        robot_port: int = DEFAULT_PORT,
        identity: str = "woosh-agent",
        logger: Optional[logging.Logger] = None,
        log_dir: Optional[str] = "logs",
        connect_status_callback: Optional[Callable[[bool], None]] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
        auto_connect: bool = False,
    ):
        """初始化机器人代理

        Args:
            robot_ip: 机器人IP地址
            robot_port: 机器人端口，默认5480
            identity: 客户端标识，默认"woosh-agent"
            logger: 日志记录器，如果为None则创建默认日志记录器
            log_dir: 日志文件目录，默认"logs"
            connect_status_callback: 连接状态变化回调函数
            loop: 事件循环，如果为None则使用当前事件循环或创建新的
            auto_connect: 是否在初始化时自动连接，默认False
        """
        # 参数验证
        if not robot_ip:
            raise ValueError("robot_ip不能为空")
        if not 0 < robot_port < 65536:
            raise ValueError("端口号无效")

        # 创建日志记录器
        if logger is None:
            self.logger = create_logger(
                name="woosh_robot_agent",
                level="INFO",
                log_dir=log_dir,
                console=True,
                file=True,
            )
        else:
            self.logger = logger

        # 保存外部连接状态回调函数
        self.external_connect_callback = connect_status_callback

        # 初始化或获取事件循环
        if loop is None:
            try:
                self.loop = asyncio.get_event_loop()
            except RuntimeError:
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
        else:
            self.loop = loop

        # 创建通信设置
        self.settings = CommuSettings(
            addr=robot_ip,
            port=robot_port,
            identity=identity,
            logger=self.logger,
            connect_status_callback=self._on_connection_status_changed,
            loop=self.loop,
        )

        # 创建机器人实例
        self.robot = WooshRobot(self.settings)

        # 初始化连接状态
        self.is_connected = False

        # 初始化RobotInfo及其组件
        self._init_robot_data()

        # 初始化锁对象
        self._task_lock = asyncio.Lock()
        self._status_lock = asyncio.Lock()

        # 自动连接设置
        if auto_connect:
            self.logger.info("初始化时自动连接...")
            # 创建连接任务但不等待完成
            asyncio.create_task(self._connect())

    def _init_robot_data(self):
        """初始化机器人数据结构"""
        # 主信息对象
        self.robot_info = RobotInfo()

        # 各组件信息
        self.general = General()
        self.state = RobotState()
        self.mode = Mode()
        self.pose_speed = PoseSpeed()
        self.battery = Battery()
        self.network = Network()
        self.scene = Scene()
        self.task_proc = TaskProc()
        self.device_state = DeviceState()
        self.hardware_state = HardwareState()
        self.operation_state = OperationState()
        self.model = Model()
        self.nav_path = NavPath()

        # 任务相关数据结构
        self.task_map: Dict[int, TaskProc] = (
            {}
        )  # 任务历史字典，键为robot_task_id，值为TaskProc对象
        self.task_ids: Deque[int] = deque(
            maxlen=50
        )  # 任务ID时间顺序列表，用于维护最新的50个任务

        # 状态和计数相关数据
        self.status_codes = StatusCodes()
        self.abnormal_codes = AbnormalCodes()
        self.count_operation = CountOperation()
        self.count_task = CountTask()
        self.count_status = CountStatus()

    # ===== 对外接口方法 =====

    @classmethod
    async def create(
        cls,
        robot_ip: str,
        robot_port: int = DEFAULT_PORT,
        identity: str = "woosh-agent",
        logger: Optional[logging.Logger] = None,
        log_dir: Optional[str] = "logs",
        connect_status_callback: Optional[Callable[[bool], None]] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ) -> "WooshRobotAgent":
        """异步工厂方法，创建并初始化机器人代理

        Args:
            robot_ip: 机器人IP地址
            robot_port: 机器人端口，默认5480
            identity: 客户端标识，默认"woosh-agent"
            logger: 日志记录器，如果为None则创建默认日志记录器
            log_dir: 日志文件目录，默认"logs"
            connect_status_callback: 连接状态变化回调函数
            loop: 事件循环，如果为None则使用当前事件循环或创建新的

        Returns:
            WooshRobotAgent: 已初始化并连接的机器人代理实例
        """
        # 创建实例但不自动连接
        agent = cls(
            robot_ip=robot_ip,
            robot_port=robot_port,
            identity=identity,
            logger=logger,
            log_dir=log_dir,
            connect_status_callback=connect_status_callback,
            loop=loop,
            auto_connect=False,  # 不自动连接
        )

        # 手动执行连接，并等待连接完成
        await agent._connect()

        # 返回已连接的实例
        return agent

    async def get_task_proc(self, task_id: int = None) -> TaskProc:
        """线程安全地获取指定任务进度

        Args:
            task_id: 任务ID，如果为None则返回最新任务

        Returns:
            TaskProc: 任务进度对象，如果指定任务不存在则返回None
        """
        if task_id:
            async with self._task_lock:
                return self.task_map.get(task_id, None)
        else:
            return self.task_proc  # 这个是单独的最新任务，通常在其他方法中有锁保护

    async def close(self) -> None:
        """显式关闭连接并清理资源

        推荐使用此方法代替依赖析构函数
        """
        try:
            # 取消所有订阅
            await self._unsubscribe_all()

            # 关闭机器人连接
            if hasattr(self.robot, "close") and callable(getattr(self.robot, "close")):
                await self.robot.close()

            # 清理锁
            self._task_lock = None
            self._status_lock = None

            self.logger.info("机器人代理已关闭")
        except Exception as e:
            self.logger.error(f"关闭机器人代理时发生错误: {str(e)}")

    def __del__(self):
        """析构函数，清理资源"""
        try:
            # 检查循环是否关闭和运行
            if hasattr(self, "loop") and not self.loop.is_closed():
                # 取消所有订阅
                if self.loop.is_running():
                    # 循环正在运行，创建任务
                    asyncio.create_task(self._unsubscribe_all())
                else:
                    # 循环未运行，可以同步等待
                    self.loop.run_until_complete(self._unsubscribe_all())
        except Exception as e:
            if hasattr(self, "logger"):
                self.logger.error(f"清理资源时发生错误: {str(e)}")

    # ===== 连接管理方法 =====

    async def _connect(self) -> None:
        """连接到机器人并初始化"""
        try:
            self.logger.info(
                f"正在连接到机器人: {self.settings.addr}:{self.settings.port}"
            )
            # 运行机器人实例，连接状态变化会通过回调函数通知
            await self.robot.run()
            # 订阅所有信息更新
            await self._subscribe_all()
            self.logger.info("连接并订阅完成")

        except Exception as e:
            self.logger.error(f"连接机器人时发生错误: {str(e)}")
            raise  # 重新抛出异常，让调用者知道连接失败

    def _on_connection_status_changed(self, connected: bool) -> None:
        """内部连接状态变化处理函数

        Args:
            connected: 当前的连接状态
        """
        # 更新内部连接状态
        self.is_connected = connected

        # 记录日志
        if connected:
            self.logger.info(f"连接到机器人: {self.settings.addr}:{self.settings.port}")

            # 在连接成功后获取机器人信息
            asyncio.create_task(self._update_robot_info_on_connect())
        else:
            self.logger.warning(
                f"与机器人的连接已断开: {self.settings.addr}:{self.settings.port}"
            )

        # 调用外部回调函数
        if self.external_connect_callback:
            try:
                self.external_connect_callback(connected)
            except Exception as e:
                self.logger.error(f"执行连接状态回调时出错: {str(e)}")

    async def _update_robot_info_on_connect(self) -> None:
        """连接成功后获取机器人信息的辅助方法"""
        try:
            self.logger.info("连接成功，正在获取机器人信息...")
            await self._get_robot_info()
        except Exception as e:
            self.logger.error(f"连接后获取机器人信息时出错: {str(e)}")

    # ===== 订阅管理方法 =====

    async def _subscribe_all(self) -> bool:
        """订阅所有机器人信息更新

        Returns:
            bool: 是否全部订阅成功
        """
        success = True

        # 订阅配置列表，每项为(订阅方法, 回调函数, 打印级别)
        subscriptions = [
            (self.robot.robot_state_sub, self._update_robot_state, FULL_PRINT),
            (self.robot.robot_mode_sub, self._update_mode, FULL_PRINT),
            (self.robot.robot_pose_speed_sub, self._update_pose_speed, NO_PRINT),
            (self.robot.robot_battery_sub, self._update_battery, FULL_PRINT),
            (self.robot.robot_network_sub, self._update_network, HEAD_ONLY),
            (self.robot.robot_scene_sub, self._update_scene, FULL_PRINT),
            (self.robot.robot_task_process_sub, self._update_task_proc, FULL_PRINT),
            (self.robot.robot_device_state_sub, self._update_device_state, FULL_PRINT),
            (
                self.robot.robot_hardware_state_sub,
                self._update_hardware_state,
                HEAD_ONLY,
            ),
            (
                self.robot.robot_operation_state_sub,
                self._update_operation_state,
                FULL_PRINT,
            ),
            (self.robot.robot_model_sub, self._update_model, HEAD_ONLY),
            (self.robot.robot_nav_path_sub, self._update_nav_path, HEAD_ONLY),
            (
                self.robot.robot_status_code_sub,
                self._update_robot_status_code,
                NO_PRINT,
            ),
            (
                self.robot.robot_abnormal_codes_sub,
                self._update_abnormal_codes,
                HEAD_ONLY,
            ),
            (
                self.robot.robot_count_operation_sub,
                self._update_count_operation,
                NO_PRINT,
            ),
            (self.robot.robot_count_task_sub, self._update_count_task, NO_PRINT),
            (self.robot.robot_count_status_sub, self._update_count_status, NO_PRINT),
        ]

        # 执行所有订阅
        for sub_method, callback, print_level in subscriptions:
            try:
                sub_success = await sub_method(callback, print_level)
                success &= sub_success
                if not sub_success:
                    self.logger.warning(f"订阅失败: {sub_method.__name__}")
            except Exception as e:
                self.logger.error(f"订阅出错: {str(e)}")
                success = False

        if success:
            self.logger.info("成功订阅所有机器人信息")
        else:
            self.logger.warning("部分机器人信息订阅失败")

        return success

    async def _unsubscribe_all(self) -> None:
        """取消所有订阅"""
        try:
            # 定义所有需要取消订阅的主题
            topics = [
                "woosh.robot.General",
                "woosh.robot.RobotState",
                "woosh.robot.Mode",
                "woosh.robot.PoseSpeed",
                "woosh.robot.Battery",
                "woosh.robot.Network",
                "woosh.robot.Scene",
                "woosh.robot.TaskProc",
                "woosh.robot.DeviceState",
                "woosh.robot.HardwareState",
                "woosh.robot.OperationState",
                "woosh.robot.Model",
                "woosh.robot.NavPath",
                "woosh.robot.StatusCode",
                "woosh.robot.AbnormalCodes",
                "woosh.robot.count.Operation",
                "woosh.robot.count.Task",
                "woosh.robot.count.Status",
            ]

            # 依次取消所有订阅
            for topic in topics:
                try:
                    await self.robot.unsubscribe(topic)
                except Exception as e:
                    self.logger.warning(f"取消订阅 {topic} 时出错: {str(e)}")

            self.logger.info("已取消所有订阅")
        except Exception as e:
            self.logger.error(f"取消订阅时发生错误: {str(e)}")

    # ===== 机器人信息获取方法 =====

    async def _get_robot_info(self) -> None:
        """获取机器人的完整信息并更新缓存"""
        try:
            # 获取机器人信息
            info, ok, msg = await self.robot.robot_info_req(
                RobotInfo(), HEAD_ONLY, HEAD_ONLY
            )
            if ok and info:
                # 使用CopyFrom更新整个robot_info
                self.robot_info.CopyFrom(info)

                # 更新各个子信息引用，保持与robot_info的同步
                self.general = self.robot_info.genral
                self.state = RobotState(state=info.state)
                self.mode = self.robot_info.mode
                self.pose_speed = self.robot_info.pose_speed
                self.battery = self.robot_info.battery
                self.network = self.robot_info.network
                self.scene = self.robot_info.scene
                self.task_proc = self.robot_info.task_proc
                self.device_state = self.robot_info.device_state
                self.hardware_state = self.robot_info.hardware_state
                self.operation_state = self.robot_info.operation_state
                self.model = self.robot_info.model

                # 从robot_info更新任务历史，保持单向更新
                await self._update_task_history_from_info()

                # 更新状态码和计数信息
                self.abnormal_codes = self.robot_info.abnormal_codes
                self.count_operation = self.robot_info.count_operation
                self.count_task = self.robot_info.count_task
                self.count_status = self.robot_info.count_error

                self.logger.info(f"成功获取机器人信息，机器人ID: {info.robot_id}")
            else:
                self.logger.error(f"获取机器人信息失败: {msg}")
        except Exception as e:
            self.logger.error(f"获取机器人信息时发生错误: {str(e)}")

    async def _update_task_history_from_info(self) -> None:
        """从robot_info更新任务历史记录"""
        if self.robot_info.task_history and self.robot_info.task_history.tes:
            # 复制任务历史记录，并保持最多50条
            sorted_tasks = sorted(
                self.robot_info.task_history.tes,
                key=lambda t: t.time if hasattr(t, "time") else 0,
                reverse=True,
            )

            # 将任务添加到映射和ID列表
            async with self._task_lock:
                for task in sorted_tasks[:50]:
                    # 创建任务副本
                    new_task = TaskProc()
                    new_task.CopyFrom(task)

                    # 添加到映射（存在则更新，不存在则添加）
                    self.task_map[task.robot_task_id] = new_task

                    # 记录ID（按时间从新到旧），存在则不添加
                    if task.robot_task_id not in self.task_ids:
                        self.task_ids.append(task.robot_task_id)

            self.logger.debug(f"从robot_info更新任务映射: {len(self.task_map)}条")

    # ===== 状态更新回调方法 =====

    # 基础状态更新方法
    def _update_robot_state(self, data: RobotState) -> None:
        """更新机器人状态"""
        self.state = data
        self.robot_info.state = data.state

    def _update_mode(self, data: Mode) -> None:
        """更新机器人模式"""
        self.mode = data
        self.robot_info.mode.CopyFrom(data)

    def _update_pose_speed(self, data: PoseSpeed) -> None:
        """更新位置和速度信息"""
        self.pose_speed = data
        self.robot_info.pose_speed.CopyFrom(data)

    def _update_battery(self, data: Battery) -> None:
        """更新电池信息"""
        self.battery = data
        self.robot_info.battery.CopyFrom(data)

    def _update_network(self, data: Network) -> None:
        """更新网络信息"""
        self.network = data
        self.robot_info.network.CopyFrom(data)

    def _update_scene(self, data: Scene) -> None:
        """更新场景信息"""
        self.scene = data
        self.robot_info.scene.CopyFrom(data)

    def _update_model(self, data: Model) -> None:
        """更新模型信息"""
        self.model = data
        self.robot_info.model.CopyFrom(data)

    def _update_nav_path(self, data: NavPath) -> None:
        """更新导航路径"""
        self.nav_path = data

    # 设备和硬件状态更新方法
    def _update_device_state(self, data: DeviceState) -> None:
        """更新设备状态"""
        self.device_state = data
        self.robot_info.device_state.CopyFrom(data)

    def _update_hardware_state(self, data: HardwareState) -> None:
        """更新硬件状态"""
        self.hardware_state = data
        self.robot_info.hardware_state.CopyFrom(data)

    def _update_operation_state(self, data: OperationState) -> None:
        """更新操作状态"""
        self.operation_state = data
        self.robot_info.operation_state.CopyFrom(data)

    # 任务相关更新方法
    async def _update_task_proc(self, data: TaskProc) -> None:
        """更新任务进度"""
        self.task_proc = data
        self.robot_info.task_proc.CopyFrom(data)

        if data.state == kTaskCompleted:
            self.logger.info(f"任务完成: {data.robot_task_id}")
        elif data.state == kTaskFailed:
            self.logger.warning(f"任务失败: {data.robot_task_id}")
        elif data.state == kTaskCanceled:
            self.logger.warning(f"任务取消: {data.robot_task_id}")

        # 同时更新任务历史记录
        await self._update_task_history(data)

    async def _update_task_history(self, task_proc: TaskProc) -> None:
        """线程安全地更新任务历史记录"""
        async with self._task_lock:
            new_task = TaskProc()
            new_task.CopyFrom(task_proc)
            task_id = task_proc.robot_task_id

            # 如果任务ID已存在，先移除再添加，确保最新的在列表末尾
            if task_id in self.task_map:
                self.task_ids.remove(task_id)
            self.task_ids.append(task_id)
            self.task_map[task_id] = new_task

            self.logger.debug(
                f"更新任务历史: 任务ID {task_id}, 类型 {task_proc.type}, 当前任务数 {len(self.task_map)}"
            )

    # 状态码和计数信息更新方法
    async def _update_robot_status_code(self, status_code: StatusCode) -> None:
        """更新机器人状态码"""
        try:
            async with self._status_lock:
                new_status_code = StatusCode()
                new_status_code.CopyFrom(status_code)
                # 将新状态码插入到列表开头
                self.status_codes.scs.insert(0, new_status_code)
                # 保持最多50条记录
                while len(self.status_codes.scs) > 50:
                    self.status_codes.scs.pop()
                self.logger.debug(f"更新状态码:  {status_code.code}")
        except Exception as e:
            self.logger.error(f"更新状态码时发生错误: {str(e)}")

    def _update_abnormal_codes(self, data: AbnormalCodes) -> None:
        """更新异常代码"""
        self.abnormal_codes = data
        self.robot_info.abnormal_codes.CopyFrom(data)

    def _update_count_operation(self, data: CountOperation) -> None:
        """更新操作计数"""
        self.count_operation = data
        self.robot_info.count_operation.CopyFrom(data)

    def _update_count_task(self, data: CountTask) -> None:
        """更新任务计数"""
        self.count_task = data
        self.robot_info.count_task.CopyFrom(data)

    def _update_count_status(self, data: CountStatus) -> None:
        """更新状态计数"""
        self.count_status = data
        self.robot_info.count_error.CopyFrom(data)


# ===== 示例用法 =====
async def main():
    """示例主函数：创建并运行机器人代理"""

    # 定义连接状态回调函数
    def on_connection_changed(connected: bool):
        print(f"连接状态变化: {'已连接' if connected else '已断开'}")

    agent = None
    try:
        # 使用异步工厂方法创建实例
        agent = await WooshRobotAgent.create(
            robot_ip="172.20.254.63",  # 替换为实际机器人IP
            robot_port=10003,
            identity="woosh-agent-demo",
            connect_status_callback=on_connection_changed,
        )

        agent.logger.info("机器人代理运行中，按Ctrl+C退出...")
        while True:
            try:
                # 检查连接状态
                if not agent.is_connected:
                    agent.logger.warning("机器人连接已断开，退出程序")
                    break

                # 打印电池电量
                if agent.battery:
                    agent.logger.info(
                        f"电池电量: {agent.battery.power}%, 充电状态: {agent.battery.charge_state}"
                    )
                # 每5秒打印一次
                await asyncio.sleep(5)
            except asyncio.CancelledError:
                # 正确处理取消操作
                break
            except Exception as e:
                agent.logger.error(f"运行时发生错误: {str(e)}")
                break

    except KeyboardInterrupt:
        agent.logger.info("收到退出信号，正在关闭程序...")
    except Exception as e:
        if agent:
            agent.logger.error(f"程序发生错误: {str(e)}")
    finally:
        # 确保在任何情况下都清理资源
        if agent:
            try:
                await agent.close()
            except Exception as e:
                if agent.logger:
                    agent.logger.error(f"关闭连接时发生错误: {str(e)}")
            agent.logger.info("程序已终止")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # 确保程序可以正常退出
        pass
