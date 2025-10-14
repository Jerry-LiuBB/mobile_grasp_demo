from typing import Any, Optional, Type, Callable, Tuple
import json
import inspect

import google.protobuf.message as pbmsg

from woosh.message_serializer import MessageSerializer
from woosh.ws_commu import AsyncWebSocket
from woosh.message_pack import NotifyPack

from woosh.proto.robot.robot_pb2 import (
    RobotInfo as PbRobotInfo,
    General,
    RobotState,
    Setting,
    Mode,
    PoseSpeed,
    Battery,
    Network,
    TaskProc,
    DeviceState,
    Scene,
    HardwareState,
    OperationState,
    Model,
    NavPath,
    TaskHistory,
    ScannerData,
)
from woosh.proto.robot.robot_setting_pb2 import (
    Identity as SetIdentity,
    Server as SetServer,
    AutoCharge as SetAutoCharge,
    AutoPark as SetAutoPark,
    GoodsCheck as SetGoodsCheck,
    Power as SetPower,
    Sound as SetSound,
)
from woosh.proto.robot.robot_pack_pb2 import (
    Deployment,
    DeploymentResponse,
    PlanNavPath,
)
from woosh.proto.robot.robot_count_pb2 import (
    StatusCode,
    StatusCodes,
    AbnormalCodes,
)

from woosh.proto.map.map_pack_pb2 import (
    SceneList,
    SceneData,
    SceneDataEasy,
    Download as DownloadMap,
    DownloadResponse as DownloadMapResponse,
    Upload as UploadMap,
    Rename as RenameMap,
    Delete as DeleteMap,
)
from woosh.proto.map.mark_pb2 import Storage
from woosh.proto.map.storage_edit_pb2 import (
    Create as StorageCreate,
    Delete as StorageDelete,
    Update as StorageUpdate,
    Find as StorageFind,
)

from woosh.proto.device.device_pb2 import RfRemoteController

from woosh_interface import (
    ICommon,
    IRobotInfo,
    IRobotSetting,
    IMapInfo,
    IMapEdit,
    IDeviceInfo,
    IRobotProxy,
    CommuSettings,
    PrintPackLevel,
    NO_PRINT,
)


class RobotCommunication:
    """机器人通信类，负责处理与机器人的WebSocket通信"""

    def __init__(self, settings: CommuSettings) -> None:
        """初始化通信类

        Args:
            settings: 通信设置，包含连接参数和回调函数
        """
        self.settings = settings
        self.logger = settings.logger
        self._connected = False
        self._sub_print_levels = {}  # 订阅消息打印级别

        # 创建WebSocket连接
        self._ws = AsyncWebSocket(
            addr=settings.addr,
            port=settings.port,
            poll_timeout=settings.poll_timeout,
            logger=settings.logger,
            loop=settings.loop,
            connection_callback=self._on_connection_change,
        )

        self.logger.info("RobotCommunication initialized")

    def is_connected(self) -> bool:
        """检查是否已连接

        Returns:
            bool: 是否已连接
        """
        return self._ws.is_connected()

    async def start(self) -> bool:
        """启动通信服务

        Returns:
            bool: 启动是否成功
        """
        if self._ws.is_connected():
            self.logger.info("Communication already started")
            return True

        self.logger.info("Starting communication...")
        success = await self.connect()
        if success:
            self.logger.info("Communication started successfully")
        else:
            self.logger.error("Failed to start communication")
        return success

    async def stop(self) -> None:
        """停止通信服务"""
        if not self._ws.is_connected():
            self.logger.info("Communication already stopped")
            return

        self.logger.info("Stopping communication...")
        await self.disconnect()
        self.logger.info("Communication stopped successfully")

    async def connect(self) -> bool:
        """建立WebSocket连接

        Args:

        Returns:
            bool: 连接是否成功
        """
        try:
            self.logger.info("Connecting to server...")

            # 使用AsyncWebSocket的connect方法，等待连接建立或超时
            success = await self._ws.connect()

            if success:
                self.logger.info("Connected to server successfully")
            else:
                self.logger.error("Connection failed: timeout")

            return success
        except Exception as e:
            self.logger.error("Connection failed", exc_info=e)
            return False

    async def disconnect(self) -> None:
        """断开WebSocket连接"""
        try:
            self.logger.info("Disconnecting from server...")
            await self._ws.disconnect()
            self.logger.info("Disconnected from server successfully")
        except Exception as e:
            self.logger.error("Disconnect failed", exc_info=e)
        finally:
            self._connected = False

    async def ensure_connected(self) -> bool:
        """确保已连接，如果未连接则尝试重连

        Returns:
            bool: 是否已连接
        """
        if self._ws.is_connected():
            return True

        self.logger.info("Not connected, attempting to reconnect...")
        return await self._ws.ensure_connected()

    async def request(
        self,
        msg: pbmsg.Message,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[pbmsg.Message], bool, str]:
        """发送请求消息

        Args:
            msg: 请求消息
            req_ppl: 请求打印级别，默认为NO_PRINT
            rep_ppl: 响应打印级别，默认为NO_PRINT

        Returns:
            Tuple[Optional[pbmsg.Message], bool, str]: (响应消息, 是否成功, 错误信息)
        """
        # 确保已连接
        if not await self.ensure_connected():
            return None, False, "Not connected"

        try:
            topic = msg.DESCRIPTOR.full_name
            self._print_message("REQ", topic, msg, req_ppl)

            self.logger.info(f"Sending request: {topic}")
            response = await self._ws.send(topic, msg)

            resp_msg = None
            if response.ok:
                resp_msg = MessageSerializer.create_message(
                    response.type, response.body
                )

            self._print_message("RSP", topic, resp_msg, rep_ppl)

            if response.ok:
                self.logger.info(f"Request completed successfully: {topic}")
            else:
                self.logger.error(f"Request failed: {response.msg}")

            return resp_msg, response.ok, response.msg

        except Exception as e:
            self.logger.error("Request failed", exc_info=e)
            return None, False, str(e)

    async def subscribe(
        self,
        msg_type: Type[pbmsg.Message],
        callback: Callable[[pbmsg.Message], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅消息主题

        Args:
            msg_type: 消息类型
            callback: 回调函数，可以是同步函数或异步函数
            sub_ppl: 打印级别，默认为NO_PRINT

        Returns:
            bool: 订阅是否成功
        """

        try:
            topic = msg_type.DESCRIPTOR.full_name
            self.logger.info(f"Subscribing to topic: {topic}")

            self._sub_print_levels[topic] = sub_ppl

            async def on_message(notify: NotifyPack) -> None:
                try:
                    msg = MessageSerializer.deserialize(notify.body, msg_type)
                    self._print_message(
                        "SUB", topic, msg, self._sub_print_levels.get(topic, NO_PRINT)
                    )

                    # 检查回调是否为异步函数
                    if inspect.iscoroutinefunction(callback):
                        # 如果是异步函数，使用await调用
                        await callback(msg)
                    else:
                        # 如果是同步函数，直接调用
                        callback(msg)

                except Exception as e:
                    self.logger.error(
                        f"Handle subscription message failed for {topic}",
                        exc_info=e,
                    )

            await self._ws.subscribe(topic, on_message)
            self.logger.info(f"Subscribed to topic successfully: {topic}")
            return True
        except Exception as e:
            self.logger.error("Subscribe failed", exc_info=e)
            return False

    async def unsubscribe(self, topic: str) -> bool:
        """取消订阅消息主题

        Args:
            topic: 要取消订阅的主题

        Returns:
            bool: 取消订阅是否成功
        """

        try:
            self.logger.info(f"Unsubscribing from topic: {topic}")
            self._sub_print_levels.pop(topic, None)

            await self._ws.unsubscribe(topic)
            self.logger.info(f"Unsubscribed from topic successfully: {topic}")
            return True
        except Exception as e:
            self.logger.error("Unsubscribe failed", exc_info=e)
            return False

    def _print_message(
        self, msg_type: str, topic: str, msg: Any, level: PrintPackLevel
    ) -> None:
        """打印消息内容

        Args:
            msg_type: 消息类型
            topic: 消息主题
            msg: 消息内容
            level: 打印级别
        """
        if not self.logger or level == NO_PRINT:
            return

        try:
            content = f"[{msg_type}] {topic}"
            if level == PrintPackLevel.BODY and msg is not None:
                if isinstance(msg, pbmsg.Message):
                    content += f"\n{str(msg)}"
                else:
                    content += f"\n{json.dumps(msg, indent=2)}"

            self.logger.info(content)
        except Exception as e:
            self.logger.error(f"Print message failed: {str(e)}")

    def _on_connection_change(
        self, connected: bool, error: Optional[Exception]
    ) -> None:
        """WebSocket连接状态变化回调

        Args:
            connected: 是否已连接
            error: 如果发生错误，提供错误信息
        """
        self._check_connect_status(connected)

        if error:
            self.logger.error(f"Connection error: {str(error)}")

    def _check_connect_status(self, connected: bool) -> None:
        """检查并更新连接状态

        Args:
            connected: 当前连接状态
        """
        if self._connected == connected:
            return

        self._connected = connected
        status_msg = "Connected" if connected else "Disconnected"
        self.logger.info(f"Connection status changed: {status_msg}")

        try:
            if self.settings.connect_status_callback:
                self.settings.connect_status_callback(connected)
        except Exception as e:
            self.logger.error("Connection status callback failed", exc_info=e)


class Common(ICommon):
    """通用接口实现，提供基础的SDK运行控制和订阅管理功能"""

    def __init__(self, comm: RobotCommunication) -> None:
        """初始化通用接口

        Args:
            comm: 机器人通信实例
        """
        self.comm = comm
        self.logger = comm.logger
        self._started = False

    async def run(self) -> bool:
        try:
            if self._started:
                return True

            success = await self.comm.start()
            self._started = True
            return success
        except Exception as e:
            self.logger.error(f"启动失败: {str(e)}")
            return False

    async def stop(self) -> None:
        try:
            if self._started:
                await self.comm.stop()
                self._started = False
        except Exception as e:
            self.logger.error(f"停止失败: {str(e)}")

    async def unsubscribe(self, topic: str) -> bool:
        try:
            return await self.comm.unsubscribe(topic)
        except Exception as e:
            self.logger.error(f"取消订阅失败: {str(e)}")
            return False


class RobotInfo(IRobotInfo):
    """机器人信息接口实现"""

    def __init__(self, comm: RobotCommunication) -> None:
        self.comm = comm
        self.logger = comm.logger

    async def robot_info_req(
        self,
        robot_info: PbRobotInfo,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[PbRobotInfo], bool, str]:
        try:
            return await self.comm.request(robot_info, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"机器人信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_general_req(
        self,
        general: General,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[General], bool, str]:
        try:
            return await self.comm.request(general, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"常规信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_setting_req(
        self,
        setting: Setting,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Setting], bool, str]:
        try:
            return await self.comm.request(setting, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"配置信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_setting_sub(
        self, callback: Callable[[Setting], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Setting, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"配置信息订阅失败: {str(e)}")
            return False

    async def robot_state_req(
        self,
        robot_state: RobotState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RobotState], bool, str]:
        try:
            return await self.comm.request(robot_state, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"机器人状态请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_state_sub(
        self, callback: Callable[[RobotState], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(RobotState, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"机器人状态订阅失败: {str(e)}")
            return False

    async def robot_mode_req(
        self,
        mode: Mode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Mode], bool, str]:
        try:
            return await self.comm.request(mode, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"机器人模式请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_mode_sub(
        self, callback: Callable[[Mode], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Mode, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"机器人模式订阅失败: {str(e)}")
            return False

    async def robot_pose_speed_req(
        self,
        pose_speed: PoseSpeed,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[PoseSpeed], bool, str]:
        try:
            return await self.comm.request(pose_speed, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"位姿速度请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_pose_speed_sub(
        self, callback: Callable[[PoseSpeed], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(PoseSpeed, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"位姿速度订阅失败: {str(e)}")
            return False

    async def robot_battery_req(
        self,
        battery: Battery,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Battery], bool, str]:
        try:
            return await self.comm.request(battery, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"电量信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_battery_sub(
        self, callback: Callable[[Battery], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Battery, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"电量信息订阅失败: {str(e)}")
            return False

    async def robot_network_req(
        self,
        network: Network,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Network], bool, str]:
        try:
            return await self.comm.request(network, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"网络信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_network_sub(
        self, callback: Callable[[Network], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Network, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"网络信息订阅失败: {str(e)}")
            return False

    async def robot_scene_req(
        self,
        scene: Scene,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Scene], bool, str]:
        try:
            return await self.comm.request(scene, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"场景信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_scene_sub(
        self, callback: Callable[[Scene], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Scene, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"场景信息订阅失败: {str(e)}")
            return False

    async def robot_task_process_req(
        self,
        task_process: TaskProc,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[TaskProc], bool, str]:
        try:
            return await self.comm.request(task_process, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"任务进度信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_task_process_sub(
        self, callback: Callable[[TaskProc], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(TaskProc, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"任务进度信息订阅失败: {str(e)}")
            return False

    async def robot_device_state_req(
        self,
        device_state: DeviceState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DeviceState], bool, str]:
        try:
            return await self.comm.request(device_state, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设备状态信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_device_state_sub(
        self,
        callback: Callable[[DeviceState], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(DeviceState, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"设备状态信息订阅失败: {str(e)}")
            return False

    async def robot_hardware_state_req(
        self,
        hardware_state: HardwareState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[HardwareState], bool, str]:
        try:
            return await self.comm.request(hardware_state, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"硬件状态信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_hardware_state_sub(
        self,
        callback: Callable[[HardwareState], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(HardwareState, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"硬件状态信息订阅失败: {str(e)}")
            return False

    async def robot_operation_state_req(
        self,
        operation_state: OperationState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[OperationState], bool, str]:
        try:
            return await self.comm.request(operation_state, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"运行状态信息请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_operation_state_sub(
        self,
        callback: Callable[[OperationState], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(OperationState, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"运行状态信息订阅失败: {str(e)}")
            return False

    async def robot_model_req(
        self,
        model: Model,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Model], bool, str]:
        try:
            return await self.comm.request(model, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"机器人模型请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_model_sub(
        self, callback: Callable[[Model], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Model, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"机器人模型订阅失败: {str(e)}")
            return False

    async def robot_nav_path_req(
        self,
        nav_path: NavPath,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[NavPath], bool, str]:
        try:
            return await self.comm.request(nav_path, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"机器人导航路径请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_nav_path_sub(
        self, callback: Callable[[NavPath], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(NavPath, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"机器人导航路径订阅失败: {str(e)}")
            return False

    async def robot_task_history_req(
        self,
        task_history: TaskHistory,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[TaskHistory], bool, str]:
        try:
            return await self.comm.request(task_history, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"历史任务请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_status_codes_req(
        self,
        status_codes: StatusCodes,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[StatusCodes], bool, str]:
        try:
            return await self.comm.request(status_codes, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"状态码请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_status_code_sub(
        self, callback: Callable[[StatusCode], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(StatusCode, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"状态码订阅失败: {str(e)}")
            return False

    async def robot_abnormal_codes_req(
        self,
        abnormal_codes: AbnormalCodes,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[AbnormalCodes], bool, str]:
        try:
            return await self.comm.request(abnormal_codes, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"未恢复的异常码请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_abnormal_codes_sub(
        self,
        callback: Callable[[AbnormalCodes], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(AbnormalCodes, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"未恢复的异常码订阅失败: {str(e)}")
            return False


class RobotSetting(IRobotSetting):
    """机器人设置接口实现"""

    def __init__(self, comm: RobotCommunication) -> None:
        self.comm = comm
        self.logger = comm.logger

    async def set_identity(
        self,
        identity: SetIdentity,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetIdentity], bool, str]:
        try:
            return await self.comm.request(identity, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设置标识失败: {str(e)}")
            return None, False, str(e)

    async def set_server(
        self,
        server: SetServer,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetServer], bool, str]:
        try:
            return await self.comm.request(server, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设置服务器失败: {str(e)}")
            return None, False, str(e)

    async def auto_charge(
        self,
        charge: SetAutoCharge,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetAutoCharge], bool, str]:
        try:
            return await self.comm.request(charge, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"自动充电设置失败: {str(e)}")
            return None, False, str(e)

    async def auto_park(
        self,
        park: SetAutoPark,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetAutoPark], bool, str]:
        try:
            return await self.comm.request(park, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"自动泊车设置失败: {str(e)}")
            return None, False, str(e)

    async def goods_check(
        self,
        check: SetGoodsCheck,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetGoodsCheck], bool, str]:
        try:
            return await self.comm.request(check, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"货物检测设置失败: {str(e)}")
            return None, False, str(e)

    async def config_power(
        self,
        power: SetPower,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetPower], bool, str]:
        try:
            return await self.comm.request(power, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"电量配置失败: {str(e)}")
            return None, False, str(e)

    async def set_sound(
        self,
        sound: SetSound,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetSound], bool, str]:
        try:
            return await self.comm.request(sound, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"声音设置失败: {str(e)}")
            return None, False, str(e)


class MapInfo(IMapInfo):
    """地图信息接口实现"""

    def __init__(self, comm: RobotCommunication) -> None:
        self.comm = comm
        self.logger = comm.logger

    async def scene_list_req(
        self,
        scene_list: SceneList,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SceneList], bool, str]:
        try:
            return await self.comm.request(scene_list, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"场景列表请求失败: {str(e)}")
            return None, False, str(e)

    async def scene_data_req(
        self,
        scene_data: SceneData,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SceneData], bool, str]:
        try:
            return await self.comm.request(scene_data, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"场景数据请求失败: {str(e)}")
            return None, False, str(e)

    async def scene_data_easy_req(
        self,
        scene_data: SceneDataEasy,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SceneDataEasy], bool, str]:
        try:
            return await self.comm.request(scene_data, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"简易版场景数据请求失败: {str(e)}")
            return None, False, str(e)

    async def download_map(
        self,
        map_data: DownloadMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DownloadMapResponse], bool, str]:
        try:
            return await self.comm.request(map_data, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"地图下载失败: {str(e)}")
            return None, False, str(e)

    async def upload_map(
        self,
        map_data: UploadMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[UploadMap], bool, str]:
        try:
            return await self.comm.request(map_data, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"地图上传失败: {str(e)}")
            return None, False, str(e)

    async def rename_map(
        self,
        rename: RenameMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RenameMap], bool, str]:
        try:
            return await self.comm.request(rename, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"地图或场景重命名失败: {str(e)}")
            return None, False, str(e)

    async def delete_map(
        self,
        delete: DeleteMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DeleteMap], bool, str]:
        try:
            return await self.comm.request(delete, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"地图或场景删除失败: {str(e)}")
            return None, False, str(e)


class MapEdit(IMapEdit):
    """地图编辑接口实现"""

    def __init__(self, comm: RobotCommunication) -> None:
        self.comm = comm
        self.logger = comm.logger

    async def create_storage(
        self,
        storage: StorageCreate,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Storage], bool, str]:
        try:
            return await self.comm.request(storage, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"创建储位失败: {str(e)}")
            return None, False, str(e)

    async def delete_storage(
        self,
        storage: StorageDelete,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[StorageDelete], bool, str]:
        try:
            return await self.comm.request(storage, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"删除储位失败: {str(e)}")
            return None, False, str(e)

    async def update_storage(
        self,
        storage: StorageUpdate,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Storage], bool, str]:
        try:
            return await self.comm.request(storage, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"更新储位失败: {str(e)}")
            return None, False, str(e)

    async def find_storage(
        self,
        storage: StorageFind,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Storage], bool, str]:
        try:
            return await self.comm.request(storage, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"查找储位失败: {str(e)}")
            return None, False, str(e)


class DeviceInfo(IDeviceInfo):
    """设备信息接口实现"""

    def __init__(self, comm: RobotCommunication) -> None:
        self.comm = comm
        self.logger = comm.logger

    async def rf_remote_controller_sub(
        self,
        callback: Callable[[RfRemoteController], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(RfRemoteController, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"射频遥控器信号订阅失败: {str(e)}")
            return False


class RobotProxy(IRobotProxy):
    """机器人代理接口实现"""

    def __init__(self, comm: RobotCommunication) -> None:
        self.comm = comm
        self.logger = comm.logger

    async def deployment_req(
        self,
        deployment: Deployment,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DeploymentResponse], bool, str]:
        try:
            return await self.comm.request(deployment, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"部署请求失败: {str(e)}")
            return None, False, str(e)

    async def plan_nav_path_req(
        self,
        plan: PlanNavPath,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[PlanNavPath], bool, str]:
        try:
            return await self.comm.request(plan, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"规划导航路径请求失败: {str(e)}")
            return None, False, str(e)

    async def scanner_data_req(
        self,
        scanner: ScannerData,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ScannerData], bool, str]:
        try:
            return await self.comm.request(scanner, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"雷达数据请求失败: {str(e)}")
            return None, False, str(e)
