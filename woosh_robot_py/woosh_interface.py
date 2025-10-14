from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional, Tuple
import enum
import logging
import asyncio

from woosh.proto.device.device_pb2 import RfRemoteController

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

from woosh.proto.robot.robot_pb2 import (
    RobotInfo as PbRobotInfo,
    General,
    Setting,
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
    TaskHistory,
    ScannerData,
)
from woosh.proto.robot.robot_pack_pb2 import (
    InitRobot,
    SetRobotPose,
    SetOccupancy,
    SetMuteCall,
    SetProgramMute,
    SwitchControlMode,
    SwitchWorkMode,
    SwitchFootPrint,
    SwitchMap,
    BuildMap,
    BuildMapData,
    Deployment,
    DeploymentResponse,
    ExecTask,
    ActionOrder,
    PlanNavPath,
    ChangeNavPath,
    ChangeNavMode,
    Speak,
    LED,
    Twist,
    Follow,
    RobotWiFi,
    CountData,
    CountDataResponse,
    ExecPreTask,
)
from woosh.proto.robot.robot_count_pb2 import (
    StatusCode,
    StatusCodes,
    AbnormalCodes,
    Operation as CountOperation,
    Task as CountTask,
    Status as CountStatus,
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

from woosh.proto.ros.ros_pack_pb2 import CallAction, Feedbacks

from woosh.proto.task.woosh_task_pb2 import RepeatTask

from woosh.logger import create_logger


class PrintPackLevel(enum.Enum):
    """打印包级别"""

    NONE = 0  # 不打印
    HEAD = 1  # 打印包头
    BODY = 3  # 打印包头和包体


NO_PRINT = PrintPackLevel.NONE  # 不打印
HEAD_ONLY = PrintPackLevel.HEAD  # 只打印包头
FULL_PRINT = PrintPackLevel.BODY  # 打印完整内容


@dataclass
class CommuSettings:
    """通信连接设置"""

    addr: str  # WebSocket服务器地址
    port: int  # WebSocket服务器端口
    identity: str = "WOOSDK"  # SDK标识
    poll_timeout: int = 10000  # 轮询超时时间（毫秒）
    print_pack_max_byte_size: int = 2048  # 打印包最大字节数
    connect_status_callback: Optional[Callable[[bool], None]] = None  # 连接状态回调函数
    loop: Optional[asyncio.AbstractEventLoop] = None  # 事件循环

    # 日志配置
    log_level: str = "INFO"  # 日志级别
    log_dir: Optional[str] = None  # 日志文件目录
    log_to_console: bool = True  # 是否输出到控制台
    log_to_file: bool = False  # 是否输出到文件
    logger: Optional[logging.Logger] = None  # 日志器实例

    def __post_init__(self):
        """初始化后处理

        如果没有提供logger实例，则根据配置创建一个新的logger
        如果没有提供loop实例，则获取当前事件循环或创建一个新的
        """
        # 初始化日志器
        if self.logger is None:
            self.logger = create_logger(
                name=self.identity,
                level=self.log_level,
                log_dir=self.log_dir,
                console=self.log_to_console,
                file=self.log_to_file,
            )

        # 初始化事件循环
        if self.loop is None:
            try:
                self.loop = asyncio.get_event_loop()
            except RuntimeError:
                # 如果没有运行中的事件循环，创建一个新的
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)


class ICommon(ABC):
    """通用接口"""

    @abstractmethod
    async def run(self, loop: Optional[asyncio.AbstractEventLoop] = None) -> bool:
        """运行SDK

        Args:
            loop: 可选的事件循环，如果提供则使用该循环创建任务

        Returns:
            bool: 启动是否成功
        """
        pass

    @abstractmethod
    async def unsubscribe(self, topic: str) -> bool:
        """取消订阅"""
        pass


class IRobotInfo(ABC):
    """机器人信息相关接口"""

    @abstractmethod
    async def robot_info_req(
        self,
        robot_info: PbRobotInfo,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[PbRobotInfo], bool, str]:
        """请求机器人基本信息

        Args:
            robot_info: 机器人信息请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[PbRobotInfo], bool, str]: (机器人信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_general_req(
        self,
        general: General,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[General], bool, str]:
        """请求机器人常规信息

        Args:
            general: 常规信息请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[General], bool, str]: (常规信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_setting_req(
        self,
        setting: Setting,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Setting], bool, str]:
        """请求机器人配置信息

        Args:
            setting: 配置信息请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Setting], bool, str]: (配置信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_setting_sub(
        self, callback: Callable[[Setting], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人配置信息变更

        Args:
            callback: 配置信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_state_req(
        self,
        robot_state: RobotState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RobotState], bool, str]:
        """请求机器人状态信息

        Args:
            robot_state: 机器人状态请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[RobotState], bool, str]: (状态信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_state_sub(
        self, callback: Callable[[RobotState], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人状态变更

        Args:
            callback: 状态变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_mode_req(
        self,
        mode: Mode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Mode], bool, str]:
        """请求机器人模式信息

        Args:
            mode: 机器人模式请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Mode], bool, str]: (模式信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_mode_sub(
        self, callback: Callable[[Mode], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人模式变更

        Args:
            callback: 模式变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_pose_speed_req(
        self,
        pose_speed: PoseSpeed,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[PoseSpeed], bool, str]:
        """请求机器人位姿和速度信息

        Args:
            pose_speed: 位姿速度请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[PoseSpeed], bool, str]: (位姿速度信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_pose_speed_sub(
        self, callback: Callable[[PoseSpeed], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人位姿和速度变更

        Args:
            callback: 位姿速度变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_battery_req(
        self,
        battery: Battery,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Battery], bool, str]:
        """请求机器人电池信息

        Args:
            battery: 电池信息请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Battery], bool, str]: (电池信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_battery_sub(
        self, callback: Callable[[Battery], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人电池信息变更

        Args:
            callback: 电池信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_network_req(
        self,
        network: Network,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Network], bool, str]:
        """请求机器人网络信息

        Args:
            network: 网络信息请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Network], bool, str]: (网络信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_network_sub(
        self, callback: Callable[[Network], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人网络信息变更

        Args:
            callback: 网络信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_scene_req(
        self,
        scene: Scene,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Scene], bool, str]:
        """请求机器人场景信息

        Args:
            scene: 场景信息请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Scene], bool, str]: (场景信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_scene_sub(
        self, callback: Callable[[Scene], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人场景信息变更

        Args:
            callback: 场景信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_task_process_req(
        self,
        task_process: TaskProc,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[TaskProc], bool, str]:
        """请求机器人任务进度信息

        Args:
            task_process: 任务进度请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[TaskProc], bool, str]: (任务进度信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_task_process_sub(
        self, callback: Callable[[TaskProc], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人任务进度信息变更

        Args:
            callback: 任务进度信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_device_state_req(
        self,
        device_state: DeviceState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DeviceState], bool, str]:
        """请求机器人设备状态信息

        Args:
            device_state: 设备状态请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[DeviceState], bool, str]: (设备状态信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_device_state_sub(
        self,
        callback: Callable[[DeviceState], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅机器人设备状态信息变更

        Args:
            callback: 设备状态信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_hardware_state_req(
        self,
        hardware_state: HardwareState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[HardwareState], bool, str]:
        """请求机器人硬件状态信息

        Args:
            hardware_state: 硬件状态请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[HardwareState], bool, str]: (硬件状态信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_hardware_state_sub(
        self,
        callback: Callable[[HardwareState], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅机器人硬件状态信息变更

        Args:
            callback: 硬件状态信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_operation_state_req(
        self,
        operation_state: OperationState,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[OperationState], bool, str]:
        """请求机器人运行状态信息

        Args:
            operation_state: 运行状态请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[OperationState], bool, str]: (运行状态信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_operation_state_sub(
        self,
        callback: Callable[[OperationState], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅机器人运行状态信息变更

        Args:
            callback: 运行状态信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_model_req(
        self,
        model: Model,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Model], bool, str]:
        """请求机器人模型信息

        Args:
            model: 机器人模型请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Model], bool, str]: (模型信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_model_sub(
        self, callback: Callable[[Model], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人模型信息变更

        Args:
            callback: 模型信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_nav_path_req(
        self,
        nav_path: NavPath,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[NavPath], bool, str]:
        """请求机器人导航路径信息

        Args:
            nav_path: 导航路径请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[NavPath], bool, str]: (导航路径信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_nav_path_sub(
        self, callback: Callable[[NavPath], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人导航路径信息变更

        Args:
            callback: 导航路径信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_task_history_req(
        self,
        task_history: TaskHistory,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[TaskHistory], bool, str]:
        """请求机器人历史任务信息

        Args:
            task_history: 历史任务请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[TaskHistory], bool, str]: (历史任务信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_status_codes_req(
        self,
        status_codes: StatusCodes,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[StatusCodes], bool, str]:
        """请求机器人状态码信息

        Args:
            status_codes: 状态码请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[StatusCodes], bool, str]: (状态码信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_status_code_sub(
        self, callback: Callable[[StatusCode], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅机器人状态码信息变更

        Args:
            callback: 状态码信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_abnormal_codes_req(
        self,
        abnormal_codes: AbnormalCodes,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[AbnormalCodes], bool, str]:
        """请求机器人异常码信息

        Args:
            abnormal_codes: 异常码请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[AbnormalCodes], bool, str]: (异常码信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_abnormal_codes_sub(
        self,
        callback: Callable[[AbnormalCodes], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅机器人异常码信息变更

        Args:
            callback: 异常码信息变更回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass


class IRobotSetting(ABC):
    """机器人设置接口"""

    @abstractmethod
    async def set_identity(
        self,
        identity: SetIdentity,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetIdentity], bool, str]:
        """设置机器人标识信息

        Args:
            identity: 标识信息设置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetIdentity], bool, str]: (标识信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def set_server(
        self,
        server: SetServer,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetServer], bool, str]:
        """设置服务器配置信息

        Args:
            server: 服务器配置设置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetServer], bool, str]: (服务器配置信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def auto_charge(
        self,
        charge: SetAutoCharge,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetAutoCharge], bool, str]:
        """配置自动充电参数

        Args:
            charge: 自动充电配置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetAutoCharge], bool, str]: (自动充电配置信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def auto_park(
        self,
        park: SetAutoPark,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetAutoPark], bool, str]:
        """配置自动泊车参数

        Args:
            park: 自动泊车配置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetAutoPark], bool, str]: (自动泊车配置信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def goods_check(
        self,
        check: SetGoodsCheck,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetGoodsCheck], bool, str]:
        """配置货物检测参数

        Args:
            check: 货物检测配置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetGoodsCheck], bool, str]: (货物检测配置信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def config_power(
        self,
        power: SetPower,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetPower], bool, str]:
        """配置电量管理参数

        Args:
            power: 电量管理配置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetPower], bool, str]: (电量管理配置信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def set_sound(
        self,
        sound: SetSound,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetSound], bool, str]:
        """配置声音系统参数

        Args:
            sound: 声音系统配置消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetSound], bool, str]: (声音系统配置信息, 是否成功, 错误信息)
        """
        pass


class IMapInfo(ABC):
    """地图信息接口"""

    @abstractmethod
    async def scene_list_req(
        self,
        scene_list: SceneList,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SceneList], bool, str]:
        """请求场景列表信息

        Args:
            scene_list: 场景列表请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SceneList], bool, str]: (场景列表信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def scene_data_req(
        self,
        scene_data: SceneData,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SceneData], bool, str]:
        """请求场景数据信息

        Args:
            scene_data: 场景数据请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SceneData], bool, str]: (场景数据信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def scene_data_easy_req(
        self,
        scene_data: SceneDataEasy,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SceneDataEasy], bool, str]:
        """请求简易版场景数据信息

        Args:
            scene_data: 简易版场景数据请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SceneDataEasy], bool, str]: (简易版场景数据信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def download_map(
        self,
        map_data: DownloadMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DownloadMapResponse], bool, str]:
        """下载地图数据

        Args:
            map_data: 地图下载请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[DownloadMapResponse], bool, str]: (地图下载响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def upload_map(
        self,
        map_data: UploadMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[UploadMap], bool, str]:
        """上传地图数据

        Args:
            map_data: 地图上传请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[UploadMap], bool, str]: (地图上传响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def rename_map(
        self,
        rename: RenameMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RenameMap], bool, str]:
        """重命名地图或场景

        Args:
            rename: 地图或场景重命名请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[RenameMap], bool, str]: (重命名响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def delete_map(
        self,
        delete: DeleteMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DeleteMap], bool, str]:
        """删除地图或场景

        Args:
            delete: 地图或场景删除请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[DeleteMap], bool, str]: (删除响应信息, 是否成功, 错误信息)
        """
        pass


class IMapEdit(ABC):
    """地图编辑接口"""

    @abstractmethod
    async def create_storage(
        self,
        storage: StorageCreate,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Storage], bool, str]:
        """创建新的储位

        Args:
            storage: 储位创建请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Storage], bool, str]: (储位信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def delete_storage(
        self,
        storage: StorageDelete,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[StorageDelete], bool, str]:
        """删除现有储位

        Args:
            storage: 储位删除请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[StorageDelete], bool, str]: (删除响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def update_storage(
        self,
        storage: StorageUpdate,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Storage], bool, str]:
        """更新储位信息

        Args:
            storage: 储位更新请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Storage], bool, str]: (更新后的储位信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def find_storage(
        self,
        storage: StorageFind,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Storage], bool, str]:
        """查找储位

        Args:
            storage: 储位查找请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Storage], bool, str]: (查找到的储位信息, 是否成功, 错误信息)
        """
        pass


class IDeviceInfo(ABC):
    """设备信息接口"""

    @abstractmethod
    async def rf_remote_controller_sub(
        self,
        callback: Callable[[RfRemoteController], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅射频遥控器信号

        Args:
            callback: 射频遥控器信号回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass


class IRobotProxy(ABC):
    """代理机器人接口"""

    @abstractmethod
    async def deployment_req(
        self,
        deployment: Deployment,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[DeploymentResponse], bool, str]:
        """发送机器人部署请求

        Args:
            deployment: 部署请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[DeploymentResponse], bool, str]: (部署响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def plan_nav_path_req(
        self,
        plan: PlanNavPath,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[PlanNavPath], bool, str]:
        """请求规划导航路径

        Args:
            plan: 导航路径规划请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[PlanNavPath], bool, str]: (规划的导航路径信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def scanner_data_req(
        self,
        scanner: ScannerData,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ScannerData], bool, str]:
        """请求雷达扫描数据

        Args:
            scanner: 雷达数据请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[ScannerData], bool, str]: (雷达扫描数据, 是否成功, 错误信息)
        """
        pass


class RobotInterface(ABC):
    """机器人控制接口，提供机器人的基本控制功能"""

    @abstractmethod
    async def init_robot_req(
        self,
        init_data: InitRobot,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[InitRobot], bool, str]:
        """初始化机器人

        Args:
            init_data: 机器人初始化请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[InitRobot], bool, str]: (初始化响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def set_robot_pose_req(
        self,
        pose: SetRobotPose,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetRobotPose], bool, str]:
        """设置机器人位姿

        Args:
            pose: 机器人位姿设置请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetRobotPose], bool, str]: (位姿设置响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def set_occupancy_req(
        self,
        occupancy: SetOccupancy,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetOccupancy], bool, str]:
        """设置占用状态

        Args:
            occupancy: 占用状态设置请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetOccupancy], bool, str]: (占用状态设置响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def set_mute_call_req(
        self,
        mute: SetMuteCall,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetMuteCall], bool, str]:
        """设置静音呼叫

        Args:
            mute: 静音呼叫设置请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetMuteCall], bool, str]: (静音呼叫设置响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def set_program_mute_req(
        self,
        program_mute: SetProgramMute,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetProgramMute], bool, str]:
        """设置程序静音

        Args:
            program_mute: 程序静音设置请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SetProgramMute], bool, str]: (程序静音设置响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def switch_control_mode_req(
        self,
        mode: SwitchControlMode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Mode], bool, str]:
        """切换控制模式

        Args:
            mode: 控制模式切换请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Mode], bool, str]: (控制模式信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def switch_work_mode_req(
        self,
        mode: SwitchWorkMode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Mode], bool, str]:
        """切换工作模式

        Args:
            mode: 工作模式切换请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Mode], bool, str]: (工作模式信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def switch_foot_print_req(
        self,
        foot_print: SwitchFootPrint,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SwitchFootPrint], bool, str]:
        """切换模型显示

        Args:
            foot_print: 模型显示切换请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SwitchFootPrint], bool, str]: (模型显示设置响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def switch_map_req(
        self,
        map_data: SwitchMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SwitchMap], bool, str]:
        """切换地图

        Args:
            map_data: 地图切换请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[SwitchMap], bool, str]: (地图切换响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def build_map_req(
        self,
        build: BuildMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[BuildMap], bool, str]:
        """请求建图

        Args:
            build: 建图请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[BuildMap], bool, str]: (建图响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def build_map_data_sub(
        self,
        callback: Callable[[BuildMapData], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅建图数据

        Args:
            callback: 建图数据回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def exec_pre_task_req(
        self,
        task: ExecPreTask,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ExecPreTask], bool, str]:
        """执行预设任务

        Args:
            task: 预设任务执行请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[ExecPreTask], bool, str]: (预设任务执行响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def exec_task_req(
        self,
        task: ExecTask,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ExecTask], bool, str]:
        """执行任务

        Args:
            task: 任务执行请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[ExecTask], bool, str]: (任务执行响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def action_order_req(
        self,
        order: ActionOrder,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ActionOrder], bool, str]:
        """发送动作指令

        Args:
            order: 动作指令请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[ActionOrder], bool, str]: (动作指令响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def repeat_task_req(
        self,
        task: RepeatTask,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RepeatTask], bool, str]:
        """重复执行任务

        Args:
            task: 任务重复执行请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[RepeatTask], bool, str]: (任务重复执行响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def change_nav_path_req(
        self,
        path: ChangeNavPath,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ChangeNavPath], bool, str]:
        """修改导航路径

        Args:
            path: 导航路径修改请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[ChangeNavPath], bool, str]: (导航路径修改响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def change_nav_mode_req(
        self,
        mode: ChangeNavMode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ChangeNavMode], bool, str]:
        """修改导航模式

        Args:
            mode: 导航模式修改请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[ChangeNavMode], bool, str]: (导航模式修改响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def speak_req(
        self,
        speak: Speak,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Speak], bool, str]:
        """语音播报请求

        Args:
            speak: 语音播报请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Speak], bool, str]: (语音播报响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def led_ctrl_req(
        self,
        led: LED,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[LED], bool, str]:
        """LED控制请求

        Args:
            led: LED控制请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[LED], bool, str]: (LED控制响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def twist_req(
        self,
        twist: Twist,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Twist], bool, str]:
        """速度控制请求

        Args:
            twist: 速度控制请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Twist], bool, str]: (速度控制响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def follow_req(
        self,
        follow: Follow,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Follow], bool, str]:
        """跟随控制请求

        Args:
            follow: 跟随控制请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[Follow], bool, str]: (跟随控制响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_wifi_req(
        self,
        wifi: RobotWiFi,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RobotWiFi], bool, str]:
        """WiFi列表请求

        Args:
            wifi: WiFi控制请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[RobotWiFi], bool, str]: (WiFi列表响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def count_data_req(
        self,
        count: CountData,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[CountDataResponse], bool, str]:
        """请求统计数据

        Args:
            count: 统计数据请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[CountDataResponse], bool, str]: (统计数据响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def robot_count_operation_sub(
        self,
        callback: Callable[[CountOperation], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅操作计数信息

        Args:
            callback: 操作计数信息回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_count_task_sub(
        self, callback: Callable[[CountTask], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅任务计数信息

        Args:
            callback: 任务计数信息回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def robot_count_status_sub(
        self,
        callback: Callable[[CountStatus], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅状态计数信息

        Args:
            callback: 状态计数信息回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def scanner_data_sub(
        self,
        callback: Callable[[ScannerData], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        """订阅雷达数据

        Args:
            callback: 雷达数据回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass

    @abstractmethod
    async def call_action_req(
        self,
        action: CallAction,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[CallAction], bool, str]:
        """ROS CallAction请求

        Args:
            action: ROS CallAction请求消息
            req_ppl: 请求打印级别，默认不打印
            rep_ppl: 响应打印级别，默认不打印

        Returns:
            Tuple[Optional[CountDataResponse], bool, str]: (CallAction响应信息, 是否成功, 错误信息)
        """
        pass

    @abstractmethod
    async def feedbacks_sub(
        self, callback: Callable[[Feedbacks], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        """订阅ROS反馈信息

        Args:
            callback: ROS反馈信息回调函数
            sub_ppl: 订阅消息打印级别，默认不打印

        Returns:
            bool: 订阅是否成功
        """
        pass
