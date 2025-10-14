from typing import Callable, Optional, Tuple

from woosh_interface import (
    CommuSettings,
    RobotInterface,
    PrintPackLevel,
    NO_PRINT,
)
from woosh_base import (
    RobotCommunication,
    Common,
    RobotInfo,
    RobotSetting,
    MapInfo,
    MapEdit,
    DeviceInfo,
    RobotProxy,
)

from woosh.proto.robot.robot_pb2 import (
    Mode,
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
    ExecPreTask,
    ExecTask,
    ActionOrder,
    ChangeNavPath,
    ChangeNavMode,
    Speak,
    LED,
    Twist,
    Follow,
    RobotWiFi,
    CountData,
    CountDataResponse,
)

from woosh.proto.robot.robot_count_pb2 import (
    Operation as CountOperation,
    Task as CountTask,
    Status as CountStatus,
)

from woosh.proto.ros.ros_pack_pb2 import CallAction, Feedbacks
from woosh.proto.task.woosh_task_pb2 import RepeatTask


class WooshRobot(
    RobotInterface,
    Common,
    RobotInfo,
    RobotSetting,
    MapInfo,
    MapEdit,
    DeviceInfo,
    RobotProxy,
):
    """机器人接口实现类"""

    def __init__(self, settings: CommuSettings):
        """初始化机器人接口"""
        self.comm = RobotCommunication(settings)
        self.logger = settings.logger

        RobotInterface.__init__(self)
        for parent in [
            Common,
            RobotInfo,
            RobotSetting,
            MapInfo,
            MapEdit,
            DeviceInfo,
            RobotProxy,
        ]:
            parent.__init__(self, self.comm)

    # RobotInterface methods
    async def init_robot_req(
        self,
        init_robot: InitRobot,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[InitRobot], bool, str]:
        try:
            return await self.comm.request(init_robot, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"初始化机器人失败: {str(e)}")
            return None, False, str(e)

    async def set_robot_pose_req(
        self,
        pose: SetRobotPose,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetRobotPose], bool, str]:
        try:
            return await self.comm.request(pose, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设置机器人位姿失败: {str(e)}")
            return None, False, str(e)

    async def set_occupancy_req(
        self,
        occupancy: SetOccupancy,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetOccupancy], bool, str]:
        try:
            return await self.comm.request(occupancy, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设置占用状态失败: {str(e)}")
            return None, False, str(e)

    async def set_mute_call_req(
        self,
        mute: SetMuteCall,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetMuteCall], bool, str]:
        try:
            return await self.comm.request(mute, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设置静音呼叫失败: {str(e)}")
            return None, False, str(e)

    async def set_program_mute_req(
        self,
        mute: SetProgramMute,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SetProgramMute], bool, str]:
        try:
            return await self.comm.request(mute, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"设置程序静音失败: {str(e)}")
            return None, False, str(e)

    async def switch_control_mode_req(
        self,
        mode: SwitchControlMode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Mode], bool, str]:
        try:
            return await self.comm.request(mode, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"切换控制模式失败: {str(e)}")
            return None, False, str(e)

    async def switch_work_mode_req(
        self,
        mode: SwitchWorkMode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Mode], bool, str]:
        try:
            return await self.comm.request(mode, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"切换工作模式失败: {str(e)}")
            return None, False, str(e)

    async def switch_foot_print_req(
        self,
        foot_print: SwitchFootPrint,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SwitchFootPrint], bool, str]:
        try:
            return await self.comm.request(foot_print, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"切换足迹失败: {str(e)}")
            return None, False, str(e)

    async def switch_map_req(
        self,
        map_switch: SwitchMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[SwitchMap], bool, str]:
        try:
            return await self.comm.request(map_switch, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"切换地图失败: {str(e)}")
            return None, False, str(e)

    async def build_map_req(
        self,
        build: BuildMap,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[BuildMap], bool, str]:
        try:
            return await self.comm.request(build, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"建图请求失败: {str(e)}")
            return None, False, str(e)

    async def build_map_data_sub(
        self,
        callback: Callable[[BuildMapData], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(BuildMapData, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"建图数据订阅失败: {str(e)}")
            return False

    async def exec_pre_task_req(
        self,
        task: ExecPreTask,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ExecPreTask], bool, str]:
        try:
            return await self.comm.request(task, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"执行预设任务失败: {str(e)}")
            return None, False, str(e)

    async def exec_task_req(
        self,
        task: ExecTask,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ExecTask], bool, str]:
        try:
            return await self.comm.request(task, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"执行任务失败: {str(e)}")
            return None, False, str(e)

    async def action_order_req(
        self,
        order: ActionOrder,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ActionOrder], bool, str]:
        try:
            return await self.comm.request(order, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"动作指令失败: {str(e)}")
            return None, False, str(e)

    async def repeat_task_req(
        self,
        task: RepeatTask,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RepeatTask], bool, str]:
        try:
            return await self.comm.request(task, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"重复任务失败: {str(e)}")
            return None, False, str(e)

    async def change_nav_path_req(
        self,
        change: ChangeNavPath,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ChangeNavPath], bool, str]:
        try:
            return await self.comm.request(change, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"修改导航路径失败: {str(e)}")
            return None, False, str(e)

    async def change_nav_mode_req(
        self,
        mode: ChangeNavMode,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[ChangeNavMode], bool, str]:
        try:
            return await self.comm.request(mode, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"修改导航模式失败: {str(e)}")
            return None, False, str(e)

    async def speak_req(
        self,
        speak: Speak,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Speak], bool, str]:
        try:
            return await self.comm.request(speak, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"语音播报失败: {str(e)}")
            return None, False, str(e)

    async def led_ctrl_req(
        self,
        led: LED,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[LED], bool, str]:
        try:
            return await self.comm.request(led, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"LED控制失败: {str(e)}")
            return None, False, str(e)

    async def twist_req(
        self,
        twist: Twist,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Twist], bool, str]:
        try:
            return await self.comm.request(twist, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"速度控制失败: {str(e)}")
            return None, False, str(e)

    async def follow_req(
        self,
        follow: Follow,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[Follow], bool, str]:
        try:
            return await self.comm.request(follow, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"跟随控制失败: {str(e)}")
            return None, False, str(e)

    async def robot_wifi_req(
        self,
        wifi: RobotWiFi,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[RobotWiFi], bool, str]:
        try:
            return await self.comm.request(wifi, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"WiFi控制失败: {str(e)}")
            return None, False, str(e)

    async def count_data_req(
        self,
        count: CountData,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[CountDataResponse], bool, str]:
        try:
            return await self.comm.request(count, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"统计数据请求失败: {str(e)}")
            return None, False, str(e)

    async def robot_count_operation_sub(
        self,
        callback: Callable[[CountOperation], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(CountOperation, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"操作计数订阅失败: {str(e)}")
            return False

    async def robot_count_task_sub(
        self, callback: Callable[[CountTask], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(CountTask, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"任务计数订阅失败: {str(e)}")
            return False

    async def robot_count_status_sub(
        self,
        callback: Callable[[CountStatus], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(CountStatus, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"状态计数订阅失败: {str(e)}")
            return False

    async def scanner_data_sub(
        self,
        callback: Callable[[ScannerData], None],
        sub_ppl: PrintPackLevel = NO_PRINT,
    ) -> bool:
        try:
            return await self.comm.subscribe(ScannerData, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"雷达数据订阅失败: {str(e)}")
            return False

    async def call_action_req(
        self,
        action: CallAction,
        req_ppl: PrintPackLevel = NO_PRINT,
        rep_ppl: PrintPackLevel = NO_PRINT,
    ) -> Tuple[Optional[CallAction], bool, str]:
        try:
            return await self.comm.request(action, req_ppl, rep_ppl)
        except Exception as e:
            self.logger.error(f"ROS CallAction失败: {str(e)}")
            return None, False, str(e)

    async def feedbacks_sub(
        self, callback: Callable[[Feedbacks], None], sub_ppl: PrintPackLevel = NO_PRINT
    ) -> bool:
        try:
            return await self.comm.subscribe(Feedbacks, callback, sub_ppl)
        except Exception as e:
            self.logger.error(f"ROS反馈订阅失败: {str(e)}")
            return False
