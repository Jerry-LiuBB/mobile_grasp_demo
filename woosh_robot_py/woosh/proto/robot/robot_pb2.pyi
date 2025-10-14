from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.util import navigation_pb2 as _navigation_pb2
from woosh.proto.util import action_pb2 as _action_pb2
from woosh.proto.util import task_pb2 as _task_pb2
from woosh.proto.util import robot_pb2 as _robot_pb2
from woosh.proto.robot import robot_count_pb2 as _robot_count_pb2
from woosh.proto.robot import robot_setting_pb2 as _robot_setting_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Battery(_message.Message):
    __slots__ = ["battery_cycle", "charge_cycle", "charge_state", "health", "power", "robot_id", "temp_max"]
    class ChargeState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    BATTERY_CYCLE_FIELD_NUMBER: _ClassVar[int]
    CHARGE_CYCLE_FIELD_NUMBER: _ClassVar[int]
    CHARGE_STATE_FIELD_NUMBER: _ClassVar[int]
    HEALTH_FIELD_NUMBER: _ClassVar[int]
    POWER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TEMP_MAX_FIELD_NUMBER: _ClassVar[int]
    battery_cycle: int
    charge_cycle: int
    charge_state: Battery.ChargeState
    health: int
    kAuto: Battery.ChargeState
    kChargeStateUndefined: Battery.ChargeState
    kManual: Battery.ChargeState
    kNot: Battery.ChargeState
    power: int
    robot_id: int
    temp_max: int
    def __init__(self, robot_id: _Optional[int] = ..., charge_state: _Optional[_Union[Battery.ChargeState, str]] = ..., power: _Optional[int] = ..., health: _Optional[int] = ..., charge_cycle: _Optional[int] = ..., battery_cycle: _Optional[int] = ..., temp_max: _Optional[int] = ...) -> None: ...

class BeaconData(_message.Message):
    __slots__ = ["beacon_id", "power_rate", "range", "robot_id"]
    BEACON_ID_FIELD_NUMBER: _ClassVar[int]
    POWER_RATE_FIELD_NUMBER: _ClassVar[int]
    RANGE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    beacon_id: int
    power_rate: float
    range: float
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., beacon_id: _Optional[int] = ..., range: _Optional[float] = ..., power_rate: _Optional[float] = ...) -> None: ...

class DeviceState(_message.Message):
    __slots__ = ["hardware", "robot_id", "software"]
    class HardwareBit(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class SoftwareBit(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    HARDWARE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SOFTWARE_FIELD_NUMBER: _ClassVar[int]
    hardware: int
    kBtn1: DeviceState.HardwareBit
    kBtn2: DeviceState.HardwareBit
    kBtn3: DeviceState.HardwareBit
    kBtn4: DeviceState.HardwareBit
    kBtn5: DeviceState.HardwareBit
    kBtn6: DeviceState.HardwareBit
    kBtn7: DeviceState.HardwareBit
    kBtn8: DeviceState.HardwareBit
    kEmgBtn: DeviceState.HardwareBit
    kGoodsState: DeviceState.SoftwareBit
    kHardwareBitUndefined: DeviceState.HardwareBit
    kLiftBtn: DeviceState.HardwareBit
    kLocation: DeviceState.SoftwareBit
    kMuteCall: DeviceState.SoftwareBit
    kOccupancy: DeviceState.SoftwareBit
    kProgramMute: DeviceState.SoftwareBit
    kSchedule: DeviceState.SoftwareBit
    kServoBtn: DeviceState.HardwareBit
    kSoftwareBitUndefined: DeviceState.SoftwareBit
    robot_id: int
    software: int
    def __init__(self, robot_id: _Optional[int] = ..., hardware: _Optional[int] = ..., software: _Optional[int] = ...) -> None: ...

class General(_message.Message):
    __slots__ = ["display_model", "driver_method", "model_data", "robot_id", "serial_number", "service_id", "type", "urdf_name", "version"]
    class ModelData(_message.Message):
        __slots__ = ["height", "length", "load", "weight", "width"]
        HEIGHT_FIELD_NUMBER: _ClassVar[int]
        LENGTH_FIELD_NUMBER: _ClassVar[int]
        LOAD_FIELD_NUMBER: _ClassVar[int]
        WEIGHT_FIELD_NUMBER: _ClassVar[int]
        WIDTH_FIELD_NUMBER: _ClassVar[int]
        height: int
        length: int
        load: int
        weight: int
        width: int
        def __init__(self, length: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., weight: _Optional[int] = ..., load: _Optional[int] = ...) -> None: ...
    class Version(_message.Message):
        __slots__ = ["rc", "system"]
        RC_FIELD_NUMBER: _ClassVar[int]
        SYSTEM_FIELD_NUMBER: _ClassVar[int]
        rc: str
        system: str
        def __init__(self, system: _Optional[str] = ..., rc: _Optional[str] = ...) -> None: ...
    DISPLAY_MODEL_FIELD_NUMBER: _ClassVar[int]
    DRIVER_METHOD_FIELD_NUMBER: _ClassVar[int]
    MODEL_DATA_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SERIAL_NUMBER_FIELD_NUMBER: _ClassVar[int]
    SERVICE_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    URDF_NAME_FIELD_NUMBER: _ClassVar[int]
    VERSION_FIELD_NUMBER: _ClassVar[int]
    display_model: str
    driver_method: int
    model_data: General.ModelData
    robot_id: int
    serial_number: int
    service_id: str
    type: _robot_pb2.Type
    urdf_name: str
    version: General.Version
    def __init__(self, robot_id: _Optional[int] = ..., type: _Optional[_Union[_robot_pb2.Type, str]] = ..., model_data: _Optional[_Union[General.ModelData, _Mapping]] = ..., urdf_name: _Optional[str] = ..., display_model: _Optional[str] = ..., serial_number: _Optional[int] = ..., service_id: _Optional[str] = ..., driver_method: _Optional[int] = ..., version: _Optional[_Union[General.Version, _Mapping]] = ...) -> None: ...

class HardwareState(_message.Message):
    __slots__ = ["beacon", "board", "camera", "crash", "esb", "imu", "lidar", "lift", "light", "magnetism", "motor", "plc", "power", "robot_id", "roller", "seanner", "sonar", "tractor"]
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    BEACON_FIELD_NUMBER: _ClassVar[int]
    BOARD_FIELD_NUMBER: _ClassVar[int]
    CAMERA_FIELD_NUMBER: _ClassVar[int]
    CRASH_FIELD_NUMBER: _ClassVar[int]
    ESB_FIELD_NUMBER: _ClassVar[int]
    IMU_FIELD_NUMBER: _ClassVar[int]
    LIDAR_FIELD_NUMBER: _ClassVar[int]
    LIFT_FIELD_NUMBER: _ClassVar[int]
    LIGHT_FIELD_NUMBER: _ClassVar[int]
    MAGNETISM_FIELD_NUMBER: _ClassVar[int]
    MOTOR_FIELD_NUMBER: _ClassVar[int]
    PLC_FIELD_NUMBER: _ClassVar[int]
    POWER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    ROLLER_FIELD_NUMBER: _ClassVar[int]
    SEANNER_FIELD_NUMBER: _ClassVar[int]
    SONAR_FIELD_NUMBER: _ClassVar[int]
    TRACTOR_FIELD_NUMBER: _ClassVar[int]
    beacon: HardwareState.State
    board: HardwareState.State
    camera: _containers.RepeatedScalarFieldContainer[HardwareState.State]
    crash: HardwareState.State
    esb: HardwareState.State
    imu: HardwareState.State
    kFatal: HardwareState.State
    kInfo: HardwareState.State
    kNormal: HardwareState.State
    kWarn: HardwareState.State
    lidar: _containers.RepeatedScalarFieldContainer[HardwareState.State]
    lift: _containers.RepeatedScalarFieldContainer[HardwareState.State]
    light: HardwareState.State
    magnetism: HardwareState.State
    motor: _containers.RepeatedScalarFieldContainer[HardwareState.State]
    plc: HardwareState.State
    power: HardwareState.State
    robot_id: int
    roller: _containers.RepeatedScalarFieldContainer[HardwareState.State]
    seanner: HardwareState.State
    sonar: _containers.RepeatedScalarFieldContainer[HardwareState.State]
    tractor: HardwareState.State
    def __init__(self, robot_id: _Optional[int] = ..., board: _Optional[_Union[HardwareState.State, str]] = ..., esb: _Optional[_Union[HardwareState.State, str]] = ..., crash: _Optional[_Union[HardwareState.State, str]] = ..., seanner: _Optional[_Union[HardwareState.State, str]] = ..., plc: _Optional[_Union[HardwareState.State, str]] = ..., motor: _Optional[_Iterable[_Union[HardwareState.State, str]]] = ..., power: _Optional[_Union[HardwareState.State, str]] = ..., lidar: _Optional[_Iterable[_Union[HardwareState.State, str]]] = ..., camera: _Optional[_Iterable[_Union[HardwareState.State, str]]] = ..., light: _Optional[_Union[HardwareState.State, str]] = ..., sonar: _Optional[_Iterable[_Union[HardwareState.State, str]]] = ..., magnetism: _Optional[_Union[HardwareState.State, str]] = ..., beacon: _Optional[_Union[HardwareState.State, str]] = ..., imu: _Optional[_Union[HardwareState.State, str]] = ..., lift: _Optional[_Iterable[_Union[HardwareState.State, str]]] = ..., roller: _Optional[_Iterable[_Union[HardwareState.State, str]]] = ..., tractor: _Optional[_Union[HardwareState.State, str]] = ...) -> None: ...

class Mode(_message.Message):
    __slots__ = ["ctrl", "robot_id", "work"]
    CTRL_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    WORK_FIELD_NUMBER: _ClassVar[int]
    ctrl: _robot_pb2.ControlMode
    robot_id: int
    work: _robot_pb2.WorkMode
    def __init__(self, robot_id: _Optional[int] = ..., ctrl: _Optional[_Union[_robot_pb2.ControlMode, str]] = ..., work: _Optional[_Union[_robot_pb2.WorkMode, str]] = ...) -> None: ...

class Model(_message.Message):
    __slots__ = ["model", "robot_id", "type"]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    model: _containers.RepeatedCompositeFieldContainer[_common_pb2.Point]
    robot_id: int
    type: _robot_pb2.FootPrint
    def __init__(self, robot_id: _Optional[int] = ..., model: _Optional[_Iterable[_Union[_common_pb2.Point, _Mapping]]] = ..., type: _Optional[_Union[_robot_pb2.FootPrint, str]] = ...) -> None: ...

class NavPath(_message.Message):
    __slots__ = ["path", "robot_id"]
    PATH_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    path: _navigation_pb2.Path
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., path: _Optional[_Union[_navigation_pb2.Path, _Mapping]] = ...) -> None: ...

class Network(_message.Message):
    __slots__ = ["is_connected", "robot_id", "robot_ip", "sch_ip", "wifi"]
    class WiFi(_message.Message):
        __slots__ = ["code", "list_json", "mode", "name", "strength"]
        class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
            __slots__ = []
        CODE_FIELD_NUMBER: _ClassVar[int]
        LIST_JSON_FIELD_NUMBER: _ClassVar[int]
        MODE_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        STRENGTH_FIELD_NUMBER: _ClassVar[int]
        code: int
        kAP: Network.WiFi.Mode
        kClient: Network.WiFi.Mode
        kToAP: Network.WiFi.Mode
        kToClient: Network.WiFi.Mode
        kWiFiModeUndefined: Network.WiFi.Mode
        list_json: bytes
        mode: Network.WiFi.Mode
        name: str
        strength: int
        def __init__(self, name: _Optional[str] = ..., code: _Optional[int] = ..., list_json: _Optional[bytes] = ..., strength: _Optional[int] = ..., mode: _Optional[_Union[Network.WiFi.Mode, str]] = ...) -> None: ...
    IS_CONNECTED_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    ROBOT_IP_FIELD_NUMBER: _ClassVar[int]
    SCH_IP_FIELD_NUMBER: _ClassVar[int]
    WIFI_FIELD_NUMBER: _ClassVar[int]
    is_connected: bool
    robot_id: int
    robot_ip: str
    sch_ip: str
    wifi: Network.WiFi
    def __init__(self, robot_id: _Optional[int] = ..., is_connected: bool = ..., robot_ip: _Optional[str] = ..., sch_ip: _Optional[str] = ..., wifi: _Optional[_Union[Network.WiFi, _Mapping]] = ...) -> None: ...

class OperationState(_message.Message):
    __slots__ = ["nav", "robot", "robot_id"]
    class NavBit(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class RobotBit(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    NAV_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    kGuide: OperationState.NavBit
    kImpede: OperationState.NavBit
    kInaLift: OperationState.NavBit
    kNarrow: OperationState.NavBit
    kNavBitUndefined: OperationState.NavBit
    kQRCode: OperationState.NavBit
    kRobotBitUndefined: OperationState.RobotBit
    kStage: OperationState.NavBit
    kTaskable: OperationState.RobotBit
    nav: int
    robot: int
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., nav: _Optional[int] = ..., robot: _Optional[int] = ...) -> None: ...

class PlanPath(_message.Message):
    __slots__ = ["plan_path", "robot_id"]
    PLAN_PATH_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    plan_path: _containers.RepeatedCompositeFieldContainer[_navigation_pb2.PlanPath]
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., plan_path: _Optional[_Iterable[_Union[_navigation_pb2.PlanPath, _Mapping]]] = ...) -> None: ...

class PoseSpeed(_message.Message):
    __slots__ = ["map_id", "mileage", "pose", "robot_id", "twist"]
    MAP_ID_FIELD_NUMBER: _ClassVar[int]
    MILEAGE_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TWIST_FIELD_NUMBER: _ClassVar[int]
    map_id: int
    mileage: int
    pose: _common_pb2.Pose2D
    robot_id: int
    twist: _common_pb2.Twist
    def __init__(self, robot_id: _Optional[int] = ..., twist: _Optional[_Union[_common_pb2.Twist, _Mapping]] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., map_id: _Optional[int] = ..., mileage: _Optional[int] = ...) -> None: ...

class RobotInfo(_message.Message):
    __slots__ = ["abnormal_codes", "battery", "count_error", "count_operation", "count_task", "device_state", "genral", "hardware_state", "mode", "model", "network", "operation_state", "pose_speed", "robot_id", "scene", "setting", "state", "status_codes", "task_history", "task_proc"]
    ABNORMAL_CODES_FIELD_NUMBER: _ClassVar[int]
    BATTERY_FIELD_NUMBER: _ClassVar[int]
    COUNT_ERROR_FIELD_NUMBER: _ClassVar[int]
    COUNT_OPERATION_FIELD_NUMBER: _ClassVar[int]
    COUNT_TASK_FIELD_NUMBER: _ClassVar[int]
    DEVICE_STATE_FIELD_NUMBER: _ClassVar[int]
    GENRAL_FIELD_NUMBER: _ClassVar[int]
    HARDWARE_STATE_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    NETWORK_FIELD_NUMBER: _ClassVar[int]
    OPERATION_STATE_FIELD_NUMBER: _ClassVar[int]
    POSE_SPEED_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SCENE_FIELD_NUMBER: _ClassVar[int]
    SETTING_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    STATUS_CODES_FIELD_NUMBER: _ClassVar[int]
    TASK_HISTORY_FIELD_NUMBER: _ClassVar[int]
    TASK_PROC_FIELD_NUMBER: _ClassVar[int]
    abnormal_codes: _robot_count_pb2.AbnormalCodes
    battery: Battery
    count_error: _robot_count_pb2.Status
    count_operation: _robot_count_pb2.Operation
    count_task: _robot_count_pb2.Task
    device_state: DeviceState
    genral: General
    hardware_state: HardwareState
    mode: Mode
    model: Model
    network: Network
    operation_state: OperationState
    pose_speed: PoseSpeed
    robot_id: int
    scene: Scene
    setting: Setting
    state: _robot_pb2.State
    status_codes: _robot_count_pb2.StatusCodes
    task_history: TaskHistory
    task_proc: TaskProc
    def __init__(self, robot_id: _Optional[int] = ..., genral: _Optional[_Union[General, _Mapping]] = ..., setting: _Optional[_Union[Setting, _Mapping]] = ..., state: _Optional[_Union[_robot_pb2.State, str]] = ..., mode: _Optional[_Union[Mode, _Mapping]] = ..., pose_speed: _Optional[_Union[PoseSpeed, _Mapping]] = ..., battery: _Optional[_Union[Battery, _Mapping]] = ..., network: _Optional[_Union[Network, _Mapping]] = ..., scene: _Optional[_Union[Scene, _Mapping]] = ..., task_proc: _Optional[_Union[TaskProc, _Mapping]] = ..., device_state: _Optional[_Union[DeviceState, _Mapping]] = ..., hardware_state: _Optional[_Union[HardwareState, _Mapping]] = ..., operation_state: _Optional[_Union[OperationState, _Mapping]] = ..., model: _Optional[_Union[Model, _Mapping]] = ..., task_history: _Optional[_Union[TaskHistory, _Mapping]] = ..., status_codes: _Optional[_Union[_robot_count_pb2.StatusCodes, _Mapping]] = ..., abnormal_codes: _Optional[_Union[_robot_count_pb2.AbnormalCodes, _Mapping]] = ..., count_operation: _Optional[_Union[_robot_count_pb2.Operation, _Mapping]] = ..., count_task: _Optional[_Union[_robot_count_pb2.Task, _Mapping]] = ..., count_error: _Optional[_Union[_robot_count_pb2.Status, _Mapping]] = ...) -> None: ...

class RobotState(_message.Message):
    __slots__ = ["robot_id", "state"]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    robot_id: int
    state: _robot_pb2.State
    def __init__(self, robot_id: _Optional[int] = ..., state: _Optional[_Union[_robot_pb2.State, str]] = ...) -> None: ...

class ScannerData(_message.Message):
    __slots__ = ["angle_increment", "angle_max", "angle_min", "offset", "pose", "range_max", "range_min", "ranges", "robot_id", "scan_time", "time_increment"]
    ANGLE_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MAX_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MIN_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    RANGES_FIELD_NUMBER: _ClassVar[int]
    RANGE_MAX_FIELD_NUMBER: _ClassVar[int]
    RANGE_MIN_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SCAN_TIME_FIELD_NUMBER: _ClassVar[int]
    TIME_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    angle_increment: float
    angle_max: float
    angle_min: float
    offset: _common_pb2.Vector3
    pose: _common_pb2.Pose2D
    range_max: float
    range_min: float
    ranges: _containers.RepeatedScalarFieldContainer[float]
    robot_id: int
    scan_time: float
    time_increment: float
    def __init__(self, robot_id: _Optional[int] = ..., angle_min: _Optional[float] = ..., angle_max: _Optional[float] = ..., angle_increment: _Optional[float] = ..., time_increment: _Optional[float] = ..., scan_time: _Optional[float] = ..., range_min: _Optional[float] = ..., range_max: _Optional[float] = ..., ranges: _Optional[_Iterable[float]] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., offset: _Optional[_Union[_common_pb2.Vector3, _Mapping]] = ...) -> None: ...

class Scene(_message.Message):
    __slots__ = ["map_id", "map_name", "robot_id", "scene_name", "version"]
    MAP_ID_FIELD_NUMBER: _ClassVar[int]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    VERSION_FIELD_NUMBER: _ClassVar[int]
    map_id: int
    map_name: str
    robot_id: int
    scene_name: str
    version: int
    def __init__(self, robot_id: _Optional[int] = ..., scene_name: _Optional[str] = ..., map_id: _Optional[int] = ..., map_name: _Optional[str] = ..., version: _Optional[int] = ...) -> None: ...

class Setting(_message.Message):
    __slots__ = ["allow", "identity", "power", "robot_id", "server", "sound"]
    class Allow(_message.Message):
        __slots__ = ["auto_charge", "auto_park", "goods_check", "mechanism_check"]
        AUTO_CHARGE_FIELD_NUMBER: _ClassVar[int]
        AUTO_PARK_FIELD_NUMBER: _ClassVar[int]
        GOODS_CHECK_FIELD_NUMBER: _ClassVar[int]
        MECHANISM_CHECK_FIELD_NUMBER: _ClassVar[int]
        auto_charge: bool
        auto_park: bool
        goods_check: bool
        mechanism_check: bool
        def __init__(self, auto_charge: bool = ..., auto_park: bool = ..., goods_check: bool = ..., mechanism_check: bool = ...) -> None: ...
    ALLOW_FIELD_NUMBER: _ClassVar[int]
    IDENTITY_FIELD_NUMBER: _ClassVar[int]
    POWER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SERVER_FIELD_NUMBER: _ClassVar[int]
    SOUND_FIELD_NUMBER: _ClassVar[int]
    allow: Setting.Allow
    identity: _robot_setting_pb2.Identity
    power: _robot_setting_pb2.Power
    robot_id: int
    server: _robot_setting_pb2.Server
    sound: _robot_setting_pb2.Sound
    def __init__(self, robot_id: _Optional[int] = ..., identity: _Optional[_Union[_robot_setting_pb2.Identity, _Mapping]] = ..., server: _Optional[_Union[_robot_setting_pb2.Server, _Mapping]] = ..., power: _Optional[_Union[_robot_setting_pb2.Power, _Mapping]] = ..., sound: _Optional[_Union[_robot_setting_pb2.Sound, _Mapping]] = ..., allow: _Optional[_Union[Setting.Allow, _Mapping]] = ...) -> None: ...

class TaskHistory(_message.Message):
    __slots__ = ["tes"]
    TES_FIELD_NUMBER: _ClassVar[int]
    tes: _containers.RepeatedCompositeFieldContainer[TaskProc]
    def __init__(self, tes: _Optional[_Iterable[_Union[TaskProc, _Mapping]]] = ...) -> None: ...

class TaskProc(_message.Message):
    __slots__ = ["action", "dest", "msg", "robot_id", "robot_task_id", "state", "time", "type"]
    class Action(_message.Message):
        __slots__ = ["state", "type", "wait_id"]
        STATE_FIELD_NUMBER: _ClassVar[int]
        TYPE_FIELD_NUMBER: _ClassVar[int]
        WAIT_ID_FIELD_NUMBER: _ClassVar[int]
        state: _action_pb2.State
        type: _action_pb2.Type
        wait_id: int
        def __init__(self, type: _Optional[_Union[_action_pb2.Type, str]] = ..., state: _Optional[_Union[_action_pb2.State, str]] = ..., wait_id: _Optional[int] = ...) -> None: ...
    ACTION_FIELD_NUMBER: _ClassVar[int]
    DEST_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    ROBOT_TASK_ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    action: TaskProc.Action
    dest: str
    msg: str
    robot_id: int
    robot_task_id: int
    state: _task_pb2.State
    time: int
    type: _task_pb2.Type
    def __init__(self, robot_id: _Optional[int] = ..., robot_task_id: _Optional[int] = ..., type: _Optional[_Union[_task_pb2.Type, str]] = ..., state: _Optional[_Union[_task_pb2.State, str]] = ..., action: _Optional[_Union[TaskProc.Action, _Mapping]] = ..., dest: _Optional[str] = ..., msg: _Optional[str] = ..., time: _Optional[int] = ...) -> None: ...
