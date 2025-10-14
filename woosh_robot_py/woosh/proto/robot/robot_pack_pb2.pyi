from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.util import action_pb2 as _action_pb2
from woosh.proto.util import navigation_pb2 as _navigation_pb2
from woosh.proto.util import task_pb2 as _task_pb2
from woosh.proto.util import robot_pb2 as _robot_pb2
from woosh.proto.robot import robot_pb2 as _robot_pb2_1
from woosh.proto.robot import robot_count_pb2 as _robot_count_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kArTags: DeploymentType
kBeacons: DeploymentType
kDepCancel: DeploymentMode
kDepCollect: DeploymentMode
kDepReflector: DeploymentMode
kDeploymentModeUndefined: DeploymentMode
kDeploymentTypeUndefined: DeploymentType
kMark: DeploymentType
kReflectors: DeploymentType

class ActionOrder(_message.Message):
    __slots__ = ["order"]
    ORDER_FIELD_NUMBER: _ClassVar[int]
    order: _action_pb2.Order
    def __init__(self, order: _Optional[_Union[_action_pb2.Order, str]] = ...) -> None: ...

class BuildMap(_message.Message):
    __slots__ = ["map_name", "scene_name", "type"]
    class BuildType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    kAdd: BuildMap.BuildType
    kBuildTypeUndefined: BuildMap.BuildType
    kCanel: BuildMap.BuildType
    kSave: BuildMap.BuildType
    kUpdate: BuildMap.BuildType
    map_name: str
    scene_name: str
    type: BuildMap.BuildType
    def __init__(self, type: _Optional[_Union[BuildMap.BuildType, str]] = ..., scene_name: _Optional[str] = ..., map_name: _Optional[str] = ...) -> None: ...

class BuildMapData(_message.Message):
    __slots__ = ["height", "map_data", "mark_data", "origin_x", "origin_y", "resolution", "width"]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    MAP_DATA_FIELD_NUMBER: _ClassVar[int]
    MARK_DATA_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_X_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_Y_FIELD_NUMBER: _ClassVar[int]
    RESOLUTION_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    height: int
    map_data: bytes
    mark_data: bytes
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    def __init__(self, map_data: _Optional[bytes] = ..., width: _Optional[int] = ..., height: _Optional[int] = ..., origin_x: _Optional[float] = ..., origin_y: _Optional[float] = ..., resolution: _Optional[float] = ..., mark_data: _Optional[bytes] = ...) -> None: ...

class ChangeNavMode(_message.Message):
    __slots__ = ["in_point", "nav_mode", "out_point"]
    IN_POINT_FIELD_NUMBER: _ClassVar[int]
    NAV_MODE_FIELD_NUMBER: _ClassVar[int]
    OUT_POINT_FIELD_NUMBER: _ClassVar[int]
    in_point: _common_pb2.Pose2D
    nav_mode: _navigation_pb2.ModeSetting
    out_point: _common_pb2.Pose2D
    def __init__(self, nav_mode: _Optional[_Union[_navigation_pb2.ModeSetting, _Mapping]] = ..., in_point: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., out_point: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class ChangeNavPath(_message.Message):
    __slots__ = ["paths"]
    PATHS_FIELD_NUMBER: _ClassVar[int]
    paths: _robot_pb2_1.PlanPath
    def __init__(self, paths: _Optional[_Union[_robot_pb2_1.PlanPath, _Mapping]] = ...) -> None: ...

class CountData(_message.Message):
    __slots__ = ["bucket", "count_type", "operation_id", "operation_page", "req_type", "status_id", "status_page", "task_id", "task_page"]
    class CountType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    COUNT_TYPE_FIELD_NUMBER: _ClassVar[int]
    OPERATION_ID_FIELD_NUMBER: _ClassVar[int]
    OPERATION_PAGE_FIELD_NUMBER: _ClassVar[int]
    REQ_TYPE_FIELD_NUMBER: _ClassVar[int]
    STATUS_ID_FIELD_NUMBER: _ClassVar[int]
    STATUS_PAGE_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_PAGE_FIELD_NUMBER: _ClassVar[int]
    bucket: _common_pb2.TimeBucket
    count_type: CountData.CountType
    kAll: CountData.CountType
    kGet: CountData.Type
    kOperation: CountData.CountType
    kStatus: CountData.CountType
    kSync: CountData.Type
    kTask: CountData.CountType
    operation_id: _containers.RepeatedScalarFieldContainer[int]
    operation_page: int
    req_type: CountData.Type
    status_id: _containers.RepeatedScalarFieldContainer[int]
    status_page: int
    task_id: _containers.RepeatedScalarFieldContainer[int]
    task_page: int
    def __init__(self, req_type: _Optional[_Union[CountData.Type, str]] = ..., count_type: _Optional[_Union[CountData.CountType, str]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., operation_id: _Optional[_Iterable[int]] = ..., task_id: _Optional[_Iterable[int]] = ..., status_id: _Optional[_Iterable[int]] = ..., operation_page: _Optional[int] = ..., task_page: _Optional[int] = ..., status_page: _Optional[int] = ...) -> None: ...

class CountDataResponse(_message.Message):
    __slots__ = ["abnormal_qty", "abnormals", "operation_qty", "operations", "task_qty", "tasks"]
    ABNORMALS_FIELD_NUMBER: _ClassVar[int]
    ABNORMAL_QTY_FIELD_NUMBER: _ClassVar[int]
    OPERATIONS_FIELD_NUMBER: _ClassVar[int]
    OPERATION_QTY_FIELD_NUMBER: _ClassVar[int]
    TASKS_FIELD_NUMBER: _ClassVar[int]
    TASK_QTY_FIELD_NUMBER: _ClassVar[int]
    abnormal_qty: int
    abnormals: _robot_count_pb2.Abnormals
    operation_qty: int
    operations: _robot_count_pb2.Operations
    task_qty: int
    tasks: _robot_count_pb2.Tasks
    def __init__(self, operations: _Optional[_Union[_robot_count_pb2.Operations, _Mapping]] = ..., tasks: _Optional[_Union[_robot_count_pb2.Tasks, _Mapping]] = ..., abnormals: _Optional[_Union[_robot_count_pb2.Abnormals, _Mapping]] = ..., operation_qty: _Optional[int] = ..., task_qty: _Optional[int] = ..., abnormal_qty: _Optional[int] = ...) -> None: ...

class Deployment(_message.Message):
    __slots__ = ["mode", "pose", "robot_id", "type"]
    MODE_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    mode: DeploymentMode
    pose: _common_pb2.Pose2D
    robot_id: int
    type: DeploymentType
    def __init__(self, robot_id: _Optional[int] = ..., type: _Optional[_Union[DeploymentType, str]] = ..., mode: _Optional[_Union[DeploymentMode, str]] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class DeploymentResponse(_message.Message):
    __slots__ = ["check_times", "error_id_groups", "markers", "mode", "type"]
    class ErrorID(_message.Message):
        __slots__ = ["id1", "id2", "id3"]
        ID1_FIELD_NUMBER: _ClassVar[int]
        ID2_FIELD_NUMBER: _ClassVar[int]
        ID3_FIELD_NUMBER: _ClassVar[int]
        id1: int
        id2: int
        id3: int
        def __init__(self, id1: _Optional[int] = ..., id2: _Optional[int] = ..., id3: _Optional[int] = ...) -> None: ...
    class ErrorIDGroup(_message.Message):
        __slots__ = ["error_ids"]
        ERROR_IDS_FIELD_NUMBER: _ClassVar[int]
        error_ids: _containers.RepeatedCompositeFieldContainer[DeploymentResponse.ErrorID]
        def __init__(self, error_ids: _Optional[_Iterable[_Union[DeploymentResponse.ErrorID, _Mapping]]] = ...) -> None: ...
    class Marker(_message.Message):
        __slots__ = ["id", "pose"]
        ID_FIELD_NUMBER: _ClassVar[int]
        POSE_FIELD_NUMBER: _ClassVar[int]
        id: int
        pose: _common_pb2.Pose2D
        def __init__(self, id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...
    CHECK_TIMES_FIELD_NUMBER: _ClassVar[int]
    ERROR_ID_GROUPS_FIELD_NUMBER: _ClassVar[int]
    MARKERS_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    check_times: int
    error_id_groups: _containers.RepeatedCompositeFieldContainer[DeploymentResponse.ErrorIDGroup]
    markers: _containers.RepeatedCompositeFieldContainer[DeploymentResponse.Marker]
    mode: DeploymentMode
    type: DeploymentType
    def __init__(self, type: _Optional[_Union[DeploymentType, str]] = ..., mode: _Optional[_Union[DeploymentMode, str]] = ..., markers: _Optional[_Iterable[_Union[DeploymentResponse.Marker, _Mapping]]] = ..., error_id_groups: _Optional[_Iterable[_Union[DeploymentResponse.ErrorIDGroup, _Mapping]]] = ..., check_times: _Optional[int] = ...) -> None: ...

class ExecPreTask(_message.Message):
    __slots__ = ["task_set_id"]
    TASK_SET_ID_FIELD_NUMBER: _ClassVar[int]
    task_set_id: int
    def __init__(self, task_set_id: _Optional[int] = ...) -> None: ...

class ExecTask(_message.Message):
    __slots__ = ["custom", "direction", "mark_no", "plan_path", "pose", "task_id", "task_type_no", "type"]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    MARK_NO_FIELD_NUMBER: _ClassVar[int]
    PLAN_PATH_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_TYPE_NO_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    custom: bytes
    direction: _task_pb2.Direction
    mark_no: str
    plan_path: _robot_pb2_1.PlanPath
    pose: _common_pb2.Pose2D
    task_id: int
    task_type_no: int
    type: _task_pb2.Type
    def __init__(self, task_id: _Optional[int] = ..., type: _Optional[_Union[_task_pb2.Type, str]] = ..., direction: _Optional[_Union[_task_pb2.Direction, str]] = ..., task_type_no: _Optional[int] = ..., mark_no: _Optional[str] = ..., plan_path: _Optional[_Union[_robot_pb2_1.PlanPath, _Mapping]] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., custom: _Optional[bytes] = ...) -> None: ...

class ExecTaskSet(_message.Message):
    __slots__ = ["repeat_times"]
    REPEAT_TIMES_FIELD_NUMBER: _ClassVar[int]
    repeat_times: int
    def __init__(self, repeat_times: _Optional[int] = ...) -> None: ...

class Follow(_message.Message):
    __slots__ = ["type"]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    type: bool
    def __init__(self, type: bool = ...) -> None: ...

class InitRobot(_message.Message):
    __slots__ = ["is_record", "pose"]
    IS_RECORD_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    is_record: bool
    pose: _common_pb2.Pose2D
    def __init__(self, is_record: bool = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class LED(_message.Message):
    __slots__ = ["abnormal", "color", "normal", "submodule", "urgency"]
    ABNORMAL_FIELD_NUMBER: _ClassVar[int]
    COLOR_FIELD_NUMBER: _ClassVar[int]
    NORMAL_FIELD_NUMBER: _ClassVar[int]
    SUBMODULE_FIELD_NUMBER: _ClassVar[int]
    URGENCY_FIELD_NUMBER: _ClassVar[int]
    abnormal: int
    color: int
    normal: int
    submodule: int
    urgency: int
    def __init__(self, submodule: _Optional[int] = ..., urgency: _Optional[int] = ..., abnormal: _Optional[int] = ..., normal: _Optional[int] = ..., color: _Optional[int] = ...) -> None: ...

class PlanNavPath(_message.Message):
    __slots__ = ["end", "robot_id", "start", "tolerance"]
    END_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    START_FIELD_NUMBER: _ClassVar[int]
    TOLERANCE_FIELD_NUMBER: _ClassVar[int]
    end: _common_pb2.Pose2D
    robot_id: int
    start: _common_pb2.Pose2D
    tolerance: float
    def __init__(self, robot_id: _Optional[int] = ..., start: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., end: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., tolerance: _Optional[float] = ...) -> None: ...

class RobotWiFi(_message.Message):
    __slots__ = ["content", "enable", "is_connect_now", "order"]
    class Order(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    CONTENT_FIELD_NUMBER: _ClassVar[int]
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    IS_CONNECT_NOW_FIELD_NUMBER: _ClassVar[int]
    ORDER_FIELD_NUMBER: _ClassVar[int]
    content: bytes
    enable: bool
    is_connect_now: bool
    kAdd: RobotWiFi.Order
    kForget: RobotWiFi.Order
    kHotspot: RobotWiFi.Order
    kOrderUndefined: RobotWiFi.Order
    kReconnect: RobotWiFi.Order
    kWiFiList: RobotWiFi.Order
    order: RobotWiFi.Order
    def __init__(self, order: _Optional[_Union[RobotWiFi.Order, str]] = ..., content: _Optional[bytes] = ..., is_connect_now: bool = ..., enable: bool = ...) -> None: ...

class SetMuteCall(_message.Message):
    __slots__ = ["mute"]
    MUTE_FIELD_NUMBER: _ClassVar[int]
    mute: bool
    def __init__(self, mute: bool = ...) -> None: ...

class SetOccupancy(_message.Message):
    __slots__ = ["occupy"]
    OCCUPY_FIELD_NUMBER: _ClassVar[int]
    occupy: bool
    def __init__(self, occupy: bool = ...) -> None: ...

class SetProgramMute(_message.Message):
    __slots__ = ["mute"]
    MUTE_FIELD_NUMBER: _ClassVar[int]
    mute: bool
    def __init__(self, mute: bool = ...) -> None: ...

class SetRobotPose(_message.Message):
    __slots__ = ["pose"]
    POSE_FIELD_NUMBER: _ClassVar[int]
    pose: _common_pb2.Pose2D
    def __init__(self, pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class Speak(_message.Message):
    __slots__ = ["text"]
    TEXT_FIELD_NUMBER: _ClassVar[int]
    text: str
    def __init__(self, text: _Optional[str] = ...) -> None: ...

class SwitchControlMode(_message.Message):
    __slots__ = ["mode"]
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: _robot_pb2.ControlMode
    def __init__(self, mode: _Optional[_Union[_robot_pb2.ControlMode, str]] = ...) -> None: ...

class SwitchFootPrint(_message.Message):
    __slots__ = ["type"]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    type: _robot_pb2.FootPrint
    def __init__(self, type: _Optional[_Union[_robot_pb2.FootPrint, str]] = ...) -> None: ...

class SwitchMap(_message.Message):
    __slots__ = ["file_datas", "map_name", "scene_name"]
    FILE_DATAS_FIELD_NUMBER: _ClassVar[int]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    file_datas: _containers.RepeatedCompositeFieldContainer[_common_pb2.FileData]
    map_name: str
    scene_name: str
    def __init__(self, scene_name: _Optional[str] = ..., map_name: _Optional[str] = ..., file_datas: _Optional[_Iterable[_Union[_common_pb2.FileData, _Mapping]]] = ...) -> None: ...

class SwitchWorkMode(_message.Message):
    __slots__ = ["mode"]
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: _robot_pb2.WorkMode
    def __init__(self, mode: _Optional[_Union[_robot_pb2.WorkMode, str]] = ...) -> None: ...

class Twist(_message.Message):
    __slots__ = ["angular", "linear", "linear_y"]
    ANGULAR_FIELD_NUMBER: _ClassVar[int]
    LINEAR_FIELD_NUMBER: _ClassVar[int]
    LINEAR_Y_FIELD_NUMBER: _ClassVar[int]
    angular: float
    linear: float
    linear_y: float
    def __init__(self, linear: _Optional[float] = ..., angular: _Optional[float] = ..., linear_y: _Optional[float] = ...) -> None: ...

class DeploymentType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class DeploymentMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
