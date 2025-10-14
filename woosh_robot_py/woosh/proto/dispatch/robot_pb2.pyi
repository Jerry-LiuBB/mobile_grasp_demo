from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.task import task_pb2 as _task_pb2
from google.protobuf import any_pb2 as _any_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Activity(_message.Message):
    __slots__ = ["id", "mileage", "power", "state", "time"]
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ID_FIELD_NUMBER: _ClassVar[int]
    MILEAGE_FIELD_NUMBER: _ClassVar[int]
    POWER_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    id: int
    kConnected: Activity.State
    kDisconnected: Activity.State
    kDropped: Activity.State
    kOffline: Activity.State
    kOnline: Activity.State
    mileage: int
    power: int
    state: Activity.State
    time: int
    def __init__(self, id: _Optional[int] = ..., state: _Optional[_Union[Activity.State, str]] = ..., time: _Optional[int] = ..., power: _Optional[int] = ..., mileage: _Optional[int] = ...) -> None: ...

class Activitys(_message.Message):
    __slots__ = ["activitys"]
    ACTIVITYS_FIELD_NUMBER: _ClassVar[int]
    activitys: _containers.RepeatedCompositeFieldContainer[Activity]
    def __init__(self, activitys: _Optional[_Iterable[_Union[Activity, _Mapping]]] = ...) -> None: ...

class Robot(_message.Message):
    __slots__ = ["commu", "dest", "id", "schedulable", "signout", "task", "task_set_id"]
    class Commu(_message.Message):
        __slots__ = ["ip", "online", "port", "time"]
        IP_FIELD_NUMBER: _ClassVar[int]
        ONLINE_FIELD_NUMBER: _ClassVar[int]
        PORT_FIELD_NUMBER: _ClassVar[int]
        TIME_FIELD_NUMBER: _ClassVar[int]
        ip: str
        online: bool
        port: int
        time: int
        def __init__(self, ip: _Optional[str] = ..., port: _Optional[int] = ..., online: bool = ..., time: _Optional[int] = ...) -> None: ...
    COMMU_FIELD_NUMBER: _ClassVar[int]
    DEST_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    SCHEDULABLE_FIELD_NUMBER: _ClassVar[int]
    SIGNOUT_FIELD_NUMBER: _ClassVar[int]
    TASK_FIELD_NUMBER: _ClassVar[int]
    TASK_SET_ID_FIELD_NUMBER: _ClassVar[int]
    commu: Robot.Commu
    dest: _common_pb2.Pose
    id: int
    schedulable: bool
    signout: bool
    task: _task_pb2.TaskBase
    task_set_id: int
    def __init__(self, id: _Optional[int] = ..., signout: bool = ..., schedulable: bool = ..., commu: _Optional[_Union[Robot.Commu, _Mapping]] = ..., task_set_id: _Optional[int] = ..., task: _Optional[_Union[_task_pb2.TaskBase, _Mapping]] = ..., dest: _Optional[_Union[_common_pb2.Pose, _Mapping]] = ...) -> None: ...

class RobotAnyInfo(_message.Message):
    __slots__ = ["id", "info", "type"]
    class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ID_FIELD_NUMBER: _ClassVar[int]
    INFO_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    id: int
    info: _any_pb2.Any
    kAbnormalCodes: RobotAnyInfo.Type
    kAll: RobotAnyInfo.Type
    kBattery: RobotAnyInfo.Type
    kCtrlMode: RobotAnyInfo.Type
    kDeviceState: RobotAnyInfo.Type
    kGeneral: RobotAnyInfo.Type
    kHardwareState: RobotAnyInfo.Type
    kNetwork: RobotAnyInfo.Type
    kPoseSpeed: RobotAnyInfo.Type
    kProgramState: RobotAnyInfo.Type
    kRobotModel: RobotAnyInfo.Type
    kSence: RobotAnyInfo.Type
    kSetting: RobotAnyInfo.Type
    kState: RobotAnyInfo.Type
    kStatusCodes: RobotAnyInfo.Type
    kTask: RobotAnyInfo.Type
    type: RobotAnyInfo.Type
    def __init__(self, id: _Optional[int] = ..., type: _Optional[_Union[RobotAnyInfo.Type, str]] = ..., info: _Optional[_Union[_any_pb2.Any, _Mapping]] = ...) -> None: ...

class Robots(_message.Message):
    __slots__ = ["robots"]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    robots: _containers.RepeatedCompositeFieldContainer[Robot]
    def __init__(self, robots: _Optional[_Iterable[_Union[Robot, _Mapping]]] = ...) -> None: ...
