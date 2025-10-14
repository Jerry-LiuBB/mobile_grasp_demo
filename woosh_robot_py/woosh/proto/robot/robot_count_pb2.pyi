from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.util import action_pb2 as _action_pb2
from woosh.proto.util import robot_pb2 as _robot_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AbnormalCodes(_message.Message):
    __slots__ = ["robot_id", "scs"]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SCS_FIELD_NUMBER: _ClassVar[int]
    robot_id: int
    scs: _containers.RepeatedCompositeFieldContainer[StatusCode]
    def __init__(self, robot_id: _Optional[int] = ..., scs: _Optional[_Iterable[_Union[StatusCode, _Mapping]]] = ...) -> None: ...

class Abnormals(_message.Message):
    __slots__ = ["ss"]
    SS_FIELD_NUMBER: _ClassVar[int]
    ss: _containers.RepeatedCompositeFieldContainer[Status]
    def __init__(self, ss: _Optional[_Iterable[_Union[Status, _Mapping]]] = ...) -> None: ...

class Operation(_message.Message):
    __slots__ = ["buckets", "date", "distance", "end", "id", "mileage", "robot_id", "uptime"]
    class Mileage(_message.Message):
        __slots__ = ["end", "start"]
        END_FIELD_NUMBER: _ClassVar[int]
        START_FIELD_NUMBER: _ClassVar[int]
        end: int
        start: int
        def __init__(self, start: _Optional[int] = ..., end: _Optional[int] = ...) -> None: ...
    BUCKETS_FIELD_NUMBER: _ClassVar[int]
    DATE_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    END_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    MILEAGE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    UPTIME_FIELD_NUMBER: _ClassVar[int]
    buckets: _containers.RepeatedCompositeFieldContainer[_common_pb2.TimeBucket]
    date: int
    distance: int
    end: bool
    id: int
    mileage: Operation.Mileage
    robot_id: int
    uptime: int
    def __init__(self, robot_id: _Optional[int] = ..., id: _Optional[int] = ..., date: _Optional[int] = ..., mileage: _Optional[_Union[Operation.Mileage, _Mapping]] = ..., buckets: _Optional[_Iterable[_Union[_common_pb2.TimeBucket, _Mapping]]] = ..., distance: _Optional[int] = ..., uptime: _Optional[int] = ..., end: bool = ...) -> None: ...

class Operations(_message.Message):
    __slots__ = ["ops"]
    OPS_FIELD_NUMBER: _ClassVar[int]
    ops: _containers.RepeatedCompositeFieldContainer[Operation]
    def __init__(self, ops: _Optional[_Iterable[_Union[Operation, _Mapping]]] = ...) -> None: ...

class Status(_message.Message):
    __slots__ = ["bucket", "cons", "end", "id", "info", "robot_id"]
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    CONS_FIELD_NUMBER: _ClassVar[int]
    END_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    INFO_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    bucket: _common_pb2.TimeBucket
    cons: int
    end: bool
    id: int
    info: StatusCode
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., id: _Optional[int] = ..., info: _Optional[_Union[StatusCode, _Mapping]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., cons: _Optional[int] = ..., end: bool = ...) -> None: ...

class StatusCode(_message.Message):
    __slots__ = ["code", "level", "msg", "robot_id", "robot_task_id", "state", "time", "type"]
    class Level(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    CODE_FIELD_NUMBER: _ClassVar[int]
    LEVEL_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    ROBOT_TASK_ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    code: int
    kFault: StatusCode.Level
    kNormal: StatusCode.Level
    kTips: StatusCode.Level
    kWarn: StatusCode.Level
    level: StatusCode.Level
    msg: str
    robot_id: int
    robot_task_id: int
    state: _robot_pb2.State
    time: int
    type: _action_pb2.Type
    def __init__(self, robot_id: _Optional[int] = ..., code: _Optional[int] = ..., time: _Optional[int] = ..., msg: _Optional[str] = ..., robot_task_id: _Optional[int] = ..., state: _Optional[_Union[_robot_pb2.State, str]] = ..., type: _Optional[_Union[_action_pb2.Type, str]] = ..., level: _Optional[_Union[StatusCode.Level, str]] = ...) -> None: ...

class StatusCodes(_message.Message):
    __slots__ = ["robot_id", "scs"]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    SCS_FIELD_NUMBER: _ClassVar[int]
    robot_id: int
    scs: _containers.RepeatedCompositeFieldContainer[StatusCode]
    def __init__(self, robot_id: _Optional[int] = ..., scs: _Optional[_Iterable[_Union[StatusCode, _Mapping]]] = ...) -> None: ...

class Statuses(_message.Message):
    __slots__ = ["ss"]
    SS_FIELD_NUMBER: _ClassVar[int]
    ss: _containers.RepeatedCompositeFieldContainer[Status]
    def __init__(self, ss: _Optional[_Iterable[_Union[Status, _Mapping]]] = ...) -> None: ...

class Task(_message.Message):
    __slots__ = ["bucket", "cons", "dest", "end", "id", "robot_id", "state", "task_id"]
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    CONS_FIELD_NUMBER: _ClassVar[int]
    DEST_FIELD_NUMBER: _ClassVar[int]
    END_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    bucket: _common_pb2.TimeBucket
    cons: int
    dest: str
    end: bool
    id: int
    kCancel: Task.State
    kCompleted: Task.State
    kExecuting: Task.State
    kFailed: Task.State
    robot_id: int
    state: Task.State
    task_id: int
    def __init__(self, robot_id: _Optional[int] = ..., id: _Optional[int] = ..., task_id: _Optional[int] = ..., dest: _Optional[str] = ..., state: _Optional[_Union[Task.State, str]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., cons: _Optional[int] = ..., end: bool = ...) -> None: ...

class Tasks(_message.Message):
    __slots__ = ["ts"]
    TS_FIELD_NUMBER: _ClassVar[int]
    ts: _containers.RepeatedCompositeFieldContainer[Task]
    def __init__(self, ts: _Optional[_Iterable[_Union[Task, _Mapping]]] = ...) -> None: ...
