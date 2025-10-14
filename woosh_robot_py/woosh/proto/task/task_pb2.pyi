from woosh.proto.util import task_pb2 as _task_pb2
from woosh.proto.util import robot_pb2 as _robot_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kCancel: Order
kComplete: Order
kContinue: Order
kDelete: Order
kNext: Order
kOrderUndefined: Order
kPause: Order
kRedo: Order
kReset: Order
kSetCompleted: TaskSetState
kSetDeleted: TaskSetState
kSetExecuting: TaskSetState
kSetFailed: TaskSetState
kSetUnassigned: TaskSetState
kTaskSetStateUndefined: TaskSetState

class Task(_message.Message):
    __slots__ = ["base", "exec"]
    BASE_FIELD_NUMBER: _ClassVar[int]
    EXEC_FIELD_NUMBER: _ClassVar[int]
    base: TaskBase
    exec: TaskExec
    def __init__(self, base: _Optional[_Union[TaskBase, _Mapping]] = ..., exec: _Optional[_Union[TaskExec, _Mapping]] = ...) -> None: ...

class TaskBase(_message.Message):
    __slots__ = ["cannot_cancel", "custom", "direction", "id", "mark_no", "name", "type", "type_no", "wait_time"]
    CANNOT_CANCEL_FIELD_NUMBER: _ClassVar[int]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    MARK_NO_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    TYPE_NO_FIELD_NUMBER: _ClassVar[int]
    WAIT_TIME_FIELD_NUMBER: _ClassVar[int]
    cannot_cancel: bool
    custom: bytes
    direction: _task_pb2.Direction
    id: int
    mark_no: str
    name: str
    type: _task_pb2.Type
    type_no: int
    wait_time: int
    def __init__(self, id: _Optional[int] = ..., name: _Optional[str] = ..., mark_no: _Optional[str] = ..., type: _Optional[_Union[_task_pb2.Type, str]] = ..., direction: _Optional[_Union[_task_pb2.Direction, str]] = ..., type_no: _Optional[int] = ..., wait_time: _Optional[int] = ..., cannot_cancel: bool = ..., custom: _Optional[bytes] = ...) -> None: ...

class TaskExec(_message.Message):
    __slots__ = ["action_wait_id", "state"]
    ACTION_WAIT_ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    action_wait_id: int
    state: _task_pb2.State
    def __init__(self, state: _Optional[_Union[_task_pb2.State, str]] = ..., action_wait_id: _Optional[int] = ...) -> None: ...

class TaskSet(_message.Message):
    __slots__ = ["base", "exec", "tasks", "time"]
    BASE_FIELD_NUMBER: _ClassVar[int]
    EXEC_FIELD_NUMBER: _ClassVar[int]
    TASKS_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    base: TaskSetBase
    exec: TaskSetExec
    tasks: _containers.RepeatedCompositeFieldContainer[Task]
    time: TaskSetTime
    def __init__(self, base: _Optional[_Union[TaskSetBase, _Mapping]] = ..., exec: _Optional[_Union[TaskSetExec, _Mapping]] = ..., time: _Optional[_Union[TaskSetTime, _Mapping]] = ..., tasks: _Optional[_Iterable[_Union[Task, _Mapping]]] = ...) -> None: ...

class TaskSetBase(_message.Message):
    __slots__ = ["actuator", "adapter", "id", "name", "no", "priority", "robots", "route", "rtype", "type"]
    ACTUATOR_FIELD_NUMBER: _ClassVar[int]
    ADAPTER_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    PRIORITY_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    ROUTE_FIELD_NUMBER: _ClassVar[int]
    RTYPE_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    actuator: str
    adapter: str
    id: int
    name: str
    no: str
    priority: int
    robots: _containers.RepeatedScalarFieldContainer[int]
    route: str
    rtype: _robot_pb2.Type
    type: str
    def __init__(self, id: _Optional[int] = ..., name: _Optional[str] = ..., type: _Optional[str] = ..., route: _Optional[str] = ..., no: _Optional[str] = ..., adapter: _Optional[str] = ..., actuator: _Optional[str] = ..., rtype: _Optional[_Union[_robot_pb2.Type, str]] = ..., priority: _Optional[int] = ..., robots: _Optional[_Iterable[int]] = ...) -> None: ...

class TaskSetExec(_message.Message):
    __slots__ = ["cur_task_id", "pre_task_id", "robot", "state"]
    CUR_TASK_ID_FIELD_NUMBER: _ClassVar[int]
    PRE_TASK_ID_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    cur_task_id: int
    pre_task_id: int
    robot: int
    state: TaskSetState
    def __init__(self, robot: _Optional[int] = ..., state: _Optional[_Union[TaskSetState, str]] = ..., cur_task_id: _Optional[int] = ..., pre_task_id: _Optional[int] = ...) -> None: ...

class TaskSetList(_message.Message):
    __slots__ = ["qty", "tsets"]
    QTY_FIELD_NUMBER: _ClassVar[int]
    TSETS_FIELD_NUMBER: _ClassVar[int]
    qty: int
    tsets: _containers.RepeatedCompositeFieldContainer[TaskSet]
    def __init__(self, tsets: _Optional[_Iterable[_Union[TaskSet, _Mapping]]] = ..., qty: _Optional[int] = ...) -> None: ...

class TaskSetTime(_message.Message):
    __slots__ = ["end", "generate", "start"]
    END_FIELD_NUMBER: _ClassVar[int]
    GENERATE_FIELD_NUMBER: _ClassVar[int]
    START_FIELD_NUMBER: _ClassVar[int]
    end: int
    generate: int
    start: int
    def __init__(self, generate: _Optional[int] = ..., start: _Optional[int] = ..., end: _Optional[int] = ...) -> None: ...

class Order(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class TaskSetState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
