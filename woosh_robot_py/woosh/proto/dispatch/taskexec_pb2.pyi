from woosh.proto.util import task_pb2 as _task_pb2
from woosh.proto.task import task_pb2 as _task_pb2_1
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AddTask(_message.Message):
    __slots__ = ["tset"]
    TSET_FIELD_NUMBER: _ClassVar[int]
    tset: _task_pb2_1.TaskSet
    def __init__(self, tset: _Optional[_Union[_task_pb2_1.TaskSet, _Mapping]] = ...) -> None: ...

class ExecPreTask(_message.Message):
    __slots__ = ["order", "repeat_times", "robot", "task_set_id", "tset_id"]
    class Order(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ORDER_FIELD_NUMBER: _ClassVar[int]
    REPEAT_TIMES_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    TASK_SET_ID_FIELD_NUMBER: _ClassVar[int]
    TSET_ID_FIELD_NUMBER: _ClassVar[int]
    kAdd: ExecPreTask.Order
    kDel: ExecPreTask.Order
    order: ExecPreTask.Order
    repeat_times: int
    robot: int
    task_set_id: int
    tset_id: int
    def __init__(self, task_set_id: _Optional[int] = ..., robot: _Optional[int] = ..., repeat_times: _Optional[int] = ..., order: _Optional[_Union[ExecPreTask.Order, str]] = ..., tset_id: _Optional[int] = ...) -> None: ...

class ExecPreTasks(_message.Message):
    __slots__ = ["tasks"]
    TASKS_FIELD_NUMBER: _ClassVar[int]
    tasks: _containers.RepeatedCompositeFieldContainer[ExecPreTask]
    def __init__(self, tasks: _Optional[_Iterable[_Union[ExecPreTask, _Mapping]]] = ...) -> None: ...

class FindTask(_message.Message):
    __slots__ = ["id", "only_external", "page", "robot", "route", "sort", "state", "time"]
    class ToSort(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class State(_message.Message):
        __slots__ = ["gte", "lte"]
        GTE_FIELD_NUMBER: _ClassVar[int]
        LTE_FIELD_NUMBER: _ClassVar[int]
        gte: _task_pb2_1.TaskSetState
        lte: _task_pb2_1.TaskSetState
        def __init__(self, gte: _Optional[_Union[_task_pb2_1.TaskSetState, str]] = ..., lte: _Optional[_Union[_task_pb2_1.TaskSetState, str]] = ...) -> None: ...
    class Time(_message.Message):
        __slots__ = ["gte", "lte"]
        GTE_FIELD_NUMBER: _ClassVar[int]
        LTE_FIELD_NUMBER: _ClassVar[int]
        gte: int
        lte: int
        def __init__(self, gte: _Optional[int] = ..., lte: _Optional[int] = ...) -> None: ...
    ID_FIELD_NUMBER: _ClassVar[int]
    ONLY_EXTERNAL_FIELD_NUMBER: _ClassVar[int]
    PAGE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    ROUTE_FIELD_NUMBER: _ClassVar[int]
    SORT_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    id: int
    kTaskSetID_Asc: FindTask.ToSort
    kTaskSetID_Desc: FindTask.ToSort
    kTimeEnd_Asc: FindTask.ToSort
    kTimeEnd_Desc: FindTask.ToSort
    only_external: bool
    page: int
    robot: int
    route: str
    sort: FindTask.ToSort
    state: FindTask.State
    time: FindTask.Time
    def __init__(self, id: _Optional[int] = ..., state: _Optional[_Union[FindTask.State, _Mapping]] = ..., time: _Optional[_Union[FindTask.Time, _Mapping]] = ..., sort: _Optional[_Union[FindTask.ToSort, str]] = ..., robot: _Optional[int] = ..., route: _Optional[str] = ..., only_external: bool = ..., page: _Optional[int] = ...) -> None: ...

class StickTask(_message.Message):
    __slots__ = ["id", "unstick"]
    ID_FIELD_NUMBER: _ClassVar[int]
    UNSTICK_FIELD_NUMBER: _ClassVar[int]
    id: int
    unstick: bool
    def __init__(self, id: _Optional[int] = ..., unstick: bool = ...) -> None: ...

class TaskOrder(_message.Message):
    __slots__ = ["id", "order", "robot", "task_id"]
    ID_FIELD_NUMBER: _ClassVar[int]
    ORDER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    id: int
    order: _task_pb2_1.Order
    robot: int
    task_id: int
    def __init__(self, id: _Optional[int] = ..., robot: _Optional[int] = ..., task_id: _Optional[int] = ..., order: _Optional[_Union[_task_pb2_1.Order, str]] = ...) -> None: ...

class TaskSetState(_message.Message):
    __slots__ = ["tset"]
    TSET_FIELD_NUMBER: _ClassVar[int]
    tset: _task_pb2_1.TaskSet
    def __init__(self, tset: _Optional[_Union[_task_pb2_1.TaskSet, _Mapping]] = ...) -> None: ...

class TaskState(_message.Message):
    __slots__ = ["action_wait_id", "id", "state", "task_id"]
    ACTION_WAIT_ID_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    action_wait_id: int
    id: int
    state: _task_pb2.State
    task_id: int
    def __init__(self, id: _Optional[int] = ..., task_id: _Optional[int] = ..., state: _Optional[_Union[_task_pb2.State, str]] = ..., action_wait_id: _Optional[int] = ...) -> None: ...
