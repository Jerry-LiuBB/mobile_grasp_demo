from woosh.proto.task import task_pb2 as _task_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AddTask(_message.Message):
    __slots__ = ["tset"]
    TSET_FIELD_NUMBER: _ClassVar[int]
    tset: WooshTaskSet
    def __init__(self, tset: _Optional[_Union[WooshTaskSet, _Mapping]] = ...) -> None: ...

class CallInfo(_message.Message):
    __slots__ = ["events", "guard", "type"]
    class CallType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class CallEvent(_message.Message):
        __slots__ = ["caller", "guard", "release"]
        CALLER_FIELD_NUMBER: _ClassVar[int]
        GUARD_FIELD_NUMBER: _ClassVar[int]
        RELEASE_FIELD_NUMBER: _ClassVar[int]
        caller: CallInfo.Caller
        guard: CallInfo.CallGuard
        release: int
        def __init__(self, caller: _Optional[_Union[CallInfo.Caller, _Mapping]] = ..., guard: _Optional[_Union[CallInfo.CallGuard, _Mapping]] = ..., release: _Optional[int] = ...) -> None: ...
    class CallGuard(_message.Message):
        __slots__ = ["callers", "type"]
        CALLERS_FIELD_NUMBER: _ClassVar[int]
        TYPE_FIELD_NUMBER: _ClassVar[int]
        callers: _containers.RepeatedCompositeFieldContainer[CallInfo.Caller]
        type: CallInfo.CallType
        def __init__(self, type: _Optional[_Union[CallInfo.CallType, str]] = ..., callers: _Optional[_Iterable[_Union[CallInfo.Caller, _Mapping]]] = ...) -> None: ...
    class Caller(_message.Message):
        __slots__ = ["key_id", "mac_addr", "triggered"]
        KEY_ID_FIELD_NUMBER: _ClassVar[int]
        MAC_ADDR_FIELD_NUMBER: _ClassVar[int]
        TRIGGERED_FIELD_NUMBER: _ClassVar[int]
        key_id: int
        mac_addr: str
        triggered: bool
        def __init__(self, mac_addr: _Optional[str] = ..., key_id: _Optional[int] = ..., triggered: bool = ...) -> None: ...
    EVENTS_FIELD_NUMBER: _ClassVar[int]
    GUARD_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    events: _containers.RepeatedCompositeFieldContainer[CallInfo.CallEvent]
    guard: CallInfo.CallGuard
    kAnd: CallInfo.CallType
    kCallTypeUndefined: CallInfo.CallType
    kMsger: CallInfo.CallType
    kOr: CallInfo.CallType
    type: CallInfo.CallType
    def __init__(self, type: _Optional[_Union[CallInfo.CallType, str]] = ..., events: _Optional[_Iterable[_Union[CallInfo.CallEvent, _Mapping]]] = ..., guard: _Optional[_Union[CallInfo.CallGuard, _Mapping]] = ...) -> None: ...

class CallTask(_message.Message):
    __slots__ = ["call_info", "task_set"]
    CALL_INFO_FIELD_NUMBER: _ClassVar[int]
    TASK_SET_FIELD_NUMBER: _ClassVar[int]
    call_info: CallInfo
    task_set: WooshTaskSet
    def __init__(self, call_info: _Optional[_Union[CallInfo, _Mapping]] = ..., task_set: _Optional[_Union[WooshTaskSet, _Mapping]] = ...) -> None: ...

class CallTasks(_message.Message):
    __slots__ = ["call_tasks"]
    CALL_TASKS_FIELD_NUMBER: _ClassVar[int]
    call_tasks: _containers.RepeatedCompositeFieldContainer[CallTask]
    def __init__(self, call_tasks: _Optional[_Iterable[_Union[CallTask, _Mapping]]] = ...) -> None: ...

class InsertTask(_message.Message):
    __slots__ = ["task", "tset_id"]
    TASK_FIELD_NUMBER: _ClassVar[int]
    TSET_ID_FIELD_NUMBER: _ClassVar[int]
    task: WooshTask
    tset_id: int
    def __init__(self, tset_id: _Optional[int] = ..., task: _Optional[_Union[WooshTask, _Mapping]] = ...) -> None: ...

class RepeatTask(_message.Message):
    __slots__ = ["addr", "key", "order", "task_set", "times"]
    class Order(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ADDR_FIELD_NUMBER: _ClassVar[int]
    KEY_FIELD_NUMBER: _ClassVar[int]
    ORDER_FIELD_NUMBER: _ClassVar[int]
    TASK_SET_FIELD_NUMBER: _ClassVar[int]
    TIMES_FIELD_NUMBER: _ClassVar[int]
    addr: int
    kDel: RepeatTask.Order
    kExec: RepeatTask.Order
    kOrderUndefined: RepeatTask.Order
    kPut: RepeatTask.Order
    key: int
    order: RepeatTask.Order
    task_set: WooshTaskSet
    times: int
    def __init__(self, times: _Optional[int] = ..., task_set: _Optional[_Union[WooshTaskSet, _Mapping]] = ..., addr: _Optional[int] = ..., key: _Optional[int] = ..., order: _Optional[_Union[RepeatTask.Order, str]] = ...) -> None: ...

class RepeatTasks(_message.Message):
    __slots__ = ["repeat_tasks"]
    REPEAT_TASKS_FIELD_NUMBER: _ClassVar[int]
    repeat_tasks: _containers.RepeatedCompositeFieldContainer[RepeatTask]
    def __init__(self, repeat_tasks: _Optional[_Iterable[_Union[RepeatTask, _Mapping]]] = ...) -> None: ...

class TaskInfo(_message.Message):
    __slots__ = ["call_tasks", "repeat_tasks"]
    CALL_TASKS_FIELD_NUMBER: _ClassVar[int]
    REPEAT_TASKS_FIELD_NUMBER: _ClassVar[int]
    call_tasks: _containers.RepeatedCompositeFieldContainer[CallTask]
    repeat_tasks: _containers.RepeatedCompositeFieldContainer[RepeatTask]
    def __init__(self, repeat_tasks: _Optional[_Iterable[_Union[RepeatTask, _Mapping]]] = ..., call_tasks: _Optional[_Iterable[_Union[CallTask, _Mapping]]] = ...) -> None: ...

class WooshTask(_message.Message):
    __slots__ = ["base", "consta", "custom", "exec"]
    class Consta(_message.Message):
        __slots__ = ["can_skip", "cancel_end", "cancel_next", "cancel_skip", "fail_end", "fail_next", "fail_redo", "fail_skip"]
        CANCEL_END_FIELD_NUMBER: _ClassVar[int]
        CANCEL_NEXT_FIELD_NUMBER: _ClassVar[int]
        CANCEL_SKIP_FIELD_NUMBER: _ClassVar[int]
        CAN_SKIP_FIELD_NUMBER: _ClassVar[int]
        FAIL_END_FIELD_NUMBER: _ClassVar[int]
        FAIL_NEXT_FIELD_NUMBER: _ClassVar[int]
        FAIL_REDO_FIELD_NUMBER: _ClassVar[int]
        FAIL_SKIP_FIELD_NUMBER: _ClassVar[int]
        can_skip: bool
        cancel_end: bool
        cancel_next: bool
        cancel_skip: bool
        fail_end: bool
        fail_next: bool
        fail_redo: bool
        fail_skip: bool
        def __init__(self, can_skip: bool = ..., fail_redo: bool = ..., fail_next: bool = ..., cancel_next: bool = ..., cancel_end: bool = ..., fail_end: bool = ..., cancel_skip: bool = ..., fail_skip: bool = ...) -> None: ...
    class Custom(_message.Message):
        __slots__ = ["executable", "priority"]
        EXECUTABLE_FIELD_NUMBER: _ClassVar[int]
        PRIORITY_FIELD_NUMBER: _ClassVar[int]
        executable: bool
        priority: int
        def __init__(self, priority: _Optional[int] = ..., executable: bool = ...) -> None: ...
    BASE_FIELD_NUMBER: _ClassVar[int]
    CONSTA_FIELD_NUMBER: _ClassVar[int]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    EXEC_FIELD_NUMBER: _ClassVar[int]
    base: _task_pb2.TaskBase
    consta: WooshTask.Consta
    custom: WooshTask.Custom
    exec: _task_pb2.TaskExec
    def __init__(self, base: _Optional[_Union[_task_pb2.TaskBase, _Mapping]] = ..., exec: _Optional[_Union[_task_pb2.TaskExec, _Mapping]] = ..., custom: _Optional[_Union[WooshTask.Custom, _Mapping]] = ..., consta: _Optional[_Union[WooshTask.Consta, _Mapping]] = ...) -> None: ...

class WooshTaskSet(_message.Message):
    __slots__ = ["base", "consta", "custom", "exec", "tasks", "time"]
    class Consta(_message.Message):
        __slots__ = ["token"]
        TOKEN_FIELD_NUMBER: _ClassVar[int]
        token: str
        def __init__(self, token: _Optional[str] = ...) -> None: ...
    class Custom(_message.Message):
        __slots__ = ["auto_complete"]
        AUTO_COMPLETE_FIELD_NUMBER: _ClassVar[int]
        auto_complete: bool
        def __init__(self, auto_complete: bool = ...) -> None: ...
    BASE_FIELD_NUMBER: _ClassVar[int]
    CONSTA_FIELD_NUMBER: _ClassVar[int]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    EXEC_FIELD_NUMBER: _ClassVar[int]
    TASKS_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    base: _task_pb2.TaskSetBase
    consta: WooshTaskSet.Consta
    custom: WooshTaskSet.Custom
    exec: _task_pb2.TaskSetExec
    tasks: _containers.RepeatedCompositeFieldContainer[WooshTask]
    time: _task_pb2.TaskSetTime
    def __init__(self, base: _Optional[_Union[_task_pb2.TaskSetBase, _Mapping]] = ..., exec: _Optional[_Union[_task_pb2.TaskSetExec, _Mapping]] = ..., time: _Optional[_Union[_task_pb2.TaskSetTime, _Mapping]] = ..., tasks: _Optional[_Iterable[_Union[WooshTask, _Mapping]]] = ..., custom: _Optional[_Union[WooshTaskSet.Custom, _Mapping]] = ..., consta: _Optional[_Union[WooshTaskSet.Consta, _Mapping]] = ...) -> None: ...
