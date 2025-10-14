from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kCallStateUndefined: CallState
kConfigEx: State
kExecuting: CallState
kHardwareEx: State
kNormal: State
kPaired: CallState
kPause: CallState
kSoftwareEx: State
kStateUndefined: State
kTriggered: CallState
kUntriggered: CallState

class Call(_message.Message):
    __slots__ = ["event"]
    EVENT_FIELD_NUMBER: _ClassVar[int]
    event: CallEvent
    def __init__(self, event: _Optional[_Union[CallEvent, _Mapping]] = ...) -> None: ...

class CallEvent(_message.Message):
    __slots__ = ["key", "no", "type"]
    class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    KEY_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    kChanged: CallEvent.Type
    kDown: CallEvent.Type
    kPress: CallEvent.Type
    kUp: CallEvent.Type
    key: int
    no: str
    type: CallEvent.Type
    def __init__(self, no: _Optional[str] = ..., key: _Optional[int] = ..., type: _Optional[_Union[CallEvent.Type, str]] = ...) -> None: ...

class Callbox(_message.Message):
    __slots__ = ["key_num", "keys", "no", "online", "state"]
    class Key(_message.Message):
        __slots__ = ["id", "state", "task_id", "time", "tset_id"]
        ID_FIELD_NUMBER: _ClassVar[int]
        STATE_FIELD_NUMBER: _ClassVar[int]
        TASK_ID_FIELD_NUMBER: _ClassVar[int]
        TIME_FIELD_NUMBER: _ClassVar[int]
        TSET_ID_FIELD_NUMBER: _ClassVar[int]
        id: int
        state: CallState
        task_id: int
        time: int
        tset_id: int
        def __init__(self, id: _Optional[int] = ..., state: _Optional[_Union[CallState, str]] = ..., tset_id: _Optional[int] = ..., task_id: _Optional[int] = ..., time: _Optional[int] = ...) -> None: ...
    KEYS_FIELD_NUMBER: _ClassVar[int]
    KEY_NUM_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    ONLINE_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    key_num: int
    keys: _containers.RepeatedCompositeFieldContainer[Callbox.Key]
    no: str
    online: bool
    state: State
    def __init__(self, no: _Optional[str] = ..., key_num: _Optional[int] = ..., online: bool = ..., state: _Optional[_Union[State, str]] = ..., keys: _Optional[_Iterable[_Union[Callbox.Key, _Mapping]]] = ...) -> None: ...

class Callboxs(_message.Message):
    __slots__ = ["callboxs"]
    CALLBOXS_FIELD_NUMBER: _ClassVar[int]
    callboxs: _containers.RepeatedCompositeFieldContainer[Callbox]
    def __init__(self, callboxs: _Optional[_Iterable[_Union[Callbox, _Mapping]]] = ...) -> None: ...

class Caller(_message.Message):
    __slots__ = ["key", "no", "state", "task_id", "time", "tset_id"]
    KEY_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    TSET_ID_FIELD_NUMBER: _ClassVar[int]
    key: int
    no: str
    state: CallState
    task_id: int
    time: int
    tset_id: int
    def __init__(self, no: _Optional[str] = ..., key: _Optional[int] = ..., state: _Optional[_Union[CallState, str]] = ..., tset_id: _Optional[int] = ..., task_id: _Optional[int] = ..., time: _Optional[int] = ...) -> None: ...

class Callers(_message.Message):
    __slots__ = ["callers"]
    CALLERS_FIELD_NUMBER: _ClassVar[int]
    callers: _containers.RepeatedCompositeFieldContainer[Caller]
    def __init__(self, callers: _Optional[_Iterable[_Union[Caller, _Mapping]]] = ...) -> None: ...

class Offline(_message.Message):
    __slots__ = ["no"]
    NO_FIELD_NUMBER: _ClassVar[int]
    no: str
    def __init__(self, no: _Optional[str] = ...) -> None: ...

class Online(_message.Message):
    __slots__ = ["callbox"]
    CALLBOX_FIELD_NUMBER: _ClassVar[int]
    callbox: Callbox
    def __init__(self, callbox: _Optional[_Union[Callbox, _Mapping]] = ...) -> None: ...

class WISE(_message.Message):
    __slots__ = ["ip", "models", "no", "port", "slave"]
    class Model(_message.Message):
        __slots__ = ["coil", "type2"]
        class Coil(_message.Message):
            __slots__ = ["base_addr", "base_key", "length"]
            BASE_ADDR_FIELD_NUMBER: _ClassVar[int]
            BASE_KEY_FIELD_NUMBER: _ClassVar[int]
            LENGTH_FIELD_NUMBER: _ClassVar[int]
            base_addr: int
            base_key: int
            length: int
            def __init__(self, base_addr: _Optional[int] = ..., base_key: _Optional[int] = ..., length: _Optional[int] = ...) -> None: ...
        class ModelType2(_message.Message):
            __slots__ = ["base_addr", "base_key", "length", "type"]
            class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
                __slots__ = []
            BASE_ADDR_FIELD_NUMBER: _ClassVar[int]
            BASE_KEY_FIELD_NUMBER: _ClassVar[int]
            LENGTH_FIELD_NUMBER: _ClassVar[int]
            TYPE_FIELD_NUMBER: _ClassVar[int]
            base_addr: int
            base_key: int
            kDiscreteInput: WISE.Model.ModelType2.Type
            kDiscreteOutput: WISE.Model.ModelType2.Type
            kHoldingRegister: WISE.Model.ModelType2.Type
            kInputRegister: WISE.Model.ModelType2.Type
            length: int
            type: WISE.Model.ModelType2.Type
            def __init__(self, base_addr: _Optional[int] = ..., length: _Optional[int] = ..., type: _Optional[_Union[WISE.Model.ModelType2.Type, str]] = ..., base_key: _Optional[int] = ...) -> None: ...
        COIL_FIELD_NUMBER: _ClassVar[int]
        TYPE2_FIELD_NUMBER: _ClassVar[int]
        coil: WISE.Model.Coil
        type2: WISE.Model.ModelType2
        def __init__(self, coil: _Optional[_Union[WISE.Model.Coil, _Mapping]] = ..., type2: _Optional[_Union[WISE.Model.ModelType2, _Mapping]] = ...) -> None: ...
    IP_FIELD_NUMBER: _ClassVar[int]
    MODELS_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    PORT_FIELD_NUMBER: _ClassVar[int]
    SLAVE_FIELD_NUMBER: _ClassVar[int]
    ip: str
    models: _containers.RepeatedCompositeFieldContainer[WISE.Model]
    no: str
    port: int
    slave: int
    def __init__(self, ip: _Optional[str] = ..., port: _Optional[int] = ..., slave: _Optional[int] = ..., models: _Optional[_Iterable[_Union[WISE.Model, _Mapping]]] = ..., no: _Optional[str] = ...) -> None: ...

class WISES(_message.Message):
    __slots__ = ["wises"]
    WISES_FIELD_NUMBER: _ClassVar[int]
    wises: _containers.RepeatedCompositeFieldContainer[WISE]
    def __init__(self, wises: _Optional[_Iterable[_Union[WISE, _Mapping]]] = ...) -> None: ...

class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class CallState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
