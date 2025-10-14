from woosh.proto.util import common_pb2 as _common_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ChargeSettings(_message.Message):
    __slots__ = ["order", "settings"]
    class Order(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class Setting(_message.Message):
        __slots__ = ["full_power", "guard_power", "level", "low_power", "time", "work_power"]
        FULL_POWER_FIELD_NUMBER: _ClassVar[int]
        GUARD_POWER_FIELD_NUMBER: _ClassVar[int]
        LEVEL_FIELD_NUMBER: _ClassVar[int]
        LOW_POWER_FIELD_NUMBER: _ClassVar[int]
        TIME_FIELD_NUMBER: _ClassVar[int]
        WORK_POWER_FIELD_NUMBER: _ClassVar[int]
        full_power: int
        guard_power: int
        level: int
        low_power: int
        time: int
        work_power: int
        def __init__(self, level: _Optional[int] = ..., guard_power: _Optional[int] = ..., low_power: _Optional[int] = ..., work_power: _Optional[int] = ..., full_power: _Optional[int] = ..., time: _Optional[int] = ...) -> None: ...
    ORDER_FIELD_NUMBER: _ClassVar[int]
    SETTINGS_FIELD_NUMBER: _ClassVar[int]
    kDel: ChargeSettings.Order
    kGet: ChargeSettings.Order
    kPut: ChargeSettings.Order
    order: ChargeSettings.Order
    settings: _containers.RepeatedCompositeFieldContainer[ChargeSettings.Setting]
    def __init__(self, settings: _Optional[_Iterable[_Union[ChargeSettings.Setting, _Mapping]]] = ..., order: _Optional[_Union[ChargeSettings.Order, str]] = ...) -> None: ...

class GetPos(_message.Message):
    __slots__ = ["no", "pose"]
    NO_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    no: str
    pose: _common_pb2.Pose
    def __init__(self, no: _Optional[str] = ..., pose: _Optional[_Union[_common_pb2.Pose, _Mapping]] = ...) -> None: ...

class GotoCharge(_message.Message):
    __slots__ = ["no", "robot"]
    NO_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    no: str
    robot: int
    def __init__(self, robot: _Optional[int] = ..., no: _Optional[str] = ...) -> None: ...

class PacAccount(_message.Message):
    __slots__ = ["id", "no", "robot", "state", "tset_id", "type"]
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ID_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    TSET_ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    id: int
    kCharger: PacAccount.Type
    kIdle: PacAccount.State
    kOccupy: PacAccount.State
    kPark: PacAccount.Type
    kTransiting: PacAccount.State
    no: str
    robot: int
    state: PacAccount.State
    tset_id: int
    type: PacAccount.Type
    def __init__(self, id: _Optional[int] = ..., no: _Optional[str] = ..., type: _Optional[_Union[PacAccount.Type, str]] = ..., state: _Optional[_Union[PacAccount.State, str]] = ..., robot: _Optional[int] = ..., tset_id: _Optional[int] = ...) -> None: ...

class PacAccountList(_message.Message):
    __slots__ = ["accounts"]
    ACCOUNTS_FIELD_NUMBER: _ClassVar[int]
    accounts: _containers.RepeatedCompositeFieldContainer[PacAccount]
    def __init__(self, accounts: _Optional[_Iterable[_Union[PacAccount, _Mapping]]] = ...) -> None: ...

class PrintLevel(_message.Message):
    __slots__ = ["level"]
    class Level(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    LEVEL_FIELD_NUMBER: _ClassVar[int]
    kCritical: PrintLevel.Level
    kDebug: PrintLevel.Level
    kErr: PrintLevel.Level
    kInfo: PrintLevel.Level
    kOff: PrintLevel.Level
    kTrace: PrintLevel.Level
    kWarn: PrintLevel.Level
    level: PrintLevel.Level
    def __init__(self, level: _Optional[_Union[PrintLevel.Level, str]] = ...) -> None: ...

class Scene(_message.Message):
    __slots__ = ["maps", "scene", "version"]
    class Map(_message.Message):
        __slots__ = ["end", "id", "name", "origin", "resolution"]
        END_FIELD_NUMBER: _ClassVar[int]
        ID_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        ORIGIN_FIELD_NUMBER: _ClassVar[int]
        RESOLUTION_FIELD_NUMBER: _ClassVar[int]
        end: _common_pb2.Pose2D
        id: int
        name: str
        origin: _common_pb2.Pose2D
        resolution: float
        def __init__(self, id: _Optional[int] = ..., name: _Optional[str] = ..., resolution: _Optional[float] = ..., origin: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., end: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...
    MAPS_FIELD_NUMBER: _ClassVar[int]
    SCENE_FIELD_NUMBER: _ClassVar[int]
    VERSION_FIELD_NUMBER: _ClassVar[int]
    maps: _containers.RepeatedCompositeFieldContainer[Scene.Map]
    scene: str
    version: int
    def __init__(self, scene: _Optional[str] = ..., version: _Optional[int] = ..., maps: _Optional[_Iterable[_Union[Scene.Map, _Mapping]]] = ...) -> None: ...

class SceneSettings(_message.Message):
    __slots__ = ["name"]
    NAME_FIELD_NUMBER: _ClassVar[int]
    name: str
    def __init__(self, name: _Optional[str] = ...) -> None: ...

class SwitchScene(_message.Message):
    __slots__ = ["name"]
    NAME_FIELD_NUMBER: _ClassVar[int]
    name: str
    def __init__(self, name: _Optional[str] = ...) -> None: ...
