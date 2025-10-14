from woosh.proto.util import common_pb2 as _common_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kAccurate: ArrType
kArrTypeUndefined: ArrType
kAvoid: Mode
kMagnetic: Mode
kModeUndefined: Mode
kNarrow: Mode
kNavWait: Mode
kOvertime: Mode
kQrcode: Mode
kTimeout: Mode
kVague: ArrType

class LocalPath(_message.Message):
    __slots__ = ["points", "time"]
    class Point(_message.Message):
        __slots__ = ["expend", "pose"]
        EXPEND_FIELD_NUMBER: _ClassVar[int]
        POSE_FIELD_NUMBER: _ClassVar[int]
        expend: int
        pose: _common_pb2.Pose2D
        def __init__(self, pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., expend: _Optional[int] = ...) -> None: ...
    POINTS_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[LocalPath.Point]
    time: int
    def __init__(self, points: _Optional[_Iterable[_Union[LocalPath.Point, _Mapping]]] = ..., time: _Optional[int] = ...) -> None: ...

class ModeSetting(_message.Message):
    __slots__ = ["capacity", "max_speed", "mode", "permitted_passage", "type", "wait_timeout"]
    CAPACITY_FIELD_NUMBER: _ClassVar[int]
    MAX_SPEED_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    PERMITTED_PASSAGE_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    WAIT_TIMEOUT_FIELD_NUMBER: _ClassVar[int]
    capacity: int
    max_speed: float
    mode: Mode
    permitted_passage: bool
    type: ArrType
    wait_timeout: int
    def __init__(self, type: _Optional[_Union[ArrType, str]] = ..., mode: _Optional[_Union[Mode, str]] = ..., wait_timeout: _Optional[int] = ..., max_speed: _Optional[float] = ..., permitted_passage: bool = ..., capacity: _Optional[int] = ...) -> None: ...

class Path(_message.Message):
    __slots__ = ["poses"]
    POSES_FIELD_NUMBER: _ClassVar[int]
    poses: _containers.RepeatedCompositeFieldContainer[_common_pb2.Pose2D]
    def __init__(self, poses: _Optional[_Iterable[_Union[_common_pb2.Pose2D, _Mapping]]] = ...) -> None: ...

class PlanPath(_message.Message):
    __slots__ = ["dest_map_id", "map_id", "optimal", "path", "target", "wormhole_id"]
    class Optimal(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    DEST_MAP_ID_FIELD_NUMBER: _ClassVar[int]
    MAP_ID_FIELD_NUMBER: _ClassVar[int]
    OPTIMAL_FIELD_NUMBER: _ClassVar[int]
    PATH_FIELD_NUMBER: _ClassVar[int]
    TARGET_FIELD_NUMBER: _ClassVar[int]
    WORMHOLE_ID_FIELD_NUMBER: _ClassVar[int]
    dest_map_id: int
    kDestOptimal: PlanPath.Optimal
    kOptimal: PlanPath.Optimal
    kOptimalUndefined: PlanPath.Optimal
    kStrict: PlanPath.Optimal
    map_id: int
    optimal: PlanPath.Optimal
    path: Path
    target: _common_pb2.Pose2D
    wormhole_id: int
    def __init__(self, path: _Optional[_Union[Path, _Mapping]] = ..., map_id: _Optional[int] = ..., wormhole_id: _Optional[int] = ..., dest_map_id: _Optional[int] = ..., target: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., optimal: _Optional[_Union[PlanPath.Optimal, str]] = ...) -> None: ...

class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class ArrType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
