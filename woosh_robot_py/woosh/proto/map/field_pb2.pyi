from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.util import navigation_pb2 as _navigation_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kMutex: GuideType
kSegment: GuideType
kTurn: GuideType
kWait: GuideType
kWormhole: GuideType

class AutoDoorField(_message.Message):
    __slots__ = ["custom", "guide_points", "nav_filed", "offset", "out", "radius"]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    IN_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    OUT_FIELD_NUMBER: _ClassVar[int]
    RADIUS_FIELD_NUMBER: _ClassVar[int]
    custom: bytes
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    nav_filed: NavField
    offset: float
    out: GuidePose
    radius: float
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., out: _Optional[_Union[GuidePose, _Mapping]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ..., radius: _Optional[float] = ..., offset: _Optional[float] = ..., custom: _Optional[bytes] = ..., **kwargs) -> None: ...

class BidirectionWay(_message.Message):
    __slots__ = ["guide_points", "nav_filed"]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ...) -> None: ...

class Crossing(_message.Message):
    __slots__ = ["guide_points", "nav_filed"]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ...) -> None: ...

class FieldInfo(_message.Message):
    __slots__ = ["autodoor_fields", "bidirection_ways", "crossings", "fire_fields", "guide_fields", "guide_markers", "locate_fields", "narrow_passages", "odom_fields", "one_ways", "ramp_fields", "single_ways", "speed_limit_fields", "warn_fields"]
    AUTODOOR_FIELDS_FIELD_NUMBER: _ClassVar[int]
    BIDIRECTION_WAYS_FIELD_NUMBER: _ClassVar[int]
    CROSSINGS_FIELD_NUMBER: _ClassVar[int]
    FIRE_FIELDS_FIELD_NUMBER: _ClassVar[int]
    GUIDE_FIELDS_FIELD_NUMBER: _ClassVar[int]
    GUIDE_MARKERS_FIELD_NUMBER: _ClassVar[int]
    LOCATE_FIELDS_FIELD_NUMBER: _ClassVar[int]
    NARROW_PASSAGES_FIELD_NUMBER: _ClassVar[int]
    ODOM_FIELDS_FIELD_NUMBER: _ClassVar[int]
    ONE_WAYS_FIELD_NUMBER: _ClassVar[int]
    RAMP_FIELDS_FIELD_NUMBER: _ClassVar[int]
    SINGLE_WAYS_FIELD_NUMBER: _ClassVar[int]
    SPEED_LIMIT_FIELDS_FIELD_NUMBER: _ClassVar[int]
    WARN_FIELDS_FIELD_NUMBER: _ClassVar[int]
    autodoor_fields: _containers.RepeatedCompositeFieldContainer[AutoDoorField]
    bidirection_ways: _containers.RepeatedCompositeFieldContainer[BidirectionWay]
    crossings: _containers.RepeatedCompositeFieldContainer[Crossing]
    fire_fields: _containers.RepeatedCompositeFieldContainer[FireField]
    guide_fields: _containers.RepeatedCompositeFieldContainer[GuideField]
    guide_markers: _containers.RepeatedCompositeFieldContainer[GuideMarker]
    locate_fields: _containers.RepeatedCompositeFieldContainer[LocateField]
    narrow_passages: _containers.RepeatedCompositeFieldContainer[NarrowPassage]
    odom_fields: _containers.RepeatedCompositeFieldContainer[OdomField]
    one_ways: _containers.RepeatedCompositeFieldContainer[OneWay]
    ramp_fields: _containers.RepeatedCompositeFieldContainer[RampField]
    single_ways: _containers.RepeatedCompositeFieldContainer[SingleWay]
    speed_limit_fields: _containers.RepeatedCompositeFieldContainer[SpeedLimitField]
    warn_fields: _containers.RepeatedCompositeFieldContainer[WarnField]
    def __init__(self, guide_markers: _Optional[_Iterable[_Union[GuideMarker, _Mapping]]] = ..., bidirection_ways: _Optional[_Iterable[_Union[BidirectionWay, _Mapping]]] = ..., single_ways: _Optional[_Iterable[_Union[SingleWay, _Mapping]]] = ..., one_ways: _Optional[_Iterable[_Union[OneWay, _Mapping]]] = ..., crossings: _Optional[_Iterable[_Union[Crossing, _Mapping]]] = ..., narrow_passages: _Optional[_Iterable[_Union[NarrowPassage, _Mapping]]] = ..., locate_fields: _Optional[_Iterable[_Union[LocateField, _Mapping]]] = ..., ramp_fields: _Optional[_Iterable[_Union[RampField, _Mapping]]] = ..., odom_fields: _Optional[_Iterable[_Union[OdomField, _Mapping]]] = ..., autodoor_fields: _Optional[_Iterable[_Union[AutoDoorField, _Mapping]]] = ..., speed_limit_fields: _Optional[_Iterable[_Union[SpeedLimitField, _Mapping]]] = ..., fire_fields: _Optional[_Iterable[_Union[FireField, _Mapping]]] = ..., warn_fields: _Optional[_Iterable[_Union[WarnField, _Mapping]]] = ..., guide_fields: _Optional[_Iterable[_Union[GuideField, _Mapping]]] = ...) -> None: ...

class FireField(_message.Message):
    __slots__ = ["nav_filed"]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ...) -> None: ...

class GuideField(_message.Message):
    __slots__ = ["guide_points", "nav_filed", "type"]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    nav_filed: NavField
    type: GuideType
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ..., type: _Optional[_Union[GuideType, str]] = ...) -> None: ...

class GuideMarker(_message.Message):
    __slots__ = ["description", "id", "no", "pos", "type"]
    DESCRIPTION_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    POS_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    description: str
    id: int
    no: str
    pos: _common_pb2.Pose2D
    type: GuideType
    def __init__(self, id: _Optional[int] = ..., pos: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., type: _Optional[_Union[GuideType, str]] = ..., no: _Optional[str] = ..., description: _Optional[str] = ...) -> None: ...

class GuidePoint(_message.Message):
    __slots__ = ["id", "point"]
    ID_FIELD_NUMBER: _ClassVar[int]
    POINT_FIELD_NUMBER: _ClassVar[int]
    id: int
    point: _common_pb2.Point
    def __init__(self, id: _Optional[int] = ..., point: _Optional[_Union[_common_pb2.Point, _Mapping]] = ...) -> None: ...

class GuidePose(_message.Message):
    __slots__ = ["id", "pose"]
    ID_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    id: int
    pose: _common_pb2.Pose2D
    def __init__(self, id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class LocateField(_message.Message):
    __slots__ = ["nav_filed"]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ...) -> None: ...

class NarrowPassage(_message.Message):
    __slots__ = ["guide_points", "nav_filed", "offset", "out", "radius"]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    IN_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    OUT_FIELD_NUMBER: _ClassVar[int]
    RADIUS_FIELD_NUMBER: _ClassVar[int]
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    nav_filed: NavField
    offset: float
    out: GuidePose
    radius: float
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., out: _Optional[_Union[GuidePose, _Mapping]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ..., radius: _Optional[float] = ..., offset: _Optional[float] = ..., **kwargs) -> None: ...

class NavField(_message.Message):
    __slots__ = ["capacity", "guide_markers", "id", "max_speed", "nav_mode", "vertexs", "wait_time_out"]
    CAPACITY_FIELD_NUMBER: _ClassVar[int]
    GUIDE_MARKERS_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    MAX_SPEED_FIELD_NUMBER: _ClassVar[int]
    NAV_MODE_FIELD_NUMBER: _ClassVar[int]
    VERTEXS_FIELD_NUMBER: _ClassVar[int]
    WAIT_TIME_OUT_FIELD_NUMBER: _ClassVar[int]
    capacity: int
    guide_markers: _containers.RepeatedScalarFieldContainer[int]
    id: int
    max_speed: float
    nav_mode: _navigation_pb2.Mode
    vertexs: _containers.RepeatedCompositeFieldContainer[_common_pb2.Point]
    wait_time_out: float
    def __init__(self, id: _Optional[int] = ..., nav_mode: _Optional[_Union[_navigation_pb2.Mode, str]] = ..., wait_time_out: _Optional[float] = ..., max_speed: _Optional[float] = ..., capacity: _Optional[int] = ..., vertexs: _Optional[_Iterable[_Union[_common_pb2.Point, _Mapping]]] = ..., guide_markers: _Optional[_Iterable[int]] = ...) -> None: ...

class OdomField(_message.Message):
    __slots__ = ["nav_filed"]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ...) -> None: ...

class OneWay(_message.Message):
    __slots__ = ["direction", "guide_points", "nav_filed"]
    class Direction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    direction: OneWay.Direction
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    kDown: OneWay.Direction
    kLeft: OneWay.Direction
    kRight: OneWay.Direction
    kUp: OneWay.Direction
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., direction: _Optional[_Union[OneWay.Direction, str]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ...) -> None: ...

class RampField(_message.Message):
    __slots__ = ["arrow_pose", "nav_filed", "slope_angle"]
    ARROW_POSE_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    SLOPE_ANGLE_FIELD_NUMBER: _ClassVar[int]
    arrow_pose: _common_pb2.Pose2D
    nav_filed: NavField
    slope_angle: float
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., slope_angle: _Optional[float] = ..., arrow_pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class SingleWay(_message.Message):
    __slots__ = ["guide_points", "nav_filed"]
    GUIDE_POINTS_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    guide_points: _containers.RepeatedCompositeFieldContainer[GuidePoint]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., guide_points: _Optional[_Iterable[_Union[GuidePoint, _Mapping]]] = ...) -> None: ...

class SpeedLimitField(_message.Message):
    __slots__ = ["nav_filed"]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ...) -> None: ...

class WarnField(_message.Message):
    __slots__ = ["custom", "nav_filed"]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    NAV_FILED_FIELD_NUMBER: _ClassVar[int]
    custom: bytes
    nav_filed: NavField
    def __init__(self, nav_filed: _Optional[_Union[NavField, _Mapping]] = ..., custom: _Optional[bytes] = ...) -> None: ...

class GuideType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
