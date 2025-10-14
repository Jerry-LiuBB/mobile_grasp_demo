from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.util import navigation_pb2 as _navigation_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ARTag(_message.Message):
    __slots__ = ["id", "pose"]
    ID_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    id: int
    pose: _common_pb2.Pose2D
    def __init__(self, id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...

class Beacon(_message.Message):
    __slots__ = ["group", "id", "point"]
    class Group(_message.Message):
        __slots__ = ["group_id", "max_range"]
        GROUP_ID_FIELD_NUMBER: _ClassVar[int]
        MAX_RANGE_FIELD_NUMBER: _ClassVar[int]
        group_id: int
        max_range: float
        def __init__(self, group_id: _Optional[int] = ..., max_range: _Optional[float] = ...) -> None: ...
    GROUP_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    POINT_FIELD_NUMBER: _ClassVar[int]
    group: _containers.RepeatedCompositeFieldContainer[Beacon.Group]
    id: int
    point: _common_pb2.Point
    def __init__(self, id: _Optional[int] = ..., group: _Optional[_Iterable[_Union[Beacon.Group, _Mapping]]] = ..., point: _Optional[_Union[_common_pb2.Point, _Mapping]] = ...) -> None: ...

class Dock(_message.Message):
    __slots__ = ["board_type", "identify", "tag", "verify"]
    class FeatureBoardType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class Identify(_message.Message):
        __slots__ = ["artag", "rfid"]
        ARTAG_FIELD_NUMBER: _ClassVar[int]
        RFID_FIELD_NUMBER: _ClassVar[int]
        artag: int
        rfid: int
        def __init__(self, artag: _Optional[int] = ..., rfid: _Optional[int] = ...) -> None: ...
    class Verify(_message.Message):
        __slots__ = ["back_feature", "bottom_artag", "bottom_magnetic", "top_photoelec"]
        BACK_FEATURE_FIELD_NUMBER: _ClassVar[int]
        BOTTOM_ARTAG_FIELD_NUMBER: _ClassVar[int]
        BOTTOM_MAGNETIC_FIELD_NUMBER: _ClassVar[int]
        TOP_PHOTOELEC_FIELD_NUMBER: _ClassVar[int]
        back_feature: bool
        bottom_artag: bool
        bottom_magnetic: bool
        top_photoelec: bool
        def __init__(self, top_photoelec: bool = ..., bottom_magnetic: bool = ..., back_feature: bool = ..., bottom_artag: bool = ...) -> None: ...
    BOARD_TYPE_FIELD_NUMBER: _ClassVar[int]
    IDENTIFY_FIELD_NUMBER: _ClassVar[int]
    TAG_FIELD_NUMBER: _ClassVar[int]
    VERIFY_FIELD_NUMBER: _ClassVar[int]
    board_type: Dock.FeatureBoardType
    identify: Dock.Identify
    kConvexShape: Dock.FeatureBoardType
    kElevatorShape: Dock.FeatureBoardType
    kFeatureBoardTypeUndefined: Dock.FeatureBoardType
    kFourPointRect: Dock.FeatureBoardType
    kMagnetism: Dock.FeatureBoardType
    kMagnetismRfid: Dock.FeatureBoardType
    kNarrowPassageShape: Dock.FeatureBoardType
    kParallelShape: Dock.FeatureBoardType
    kVLShape: Dock.FeatureBoardType
    tag: str
    verify: Dock.Verify
    def __init__(self, identify: _Optional[_Union[Dock.Identify, _Mapping]] = ..., verify: _Optional[_Union[Dock.Verify, _Mapping]] = ..., board_type: _Optional[_Union[Dock.FeatureBoardType, str]] = ..., tag: _Optional[str] = ...) -> None: ...

class Identity(_message.Message):
    __slots__ = ["desc", "id", "no"]
    DESC_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    NO_FIELD_NUMBER: _ClassVar[int]
    desc: str
    id: int
    no: str
    def __init__(self, id: _Optional[int] = ..., no: _Optional[str] = ..., desc: _Optional[str] = ...) -> None: ...

class MarkInfo(_message.Message):
    __slots__ = ["artags", "beacons", "floor_name", "reflectors", "sdkv", "storages", "version", "wormholes"]
    ARTAGS_FIELD_NUMBER: _ClassVar[int]
    BEACONS_FIELD_NUMBER: _ClassVar[int]
    FLOOR_NAME_FIELD_NUMBER: _ClassVar[int]
    REFLECTORS_FIELD_NUMBER: _ClassVar[int]
    SDKV_FIELD_NUMBER: _ClassVar[int]
    STORAGES_FIELD_NUMBER: _ClassVar[int]
    VERSION_FIELD_NUMBER: _ClassVar[int]
    WORMHOLES_FIELD_NUMBER: _ClassVar[int]
    artags: _containers.RepeatedCompositeFieldContainer[ARTag]
    beacons: _containers.RepeatedCompositeFieldContainer[Beacon]
    floor_name: str
    reflectors: _containers.RepeatedCompositeFieldContainer[Reflector]
    sdkv: str
    storages: _containers.RepeatedCompositeFieldContainer[Storage]
    version: int
    wormholes: _containers.RepeatedCompositeFieldContainer[Wormhole]
    def __init__(self, version: _Optional[int] = ..., sdkv: _Optional[str] = ..., floor_name: _Optional[str] = ..., storages: _Optional[_Iterable[_Union[Storage, _Mapping]]] = ..., wormholes: _Optional[_Iterable[_Union[Wormhole, _Mapping]]] = ..., beacons: _Optional[_Iterable[_Union[Beacon, _Mapping]]] = ..., artags: _Optional[_Iterable[_Union[ARTag, _Mapping]]] = ..., reflectors: _Optional[_Iterable[_Union[Reflector, _Mapping]]] = ...) -> None: ...

class Navigation(_message.Message):
    __slots__ = ["arr"]
    ARR_FIELD_NUMBER: _ClassVar[int]
    arr: _navigation_pb2.ArrType
    def __init__(self, arr: _Optional[_Union[_navigation_pb2.ArrType, str]] = ...) -> None: ...

class Pose(_message.Message):
    __slots__ = ["adjust", "dock", "real"]
    class Adjust(_message.Message):
        __slots__ = ["pose"]
        POSE_FIELD_NUMBER: _ClassVar[int]
        pose: _common_pb2.Pose2D
        def __init__(self, pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ...) -> None: ...
    ADJUST_FIELD_NUMBER: _ClassVar[int]
    DOCK_FIELD_NUMBER: _ClassVar[int]
    REAL_FIELD_NUMBER: _ClassVar[int]
    adjust: Pose.Adjust
    dock: _common_pb2.Pose2D
    real: _common_pb2.Pose2D
    def __init__(self, dock: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., real: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., adjust: _Optional[_Union[Pose.Adjust, _Mapping]] = ...) -> None: ...

class Reflector(_message.Message):
    __slots__ = ["group", "id", "pose"]
    GROUP_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    group: _containers.RepeatedScalarFieldContainer[int]
    id: int
    pose: _common_pb2.Pose2D
    def __init__(self, id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., group: _Optional[_Iterable[int]] = ...) -> None: ...

class Storage(_message.Message):
    __slots__ = ["arm_worktop", "charger", "custom", "dock", "identity", "lift_trolley", "nav", "pallet_platform", "parkspot", "pose", "rack", "roller_station", "tractor_trailer"]
    class ArmWorktop(_message.Message):
        __slots__ = []
        def __init__(self) -> None: ...
    class Charger(_message.Message):
        __slots__ = ["parking", "robots"]
        PARKING_FIELD_NUMBER: _ClassVar[int]
        ROBOTS_FIELD_NUMBER: _ClassVar[int]
        parking: bool
        robots: _containers.RepeatedScalarFieldContainer[int]
        def __init__(self, parking: bool = ..., robots: _Optional[_Iterable[int]] = ...) -> None: ...
    class LiftTrolley(_message.Message):
        __slots__ = []
        def __init__(self) -> None: ...
    class PalletPlatform(_message.Message):
        __slots__ = ["detector"]
        DETECTOR_FIELD_NUMBER: _ClassVar[int]
        detector: str
        def __init__(self, detector: _Optional[str] = ...) -> None: ...
    class Parkspot(_message.Message):
        __slots__ = ["robots"]
        ROBOTS_FIELD_NUMBER: _ClassVar[int]
        robots: _containers.RepeatedScalarFieldContainer[int]
        def __init__(self, robots: _Optional[_Iterable[int]] = ...) -> None: ...
    class Rack(_message.Message):
        __slots__ = ["numbers"]
        NUMBERS_FIELD_NUMBER: _ClassVar[int]
        numbers: _containers.RepeatedScalarFieldContainer[str]
        def __init__(self, numbers: _Optional[_Iterable[str]] = ...) -> None: ...
    class RollerStation(_message.Message):
        __slots__ = ["mode"]
        class RollerMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
            __slots__ = []
        MODE_FIELD_NUMBER: _ClassVar[int]
        kCheck: Storage.RollerStation.RollerMode
        kProcessctrl: Storage.RollerStation.RollerMode
        kRollerModeUndefined: Storage.RollerStation.RollerMode
        mode: Storage.RollerStation.RollerMode
        def __init__(self, mode: _Optional[_Union[Storage.RollerStation.RollerMode, str]] = ...) -> None: ...
    class TractorTrailer(_message.Message):
        __slots__ = []
        def __init__(self) -> None: ...
    ARM_WORKTOP_FIELD_NUMBER: _ClassVar[int]
    CHARGER_FIELD_NUMBER: _ClassVar[int]
    CUSTOM_FIELD_NUMBER: _ClassVar[int]
    DOCK_FIELD_NUMBER: _ClassVar[int]
    IDENTITY_FIELD_NUMBER: _ClassVar[int]
    LIFT_TROLLEY_FIELD_NUMBER: _ClassVar[int]
    NAV_FIELD_NUMBER: _ClassVar[int]
    PALLET_PLATFORM_FIELD_NUMBER: _ClassVar[int]
    PARKSPOT_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    RACK_FIELD_NUMBER: _ClassVar[int]
    ROLLER_STATION_FIELD_NUMBER: _ClassVar[int]
    TRACTOR_TRAILER_FIELD_NUMBER: _ClassVar[int]
    arm_worktop: Storage.ArmWorktop
    charger: Storage.Charger
    custom: bytes
    dock: Dock
    identity: Identity
    lift_trolley: Storage.LiftTrolley
    nav: Navigation
    pallet_platform: Storage.PalletPlatform
    parkspot: Storage.Parkspot
    pose: Pose
    rack: Storage.Rack
    roller_station: Storage.RollerStation
    tractor_trailer: Storage.TractorTrailer
    def __init__(self, identity: _Optional[_Union[Identity, _Mapping]] = ..., pose: _Optional[_Union[Pose, _Mapping]] = ..., nav: _Optional[_Union[Navigation, _Mapping]] = ..., dock: _Optional[_Union[Dock, _Mapping]] = ..., custom: _Optional[bytes] = ..., rack: _Optional[_Union[Storage.Rack, _Mapping]] = ..., parkspot: _Optional[_Union[Storage.Parkspot, _Mapping]] = ..., charger: _Optional[_Union[Storage.Charger, _Mapping]] = ..., pallet_platform: _Optional[_Union[Storage.PalletPlatform, _Mapping]] = ..., lift_trolley: _Optional[_Union[Storage.LiftTrolley, _Mapping]] = ..., tractor_trailer: _Optional[_Union[Storage.TractorTrailer, _Mapping]] = ..., roller_station: _Optional[_Union[Storage.RollerStation, _Mapping]] = ..., arm_worktop: _Optional[_Union[Storage.ArmWorktop, _Mapping]] = ...) -> None: ...

class Storages(_message.Message):
    __slots__ = ["arm_worktops", "bases", "chargers", "lift_trolleys", "pallet_platforms", "parkspots", "racks", "roller_stations", "tractor_trailers"]
    class Base(_message.Message):
        __slots__ = ["custom", "identity", "pose"]
        CUSTOM_FIELD_NUMBER: _ClassVar[int]
        IDENTITY_FIELD_NUMBER: _ClassVar[int]
        POSE_FIELD_NUMBER: _ClassVar[int]
        custom: bytes
        identity: Identity
        pose: Pose
        def __init__(self, identity: _Optional[_Union[Identity, _Mapping]] = ..., pose: _Optional[_Union[Pose, _Mapping]] = ..., custom: _Optional[bytes] = ...) -> None: ...
    ARM_WORKTOPS_FIELD_NUMBER: _ClassVar[int]
    BASES_FIELD_NUMBER: _ClassVar[int]
    CHARGERS_FIELD_NUMBER: _ClassVar[int]
    LIFT_TROLLEYS_FIELD_NUMBER: _ClassVar[int]
    PALLET_PLATFORMS_FIELD_NUMBER: _ClassVar[int]
    PARKSPOTS_FIELD_NUMBER: _ClassVar[int]
    RACKS_FIELD_NUMBER: _ClassVar[int]
    ROLLER_STATIONS_FIELD_NUMBER: _ClassVar[int]
    TRACTOR_TRAILERS_FIELD_NUMBER: _ClassVar[int]
    arm_worktops: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    bases: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    chargers: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    lift_trolleys: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    pallet_platforms: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    parkspots: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    racks: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    roller_stations: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    tractor_trailers: _containers.RepeatedCompositeFieldContainer[Storages.Base]
    def __init__(self, bases: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., racks: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., parkspots: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., chargers: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., pallet_platforms: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., lift_trolleys: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., tractor_trailers: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., roller_stations: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ..., arm_worktops: _Optional[_Iterable[_Union[Storages.Base, _Mapping]]] = ...) -> None: ...

class Wormhole(_message.Message):
    __slots__ = ["elevator", "ferry_field", "identity", "nav", "other_fields", "pose", "reach_maps", "token", "wormhole_field"]
    class Elevator(_message.Message):
        __slots__ = ["desc", "ip", "port", "tag", "type"]
        class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
            __slots__ = []
        DESC_FIELD_NUMBER: _ClassVar[int]
        IP_FIELD_NUMBER: _ClassVar[int]
        PORT_FIELD_NUMBER: _ClassVar[int]
        TAG_FIELD_NUMBER: _ClassVar[int]
        TYPE_FIELD_NUMBER: _ClassVar[int]
        desc: str
        ip: str
        kNarrow: Wormhole.Elevator.Type
        kTypeUndefined: Wormhole.Elevator.Type
        kWide: Wormhole.Elevator.Type
        port: int
        tag: str
        type: Wormhole.Elevator.Type
        def __init__(self, type: _Optional[_Union[Wormhole.Elevator.Type, str]] = ..., ip: _Optional[str] = ..., port: _Optional[int] = ..., tag: _Optional[str] = ..., desc: _Optional[str] = ...) -> None: ...
    ELEVATOR_FIELD_NUMBER: _ClassVar[int]
    FERRY_FIELD_FIELD_NUMBER: _ClassVar[int]
    IDENTITY_FIELD_NUMBER: _ClassVar[int]
    NAV_FIELD_NUMBER: _ClassVar[int]
    OTHER_FIELDS_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    REACH_MAPS_FIELD_NUMBER: _ClassVar[int]
    TOKEN_FIELD_NUMBER: _ClassVar[int]
    WORMHOLE_FIELD_FIELD_NUMBER: _ClassVar[int]
    elevator: Wormhole.Elevator
    ferry_field: int
    identity: Identity
    nav: Navigation
    other_fields: _containers.RepeatedScalarFieldContainer[int]
    pose: Pose
    reach_maps: _containers.RepeatedScalarFieldContainer[int]
    token: str
    wormhole_field: int
    def __init__(self, identity: _Optional[_Union[Identity, _Mapping]] = ..., pose: _Optional[_Union[Pose, _Mapping]] = ..., nav: _Optional[_Union[Navigation, _Mapping]] = ..., token: _Optional[str] = ..., reach_maps: _Optional[_Iterable[int]] = ..., wormhole_field: _Optional[int] = ..., ferry_field: _Optional[int] = ..., other_fields: _Optional[_Iterable[int]] = ..., elevator: _Optional[_Union[Wormhole.Elevator, _Mapping]] = ...) -> None: ...
