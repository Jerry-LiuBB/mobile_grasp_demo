from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from typing import ClassVar as _ClassVar

DESCRIPTOR: _descriptor.FileDescriptor
kARTag: MarkerType
kAutoDoor: FieldType
kBeacon: MarkerType
kBidirectPath: PathType
kBidirectionWay: FieldType
kCrossing: FieldType
kFieldTypeUndefined: FieldType
kFireField: FieldType
kGuideField: FieldType
kLocate: FieldType
kMarkerTypeUndefined: MarkerType
kMonoPath: PathType
kNarrowPassage: FieldType
kOdom: FieldType
kOneWay: FieldType
kPathTypeUndefined: PathType
kRamp: FieldType
kReflector: MarkerType
kSingleWay: FieldType
kSpeedLimit: FieldType
kStorage: MarkerType
kWarnField: FieldType
kWormhole: MarkerType

class MarkerType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class FieldType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class PathType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
