from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from typing import ClassVar as _ClassVar

DESCRIPTOR: _descriptor.FileDescriptor
kActionWait: State
kCanceled: State
kCarry: Type
kCharge: Type
kCompleted: State
kCutting: Direction
kDirectionUndefined: Direction
kExecuting: State
kFailed: State
kFeeding: Direction
kInit: State
kParking: Type
kPaused: State
kPick: Type
kReady: State
kStateUndefined: State
kTaskWait: State
kTypeUndefined: Type

class Direction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
