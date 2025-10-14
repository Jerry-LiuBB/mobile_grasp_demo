from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from typing import ClassVar as _ClassVar

DESCRIPTOR: _descriptor.FileDescriptor
kCancel: Order
kCarry: Type
kCharge: Type
kContinue: Order
kNav: Type
kOrderUndefined: Order
kPause: Order
kRecover: Order
kReleaseCtrl: Order
kRosCancel: State
kRosExecuting: State
kRosFailure: State
kRosSuccess: State
kRosWarning: State
kSecondposEnter: Type
kSecondposQuit: Type
kStart: Order
kStateUndefined: State
kStepCtrl: Type
kSuspend: State
kTmCtrl: Order
kTraffiCtrl: State
kTypeUndefined: Type
kWait: Type
kWaitBreak: Order

class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class Order(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
