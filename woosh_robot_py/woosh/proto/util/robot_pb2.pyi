from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from typing import ClassVar as _ClassVar

DESCRIPTOR: _descriptor.FileDescriptor
kArmRobot_14: Type
kAuto: ControlMode
kBaseRobot_200: Type
kCharging: State
kComplexRobot: Type
kControlModeUndefined: ControlMode
kDeployMode: WorkMode
kDock: FootPrint
kExpand: FootPrint
kFault: State
kFollowing: State
kIdle: State
kMaintain: ControlMode
kManual: ControlMode
kMapping: State
kOriginal: FootPrint
kPalletLiftRobot_500: Type
kParking: State
kRollerRobot_500: Type
kScheduleMode: WorkMode
kShelfLiftRobot_500: Type
kSpare: FootPrint
kStateUndefined: State
kTask: State
kTaskMode: WorkMode
kTractorRobot_500: Type
kTypeUndefined: Type
kUninit: State
kWarning: State
kWorkModeUndefined: WorkMode

class Type(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class ControlMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class WorkMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class FootPrint(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
