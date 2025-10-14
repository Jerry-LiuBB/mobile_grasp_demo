from woosh.proto.util import common_pb2 as _common_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kCancel: ControlAction
kExecute: ControlAction
kPause: ControlAction
kResume: ControlAction

class AnyAction(_message.Message):
    __slots__ = ["type", "value"]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    VALUE_FIELD_NUMBER: _ClassVar[int]
    type: str
    value: bytes
    def __init__(self, type: _Optional[str] = ..., value: _Optional[bytes] = ...) -> None: ...

class ChargeControl(_message.Message):
    __slots__ = ["action", "execute_mode", "mode"]
    class ExecuteMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ACTION_FIELD_NUMBER: _ClassVar[int]
    EXECUTE_MODE_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    action: ControlAction
    execute_mode: ChargeControl.ExecuteMode
    kAuto: ChargeControl.ExecuteMode
    kCheck: ChargeControl.Mode
    kExit: ChargeControl.ExecuteMode
    kNoneExecuteMode: ChargeControl.ExecuteMode
    kNoneMode: ChargeControl.Mode
    kProcessctrl: ChargeControl.Mode
    mode: ChargeControl.Mode
    def __init__(self, execute_mode: _Optional[_Union[ChargeControl.ExecuteMode, str]] = ..., mode: _Optional[_Union[ChargeControl.Mode, str]] = ..., action: _Optional[_Union[ControlAction, str]] = ...) -> None: ...

class LiftControl(_message.Message):
    __slots__ = ["action", "execute_mode"]
    class ExecuteMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ACTION_FIELD_NUMBER: _ClassVar[int]
    EXECUTE_MODE_FIELD_NUMBER: _ClassVar[int]
    action: ControlAction
    execute_mode: LiftControl.ExecuteMode
    kDown: LiftControl.ExecuteMode
    kNoneExecuteMode: LiftControl.ExecuteMode
    kUp: LiftControl.ExecuteMode
    def __init__(self, execute_mode: _Optional[_Union[LiftControl.ExecuteMode, str]] = ..., action: _Optional[_Union[ControlAction, str]] = ...) -> None: ...

class LiftControl2(_message.Message):
    __slots__ = ["action", "height"]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    action: ControlAction
    height: float
    def __init__(self, height: _Optional[float] = ..., action: _Optional[_Union[ControlAction, str]] = ...) -> None: ...

class LiftControl3(_message.Message):
    __slots__ = ["action", "execute_mode", "flags", "height", "speed"]
    class ExecuteMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class FlagsBit(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ACTION_FIELD_NUMBER: _ClassVar[int]
    EXECUTE_MODE_FIELD_NUMBER: _ClassVar[int]
    FLAGS_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    SPEED_FIELD_NUMBER: _ClassVar[int]
    action: ControlAction
    execute_mode: LiftControl3.ExecuteMode
    flags: int
    height: float
    kAbsolute: LiftControl3.ExecuteMode
    kCalibration: LiftControl3.ExecuteMode
    kFB0: LiftControl3.FlagsBit
    kFB1: LiftControl3.FlagsBit
    kFB2: LiftControl3.FlagsBit
    kFB3: LiftControl3.FlagsBit
    kFB4: LiftControl3.FlagsBit
    kFB5: LiftControl3.FlagsBit
    kFB6: LiftControl3.FlagsBit
    kFB7: LiftControl3.FlagsBit
    kFlagsBitUndefined: LiftControl3.FlagsBit
    kQuery: LiftControl3.ExecuteMode
    kRelative: LiftControl3.ExecuteMode
    kTsetMode: LiftControl3.ExecuteMode
    speed: float
    def __init__(self, execute_mode: _Optional[_Union[LiftControl3.ExecuteMode, str]] = ..., speed: _Optional[float] = ..., height: _Optional[float] = ..., flags: _Optional[int] = ..., action: _Optional[_Union[ControlAction, str]] = ...) -> None: ...

class MoveBase(_message.Message):
    __slots__ = ["action", "execution_mode", "poses", "target_pose"]
    class ExecutionMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ACTION_FIELD_NUMBER: _ClassVar[int]
    EXECUTION_MODE_FIELD_NUMBER: _ClassVar[int]
    POSES_FIELD_NUMBER: _ClassVar[int]
    TARGET_POSE_FIELD_NUMBER: _ClassVar[int]
    action: ControlAction
    execution_mode: MoveBase.ExecutionMode
    kFree: MoveBase.ExecutionMode
    kOneByOne: MoveBase.ExecutionMode
    poses: _containers.RepeatedCompositeFieldContainer[_common_pb2.Pose2D]
    target_pose: _common_pb2.Pose2D
    def __init__(self, poses: _Optional[_Iterable[_Union[_common_pb2.Pose2D, _Mapping]]] = ..., target_pose: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., execution_mode: _Optional[_Union[MoveBase.ExecutionMode, str]] = ..., action: _Optional[_Union[ControlAction, str]] = ...) -> None: ...

class NavigationMode(_message.Message):
    __slots__ = ["capacity", "id", "max_speed", "mode", "permitted_passage", "wait_time"]
    class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    CAPACITY_FIELD_NUMBER: _ClassVar[int]
    ID_FIELD_NUMBER: _ClassVar[int]
    MAX_SPEED_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    PERMITTED_PASSAGE_FIELD_NUMBER: _ClassVar[int]
    WAIT_TIME_FIELD_NUMBER: _ClassVar[int]
    capacity: int
    id: int
    kAccurateAvoid: NavigationMode.Mode
    kVagueAvoid: NavigationMode.Mode
    kWaitAvoid: NavigationMode.Mode
    kWaitNav: NavigationMode.Mode
    max_speed: float
    mode: NavigationMode.Mode
    permitted_passage: bool
    wait_time: float
    def __init__(self, id: _Optional[int] = ..., mode: _Optional[_Union[NavigationMode.Mode, str]] = ..., wait_time: _Optional[float] = ..., max_speed: _Optional[float] = ..., permitted_passage: bool = ..., capacity: _Optional[int] = ...) -> None: ...

class StepControl(_message.Message):
    __slots__ = ["action", "avoid", "steps"]
    class Step(_message.Message):
        __slots__ = ["angle", "mode", "speed", "value"]
        class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
            __slots__ = []
        ANGLE_FIELD_NUMBER: _ClassVar[int]
        MODE_FIELD_NUMBER: _ClassVar[int]
        SPEED_FIELD_NUMBER: _ClassVar[int]
        VALUE_FIELD_NUMBER: _ClassVar[int]
        angle: float
        kDiagonalize: StepControl.Step.Mode
        kLateral: StepControl.Step.Mode
        kNone: StepControl.Step.Mode
        kRotate: StepControl.Step.Mode
        kStraight: StepControl.Step.Mode
        mode: StepControl.Step.Mode
        speed: float
        value: float
        def __init__(self, mode: _Optional[_Union[StepControl.Step.Mode, str]] = ..., value: _Optional[float] = ..., speed: _Optional[float] = ..., angle: _Optional[float] = ...) -> None: ...
    ACTION_FIELD_NUMBER: _ClassVar[int]
    AVOID_FIELD_NUMBER: _ClassVar[int]
    STEPS_FIELD_NUMBER: _ClassVar[int]
    action: ControlAction
    avoid: int
    steps: _containers.RepeatedCompositeFieldContainer[StepControl.Step]
    def __init__(self, steps: _Optional[_Iterable[_Union[StepControl.Step, _Mapping]]] = ..., avoid: _Optional[int] = ..., action: _Optional[_Union[ControlAction, str]] = ...) -> None: ...

class ControlAction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
