from woosh.proto.ros import action_pb2 as _action_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kRosCancel: State
kRosErrMsg: State
kRosExecuteFailed: State
kRosExecuting: State
kRosFailure: State
kRosNone: State
kRosPause: State
kRosPauseFailed: State
kRosSuccess: State
kRosWiFiCode: State
kRosWiFiJson: State

class CallAction(_message.Message):
    __slots__ = ["any_action", "charge_control", "lift_control", "lift_control2", "lift_control3", "move_base", "nav_mode", "step_control"]
    ANY_ACTION_FIELD_NUMBER: _ClassVar[int]
    CHARGE_CONTROL_FIELD_NUMBER: _ClassVar[int]
    LIFT_CONTROL2_FIELD_NUMBER: _ClassVar[int]
    LIFT_CONTROL3_FIELD_NUMBER: _ClassVar[int]
    LIFT_CONTROL_FIELD_NUMBER: _ClassVar[int]
    MOVE_BASE_FIELD_NUMBER: _ClassVar[int]
    NAV_MODE_FIELD_NUMBER: _ClassVar[int]
    STEP_CONTROL_FIELD_NUMBER: _ClassVar[int]
    any_action: _action_pb2.AnyAction
    charge_control: _action_pb2.ChargeControl
    lift_control: _action_pb2.LiftControl
    lift_control2: _action_pb2.LiftControl2
    lift_control3: _action_pb2.LiftControl3
    move_base: _action_pb2.MoveBase
    nav_mode: _action_pb2.NavigationMode
    step_control: _action_pb2.StepControl
    def __init__(self, charge_control: _Optional[_Union[_action_pb2.ChargeControl, _Mapping]] = ..., lift_control: _Optional[_Union[_action_pb2.LiftControl, _Mapping]] = ..., lift_control2: _Optional[_Union[_action_pb2.LiftControl2, _Mapping]] = ..., step_control: _Optional[_Union[_action_pb2.StepControl, _Mapping]] = ..., lift_control3: _Optional[_Union[_action_pb2.LiftControl3, _Mapping]] = ..., move_base: _Optional[_Union[_action_pb2.MoveBase, _Mapping]] = ..., nav_mode: _Optional[_Union[_action_pb2.NavigationMode, _Mapping]] = ..., any_action: _Optional[_Union[_action_pb2.AnyAction, _Mapping]] = ...) -> None: ...

class Feedback(_message.Message):
    __slots__ = ["action", "code", "msg", "state"]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    CODE_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    action: str
    code: int
    msg: str
    state: State
    def __init__(self, action: _Optional[str] = ..., state: _Optional[_Union[State, str]] = ..., code: _Optional[int] = ..., msg: _Optional[str] = ...) -> None: ...

class Feedbacks(_message.Message):
    __slots__ = ["fbs"]
    FBS_FIELD_NUMBER: _ClassVar[int]
    fbs: _containers.RepeatedCompositeFieldContainer[Feedback]
    def __init__(self, fbs: _Optional[_Iterable[_Union[Feedback, _Mapping]]] = ...) -> None: ...

class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
