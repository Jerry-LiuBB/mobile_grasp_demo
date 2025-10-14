from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.util import navigation_pb2 as _navigation_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
kCtrlContinue: CtrlOrder
kCtrlNavMode: CtrlOrder
kCtrlPath: CtrlOrder
kCtrlPause: CtrlOrder

class CtrlRequest(_message.Message):
    __slots__ = ["nav_mode_setting", "order", "robot_id"]
    NAV_MODE_SETTING_FIELD_NUMBER: _ClassVar[int]
    ORDER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    nav_mode_setting: _navigation_pb2.ModeSetting
    order: CtrlOrder
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., order: _Optional[_Union[CtrlOrder, str]] = ..., nav_mode_setting: _Optional[_Union[_navigation_pb2.ModeSetting, _Mapping]] = ...) -> None: ...

class CtrlResponse(_message.Message):
    __slots__ = ["order", "result", "robot_id"]
    class Result(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    ORDER_FIELD_NUMBER: _ClassVar[int]
    RESULT_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    kErr: CtrlResponse.Result
    kOk: CtrlResponse.Result
    order: CtrlOrder
    result: CtrlResponse.Result
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., order: _Optional[_Union[CtrlOrder, str]] = ..., result: _Optional[_Union[CtrlResponse.Result, str]] = ...) -> None: ...

class MarkerInfo(_message.Message):
    __slots__ = ["marker_id", "pose"]
    MARKER_ID_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    marker_id: int
    pose: _common_pb2.Pose
    def __init__(self, marker_id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose, _Mapping]] = ...) -> None: ...

class PathRequest(_message.Message):
    __slots__ = ["pose", "robot_id", "task_id"]
    POSE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    pose: _common_pb2.Pose
    robot_id: int
    task_id: int
    def __init__(self, robot_id: _Optional[int] = ..., task_id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose, _Mapping]] = ...) -> None: ...

class PathResponse(_message.Message):
    __slots__ = ["result", "robot_id", "task_id"]
    class Result(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    RESULT_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    kErr: PathResponse.Result
    kOk: PathResponse.Result
    kOther: PathResponse.Result
    result: PathResponse.Result
    robot_id: int
    task_id: int
    def __init__(self, robot_id: _Optional[int] = ..., task_id: _Optional[int] = ..., result: _Optional[_Union[PathResponse.Result, str]] = ...) -> None: ...

class PosRequest(_message.Message):
    __slots__ = ["points", "robot_id", "task_id"]
    POINTS_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[MarkerInfo]
    robot_id: int
    task_id: int
    def __init__(self, robot_id: _Optional[int] = ..., task_id: _Optional[int] = ..., points: _Optional[_Iterable[_Union[MarkerInfo, _Mapping]]] = ...) -> None: ...

class PosResponse(_message.Message):
    __slots__ = ["points", "result", "robot_id", "task_id"]
    class Result(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    POINTS_FIELD_NUMBER: _ClassVar[int]
    RESULT_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    kErr: PosResponse.Result
    kOk: PosResponse.Result
    kOther: PosResponse.Result
    points: _containers.RepeatedCompositeFieldContainer[MarkerInfo]
    result: PosResponse.Result
    robot_id: int
    task_id: int
    def __init__(self, robot_id: _Optional[int] = ..., task_id: _Optional[int] = ..., result: _Optional[_Union[PosResponse.Result, str]] = ..., points: _Optional[_Iterable[_Union[MarkerInfo, _Mapping]]] = ...) -> None: ...

class RobotSortRequest(_message.Message):
    __slots__ = ["pose", "robots", "task_id"]
    POSE_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    pose: _common_pb2.Pose
    robots: _containers.RepeatedScalarFieldContainer[int]
    task_id: int
    def __init__(self, task_id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose, _Mapping]] = ..., robots: _Optional[_Iterable[int]] = ...) -> None: ...

class RobotSortResponse(_message.Message):
    __slots__ = ["result", "robots", "task_id"]
    class Result(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    RESULT_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    kErr: RobotSortResponse.Result
    kOk: RobotSortResponse.Result
    kOther: RobotSortResponse.Result
    result: RobotSortResponse.Result
    robots: _containers.RepeatedScalarFieldContainer[int]
    task_id: int
    def __init__(self, task_id: _Optional[int] = ..., result: _Optional[_Union[RobotSortResponse.Result, str]] = ..., robots: _Optional[_Iterable[int]] = ...) -> None: ...

class TaskSortRequest(_message.Message):
    __slots__ = ["pose", "robot_id", "robots"]
    POSE_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    pose: _common_pb2.Pose
    robot_id: int
    robots: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robot_id: _Optional[int] = ..., pose: _Optional[_Union[_common_pb2.Pose, _Mapping]] = ..., robots: _Optional[_Iterable[int]] = ...) -> None: ...

class TaskSortResponse(_message.Message):
    __slots__ = ["result", "robot_id", "robots"]
    class Result(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    RESULT_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    kErr: TaskSortResponse.Result
    kOk: TaskSortResponse.Result
    kOther: TaskSortResponse.Result
    result: TaskSortResponse.Result
    robot_id: int
    robots: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robot_id: _Optional[int] = ..., result: _Optional[_Union[TaskSortResponse.Result, str]] = ..., robots: _Optional[_Iterable[int]] = ...) -> None: ...

class CtrlOrder(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
