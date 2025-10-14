from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Fire(_message.Message):
    __slots__ = ["drill", "warn"]
    DRILL_FIELD_NUMBER: _ClassVar[int]
    WARN_FIELD_NUMBER: _ClassVar[int]
    drill: bool
    warn: bool
    def __init__(self, warn: bool = ..., drill: bool = ...) -> None: ...

class FireTime(_message.Message):
    __slots__ = ["remain_time", "start_time"]
    REMAIN_TIME_FIELD_NUMBER: _ClassVar[int]
    START_TIME_FIELD_NUMBER: _ClassVar[int]
    remain_time: int
    start_time: int
    def __init__(self, start_time: _Optional[int] = ..., remain_time: _Optional[int] = ...) -> None: ...

class GetFireInfo(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class GetFireTimeInfo(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class GetLiftInfo(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class Lift(_message.Message):
    __slots__ = ["grating"]
    GRATING_FIELD_NUMBER: _ClassVar[int]
    grating: bool
    def __init__(self, grating: bool = ...) -> None: ...

class SetFireTimeInfo(_message.Message):
    __slots__ = ["ftime"]
    FTIME_FIELD_NUMBER: _ClassVar[int]
    ftime: FireTime
    def __init__(self, ftime: _Optional[_Union[FireTime, _Mapping]] = ...) -> None: ...
