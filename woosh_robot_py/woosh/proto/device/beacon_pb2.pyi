from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Beacon(_message.Message):
    __slots__ = ["id", "power", "time"]
    ID_FIELD_NUMBER: _ClassVar[int]
    POWER_FIELD_NUMBER: _ClassVar[int]
    TIME_FIELD_NUMBER: _ClassVar[int]
    id: int
    power: float
    time: int
    def __init__(self, id: _Optional[int] = ..., power: _Optional[float] = ..., time: _Optional[int] = ...) -> None: ...

class Beacons(_message.Message):
    __slots__ = ["beacons"]
    BEACONS_FIELD_NUMBER: _ClassVar[int]
    beacons: _containers.RepeatedCompositeFieldContainer[Beacon]
    def __init__(self, beacons: _Optional[_Iterable[_Union[Beacon, _Mapping]]] = ...) -> None: ...
