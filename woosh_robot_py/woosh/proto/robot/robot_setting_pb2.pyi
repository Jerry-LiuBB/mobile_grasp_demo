from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class AutoCharge(_message.Message):
    __slots__ = ["allow", "robot_id"]
    ALLOW_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    allow: bool
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., allow: bool = ...) -> None: ...

class AutoPark(_message.Message):
    __slots__ = ["allow", "robot_id"]
    ALLOW_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    allow: bool
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., allow: bool = ...) -> None: ...

class GoodsCheck(_message.Message):
    __slots__ = ["allow", "robot_id"]
    ALLOW_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    allow: bool
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., allow: bool = ...) -> None: ...

class Identity(_message.Message):
    __slots__ = ["name", "robot_id"]
    NAME_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    name: str
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., name: _Optional[str] = ...) -> None: ...

class Power(_message.Message):
    __slots__ = ["alarm", "full", "idle", "low", "robot_id"]
    ALARM_FIELD_NUMBER: _ClassVar[int]
    FULL_FIELD_NUMBER: _ClassVar[int]
    IDLE_FIELD_NUMBER: _ClassVar[int]
    LOW_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    alarm: int
    full: int
    idle: int
    low: int
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., alarm: _Optional[int] = ..., low: _Optional[int] = ..., idle: _Optional[int] = ..., full: _Optional[int] = ...) -> None: ...

class Server(_message.Message):
    __slots__ = ["ip", "port", "robot_id"]
    IP_FIELD_NUMBER: _ClassVar[int]
    PORT_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    ip: str
    port: int
    robot_id: int
    def __init__(self, robot_id: _Optional[int] = ..., ip: _Optional[str] = ..., port: _Optional[int] = ...) -> None: ...

class Sound(_message.Message):
    __slots__ = ["mute", "robot_id", "volume"]
    MUTE_FIELD_NUMBER: _ClassVar[int]
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    VOLUME_FIELD_NUMBER: _ClassVar[int]
    mute: bool
    robot_id: int
    volume: int
    def __init__(self, robot_id: _Optional[int] = ..., mute: bool = ..., volume: _Optional[int] = ...) -> None: ...
