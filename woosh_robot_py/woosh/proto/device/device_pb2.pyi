from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class RfRemoteController(_message.Message):
    __slots__ = ["address", "keys"]
    ADDRESS_FIELD_NUMBER: _ClassVar[int]
    KEYS_FIELD_NUMBER: _ClassVar[int]
    address: int
    keys: int
    def __init__(self, address: _Optional[int] = ..., keys: _Optional[int] = ...) -> None: ...
