from woosh.proto.map import mark_pb2 as _mark_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Create(_message.Message):
    __slots__ = ["storage"]
    STORAGE_FIELD_NUMBER: _ClassVar[int]
    storage: _mark_pb2.Storage
    def __init__(self, storage: _Optional[_Union[_mark_pb2.Storage, _Mapping]] = ...) -> None: ...

class Delete(_message.Message):
    __slots__ = ["no"]
    NO_FIELD_NUMBER: _ClassVar[int]
    no: str
    def __init__(self, no: _Optional[str] = ...) -> None: ...

class Find(_message.Message):
    __slots__ = ["no"]
    NO_FIELD_NUMBER: _ClassVar[int]
    no: str
    def __init__(self, no: _Optional[str] = ...) -> None: ...

class Storages(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class Update(_message.Message):
    __slots__ = ["storage"]
    STORAGE_FIELD_NUMBER: _ClassVar[int]
    storage: _mark_pb2.Storage
    def __init__(self, storage: _Optional[_Union[_mark_pb2.Storage, _Mapping]] = ...) -> None: ...
