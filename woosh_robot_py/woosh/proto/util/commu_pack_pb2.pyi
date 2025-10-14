from google.protobuf import any_pb2 as _any_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class MessagePack(_message.Message):
    __slots__ = ["body", "msg", "ok"]
    BODY_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    OK_FIELD_NUMBER: _ClassVar[int]
    body: _any_pb2.Any
    msg: str
    ok: bool
    def __init__(self, body: _Optional[_Union[_any_pb2.Any, _Mapping]] = ..., ok: bool = ..., msg: _Optional[str] = ...) -> None: ...

class MessagePackJson(_message.Message):
    __slots__ = ["body", "msg", "ok", "sn", "type"]
    BODY_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    OK_FIELD_NUMBER: _ClassVar[int]
    SN_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    body: _any_pb2.Any
    msg: str
    ok: bool
    sn: int
    type: str
    def __init__(self, type: _Optional[str] = ..., body: _Optional[_Union[_any_pb2.Any, _Mapping]] = ..., ok: bool = ..., msg: _Optional[str] = ..., sn: _Optional[int] = ...) -> None: ...

class NotifyPackJson(_message.Message):
    __slots__ = ["body", "type"]
    BODY_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    body: _any_pb2.Any
    type: str
    def __init__(self, type: _Optional[str] = ..., body: _Optional[_Union[_any_pb2.Any, _Mapping]] = ...) -> None: ...

class RequestPackJson(_message.Message):
    __slots__ = ["body", "type"]
    BODY_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    body: _any_pb2.Any
    type: str
    def __init__(self, type: _Optional[str] = ..., body: _Optional[_Union[_any_pb2.Any, _Mapping]] = ...) -> None: ...

class ResponsePackJson(_message.Message):
    __slots__ = ["body", "msg", "ok", "type"]
    BODY_FIELD_NUMBER: _ClassVar[int]
    MSG_FIELD_NUMBER: _ClassVar[int]
    OK_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    body: _any_pb2.Any
    msg: str
    ok: bool
    type: str
    def __init__(self, type: _Optional[str] = ..., body: _Optional[_Union[_any_pb2.Any, _Mapping]] = ..., ok: bool = ..., msg: _Optional[str] = ...) -> None: ...

class Subscription(_message.Message):
    __slots__ = ["sub", "topics"]
    SUB_FIELD_NUMBER: _ClassVar[int]
    TOPICS_FIELD_NUMBER: _ClassVar[int]
    sub: bool
    topics: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, sub: bool = ..., topics: _Optional[_Iterable[str]] = ...) -> None: ...
