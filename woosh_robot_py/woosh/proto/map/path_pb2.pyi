from woosh.proto.util import common_pb2 as _common_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class BidirectPath(_message.Message):
    __slots__ = ["allow_avoid", "path", "width"]
    ALLOW_AVOID_FIELD_NUMBER: _ClassVar[int]
    PATH_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    allow_avoid: bool
    path: _containers.RepeatedCompositeFieldContainer[_common_pb2.Point]
    width: float
    def __init__(self, path: _Optional[_Iterable[_Union[_common_pb2.Point, _Mapping]]] = ..., width: _Optional[float] = ..., allow_avoid: bool = ...) -> None: ...

class MonoPath(_message.Message):
    __slots__ = ["allow_avoid", "path", "width"]
    ALLOW_AVOID_FIELD_NUMBER: _ClassVar[int]
    PATH_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    allow_avoid: bool
    path: _containers.RepeatedCompositeFieldContainer[_common_pb2.Point]
    width: float
    def __init__(self, path: _Optional[_Iterable[_Union[_common_pb2.Point, _Mapping]]] = ..., width: _Optional[float] = ..., allow_avoid: bool = ...) -> None: ...

class PathInfo(_message.Message):
    __slots__ = ["bidirect_paths", "mono_paths"]
    BIDIRECT_PATHS_FIELD_NUMBER: _ClassVar[int]
    MONO_PATHS_FIELD_NUMBER: _ClassVar[int]
    bidirect_paths: _containers.RepeatedCompositeFieldContainer[BidirectPath]
    mono_paths: _containers.RepeatedCompositeFieldContainer[MonoPath]
    def __init__(self, mono_paths: _Optional[_Iterable[_Union[MonoPath, _Mapping]]] = ..., bidirect_paths: _Optional[_Iterable[_Union[BidirectPath, _Mapping]]] = ...) -> None: ...
