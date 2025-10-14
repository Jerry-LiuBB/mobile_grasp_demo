from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.map import mark_pb2 as _mark_pb2
from woosh.proto.map import field_pb2 as _field_pb2
from woosh.proto.map import path_pb2 as _path_pb2
from woosh.proto.task import woosh_task_pb2 as _woosh_task_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Delete(_message.Message):
    __slots__ = ["map_name", "scene_name"]
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    map_name: str
    scene_name: str
    def __init__(self, scene_name: _Optional[str] = ..., map_name: _Optional[str] = ...) -> None: ...

class Download(_message.Message):
    __slots__ = ["scene_name"]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    scene_name: str
    def __init__(self, scene_name: _Optional[str] = ...) -> None: ...

class DownloadResponse(_message.Message):
    __slots__ = ["file_datas"]
    FILE_DATAS_FIELD_NUMBER: _ClassVar[int]
    file_datas: _containers.RepeatedCompositeFieldContainer[_common_pb2.FileData]
    def __init__(self, file_datas: _Optional[_Iterable[_Union[_common_pb2.FileData, _Mapping]]] = ...) -> None: ...

class Rename(_message.Message):
    __slots__ = ["new_map_name", "new_scene_name", "old_map_name", "old_scene_name"]
    NEW_MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    NEW_SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    OLD_MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    OLD_SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    new_map_name: str
    new_scene_name: str
    old_map_name: str
    old_scene_name: str
    def __init__(self, old_scene_name: _Optional[str] = ..., new_scene_name: _Optional[str] = ..., old_map_name: _Optional[str] = ..., new_map_name: _Optional[str] = ...) -> None: ...

class SceneData(_message.Message):
    __slots__ = ["action_group", "maps", "name", "task_info"]
    class Map(_message.Message):
        __slots__ = ["end", "field_info", "id", "keepout_png", "map_png", "mark_info", "name", "origin", "path_info", "resolution", "time_write", "traffic_keepout_png"]
        END_FIELD_NUMBER: _ClassVar[int]
        FIELD_INFO_FIELD_NUMBER: _ClassVar[int]
        ID_FIELD_NUMBER: _ClassVar[int]
        KEEPOUT_PNG_FIELD_NUMBER: _ClassVar[int]
        MAP_PNG_FIELD_NUMBER: _ClassVar[int]
        MARK_INFO_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        ORIGIN_FIELD_NUMBER: _ClassVar[int]
        PATH_INFO_FIELD_NUMBER: _ClassVar[int]
        RESOLUTION_FIELD_NUMBER: _ClassVar[int]
        TIME_WRITE_FIELD_NUMBER: _ClassVar[int]
        TRAFFIC_KEEPOUT_PNG_FIELD_NUMBER: _ClassVar[int]
        end: _common_pb2.Pose2D
        field_info: _field_pb2.FieldInfo
        id: int
        keepout_png: bytes
        map_png: bytes
        mark_info: _mark_pb2.MarkInfo
        name: str
        origin: _common_pb2.Pose2D
        path_info: _path_pb2.PathInfo
        resolution: float
        time_write: str
        traffic_keepout_png: bytes
        def __init__(self, id: _Optional[int] = ..., name: _Optional[str] = ..., resolution: _Optional[float] = ..., origin: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., end: _Optional[_Union[_common_pb2.Pose2D, _Mapping]] = ..., map_png: _Optional[bytes] = ..., keepout_png: _Optional[bytes] = ..., traffic_keepout_png: _Optional[bytes] = ..., mark_info: _Optional[_Union[_mark_pb2.MarkInfo, _Mapping]] = ..., field_info: _Optional[_Union[_field_pb2.FieldInfo, _Mapping]] = ..., path_info: _Optional[_Union[_path_pb2.PathInfo, _Mapping]] = ..., time_write: _Optional[str] = ...) -> None: ...
    ACTION_GROUP_FIELD_NUMBER: _ClassVar[int]
    MAPS_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    TASK_INFO_FIELD_NUMBER: _ClassVar[int]
    action_group: str
    maps: _containers.RepeatedCompositeFieldContainer[SceneData.Map]
    name: str
    task_info: _woosh_task_pb2.TaskInfo
    def __init__(self, name: _Optional[str] = ..., maps: _Optional[_Iterable[_Union[SceneData.Map, _Mapping]]] = ..., task_info: _Optional[_Union[_woosh_task_pb2.TaskInfo, _Mapping]] = ..., action_group: _Optional[str] = ...) -> None: ...

class SceneDataEasy(_message.Message):
    __slots__ = ["maps", "name"]
    class Map(_message.Message):
        __slots__ = ["floor", "id", "name", "storages", "version"]
        FLOOR_FIELD_NUMBER: _ClassVar[int]
        ID_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        STORAGES_FIELD_NUMBER: _ClassVar[int]
        VERSION_FIELD_NUMBER: _ClassVar[int]
        floor: str
        id: int
        name: str
        storages: _mark_pb2.Storages
        version: int
        def __init__(self, id: _Optional[int] = ..., name: _Optional[str] = ..., floor: _Optional[str] = ..., version: _Optional[int] = ..., storages: _Optional[_Union[_mark_pb2.Storages, _Mapping]] = ...) -> None: ...
    MAPS_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    maps: _containers.RepeatedCompositeFieldContainer[SceneDataEasy.Map]
    name: str
    def __init__(self, name: _Optional[str] = ..., maps: _Optional[_Iterable[_Union[SceneDataEasy.Map, _Mapping]]] = ...) -> None: ...

class SceneFileMD5(_message.Message):
    __slots__ = ["maps", "scene_name", "scenes_md5s"]
    class MapFileInfo(_message.Message):
        __slots__ = ["map_md5s", "map_name", "version"]
        MAP_MD5S_FIELD_NUMBER: _ClassVar[int]
        MAP_NAME_FIELD_NUMBER: _ClassVar[int]
        VERSION_FIELD_NUMBER: _ClassVar[int]
        map_md5s: _containers.RepeatedCompositeFieldContainer[_common_pb2.FileMD5]
        map_name: str
        version: int
        def __init__(self, map_name: _Optional[str] = ..., version: _Optional[int] = ..., map_md5s: _Optional[_Iterable[_Union[_common_pb2.FileMD5, _Mapping]]] = ...) -> None: ...
    MAPS_FIELD_NUMBER: _ClassVar[int]
    SCENES_MD5S_FIELD_NUMBER: _ClassVar[int]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    maps: _containers.RepeatedCompositeFieldContainer[SceneFileMD5.MapFileInfo]
    scene_name: str
    scenes_md5s: _containers.RepeatedCompositeFieldContainer[_common_pb2.FileMD5]
    def __init__(self, scene_name: _Optional[str] = ..., scenes_md5s: _Optional[_Iterable[_Union[_common_pb2.FileMD5, _Mapping]]] = ..., maps: _Optional[_Iterable[_Union[SceneFileMD5.MapFileInfo, _Mapping]]] = ...) -> None: ...

class SceneList(_message.Message):
    __slots__ = ["scenes"]
    class Scene(_message.Message):
        __slots__ = ["maps", "name"]
        MAPS_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        maps: _containers.RepeatedScalarFieldContainer[str]
        name: str
        def __init__(self, name: _Optional[str] = ..., maps: _Optional[_Iterable[str]] = ...) -> None: ...
    SCENES_FIELD_NUMBER: _ClassVar[int]
    scenes: _containers.RepeatedCompositeFieldContainer[SceneList.Scene]
    def __init__(self, scenes: _Optional[_Iterable[_Union[SceneList.Scene, _Mapping]]] = ...) -> None: ...

class SceneMd5(_message.Message):
    __slots__ = ["scene_name"]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    scene_name: str
    def __init__(self, scene_name: _Optional[str] = ...) -> None: ...

class SceneMd5Response(_message.Message):
    __slots__ = ["scenes"]
    SCENES_FIELD_NUMBER: _ClassVar[int]
    scenes: _containers.RepeatedCompositeFieldContainer[SceneFileMD5]
    def __init__(self, scenes: _Optional[_Iterable[_Union[SceneFileMD5, _Mapping]]] = ...) -> None: ...

class SceneSync(_message.Message):
    __slots__ = ["scenes"]
    SCENES_FIELD_NUMBER: _ClassVar[int]
    scenes: _containers.RepeatedCompositeFieldContainer[SceneFileMD5]
    def __init__(self, scenes: _Optional[_Iterable[_Union[SceneFileMD5, _Mapping]]] = ...) -> None: ...

class SceneSyncResponse(_message.Message):
    __slots__ = ["file_datas"]
    FILE_DATAS_FIELD_NUMBER: _ClassVar[int]
    file_datas: _containers.RepeatedCompositeFieldContainer[_common_pb2.FileData]
    def __init__(self, file_datas: _Optional[_Iterable[_Union[_common_pb2.FileData, _Mapping]]] = ...) -> None: ...

class Upload(_message.Message):
    __slots__ = ["file_datas", "method", "scene_name"]
    class Method(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    FILE_DATAS_FIELD_NUMBER: _ClassVar[int]
    METHOD_FIELD_NUMBER: _ClassVar[int]
    SCENE_NAME_FIELD_NUMBER: _ClassVar[int]
    file_datas: _containers.RepeatedCompositeFieldContainer[_common_pb2.FileData]
    kFull: Upload.Method
    kIncr: Upload.Method
    method: Upload.Method
    scene_name: str
    def __init__(self, scene_name: _Optional[str] = ..., file_datas: _Optional[_Iterable[_Union[_common_pb2.FileData, _Mapping]]] = ..., method: _Optional[_Union[Upload.Method, str]] = ...) -> None: ...
