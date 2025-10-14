from woosh.proto.util import common_pb2 as _common_pb2
from woosh.proto.robot import robot_count_pb2 as _robot_count_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AbnormalCount(_message.Message):
    __slots__ = ["fault", "fault_time", "warn", "warn_time"]
    FAULT_FIELD_NUMBER: _ClassVar[int]
    FAULT_TIME_FIELD_NUMBER: _ClassVar[int]
    WARN_FIELD_NUMBER: _ClassVar[int]
    WARN_TIME_FIELD_NUMBER: _ClassVar[int]
    fault: int
    fault_time: int
    warn: int
    warn_time: int
    def __init__(self, warn: _Optional[int] = ..., fault: _Optional[int] = ..., warn_time: _Optional[int] = ..., fault_time: _Optional[int] = ...) -> None: ...

class Abnormals(_message.Message):
    __slots__ = ["abns", "bucket", "page", "qty", "robots"]
    ABNS_FIELD_NUMBER: _ClassVar[int]
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    PAGE_FIELD_NUMBER: _ClassVar[int]
    QTY_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    abns: _containers.RepeatedCompositeFieldContainer[_robot_count_pb2.Status]
    bucket: _common_pb2.TimeBucket
    page: int
    qty: int
    robots: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robots: _Optional[_Iterable[int]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., page: _Optional[int] = ..., qty: _Optional[int] = ..., abns: _Optional[_Iterable[_Union[_robot_count_pb2.Status, _Mapping]]] = ...) -> None: ...

class DailyTask(_message.Message):
    __slots__ = ["completed", "day", "failed", "total"]
    COMPLETED_FIELD_NUMBER: _ClassVar[int]
    DAY_FIELD_NUMBER: _ClassVar[int]
    FAILED_FIELD_NUMBER: _ClassVar[int]
    TOTAL_FIELD_NUMBER: _ClassVar[int]
    completed: int
    day: str
    failed: int
    total: int
    def __init__(self, day: _Optional[str] = ..., completed: _Optional[int] = ..., failed: _Optional[int] = ..., total: _Optional[int] = ...) -> None: ...

class DailyTasks(_message.Message):
    __slots__ = ["days"]
    DAYS_FIELD_NUMBER: _ClassVar[int]
    days: _containers.RepeatedCompositeFieldContainer[DailyTask]
    def __init__(self, days: _Optional[_Iterable[_Union[DailyTask, _Mapping]]] = ...) -> None: ...

class Reliability(_message.Message):
    __slots__ = ["mtbf", "mttr"]
    MTBF_FIELD_NUMBER: _ClassVar[int]
    MTTR_FIELD_NUMBER: _ClassVar[int]
    mtbf: int
    mttr: int
    def __init__(self, mttr: _Optional[int] = ..., mtbf: _Optional[int] = ...) -> None: ...

class RobotOperation(_message.Message):
    __slots__ = ["abn_count", "abn_rate", "last_time", "mileage", "rel", "robot", "task_rate", "task_time", "uptime"]
    ABN_COUNT_FIELD_NUMBER: _ClassVar[int]
    ABN_RATE_FIELD_NUMBER: _ClassVar[int]
    LAST_TIME_FIELD_NUMBER: _ClassVar[int]
    MILEAGE_FIELD_NUMBER: _ClassVar[int]
    REL_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    TASK_RATE_FIELD_NUMBER: _ClassVar[int]
    TASK_TIME_FIELD_NUMBER: _ClassVar[int]
    UPTIME_FIELD_NUMBER: _ClassVar[int]
    abn_count: AbnormalCount
    abn_rate: int
    last_time: int
    mileage: int
    rel: Reliability
    robot: int
    task_rate: int
    task_time: int
    uptime: int
    def __init__(self, robot: _Optional[int] = ..., uptime: _Optional[int] = ..., mileage: _Optional[int] = ..., last_time: _Optional[int] = ..., task_time: _Optional[int] = ..., abn_count: _Optional[_Union[AbnormalCount, _Mapping]] = ..., abn_rate: _Optional[int] = ..., task_rate: _Optional[int] = ..., rel: _Optional[_Union[Reliability, _Mapping]] = ...) -> None: ...

class RobotOperations(_message.Message):
    __slots__ = ["bucket", "ops", "robots"]
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    OPS_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    bucket: _common_pb2.TimeBucket
    ops: _containers.RepeatedCompositeFieldContainer[RobotOperation]
    robots: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robots: _Optional[_Iterable[int]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., ops: _Optional[_Iterable[_Union[RobotOperation, _Mapping]]] = ...) -> None: ...

class RobotTaskCount(_message.Message):
    __slots__ = ["count", "robot"]
    COUNT_FIELD_NUMBER: _ClassVar[int]
    ROBOT_FIELD_NUMBER: _ClassVar[int]
    count: TaskCount
    robot: int
    def __init__(self, robot: _Optional[int] = ..., count: _Optional[_Union[TaskCount, _Mapping]] = ...) -> None: ...

class RobotTaskCounts(_message.Message):
    __slots__ = ["bucket", "count", "counts", "robots"]
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    COUNTS_FIELD_NUMBER: _ClassVar[int]
    COUNT_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    bucket: _common_pb2.TimeBucket
    count: TaskCount
    counts: _containers.RepeatedCompositeFieldContainer[RobotTaskCount]
    robots: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robots: _Optional[_Iterable[int]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., count: _Optional[_Union[TaskCount, _Mapping]] = ..., counts: _Optional[_Iterable[_Union[RobotTaskCount, _Mapping]]] = ...) -> None: ...

class RobotTimeline(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class RouteAvg(_message.Message):
    __slots__ = ["num", "route", "sum"]
    NUM_FIELD_NUMBER: _ClassVar[int]
    ROUTE_FIELD_NUMBER: _ClassVar[int]
    SUM_FIELD_NUMBER: _ClassVar[int]
    num: int
    route: str
    sum: int
    def __init__(self, route: _Optional[str] = ..., sum: _Optional[int] = ..., num: _Optional[int] = ...) -> None: ...

class RouteAvgs(_message.Message):
    __slots__ = ["avgs", "bucket", "robots"]
    AVGS_FIELD_NUMBER: _ClassVar[int]
    BUCKET_FIELD_NUMBER: _ClassVar[int]
    ROBOTS_FIELD_NUMBER: _ClassVar[int]
    avgs: _containers.RepeatedCompositeFieldContainer[RouteAvg]
    bucket: _common_pb2.TimeBucket
    robots: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robots: _Optional[_Iterable[int]] = ..., bucket: _Optional[_Union[_common_pb2.TimeBucket, _Mapping]] = ..., avgs: _Optional[_Iterable[_Union[RouteAvg, _Mapping]]] = ...) -> None: ...

class TaskCount(_message.Message):
    __slots__ = ["completed", "failed", "total"]
    COMPLETED_FIELD_NUMBER: _ClassVar[int]
    FAILED_FIELD_NUMBER: _ClassVar[int]
    TOTAL_FIELD_NUMBER: _ClassVar[int]
    completed: int
    failed: int
    total: int
    def __init__(self, completed: _Optional[int] = ..., failed: _Optional[int] = ..., total: _Optional[int] = ...) -> None: ...
