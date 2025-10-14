"""消息包模块

定义了各种消息包类型，用于WebSocket通信中的消息传输。
包括基础消息包、请求包、响应包、订阅包和通知包。
"""

from dataclasses import dataclass, field
from typing import List, Any, Optional
import time

from .message_serializer import MessageSerializer

# Constants
SUBSCRIPTION_TOPIC = "woosh.Subscription"


@dataclass
class MessagePack:
    """消息包基类

    用于WebSocket通信的基础消息格式，包含消息类型和消息体。
    """

    type: str
    body: dict

    def to_dict(self) -> dict:
        """转换为字典格式

        Returns:
            dict: 包含type和body
        """
        return {"type": self.type, "body": self.body}


@dataclass
class RequestPack(MessagePack):
    """请求包

    用于客户端发送请求的消息格式，在基础消息之上增加序列号。
    """

    sn: int
    timestamp: int = field(default_factory=lambda: int(time.time() * 1000))

    def to_dict(self) -> dict:
        """转换为字典格式

        Returns:
            dict: 包含type、body、timestamp和sn的字典
        """
        data = super().to_dict()
        data["sn"] = self.sn
        data["timestamp"] = self.timestamp
        return data


@dataclass
class ResponsePack(MessagePack):
    """响应包

    用于服务器响应请求的消息格式，包含请求处理的结果信息。
    """

    ok: bool
    msg: str
    sn: int
    timestamp: int

    def to_dict(self) -> dict:
        """转换为字典格式

        Returns:
            dict: 包含完整响应信息的字典
        """
        data = super().to_dict()
        data.update({"ok": self.ok, "msg": self.msg, "sn": self.sn, "timestamp": self.timestamp})
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "ResponsePack":
        """从字典创建响应包

        Args:
            data: 响应数据字典

        Returns:
            ResponsePack: 响应包实例
        """
        return cls(
            type=data.get("type", ""),
            body=data.get("body", {}),
            timestamp=data.get("timestamp", 0),
            ok=data.get("ok", False),
            msg=data.get("msg", ""),
            sn=data.get("sn", 0),
        )

    @classmethod
    def error(cls, msg: str, sn: int = 0) -> "ResponsePack":
        """创建错误响应包

        Args:
            msg: 错误信息
            sn: 序列号

        Returns:
            ResponsePack: 错误响应包实例
        """
        return cls(type="error", body={}, ok=False, msg=msg, sn=sn)


@dataclass
class SubscriptionPack(MessagePack):
    """订阅包

    用于订阅和取消订阅主题的消息格式。
    """

    topics: List[str]
    sub: bool

    def to_dict(self) -> dict:
        """转换为字典格式

        Returns:
            dict: 包含订阅信息的字典
        """
        data = super().to_dict()
        data["body"].update({"topics": self.topics, "sub": self.sub})
        return data

    @classmethod
    def create(cls, topics: List[str], sub: bool) -> "SubscriptionPack":
        """创建订阅包

        Args:
            topics: 主题列表
            sub: 是否订阅

        Returns:
            SubscriptionPack: 订阅包实例
        """
        return cls(type=SUBSCRIPTION_TOPIC, body={}, topics=topics, sub=sub)


@dataclass
class NotifyPack(MessagePack):
    """通知包

    用于服务器推送的通知消息，包括订阅消息和其他异步通知。
    """
    
    timestamp: int

    @classmethod
    def from_dict(cls, data: dict) -> "NotifyPack":
        """从字典创建通知包

        Args:
            data: 通知数据字典

        Returns:
            NotifyPack: 通知包实例
        """
        return cls(
            type=data.get("type", ""),
            body=data.get("body", {}),
            timestamp=data.get("timestamp", 0)
        )

    @classmethod
    def create(cls, topic: str, data: Any) -> "NotifyPack":
        """创建通知包

        Args:
            topic: 通知主题
            data: 通知数据

        Returns:
            NotifyPack: 通知包实例
        """
        return cls(type=topic, body=MessageSerializer.serialize(data))


class MessagePackGuards:
    """消息包类型守卫

    提供类似TypeScript版本的类型检查功能
    """
    
    @staticmethod
    def is_message_pack(obj: Any) -> bool:
        """检查是否为基础消息包"""
        return (
            isinstance(obj, dict) and
            "type" in obj and isinstance(obj["type"], str) and
            "body" in obj and isinstance(obj["body"], dict)
        )
    
    @staticmethod
    def is_request_pack(obj: Any) -> bool:
        """检查是否为请求包"""
        return (
            MessagePackGuards.is_message_pack(obj) and
            "sn" in obj and isinstance(obj["sn"], int)
        )
    
    @staticmethod
    def is_response_pack(obj: Any) -> bool:
        """检查是否为响应包"""
        return (
            MessagePackGuards.is_message_pack(obj) and
            "sn" in obj and isinstance(obj["sn"], int) and
            "ok" in obj and isinstance(obj["ok"], bool) and
            "msg" in obj and isinstance(obj["msg"], str)
        )
