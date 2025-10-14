"""消息序列化模块

提供protobuf消息和字典之间的序列化和反序列化功能。
支持protobuf消息、字典和JSON格式之间的转换。
"""

from typing import Union, Optional, Type, Any, Dict, TypeVar, cast
import json
import google.protobuf.message as pbmsg
from google.protobuf.json_format import MessageToDict, ParseDict, Parse as ParseJson
from google.protobuf import descriptor_pool
from google.protobuf.message_factory import MessageFactory

# 类型变量定义，用于改进类型提示
T = TypeVar("T", bound=pbmsg.Message)


class SerializationError(Exception):
    """序列化错误"""

    pass


class DeserializationError(Exception):
    """反序列化错误"""

    pass


class MessageTypeError(Exception):
    """消息类型错误"""

    pass


class MessageSerializer:
    """消息序列化器

    提供以下功能:
    1. protobuf消息序列化为字典
    2. 字典反序列化为protobuf消息
    3. JSON字符串和protobuf消息互转
    4. 通过消息类型名称创建protobuf消息实例
    """

    @staticmethod
    def get_message_type(message_type_name: str) -> Type[pbmsg.Message]:
        """通过消息类型名称获取protobuf消息类型

        Args:
            message_type_name: protobuf消息类型的全名

        Returns:
            Type[pbmsg.Message]: protobuf消息类型

        Raises:
            MessageTypeError: 消息类型不存在或无效
        """
        try:
            # 从描述符池中查找消息类型
            descriptor = descriptor_pool.Default().FindMessageTypeByName(
                message_type_name
            )
            if descriptor is None:
                raise MessageTypeError(f"Message type {message_type_name} not found")

            # 使用消息工厂创建消息类型
            return MessageFactory().GetPrototype(descriptor)
        except Exception as e:
            raise MessageTypeError(f"Failed to get message type: {str(e)}")

    @staticmethod
    def create_message(
        message_type_name: str, data: Optional[Dict[str, Any]] = None
    ) -> pbmsg.Message:
        """创建并初始化protobuf消息实例

        Args:
            message_type_name: protobuf消息类型的全名
            data: 用于初始化消息的字典数据

        Returns:
            pbmsg.Message: 创建的protobuf消息实例

        Raises:
            MessageTypeError: 消息类型不存在或无效
            DeserializationError: 数据反序列化失败
        """
        try:
            # 获取消息类型并创建实例
            message_type = MessageSerializer.get_message_type(message_type_name)
            message = message_type()

            # 如果提供了数据，进行初始化
            if data is not None:
                ParseDict(data, message)

            return message
        except MessageTypeError as e:
            raise e
        except Exception as e:
            raise DeserializationError(f"Failed to create message: {str(e)}")

    @staticmethod
    def serialize(message: Union[pbmsg.Message, dict]) -> dict:
        """序列化消息为字典

        Args:
            message: 要序列化的消息，可以是protobuf消息或字典

        Returns:
            dict: 序列化后的字典

        Raises:
            SerializationError: 序列化失败
            TypeError: 不支持的消息类型
        """
        try:
            if isinstance(message, pbmsg.Message):
                return MessageToDict(message, use_integers_for_enums=True)
            elif isinstance(message, dict):
                return message
            else:
                raise TypeError(f"Unsupported message type: {type(message)}")
        except Exception as e:
            raise SerializationError(f"Failed to serialize message: {str(e)}")

    @staticmethod
    def deserialize(
        data: Union[dict, str], message_type: Optional[Type[T]] = None
    ) -> Union[T, dict]:
        """反序列化数据为消息

        Args:
            data: 要反序列化的数据，可以是字典或JSON字符串
            message_type: 目标protobuf消息类型

        Returns:
            Union[T, dict]: 反序列化后的消息

        Raises:
            DeserializationError: 反序列化失败
        """
        try:
            # 如果数据是JSON字符串，先解析为字典
            if isinstance(data, str):
                data = json.loads(data)

            # 如果指定了消息类型，转换为protobuf消息
            if message_type and issubclass(message_type, pbmsg.Message):
                msg = message_type()
                ParseDict(data, msg)
                return cast(T, msg)

            return data
        except Exception as e:
            raise DeserializationError(f"Failed to deserialize message: {str(e)}")

    @staticmethod
    def to_json(message: Union[pbmsg.Message, dict]) -> str:
        """将消息转换为JSON字符串

        Args:
            message: 要转换的消息，可以是protobuf消息或字典

        Returns:
            str: JSON字符串

        Raises:
            SerializationError: 转换失败
            TypeError: 不支持的消息类型
        """
        try:
            if isinstance(message, pbmsg.Message):
                data = MessageSerializer.serialize(message)
            elif isinstance(message, dict):
                data = message
            else:
                raise TypeError(f"Unsupported message type: {type(message)}")

            return json.dumps(data)
        except Exception as e:
            raise SerializationError(f"Failed to convert message to JSON: {str(e)}")

    @staticmethod
    def from_json(
        json_str: str, message_type: Optional[Type[T]] = None
    ) -> Union[T, dict]:
        """从JSON字符串转换为消息

        Args:
            json_str: JSON字符串
            message_type: 目标protobuf消息类型

        Returns:
            Union[T, dict]: 转换后的消息

        Raises:
            DeserializationError: 转换失败
        """
        try:
            if message_type and issubclass(message_type, pbmsg.Message):
                msg = message_type()
                ParseJson(json_str, msg)
                return cast(T, msg)
            else:
                return json.loads(json_str)
        except Exception as e:
            raise DeserializationError(f"Failed to convert JSON to message: {str(e)}")
