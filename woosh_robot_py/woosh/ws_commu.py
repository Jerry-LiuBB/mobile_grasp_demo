"""WebSocket通信实现

基于WebSocket协议的通信实现，支持请求-响应和发布-订阅模式。
"""

import asyncio
import json
import enum
import logging
import time
from typing import Optional, Dict, Any, Callable, List, Set, Tuple

import websockets

from .message_serializer import MessageSerializer
from .message_pack import RequestPack, ResponsePack, SubscriptionPack, NotifyPack, MessagePackGuards


# 常量定义
INITIAL_RECONNECT_DELAY = 1.0
MAX_RECONNECT_DELAY = 30.0
PING_INTERVAL = 30.0
PING_TIMEOUT = 20.0


class ConnectionState(enum.Enum):
    """连接状态枚举"""

    DISCONNECTED = 0  # 已断开
    CONNECTING = 1  # 连接中
    CONNECTED = 2  # 已连接
    RECONNECTING = 3  # 重连中


class AsyncWebSocket:
    """异步WebSocket通信类

    实现了基于WebSocket的通信功能，支持：
    1. 连接管理
    2. 请求-响应模式
    3. 发布-订阅模式
    """

    def __init__(
        self,
        addr: str,
        port: int,
        poll_timeout: int,
        logger: Optional[logging.Logger] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
        connection_callback: Optional[
            Callable[[bool, Optional[Exception]], None]
        ] = None,
    ) -> None:
        """初始化WebSocket通信类

        Args:
            addr: WebSocket服务器地址
            port: WebSocket服务器端口
            poll_timeout: 轮询超时时间（毫秒）
            logger: 日志记录器，如果为None则使用默认记录器
            loop: 事件循环，如果为None则使用当前事件循环
            connection_callback: 连接状态变化回调，接收(connected, error)参数
        """
        # WebSocket相关
        self._ws_url = f"ws://{addr}:{port}"
        self._ws = None

        # 轮询超时时间（毫秒）
        self.poll_timeout = poll_timeout

        # 请求-响应相关
        self._req_num = 0
        self._response_queues: Dict[int, asyncio.Queue] = {}

        # 待发送消息队列
        self._pending_messages: asyncio.Queue = asyncio.Queue()
        self._pending_futures: Dict[int, asyncio.Future] = {}

        # 重连相关
        self._reconnect_attempts = 0
        self._reconnect_delay = INITIAL_RECONNECT_DELAY

        # Ping 相关
        self._last_ping = 0.0
        self._last_pong = 0.0

        # 维护任务
        self._maintenance_task: Optional[asyncio.Task] = None
        self._connection_future: Optional[asyncio.Future] = None

        # 连接状态相关
        self._connection_state = ConnectionState.DISCONNECTED
        self._connection_error: Optional[Exception] = None
        self._connection_state_changed = asyncio.Event()
        self._on_connection_change = connection_callback
        self._running = False

        # 设置日志记录器
        self.logger = logger or logging.getLogger(__name__)

        # 发布-订阅相关
        self._subscribed_topics: Set[str] = set()
        self._topic_callbacks: Dict[str, Callable[[NotifyPack], None]] = {}

        # 事件循环和任务管理
        self._loop = loop or asyncio.get_event_loop()
        self._active_tasks: List[asyncio.Task] = []

    # 连接状态管理方法
    def is_connected(self) -> bool:
        """检查是否已连接

        Returns:
            bool: 是否已连接
        """
        return (
            self._connection_state == ConnectionState.CONNECTED and self._ws is not None
        )

    @property
    def connected(self) -> bool:
        """是否已连接（兼容旧接口）"""
        return self.is_connected()

    @property
    def running(self) -> bool:
        """是否正在运行"""
        return self._running

    def get_connection_state(self) -> Tuple[ConnectionState, Optional[Exception]]:
        """获取当前连接状态和错误信息

        Returns:
            Tuple[ConnectionState, Optional[Exception]]: (连接状态, 错误信息)
        """
        return self._connection_state, self._connection_error

    async def wait_for_state(
        self, state: ConnectionState, timeout: Optional[float] = None
    ) -> bool:
        """等待连接状态变为指定状态

        Args:
            state: 目标连接状态
            timeout: 超时时间（秒），None表示无限等待

        Returns:
            bool: 是否达到目标状态
        """
        if self._connection_state == state:
            return True

        self._connection_state_changed.clear()

        while self._connection_state != state:
            try:
                await asyncio.wait_for(
                    self._connection_state_changed.wait(), timeout=timeout
                )
                self._connection_state_changed.clear()

                if self._connection_state == state:
                    return True
            except asyncio.TimeoutError:
                return False

        return True

    def _set_connection_state(
        self, state: ConnectionState, error: Optional[Exception] = None
    ) -> None:
        """设置连接状态

        Args:
            state: 新的连接状态
            error: 如果有错误，提供错误信息
        """
        if self._connection_state == state and self._connection_error == error:
            return

        old_state = self._connection_state
        self._connection_state = state
        self._connection_error = error

        # 通知状态变化
        self._connection_state_changed.set()

        # 如果是连接状态变化，调用回调
        is_connected = state == ConnectionState.CONNECTED
        was_connected = old_state == ConnectionState.CONNECTED

        if is_connected != was_connected and self._on_connection_change:
            self._on_connection_change(is_connected, error)

    # 任务管理方法
    def _create_task(self, coro) -> asyncio.Task:
        """创建异步任务

        Args:
            coro: 协程对象

        Returns:
            asyncio.Task: 创建的任务
        """
        task = self._loop.create_task(coro)
        self._active_tasks.append(task)
        return task

    async def _cancel_tasks(self) -> None:
        """取消所有活跃任务"""
        for task in self._active_tasks:
            if not task.done():
                task.cancel()
                try:
                    await asyncio.gather(asyncio.shield(task), return_exceptions=True)
                except (asyncio.CancelledError, asyncio.TimeoutError):
                    # 如果任务被取消或等待超时，继续处理下一个任务
                    pass
                except Exception as e:
                    # 记录其他异常但不阻止继续处理
                    self.logger.error(f"Error while cancelling task: {str(e)}")
        self._active_tasks.clear()

    # 连接管理方法
    async def ensure_connected(self, timeout: float = 5.0) -> bool:
        """确保已连接，如果未连接则尝试连接

        Args:
            timeout: 连接超时时间（秒）

        Returns:
            bool: 是否已连接
        """
        if self.is_connected():
            return True

        return await self.connect(timeout=timeout)

    async def _handle_subscription(self, notify: NotifyPack) -> None:
        """处理订阅消息

        Args:
            notify: 通知包
        """
        if notify.type in self._topic_callbacks:
            try:
                callback = self._topic_callbacks[notify.type]
                if asyncio.iscoroutinefunction(callback):
                    await callback(notify)
                else:
                    callback(notify)
            except Exception as e:
                self.logger.error(f"Subscription callback error: {str(e)}")

    async def _process_message(self, message: str) -> None:
        """处理接收到的消息"""
        try:
            data = json.loads(message) 

            # 使用MessagePackGuards验证消息格式
            if not MessagePackGuards.is_message_pack(data):
                self.logger.warning(f"Received invalid message format: {data}")
                return

            if MessagePackGuards.is_response_pack(data):  # 响应消息
                response = ResponsePack.from_dict(data)
                await self._handle_response(response)
            else:  # 订阅消息
                notify = NotifyPack.from_dict(data)
                await self._handle_subscription(notify)
        except Exception as e:
            self.logger.error(f"Process message error: {str(e)}")

    async def _receive_messages(self) -> None:
        """接收消息的后台任务"""
        try:
            while self.is_connected():
                try:
                    message = await self._ws.recv()
                    await self._process_message(message)
                except websockets.ConnectionClosed as e:
                    self.logger.warning(
                        f"WebSocket connection closed while receiving: {str(e)}"
                    )
                    break
                except Exception as e:
                    self.logger.error(f"Error receiving message: {str(e)}")
                    break
        finally:
            if self.is_connected():
                await self._handle_connection_closed()

    async def _process_pending_messages(self) -> None:
        """处理待发送的消息队列"""
        while self.is_connected():
            try:
                # 获取下一个待发送的消息
                req_num, topic, message = await self._pending_messages.get()

                try:
                    # 准备并发送请求
                    request = RequestPack(
                        type=topic,
                        body=MessageSerializer.serialize(message),
                        timestamp=int(time.time() * 1000),
                        sn=req_num
                    )
                    await self._ws.send(json.dumps(request.to_dict()))
                except Exception as e:
                    # 如果发送失败，将错误结果设置到对应的Future中
                    if req_num in self._pending_futures:
                        future = self._pending_futures.pop(req_num)
                        if not future.done():
                            future.set_result(
                                ResponsePack.error(
                                    f"Send request failed: {str(e)}", req_num
                                )
                            )
                finally:
                    self._pending_messages.task_done()
            except asyncio.CancelledError:
                # 任务被取消，退出循环
                break
            except Exception as e:
                self.logger.error(f"Process pending message error: {str(e)}")
                # 继续处理下一个消息
                continue

    async def _ping_task(self) -> None:
        """发送定期ping以保持连接活跃"""
        while self.is_connected():
            try:
                # 发送ping并等待pong响应
                pong_waiter = await self._ws.ping()
                self._last_ping = self._loop.time()

                # 等待pong响应，设置超时
                try:
                    await asyncio.wait_for(pong_waiter, timeout=PING_TIMEOUT)
                    # 记录成功接收到pong
                    self._last_pong = self._loop.time()
                    self.logger.debug(
                        f"Ping sent at {self._last_ping}, Pong received at {self._last_pong}. Time difference: {self._last_pong - self._last_ping} seconds"
                    )

                except asyncio.TimeoutError:
                    self.logger.warning(
                        f"Ping sent at {self._last_ping}, Pong received at {self._last_pong}. Ping timeout, no pong received"
                    )
                    # 主动断开连接并触发重连
                    if self.is_connected():
                        await self._handle_connection_closed(Exception("Ping timeout"))
                    break

                # 等待下一个ping周期
                await asyncio.sleep(PING_INTERVAL)
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Ping failed: {str(e)}")
                break

    async def connect(self, timeout: float = 10.0) -> bool:
        """建立WebSocket连接

        此方法不会阻塞，而是启动一个后台任务来维护连接，并等待初始连接建立或超时。

        Args:
            timeout: 连接超时时间（秒）

        Returns:
            bool: 连接是否成功
        """
        if self.is_connected():
            return True

        # 设置运行状态
        self._running = True

        # 设置连接状态为连接中
        self._set_connection_state(ConnectionState.CONNECTING)

        # 启动连接维护任务
        if not self._maintenance_task or self._maintenance_task.done():
            self._maintenance_task = self._create_task(self._maintain_connection())

        # 等待连接建立或超时
        try:
            success = await self.wait_for_state(
                ConnectionState.CONNECTED, timeout=timeout
            )
            return success
        except Exception as e:
            self.logger.error(f"Connect failed: {str(e)}")
            return False

    async def _maintain_connection(self) -> None:
        """内部方法：维护WebSocket连接，包括断线重连"""
        while self._running:
            try:
                # 如果当前状态是重连中，记录日志
                if self._connection_state == ConnectionState.RECONNECTING:
                    self.logger.info(
                        f"Reconnection attempt {self._reconnect_attempts + 1}"
                    )

                # 连接WebSocket
                async with websockets.connect(self._ws_url) as websocket:
                    self.logger.info(f"Connected to {self._ws_url}")
                    self._ws = websocket

                    # 重置重连参数
                    self._reconnect_attempts = 0
                    self._reconnect_delay = INITIAL_RECONNECT_DELAY

                    # 更新连接状态
                    self._set_connection_state(ConnectionState.CONNECTED)

                    # 重新订阅之前的主题
                    if self._subscribed_topics:
                        await self._resubscribe_topics()

                    # 先取消之前可能存在的任务
                    await self._cancel_tasks()

                    # 启动消息处理、接收任务和ping任务
                    pending_task = self._create_task(self._process_pending_messages())
                    receive_task = self._create_task(self._receive_messages())
                    ping_task = self._create_task(self._ping_task())

                    # 创建一个临时变量存储当前连接的任务，用于等待
                    connection_tasks = [pending_task, receive_task, ping_task]

                    # 等待任意一个任务结束
                    done, pending = await asyncio.wait(
                        connection_tasks,
                        return_when=asyncio.FIRST_COMPLETED,
                    )

                    # 取消未完成的任务
                    for task in pending:
                        task.cancel()
                        try:
                            await asyncio.gather(
                                asyncio.shield(task), return_exceptions=True
                            )
                        except (asyncio.CancelledError, asyncio.TimeoutError):
                            # 如果任务被取消或等待超时，继续处理
                            pass
                        except Exception as e:
                            # 记录其他异常但不阻止继续处理
                            self.logger.error(f"Error while cancelling task: {str(e)}")

            except websockets.ConnectionClosed as e:
                self.logger.warning(f"WebSocket connection closed: {str(e)}")
                await self._handle_connection_closed(e)
            except Exception as e:
                self.logger.error(f"WebSocket error: {str(e)}")
                await self._handle_connection_closed(e)

            # 如果不再运行，退出循环
            if not self._running:
                break

            # 更新连接状态为重连中
            self._set_connection_state(
                ConnectionState.RECONNECTING, error=self._connection_error
            )

            # 重连逻辑
            self._reconnect_attempts += 1

            # 计算重连延迟（指数退避）
            backoff_delay = min(self._reconnect_delay * 2, MAX_RECONNECT_DELAY)

            # 保存当前重连延迟用于下次计算
            self._reconnect_delay = backoff_delay

            # 等待重连延迟
            self.logger.info(f"Reconnecting in {backoff_delay:.1f} seconds...")
            await asyncio.sleep(backoff_delay)

    async def _handle_connection_closed(
        self, error: Optional[Exception] = None
    ) -> None:
        """处理连接断开

        Args:
            error: 导致连接断开的错误，如果有的话
        """
        # 更新连接状态
        self._set_connection_state(ConnectionState.DISCONNECTED, error=error)

        # 重置WebSocket
        self._ws = None

        # 清理所有待处理的消息
        error_msg = str(error) if error else "Connection closed"
        self._clear_pending_messages(f"Connection closed: {error_msg}")

        # 取消所有活跃任务
        await self._cancel_tasks()

    def _clear_pending_messages(self, error_message: str) -> None:
        """清理所有待处理的消息"""
        # 清理待发送消息队列
        while not self._pending_messages.empty():
            try:
                req_num, _, _ = self._pending_messages.get_nowait()
                if req_num in self._pending_futures:
                    future = self._pending_futures.pop(req_num)
                    if not future.done():
                        future.set_result(ResponsePack.error(error_message, req_num))
            except asyncio.QueueEmpty:
                break

        # 清理剩余的futures
        for req_num, future in list(self._pending_futures.items()):
            if not future.done():
                future.set_result(ResponsePack.error(error_message, req_num))
        self._pending_futures.clear()

    async def disconnect(self) -> None:
        """断开WebSocket连接"""
        self._running = False

        # 取消维护任务
        if self._maintenance_task and not self._maintenance_task.done():
            self._maintenance_task.cancel()
            try:
                await self._maintenance_task
            except asyncio.CancelledError:
                pass
            self._maintenance_task = None

        # 清理所有待处理的消息
        self._clear_pending_messages("Connection closed by user")

        # 取消所有活跃任务
        await self._cancel_tasks()

        if not self.is_connected():
            return

        # 取消所有订阅
        if self._subscribed_topics:
            try:
                await self._unsubscribe_all()
            except Exception as e:
                self.logger.error(f"Failed to unsubscribe topics: {str(e)}")

        # 关闭WebSocket连接
        if self._ws:
            try:
                await self._ws.close()
            except Exception as e:
                self.logger.error(f"Close WebSocket failed: {str(e)}")
            finally:
                self._ws = None
                # 更新连接状态
                self._set_connection_state(ConnectionState.DISCONNECTED)

        # 清理状态
        self._response_queues.clear()
        self._subscribed_topics.clear()
        self._topic_callbacks.clear()

    async def _handle_response(self, response: ResponsePack) -> None:
        """处理响应消息"""
        sn = response.sn

        # 如果存在对应的Future，设置结果
        if sn in self._pending_futures:
            future = self._pending_futures[sn]
            if not future.done():
                future.set_result(response)

        # 如果存在对应的响应队列，也发送响应
        if sn in self._response_queues:
            await self._response_queues[sn].put(response)

    async def send(self, topic: str, message: Any) -> ResponsePack:
        """发送请求消息

        Args:
            topic: 消息主题
            message: 消息内容

        Returns:
            ResponsePack: 响应包。所有请求都会进入消息队列，
                        按顺序处理并等待响应。
        """
        # 检查连接状态
        if not self.is_connected():
            if not await self.ensure_connected(timeout=5.0):
                return ResponsePack.error("Not connected and reconnection failed", 0)

        # 生成请求编号
        self._req_num = (self._req_num + 1) % 0x7FFFFFFF
        req_num = self._req_num

        # 创建响应队列和Future
        response_queue = asyncio.Queue()
        self._response_queues[req_num] = response_queue
        future = asyncio.Future()
        self._pending_futures[req_num] = future

        # 将消息加入待发送队列
        await self._pending_messages.put((req_num, topic, message))

        try:
            # 等待消息被处理
            timeout = self.poll_timeout / 1000
            response = await asyncio.wait_for(future, timeout=timeout)
            return response
        except asyncio.TimeoutError:
            return ResponsePack.error(f"Request timeout: {topic}", req_num)
        finally:
            self._response_queues.pop(req_num, None)
            self._pending_futures.pop(req_num, None)

    async def subscribe(
        self, topic: str, callback: Callable[[NotifyPack], None]
    ) -> bool:
        """订阅主题

        Args:
            topic: 主题
            callback: 回调函数，接收 NotifyPack 参数

        Returns:
            bool: 是否成功。注意：返回True不代表已经完成订阅，
                 如果连接尚未建立，会在连接建立后自动完成订阅。
        """
        if topic in self._subscribed_topics:
            return True

        # 注册回调和记录订阅主题
        self._topic_callbacks[topic] = callback
        self._subscribed_topics.add(topic)

        # 如果连接已建立，立即发送订阅请求
        if self.is_connected():
            try:
                subscription = SubscriptionPack.create([topic], True) 
                await self._ws.send(json.dumps(subscription.to_dict()))
                self.logger.info(f"Subscribed to topic: {topic}")
                return True
            except Exception as e:
                self.logger.error(f"Subscribe failed: {str(e)}")
                # 发送失败时不回滚订阅状态，因为会在重连后自动重新订阅
                return False

        # 连接未建立时，返回True表示订阅信息已记录
        # 实际订阅会在连接建立后通过_resubscribe_topics完成
        self.logger.info(f"Topic {topic} will be subscribed when connected")
        return True

    async def unsubscribe(self, topic: str) -> bool:
        """取消订阅主题"""
        if topic not in self._subscribed_topics:
            return True

        # 无论是否连接，都从内部状态中移除
        self._topic_callbacks.pop(topic, None)
        self._subscribed_topics.remove(topic)

        # 如果已连接，发送取消订阅请求
        if self.is_connected():
            try:
                subscription = SubscriptionPack.create([topic], False) 
                await self._ws.send(json.dumps(subscription.to_dict()))
                self.logger.info(f"Unsubscribed from topic: {topic}")
                return True
            except Exception as e:
                self.logger.error(f"Unsubscribe failed: {str(e)}")
                return False

        return True

    async def _resubscribe_topics(self) -> None:
        """重新订阅所有主题"""
        if not self._subscribed_topics:
            return

        try:
            topics = list(self._subscribed_topics)
            subscription = SubscriptionPack.create(topics, True) 
            await self._ws.send(json.dumps(subscription.to_dict()))
            self.logger.info(f"Resubscribed topics: {topics}")
        except Exception as e:
            self.logger.error(f"Resubscribe failed: {str(e)}")

    async def _unsubscribe_all(self) -> None:
        """取消所有订阅"""
        if not self._subscribed_topics:
            return

        try:
            topics = list(self._subscribed_topics)
            subscription = SubscriptionPack.create(topics, False) 
            await self._ws.send(json.dumps(subscription.to_dict()))
            self.logger.info(f"Unsubscribed all topics: {topics}")
        except Exception as e:
            self.logger.error(f"Unsubscribe all failed: {str(e)}")

    async def __aenter__(self):
        """异步上下文管理器入口"""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """异步上下文管理器出口"""
        await self.disconnect()

    def __del__(self):
        """析构函数，确保资源被释放"""
        if self._running:
            self._running = False
