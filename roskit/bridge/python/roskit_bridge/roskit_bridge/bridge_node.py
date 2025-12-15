"""RosWeb Bridge - Main ROS2 node with WebSocket server."""

import asyncio
import json
import time
import cbor2
import struct
import ssl
import os
from typing import Dict, Set, Any, Optional, Tuple
from dataclasses import dataclass, field
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    print("websockets package not found. Install with: pip install websockets")
    raise

from .protocol import (
    MessageType, Encoding, Flags,
    create_message, parse_message, MessageHeader
)
from .encoders import get_encoder


class LRUCache(OrderedDict):
    """Simple LRU cache with max size and optional TTL eviction."""

    def __init__(self, max_size: int = 100, ttl_seconds: float = 0):
        super().__init__()
        self.max_size = max_size
        self.ttl_seconds = ttl_seconds
        self._timestamps: Dict[Any, float] = {}

    def get(self, key, default=None):
        if key in self:
            # Check TTL
            if self.ttl_seconds > 0:
                if time.time() - self._timestamps.get(key, 0) > self.ttl_seconds:
                    self.pop(key, None)
                    self._timestamps.pop(key, None)
                    return default
            # Move to end (most recently used)
            self.move_to_end(key)
            return self[key]
        return default

    def set(self, key, value):
        if key in self:
            self.move_to_end(key)
        self[key] = value
        self._timestamps[key] = time.time()
        # Evict oldest entries if over capacity
        while len(self) > self.max_size:
            oldest = next(iter(self))
            self.pop(oldest)
            self._timestamps.pop(oldest, None)

    def evict_expired(self):
        """Remove all expired entries."""
        if self.ttl_seconds <= 0:
            return
        now = time.time()
        expired = [k for k, ts in self._timestamps.items()
                   if now - ts > self.ttl_seconds]
        for k in expired:
            self.pop(k, None)
            self._timestamps.pop(k, None)


class RateLimiter:
    """Simple token bucket rate limiter."""

    def __init__(self, rate: float, burst: int):
        """
        Args:
            rate: Tokens per second
            burst: Maximum burst size
        """
        self.rate = rate
        self.burst = burst
        self.tokens = float(burst)
        self.last_update = time.time()

    def allow(self) -> bool:
        """Check if request should be allowed."""
        now = time.time()
        elapsed = now - self.last_update
        self.last_update = now

        # Add tokens based on elapsed time
        self.tokens = min(self.burst, self.tokens + elapsed * self.rate)

        if self.tokens >= 1.0:
            self.tokens -= 1.0
            return True
        return False


@dataclass
class ClientRateLimits:
    """Per-client rate limiters."""
    subscribe: RateLimiter
    publish: RateLimiter
    service_call: RateLimiter

    @classmethod
    def create(cls, subscribe_rate: float = 10.0, publish_rate: float = 100.0,
               service_rate: float = 10.0, burst: int = 5):
        return cls(
            subscribe=RateLimiter(subscribe_rate, burst),
            publish=RateLimiter(publish_rate, burst * 10),
            service_call=RateLimiter(service_rate, burst),
        )


@dataclass
class Channel:
    """Represents a subscribed topic channel."""
    id: int
    topic: str
    msg_type: str
    subscription: Any = None
    encoder: Any = None
    throttle_ms: int = 0
    last_sent: float = 0
    clients: Set[WebSocketServerProtocol] = field(default_factory=set)


@dataclass
class ClientState:
    """State for a connected WebSocket client."""
    ws: WebSocketServerProtocol
    subscribed_channels: Set[int] = field(default_factory=set)


class RosWebBridge(Node):
    """ROS2 node that provides a high-performance WebSocket bridge."""

    def __init__(self):
        super().__init__('rosweb_bridge')

        # Declare parameters
        self.declare_parameter('port', 9091)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('max_clients', 10)
        # TLS parameters
        self.declare_parameter('tls_cert', '')
        self.declare_parameter('tls_key', '')
        # Authentication token (if set, clients must provide Authorization header)
        self.declare_parameter('auth_token', '')
        # Rate limiting parameters
        self.declare_parameter('rate_limit_subscribe', 10.0)
        self.declare_parameter('rate_limit_publish', 100.0)
        self.declare_parameter('rate_limit_service', 10.0)

        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.max_clients = self.get_parameter('max_clients').value
        self.tls_cert = self.get_parameter('tls_cert').value
        self.tls_key = self.get_parameter('tls_key').value
        self.auth_token = self.get_parameter('auth_token').value
        self._rate_limit_subscribe = self.get_parameter('rate_limit_subscribe').value
        self._rate_limit_publish = self.get_parameter('rate_limit_publish').value
        self._rate_limit_service = self.get_parameter('rate_limit_service').value

        # Channel management
        self.channels: Dict[int, Channel] = {}
        self.topic_to_channel: Dict[str, int] = {}
        self.next_channel_id = 1

        # Client management
        self.clients: Dict[WebSocketServerProtocol, ClientState] = {}
        self._client_rate_limits: Dict[WebSocketServerProtocol, ClientRateLimits] = {}

        # Server
        self.server = None
        self.server_task = None
        # LRU caches with TTL eviction for ROS resources
        self._publisher_cache: LRUCache = LRUCache(max_size=50, ttl_seconds=300)
        self._service_client_cache: LRUCache = LRUCache(max_size=20, ttl_seconds=300)

        # Periodic cache cleanup task
        self._cache_cleanup_task = None
        self.cache_cleanup_interval = float(os.environ.get('ROSKIT_CACHE_CLEANUP_INTERVAL', '60'))

        proto = 'wss' if self.tls_cert else 'ws'
        self.get_logger().info(f'RosWeb Bridge initialized on {proto}://{self.host}:{self.port}')

    async def start_server(self):
        """Start the WebSocket server."""
        # Build SSL context if TLS is configured
        ssl_context = None
        if self.tls_cert and self.tls_key:
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(self.tls_cert, self.tls_key)
            self.get_logger().info('TLS enabled')

        self.server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port,
            ssl=ssl_context,
            max_size=50 * 1024 * 1024,  # 50MB max message
            ping_interval=30,
            ping_timeout=10,
            process_request=self._process_request if self.auth_token else None,
        )

        # Start periodic cache cleanup
        self._cache_cleanup_task = asyncio.create_task(self._periodic_cache_cleanup())

        proto = 'wss' if ssl_context else 'ws'
        self.get_logger().info(f'WebSocket server started on {proto}://{self.host}:{self.port}')

    async def _process_request(self, path, request_headers):
        """Validate authentication token in request headers."""
        if not self.auth_token:
            return None  # No auth required

        auth_header = request_headers.get('Authorization', '')
        if auth_header.startswith('Bearer '):
            token = auth_header[7:]
            if token == self.auth_token:
                return None  # Auth successful
        # Check query parameter as fallback
        if '?' in path:
            query = path.split('?', 1)[1]
            for param in query.split('&'):
                if param.startswith('token='):
                    token = param[6:]
                    if token == self.auth_token:
                        return None  # Auth successful

        self.get_logger().warn('Unauthorized connection attempt')
        return (401, [('Content-Type', 'text/plain')], b'Unauthorized')

    async def _periodic_cache_cleanup(self):
        """Periodically evict expired entries from caches."""
        while True:
            await asyncio.sleep(60)  # Run every minute
            self._publisher_cache.evict_expired()
            self._service_client_cache.evict_expired()

    async def handle_client(self, websocket: WebSocketServerProtocol):
        """Handle a new WebSocket client connection."""
        if len(self.clients) >= self.max_clients:
            self.get_logger().warn('Max clients reached, rejecting connection')
            await websocket.close(1013, 'Max clients reached')
            return

        client_state = ClientState(ws=websocket)
        self.clients[websocket] = client_state
        # Initialize per-client rate limiters
        self._client_rate_limits[websocket] = ClientRateLimits.create(
            subscribe_rate=self._rate_limit_subscribe,
            publish_rate=self._rate_limit_publish,
            service_rate=self._rate_limit_service,
        )
        self.get_logger().info(f'Client connected: {websocket.remote_address}')

        # Send server info
        await self.send_server_info(websocket)

        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.cleanup_client(websocket)
            self.get_logger().info(f'Client disconnected: {websocket.remote_address}')

    async def send_server_info(self, websocket: WebSocketServerProtocol):
        """Send server capabilities to client."""
        info = {
            'name': 'RosWeb Bridge',
            'version': '0.1.0',
            'protocol_version': 1,
            'capabilities': ['subscribe', 'publish', 'service', 'param', 'service_list', 'node_list'],
            'encodings': ['cbor', 'png', 'jpeg', 'binary'],
        }
        payload = cbor2.dumps(info)
        msg = create_message(MessageType.SERVER_INFO, 0, payload, Encoding.CBOR)
        await websocket.send(msg)

    async def handle_message(self, websocket: WebSocketServerProtocol, data: bytes):
        """Handle an incoming message from a client."""
        header, payload = parse_message(data)

        if header is None:
            self.get_logger().warn('Invalid message received')
            return

        if header.message_type == MessageType.SUBSCRIBE:
            await self.handle_subscribe(websocket, payload)
        elif header.message_type == MessageType.UNSUBSCRIBE:
            await self.handle_unsubscribe(websocket, payload)
        elif header.message_type == MessageType.TOPIC_LIST:
            await self.send_topic_list(websocket)
        elif header.message_type == MessageType.PING:
            await self.send_pong(websocket)
        elif header.message_type == MessageType.PUBLISH:
            await self.handle_publish(websocket, header.channel_id, payload)
        elif header.message_type == MessageType.SERVICE_CALL:
            await self.handle_service_call(websocket, payload)
        elif header.message_type == MessageType.SERVICE_LIST:
            await self.send_service_list(websocket)
        elif header.message_type == MessageType.NODE_LIST:
            await self.send_node_list(websocket)
        else:
            self.get_logger().warn(f'Unknown message type: {header.message_type}')

    async def handle_subscribe(self, websocket: WebSocketServerProtocol, payload: bytes):
        """Handle a subscription request."""
        # Check rate limit
        rate_limits = self._client_rate_limits.get(websocket)
        if rate_limits and not rate_limits.subscribe.allow():
            await self.send_error(websocket, 'Rate limit exceeded for subscribe')
            return

        try:
            request = cbor2.loads(payload)
            topic = request['topic']
            msg_type = request.get('msg_type', '')
            throttle_ms = request.get('throttle_ms', 0)

            self.get_logger().info(f'Subscribe request: {topic} ({msg_type})')

            # Get or create channel
            channel = await self.get_or_create_channel(topic, msg_type, throttle_ms)

            if channel is None:
                await self.send_error(websocket, f'Failed to subscribe to {topic}')
                return

            # Add client to channel
            channel.clients.add(websocket)
            self.clients[websocket].subscribed_channels.add(channel.id)

            # Send channel info
            await self.send_channel_info(websocket, channel)

        except Exception as e:
            self.get_logger().error(f'Subscribe error: {e}')
            await self.send_error(websocket, str(e))

    async def get_or_create_channel(
        self, topic: str, msg_type: str, throttle_ms: int
    ) -> Optional[Channel]:
        """Get existing channel or create new subscription."""
        if topic in self.topic_to_channel:
            channel_id = self.topic_to_channel[topic]
            return self.channels[channel_id]

        # Discover message type if not provided
        if not msg_type:
            topic_names_and_types = self.get_topic_names_and_types()
            for name, types in topic_names_and_types:
                if name == topic and types:
                    msg_type = types[0]
                    break

        if not msg_type:
            self.get_logger().error(f'Could not determine message type for {topic}')
            return None

        # Create channel
        channel_id = self.next_channel_id
        self.next_channel_id += 1

        channel = Channel(
            id=channel_id,
            topic=topic,
            msg_type=msg_type,
            encoder=get_encoder(msg_type),
            throttle_ms=throttle_ms,
        )

        # Import message type dynamically
        try:
            msg_class = self._get_message_class(msg_type)
        except Exception as e:
            self.get_logger().error(f'Failed to import {msg_type}: {e}')
            return None

        # Create subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        def callback(msg):
            asyncio.run_coroutine_threadsafe(
                self.broadcast_message(channel, msg),
                asyncio.get_event_loop()
            )

        channel.subscription = self.create_subscription(
            msg_class, topic, callback, qos
        )

        self.channels[channel_id] = channel
        self.topic_to_channel[topic] = channel_id

        return channel

    def _get_message_class(self, msg_type: str):
        """Dynamically import a ROS message class."""
        # msg_type format: "package/msg/MessageName"
        parts = msg_type.split('/')
        if len(parts) == 3:
            package, _, class_name = parts
        else:
            raise ValueError(f'Invalid message type format: {msg_type}')

        module = __import__(f'{package}.msg', fromlist=[class_name])
        return getattr(module, class_name)

    def _get_service_class(self, srv_type: str):
        """Dynamically import a ROS service class."""
        parts = srv_type.split('/')
        if len(parts) == 3:
            package, _, class_name = parts
        else:
            raise ValueError(f'Invalid service type format: {srv_type}')

        module = __import__(f'{package}.srv', fromlist=[class_name])
        return getattr(module, class_name)

    def _evict_caches(self):
        """Evict expired cache entries (publishers/services)."""
        self._publisher_cache.evict_expired()
        self._service_client_cache.evict_expired()

    async def broadcast_message(self, channel: Channel, msg: Any):
        """Broadcast a ROS message to all subscribed clients."""
        if not channel.clients:
            return

        # Apply throttling
        now = time.time() * 1000  # ms
        if channel.throttle_ms > 0:
            if now - channel.last_sent < channel.throttle_ms:
                return
        channel.last_sent = now

        try:
            # Encode message
            encoded = channel.encoder.encode(msg)

            # Get timestamp
            timestamp_ns = 0
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

            # Create binary message
            binary_msg = create_message(
                MessageType.MESSAGE,
                channel.id,
                encoded,
                channel.encoder.encoding,
                timestamp_ns
            )

            # Broadcast to all clients
            disconnected = set()
            for client in channel.clients:
                try:
                    await client.send(binary_msg)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)

            # Remove disconnected clients
            for client in disconnected:
                channel.clients.discard(client)

        except Exception as e:
            self.get_logger().error(f'Broadcast error: {e}')
            # Inform clients about the failure
            for client in list(channel.clients):
                try:
                    await self.send_error(client, str(e))
                except Exception:
                    channel.clients.discard(client)

    async def handle_unsubscribe(self, websocket: WebSocketServerProtocol, payload: bytes):
        """Handle an unsubscribe request."""
        try:
            request = cbor2.loads(payload)
            channel_id = request.get('channel_id')

            if channel_id in self.channels:
                channel = self.channels[channel_id]
                channel.clients.discard(websocket)
                self.clients[websocket].subscribed_channels.discard(channel_id)

                # Clean up channel if no clients
                if not channel.clients:
                    self.destroy_subscription(channel.subscription)
                    del self.topic_to_channel[channel.topic]
                    del self.channels[channel_id]

        except Exception as e:
            self.get_logger().error(f'Unsubscribe error: {e}')

    async def send_topic_list(self, websocket: WebSocketServerProtocol):
        """Send list of available topics."""
        topics = []
        for name, types in self.get_topic_names_and_types():
            topics.append({
                'name': name,
                'types': list(types),
            })

        payload = cbor2.dumps({'topics': topics})
        msg = create_message(MessageType.TOPIC_LIST_RESPONSE, 0, payload, Encoding.CBOR)
        await websocket.send(msg)

    async def send_service_list(self, websocket: WebSocketServerProtocol):
        """Send list of available services."""
        services = []
        for name, types in self.get_service_names_and_types():
            services.append({
                'name': name,
                'types': list(types),
            })
        payload = cbor2.dumps({'services': services})
        msg = create_message(MessageType.SERVICE_LIST_RESPONSE, 0, payload, Encoding.CBOR)
        await websocket.send(msg)

    async def send_node_list(self, websocket: WebSocketServerProtocol):
        """Send list of available nodes."""
        nodes = self.get_node_names()
        payload = cbor2.dumps({'nodes': nodes})
        msg = create_message(MessageType.NODE_LIST_RESPONSE, 0, payload, Encoding.CBOR)
        await websocket.send(msg)

    async def send_channel_info(self, websocket: WebSocketServerProtocol, channel: Channel):
        """Send channel information to client."""
        info = {
            'channel_id': channel.id,
            'topic': channel.topic,
            'msg_type': channel.msg_type,
            'encoding': channel.encoder.encoding.value,
        }
        payload = cbor2.dumps(info)
        msg = create_message(MessageType.CHANNEL_INFO, channel.id, payload, Encoding.CBOR)
        await websocket.send(msg)

    async def send_pong(self, websocket: WebSocketServerProtocol):
        """Send pong response."""
        msg = create_message(MessageType.PONG, 0, b'', Encoding.RAW)
        await websocket.send(msg)

    async def send_error(self, websocket: WebSocketServerProtocol, error: str):
        """Send error message."""
        payload = cbor2.dumps({'error': error})
        msg = create_message(MessageType.ERROR, 0, payload, Encoding.CBOR)
        await websocket.send(msg)

    async def cleanup_client(self, websocket: WebSocketServerProtocol):
        """Clean up when a client disconnects."""
        if websocket not in self.clients:
            return

        client_state = self.clients[websocket]

        # Remove from all channels
        for channel_id in client_state.subscribed_channels:
            if channel_id in self.channels:
                self.channels[channel_id].clients.discard(websocket)

        del self.clients[websocket]

        # Clean up caches periodically
        await self._periodic_cache_cleanup(force=True)
        # Clean up rate limiters
        self._client_rate_limits.pop(websocket, None)

    async def handle_publish(self, websocket: WebSocketServerProtocol, channel_id: int, payload: bytes):
        """Handle a publish request from client."""
        # Check rate limit
        rate_limits = self._client_rate_limits.get(websocket)
        if rate_limits and not rate_limits.publish.allow():
            await self.send_error(websocket, 'Rate limit exceeded for publish')
            return

        try:
            # Expect CBOR with {topic, msg_type, data}
            data = cbor2.loads(payload)
            topic = data.get('topic')
            msg_type = data.get('msg_type')
            msg_payload = data.get('data')

            if not topic or not msg_type or msg_payload is None:
                await self.send_error(websocket, 'Invalid publish payload')
                return

            msg_class = self._get_message_class(msg_type)
            ros_msg = msg_class()

            # Populate ROS message fields recursively
            self._populate_ros_message(ros_msg, msg_payload)

            publisher = self._publisher_cache.get(topic)
            if publisher is None:
                publisher = self.create_publisher(msg_class, topic, 10)
                self._publisher_cache.set(topic, publisher)

            publisher.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')
            await self.send_error(websocket, f'Publish failed: {e}')

    def _populate_ros_message(self, ros_msg, data):
        """Recursively populate a ROS message from a plain dict/list."""
        if isinstance(data, dict):
            for key, value in data.items():
                if hasattr(ros_msg, key):
                    current = getattr(ros_msg, key)
                    if hasattr(current, '__slots__'):
                        self._populate_ros_message(current, value)
                    else:
                        setattr(ros_msg, key, value)
        elif isinstance(data, (list, tuple)):
            # Assuming sequence fields are already properly typed by rclpy
            for idx, value in enumerate(data):
                if idx < len(ros_msg):
                    ros_msg[idx] = value

    async def handle_service_call(self, websocket: WebSocketServerProtocol, payload: bytes):
        """Handle a service call request from client."""
        # Check rate limit
        rate_limits = self._client_rate_limits.get(websocket)
        if rate_limits and not rate_limits.service_call.allow():
            await self.send_error(websocket, 'Rate limit exceeded for service call')
            return

        try:
            data = cbor2.loads(payload)
            service_name = data.get('service')
            msg_type = data.get('msg_type')
            request_id = data.get('request_id')
            request_payload = data.get('request')

            if not service_name or not msg_type or request_id is None:
                await self.send_error(websocket, 'Invalid service call payload')
                return

            srv_class = self._get_service_class(msg_type)
            client = self._service_client_cache.get(service_name)
            if client is None:
                client = self.create_client(srv_class, service_name)
                self._service_client_cache.set(service_name, client)

            # Wait for service availability (timeout 3s)
            ready = client.wait_for_service(timeout_sec=3.0)
            if not ready:
                resp = {'request_id': request_id, 'error': f'Service {service_name} not available'}
                await websocket.send(create_message(MessageType.SERVICE_RESPONSE, 0, cbor2.dumps(resp), Encoding.CBOR))
                return

            # Build request
            ros_req = srv_class.Request()
            self._populate_ros_message(ros_req, request_payload)

            future = client.call_async(ros_req)

            # Wait with timeout
            timeout = 5.0
            start = self.get_clock().now().nanoseconds
            while not future.done():
                await asyncio.sleep(0.01)
                if (self.get_clock().now().nanoseconds - start) / 1e9 > timeout:
                    future.cancel()
                    resp = {'request_id': request_id, 'error': f'Service {service_name} timed out'}
                    await websocket.send(create_message(MessageType.SERVICE_RESPONSE, 0, cbor2.dumps(resp), Encoding.CBOR))
                    return

            if future.cancelled():
                resp = {'request_id': request_id, 'error': 'Service call cancelled'}
            elif future.exception():
                resp = {'request_id': request_id, 'error': str(future.exception())}
            else:
                ros_resp = future.result()
                resp = {'request_id': request_id, 'response': self._ros_message_to_dict(ros_resp)}

            await websocket.send(create_message(MessageType.SERVICE_RESPONSE, 0, cbor2.dumps(resp), Encoding.CBOR))

        except Exception as e:
            self.get_logger().error(f'Service call error: {e}')
            await self.send_error(websocket, f'Service call failed: {e}')

    def _ros_message_to_dict(self, msg: Any):
        """Convert ROS message to plain dict recursively."""
        if hasattr(msg, '__slots__'):
            result = {}
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                result[slot] = self._ros_message_to_dict(value)
            return result
        elif isinstance(msg, (list, tuple)):
            return [self._ros_message_to_dict(v) for v in msg]
        else:
            return msg


async def spin_ros(node: Node):
    """Spin ROS node in async context."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def main_async():
    """Async main function."""
    rclpy.init()
    bridge = RosWebBridge()

    await bridge.start_server()

    try:
        await spin_ros(bridge)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


def main():
    """Entry point."""
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
