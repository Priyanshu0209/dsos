"""
Mesh networking implementation for drone swarm communication.
Provides heartbeat protocol, neighbor discovery, and basic message routing.
"""
import asyncio
import json
import time
import uuid
from typing import Dict, List, Optional, Callable, Any
import logging
from dataclasses import dataclass, asdict
from enum import Enum

from dsos.config.settings import settings


class MessageType(Enum):
    """Types of messages in the mesh network."""
    HEARTBEAT = "heartbeat"
    DISCOVERY = "discovery"
    COMMAND = "command"
    TELEMETRY = "telemetry"
    MISSION = "mission"
    FORMATION = "formation"
    EMERGENCY = "emergency"
    ACK = "ack"


@dataclass
class MeshMessage:
    """Message structure for mesh network communication."""
    msg_id: str
    msg_type: MessageType
    sender_id: str
    recipient_id: Optional[str]  # None for broadcast
    timestamp: float
    payload: Dict[str, Any]
    hop_count: int = 0
    ttl: int = 5  # Time to live (max hops)

    def to_json(self) -> str:
        """Convert message to JSON string."""
        data = asdict(self)
        data['msg_type'] = self.msg_type.value
        return json.dumps(data)

    @classmethod
    def from_json(cls, json_str: str) -> 'MeshMessage':
        """Create message from JSON string."""
        data = json.loads(json_str)
        data['msg_type'] = MessageType(data['msg_type'])
        return cls(**data)


class MeshNetwork:
    """Manages mesh network communication for a drone agent."""

    def __init__(self, agent_id: str, message_handler: Callable[[MeshMessage], None]):
        self.agent_id = agent_id
        self.message_handlers = [message_handler] if message_handler else []
        self.logger = logging.getLogger(f"mesh.{agent_id}")

        # Network configuration
        self.heartbeat_interval = settings.getfloat('DSOS', 'heartbeat_interval_sec', 1.0)
        self.discovery_interval = settings.getfloat('DSOS', 'discovery_interval_sec', 5.0)
        self.message_ttl = settings.getint('DSOS', 'message_ttl', 5)
        self.max_neighbors = settings.getint('DSOS', 'max_neighbors', 10)
        self.neighbor_timeout = settings.getfloat('DSOS', 'neighbor_timeout_sec', 10.0)

        # Network state
        self.neighbors: Dict[str, dict] = {}  # agent_id -> {last_seen, status, etc.}
        self.running = False
        self._tasks: List[asyncio.Task] = []

        # Simulated network interface (in reality, this would be UDP sockets or similar)
        self._message_queue: asyncio.Queue = asyncio.Queue()

    def subscribe(self, handler: Callable[[MeshMessage], None]) -> None:
        """Add a message handler."""
        if handler not in self.message_handlers:
            self.message_handlers.append(handler)

    def unsubscribe(self, handler: Callable[[MeshMessage], None]) -> None:
        """Remove a message handler."""
        if handler in self.message_handlers:
            self.message_handlers.remove(handler)

    async def start(self) -> None:
        """Start the mesh network services."""
        if self.running:
            return
        self.running = True
        self.logger.info(f"Starting mesh network for agent {self.agent_id}")

        # Start background tasks
        self._tasks.append(asyncio.create_task(self._heartbeat_loop()))
        self._tasks.append(asyncio.create_task(self._discovery_loop()))
        self._tasks.append(asyncio.create_task(self._message_processor()))
        self._tasks.append(asyncio.create_task(self._neighbor_monitor()))

    async def stop(self) -> None:
        """Stop the mesh network services."""
        if not self.running:
            return
        self.running = False
        self.logger.info(f"Stopping mesh network for agent {self.agent_id}")

        # Cancel all tasks
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()

    async def send_message(self, message: MeshMessage) -> None:
        """Send a message to the mesh network."""
        # In a real implementation, this would send over UDP mesh
        # For simulation, we'll just put it in the queue to be processed by neighbors
        await self._message_queue.put(message)
        self.logger.debug(f"Queued message {message.msg_id} of type {message.msg_type.value}")

    async def broadcast_heartbeat(self) -> None:
        """Broadcast a heartbeat message to discover neighbors."""
        message = MeshMessage(
            msg_id=str(uuid.uuid4()),
            msg_type=MessageType.HEARTBEAT,
            sender_id=self.agent_id,
            recipient_id=None,  # Broadcast
            timestamp=time.time(),
            payload={
                'agent_id': self.agent_id,
                'timestamp': time.time()
            },
            ttl=self.message_ttl
        )
        await self.send_message(message)

    async def send_discovery(self) -> None:
        """Send a discovery message to find neighbors."""
        message = MeshMessage(
            msg_id=str(uuid.uuid4()),
            msg_type=MessageType.DISCOVERY,
            sender_id=self.agent_id,
            recipient_id=None,  # Broadcast
            timestamp=time.time(),
            payload={
                'agent_id': self.agent_id,
                'timestamp': time.time()
            },
            ttl=self.message_ttl
        )
        await self.send_message(message)

    async def send_telemetry(self, telemetry_data: Dict[str, Any]) -> None:
        """Send telemetry data to neighbors."""
        message = MeshMessage(
            msg_id=str(uuid.uuid4()),
            msg_type=MessageType.TELEMETRY,
            sender_id=self.agent_id,
            recipient_id=None,  # Broadcast telemetry to neighbors
            timestamp=time.time(),
            payload=telemetry_data,
            ttl=self.message_ttl
        )
        await self.send_message(message)

    async def send_command(self, target_id: str, command: Dict[str, Any]) -> None:
        """Send a command to a specific neighbor."""
        message = MeshMessage(
            msg_id=str(uuid.uuid4()),
            msg_type=MessageType.COMMAND,
            sender_id=self.agent_id,
            recipient_id=target_id,
            timestamp=time.time(),
            payload=command,
            ttl=self.message_ttl
        )
        await self.send_message(message)

    async def _heartbeat_loop(self) -> None:
        """Periodically send heartbeat messages."""
        while self.running:
            await self.broadcast_heartbeat()
            await asyncio.sleep(self.heartbeat_interval)

    async def _discovery_loop(self) -> None:
        """Periodically send discovery messages."""
        while self.running:
            await self.send_discovery()
            await asyncio.sleep(self.discovery_interval)

    async def _message_processor(self) -> None:
        """Process incoming messages from the queue."""
        while self.running:
            try:
                # Wait for message with timeout to allow checking running flag
                message = await asyncio.wait_for(self._message_queue.get(), timeout=1.0)
                await self._handle_message(message)
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                self.logger.error(f"Error processing message: {e}")

    async def _handle_message(self, message: MeshMessage) -> None:
        """Handle an incoming message."""
        # Decrement TTL and discard if expired
        message.ttl -= 1
        if message.ttl <= 0:
            self.logger.debug(f"Message {message.msg_id} expired (TTL=0)")
            return

        # Ignore messages from ourselves
        if message.sender_id == self.agent_id:
            return

        # Update neighbor information
        self._update_neighbor_from_message(message)

        # Handle message based on type
        try:
            if message.msg_type == MessageType.HEARTBEAT:
                await self._handle_heartbeat(message)
            elif message.msg_type == MessageType.DISCOVERY:
                await self._handle_discovery(message)
            elif message.msg_type == MessageType.COMMAND:
                await self._handle_command(message)
            elif message.msg_type == MessageType.TELEMETRY:
                await self._handle_telemetry(message)
            elif message.msg_type == MessageType.MISSION:
                await self._handle_mission(message)
            elif message.msg_type == MessageType.FORMATION:
                await self._handle_formation(message)
            elif message.msg_type == MessageType.EMERGENCY:
                await self._handle_emergency(message)
            elif message.msg_type == MessageType.ACK:
                await self._handle_ack(message)
            else:
                self.logger.warning(f"Unknown message type: {message.msg_type}")

            # Pass message to handlers for agent-specific processing
            for handler in self.message_handlers:
                if handler:
                    handler(message)
        except Exception as e:
            self.logger.error(f"Error handling message {message.msg_id}: {e}")

    def _update_neighbor_from_message(self, message: MeshMessage) -> None:
        """Update neighbor information based on a message."""
        sender_id = message.sender_id
        if sender_id not in self.neighbors:
            self.neighbors[sender_id] = {
                'last_seen': message.timestamp,
                'message_count': 0,
                'seq_num': 0
            }
        else:
            self.neighbors[sender_id]['last_seen'] = message.timestamp

        self.neighbors[sender_id]['message_count'] += 1

    async def _handle_heartbeat(self, message: MeshMessage) -> None:
        """Handle a heartbeat message."""
        # Heartbeats are primarily for neighbor discovery
        pass

    async def _handle_discovery(self, message: MeshMessage) -> None:
        """Handle a discovery message."""
        # Respond to discovery with our own heartbeat
        await self.broadcast_heartbeat()

    async def _handle_command(self, message: MeshMessage) -> None:
        """Handle a command message."""
        if message.recipient_id == self.agent_id or message.recipient_id is None:
            # This is a command for us or a broadcast command
            self.logger.debug(f"Received command: {message.payload}")
            # In a real implementation, we would execute the command
            # For now, we'll just log it
            pass

    async def _handle_telemetry(self, message: MeshMessage) -> None:
        """Handle a telemetry message."""
        # Telemetry is just for informational purposes
        pass

    async def _handle_mission(self, message: MeshMessage) -> None:
        """Handle a mission message."""
        pass

    async def _handle_formation(self, message: MeshMessage) -> None:
        """Handle a formation message."""
        pass

    async def _handle_emergency(self, message: MeshMessage) -> None:
        """Handle an emergency message."""
        self.logger.warning(f"Received emergency message from {message.sender_id}: {message.payload}")
        # In a real implementation, we might take evasive action

    async def _handle_ack(self, message: MeshMessage) -> None:
        """Handle an acknowledgment message."""
        pass

    async def _neighbor_monitor(self) -> None:
        """Monitor neighbors and remove those that have timed out."""
        while self.running:
            now = time.time()
            to_remove = []
            for agent_id, info in self.neighbors.items():
                if now - info['last_seen'] > self.neighbor_timeout:
                    to_remove.append(agent_id)
            for agent_id in to_remove:
                self.logger.info(f"Neighbor {agent_id} timed out")
                del self.neighbors[agent_id]
            await asyncio.sleep(1.0)

    def get_neighbors(self) -> List[str]:
        """Get list of neighbor agent IDs."""
        return list(self.neighbors.keys())

    def get_neighbor_info(self, agent_id: str) -> Optional[dict]:
        """Get information about a specific neighbor."""
        return self.neighbors.get(agent_id)

    def is_connected(self) -> bool:
        """Check if we have any neighbors."""
        return len(self.neighbors) > 0