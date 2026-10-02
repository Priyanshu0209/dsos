"""
Distributed Knowledge Base for DSOS Swarm Intelligence.
Each drone maintains a local knowledge base that synchronizes with neighbors
to form a distributed global knowledge model.
"""
import asyncio
import json
import time
import threading
from typing import Dict, List, Optional, Any, Set, Tuple
from dataclasses import dataclass, field, asdict
from enum import Enum
import logging
import numpy as np
from collections import defaultdict, deque

from dsos.config.settings import settings
from dsos.agents.drone_agent import DroneAgent
from dsos.communication.mesh_network import MeshNetwork, MeshMessage, MessageType


class KnowledgeType(Enum):
    """Types of knowledge that can be stored in the knowledge base."""
    OBJECT = "object"              # Detected objects
    OBSTACLE = "obstacle"          # Obstacles in environment
    REGION = "region"              # Explored regions
    COMMUNICATION = "communication" # Communication quality metrics
    NEIGHBOR = "neighbor"          # Neighboring drone information
    MISSION = "mission"            # Mission progress and status
    OCCUPANCY = "occupancy"        # Local occupancy grid
    SEMANTIC = "semantic"          # Semantic map information
    TARGET = "target"              # Target information
    HAZARD = "hazard"              # Environmental hazards
    ENVIRONMENT = "environment"    # Environmental conditions


@dataclass
class KnowledgeItem:
    """Base class for knowledge items."""
    knowledge_id: str
    knowledge_type: KnowledgeType
    timestamp: float
    source_drone: str  # Which drone originally discovered this
    confidence: float = 1.0  # Confidence in this knowledge (0-1)
    location: Optional[List[float]] = None  # [x, y, z] if location-based
    metadata: Dict[str, Any] = field(default_factory=dict)
    ttl: float = float('inf')  # Time to live (seconds)

    def is_expired(self, current_time: float) -> bool:
        """Check if this knowledge item has expired."""
        return current_time - self.timestamp > self.ttl

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        data = asdict(self)
        data['knowledge_type'] = self.knowledge_type.value
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'KnowledgeItem':
        """Create from dictionary."""
        data['knowledge_type'] = KnowledgeType(data['knowledge_type'])
        return cls(**data)


@dataclass
class ObjectKnowledge(KnowledgeItem):
    """Knowledge about a detected object."""
    object_class: str = ""  # e.g., "person", "vehicle", "building"
    object_id: str = ""     # Unique identifier for this object instance
    bounding_box: Optional[List[float]] = None  # [x, y, z, width, height, depth]
    velocity: Optional[List[float]] = None      # Estimated velocity [vx, vy, vz]


@dataclass
class ObstacleKnowledge(KnowledgeItem):
    """Knowledge about an obstacle."""
    obstacle_type: str = ""   # e.g., "wall", "tree", "power_line"
    size: Optional[List[float]] = None  # [width, height, depth]
    is_dynamic: bool = False  # Whether the obstacle can move


@dataclass
class RegionKnowledge(KnowledgeItem):
    """Knowledge about an explored region."""
    region_type: str = ""     # e.g., "urban", "forest", "indoor"
    coverage: float = 0.0     # How thoroughly explored (0-1)
    risk_level: float = 0.0   # Risk level in this region (0-1)
    resources: List[str] = field(default_factory=list)  # Available resources


@dataclass
class CommunicationKnowledge(KnowledgeItem):
    """Knowledge about communication quality."""
    target_drone: str = ""    # Which drone this communication is with
    signal_strength: float = 0.0  # RSSI or similar metric (0-1)
    latency: float = 0.0      # Communication latency (seconds)
    packet_loss: float = 0.0  # Packet loss rate (0-1)
    bandwidth: float = 0.0    # Available bandwidth (bps)
    channel_quality: float = 0.0  # Overall channel quality (0-1)


@dataclass
class NeighborKnowledge(KnowledgeItem):
    """Knowledge about a neighboring drone."""
    status: str = ""          # Current status (idle, flying, etc.)
    battery_level: float = 0.0  # Battery percentage (0-100)
    health: float = 0.0       # Health status (0-1)
    mission_progress: float = 0.0  # Mission completion percentage (0-1)
    position: Optional[List[float]] = None  # Last known position
    velocity: Optional[List[float]] = None  # Last known velocity


class MissionKnowledge(KnowledgeItem):
    """Knowledge about a mission."""
    mission_id: str = ""          # Unique identifier for the mission
    mission_type: str = ""        # Type of mission (e.g., "surveillance", "search")
    status: str = ""              # Current status of the mission (e.g., "planned", "active", "completed")
    progress: float = 0.0         # Mission completion percentage (0-100)
    waypoints_completed: int = 0  # Number of waypoints completed
    total_waypoints: int = 0      # Total number of waypoints in the mission
    start_time: float = 0.0       # Mission start time
    end_time: float = 0.0         # Mission end time (0 if not completed)


class OccupancyKnowledge(KnowledgeItem):
    """Knowledge about occupancy of a grid cell."""
    cell_x: int = 0               # X coordinate of the cell
    cell_y: int = 0               # Y coordinate of the cell
    cell_z: int = 0               # Z coordinate of the cell
    occupancy: int = 0            # 0=unknown, 1=occupied, 2=free
    probability: float = 0.0      # Probability of occupancy (0-1)


class SemanticKnowledge(KnowledgeItem):
    """Semantic knowledge about an object or region."""
    semantic_label: str = ""      # Semantic label (e.g., "building", "tree", "road")
    confidence: float = 0.0       # Confidence in the semantic label (0-1)
    attributes: Dict[str, Any] = field(default_factory=dict)  # Additional attributes


class TargetKnowledge(KnowledgeItem):
    """Knowledge about a target."""
    target_id: str = ""           # Unique identifier for the target
    target_type: str = ""         # Type of target (e.g., "vehicle", "person", "structure")
    position: Optional[List[float]] = None  # Last known position
    velocity: Optional[List[float]] = None  # Estimated velocity
    threat_level: float = 0.0     # Threat level (0-1)
    priority: int = 0             # Priority level (higher is more important)
    discovered_time: float = 0.0  # Time when target was first discovered


class HazardKnowledge(KnowledgeItem):
    """Knowledge about a hazard."""
    hazard_type: str = ""         # Type of hazard (e.g., "fire", "chemical", "electrical")
    location: Optional[List[float]] = None  # Position of the hazard
    radius: float = 0.0           # Affected radius (meters)
    severity: float = 0.0         # Severity level (0-1)
    is_active: bool = False       # Whether the hazard is currently active
    detection_time: float = 0.0   # Time when hazard was detected


class EnvironmentKnowledge(KnowledgeItem):
    """Knowledge about environmental conditions."""
    temperature: float = 0.0      # Temperature in Celsius
    humidity: float = 0.0         # Humidity percentage
    wind_speed: float = 0.0       # Wind speed in m/s
    wind_direction: float = 0.0   # Wind direction in degrees
    pressure: float = 0.0         # Atmospheric pressure in hPa
    visibility: float = 0.0       # Visibility in meters


class DistributedKnowledgeBase:
    """
    Distributed Knowledge Base for each drone.
    Maintains local knowledge and synchronizes with neighbors.
    """

    def __init__(self, drone_id: str, mesh_network: MeshNetwork):
        self.drone_id = drone_id
        self.mesh_network = mesh_network
        self.logger = logging.getLogger(f"knowledge.{drone_id}")

        # Local knowledge storage
        self.knowledge_items: Dict[str, KnowledgeItem] = {}  # knowledge_id -> KnowledgeItem
        self.knowledge_by_type: Dict[KnowledgeType, Set[str]] = defaultdict(set)
        self.knowledge_by_location: Dict[Tuple[int, int, int], Set[str]] = defaultdict(set)  # Grid-based indexing

        # Synchronization tracking
        self.last_sync_time: Dict[str, float] = {}  # neighbor_id -> last_sync_timestamp
        self.knowledge_version: Dict[str, int] = defaultdict(int)  # knowledge_id -> version
        self.pending_updates: deque = deque()  # Knowledge items to sync

        # Configuration
        self.sync_interval = settings.getfloat('DSOS', 'knowledge_sync_interval_sec', 1.0)
        self.knowledge_retention_time = settings.getfloat('DSOS', 'knowledge_retention_time_sec', 300.0)
        self.grid_resolution = settings.getfloat('DSOS', 'knowledge_grid_resolution_m', 5.0)
        self.max_knowledge_items = settings.getint('DSOS', 'max_knowledge_items_per_drone', 10000)

        # Start background synchronization
        self._sync_task: Optional[asyncio.Task] = None
        self._cleanup_task: Optional[asyncio.Task] = None
        self._running = False

    async def start(self) -> None:
        """Start the knowledge base synchronization processes."""
        if self._running:
            return
        self._running = True
        self.logger.info(f"Starting knowledge base for drone {self.drone_id}")

        # Start background tasks
        self._sync_task = asyncio.create_task(self._synchronization_loop())
        self._cleanup_task = asyncio.create_task(self._cleanup_loop())

        # Subscribe to knowledge messages
        self.mesh_network.subscribe(self._handle_knowledge_message)

    async def stop(self) -> None:
        """Stop the knowledge base synchronization processes."""
        if not self._running:
            return
        self._running = False
        self.logger.info(f"Stopping knowledge base for drone {self.drone_id}")

        # Cancel background tasks
        if self._sync_task:
            self._sync_task.cancel()
        if self._cleanup_task:
            self._cleanup_task.cancel()

        # Unsubscribe from messages
        self.mesh_network.unsubscribe(self._handle_knowledge_message)

        # Wait for tasks to complete
        tasks = [t for t in [self._sync_task, self._cleanup_task] if t is not None]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def add_knowledge(self, knowledge: KnowledgeItem) -> bool:
        """
        Add a new knowledge item to the local knowledge base.

        Args:
            knowledge: The knowledge item to add

        Returns:
            True if added successfully, False if rejected (e.g., duplicate, capacity exceeded)
        """
        # Check if we're at capacity
        if len(self.knowledge_items) >= self.max_knowledge_items:
            # Remove oldest knowledge item to make space
            self._remove_oldest_knowledge()

        # Check for duplicates (same ID and type from same source)
        existing_key = self._get_knowledge_key(knowledge)
        if existing_key in self.knowledge_items:
            existing = self.knowledge_items[existing_key]
            # Only replace if newer or higher confidence
            if knowledge.timestamp <= existing.timestamp and knowledge.confidence <= existing.confidence:
                return False  # Don't replace older or lower confidence knowledge

        # Add the knowledge item
        self.knowledge_items[knowledge.knowledge_id] = knowledge
        self.knowledge_by_type[knowledge.knowledge_type].add(knowledge.knowledge_id)

        # Add to location-based index if applicable
        if knowledge.location:
            grid_key = self._location_to_grid_key(knowledge.location)
            self.knowledge_by_location[grid_key].add(knowledge.knowledge_id)

        # Update version tracking
        self.knowledge_version[knowledge.knowledge_id] = self.knowledge_version.get(knowledge.knowledge_id, 0) + 1

        # Mark for synchronization
        self.pending_updates.append(knowledge)
        self.logger.debug(f"Added knowledge: {knowledge.knowledge_id} ({knowledge.knowledge_type.value})")
        return True

    def get_knowledge(self, knowledge_id: str) -> Optional[KnowledgeItem]:
        """Get a knowledge item by ID."""
        return self.knowledge_items.get(knowledge_id)

    def get_knowledge_by_type(self, knowledge_type: KnowledgeType) -> List[KnowledgeItem]:
        """Get all knowledge items of a specific type."""
        ids = self.knowledge_by_type.get(knowledge_type, set())
        return [self.knowledge_items[kid] for kid in ids if kid in self.knowledge_items]

    def get_knowledge_in_region(self, center: List[float], radius: float) -> List[KnowledgeItem]:
        """
        Get all knowledge items within a spherical region.

        Args:
            center: [x, y, z] center of the region
            radius: Radius in meters

        Returns:
            List of knowledge items within the region
        """
        results = []
        # First get candidates from grid-based indexing for efficiency
        grid_center = self._location_to_grid_key(center)
        grid_radius_cells = int(radius / self.grid_resolution) + 1

        candidate_ids = set()
        for dx in range(-grid_radius_cells, grid_radius_cells + 1):
            for dy in range(-grid_radius_cells, grid_radius_cells + 1):
                for dz in range(-grid_radius_cells, grid_radius_cells + 1):
                    grid_key = (grid_center[0] + dx, grid_center[1] + dy, grid_center[2] + dz)
                    candidate_ids.update(self.knowledge_by_location.get(grid_key, set()))

        # Filter by actual distance
        for knowledge_id in candidate_ids:
            knowledge = self.knowledge_items.get(knowledge_id)
            if knowledge and knowledge.location:
                distance = np.linalg.norm(np.array(knowledge.location) - np.array(center))
                if distance <= radius:
                    results.append(knowledge)

        return results

    def get_neighbor_knowledge(self, neighbor_drone_id: str) -> List[KnowledgeItem]:
        """Get knowledge items originating from a specific neighbor."""
        return [
            knowledge for knowledge in self.knowledge_items.values()
            if knowledge.source_drone == neighbor_drone_id
        ]

    def update_knowledge(self, knowledge: KnowledgeItem) -> bool:
        """
        Update an existing knowledge item or add it if new.

        Args:
            knowledge: The knowledge item to update/add

        Returns:
            True if updated/added successfully
        """
        existing = self.knowledge_items.get(knowledge.knowledge_id)
        if existing:
            # Check if this is newer information
            if knowledge.timestamp > existing.timestamp or knowledge.confidence > existing.confidence:
                # Remove old from indices
                self.knowledge_by_type[existing.knowledge_type].discard(existing.knowledge_id)
                if existing.location:
                    grid_key = self._location_to_grid_key(existing.location)
                    self.knowledge_by_location[grid_key].discard(existing.knowledge_id)

                # Add new
                return self.add_knowledge(knowledge)
            else:
                return False  # Existing is newer or equal
        else:
            return self.add_knowledge(knowledge)

    def remove_knowledge(self, knowledge_id: str) -> bool:
        """Remove a knowledge item."""
        knowledge = self.knowledge_items.get(knowledge_id)
        if not knowledge:
            return False

        # Remove from indices
        del self.knowledge_items[knowledge_id]
        self.knowledge_by_type[knowledge.knowledge_type].discard(knowledge_id)
        if knowledge.location:
            grid_key = self._location_to_grid_key(knowledge.location)
            self.knowledge_by_location[grid_key].discard(knowledge_id)

        # Remove version tracking
        self.knowledge_version.pop(knowledge_id, None)

        self.logger.debug(f"Removed knowledge: {knowledge_id}")
        return True

    def _get_knowledge_key(self, knowledge: KnowledgeItem) -> str:
        """Generate a unique key for deduplication."""
        return f"{knowledge.knowledge_type.value}:{knowledge.source_drone}:{knowledge.knowledge_id}"

    def _location_to_grid_key(self, location: List[float]) -> Tuple[int, int, int]:
        """Convert a location to a grid key for spatial indexing."""
        x, y, z = location
        grid_x = int(x / self.grid_resolution)
        grid_y = int(y / self.grid_resolution)
        grid_z = int(z / self.grid_resolution)
        return (grid_x, grid_y, grid_z)

    async def _synchronization_loop(self) -> None:
        """Background loop for synchronizing knowledge with neighbors."""
        while self._running:
            try:
                await self._synchronize_knowledge()
                await asyncio.sleep(self.sync_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Error in knowledge synchronization loop: {e}")
                await asyncio.sleep(self.sync_interval)  # Continue after error

    async def _synchronize_knowledge(self) -> None:
        """Synchronize knowledge with neighboring drones."""
        if not self.pending_updates:
            return

        # Get current neighbors from mesh network
        neighbor_ids = self.mesh_network.get_neighbors()
        current_time = time.time()

        # Prepare knowledge update message
        updates_to_send = []
        while self.pending_updates and len(updates_to_send) < 50:  # Limit message size
            updates_to_send.append(self.pending_updates.popleft())

        if not updates_to_send:
            return

        # Create knowledge update message
        update_data = {
            'source_drone': self.drone_id,
            'timestamp': current_time,
            'updates': [knowledge.to_dict() for knowledge in updates_to_send]
        }

        # Broadcast to all neighbors
        message = MeshMessage(
            msg_id=f"knowledge_sync_{int(current_time * 1000)}",
            msg_type=MessageType.COMMAND,  # Reusing existing message type
            sender_id=self.drone_id,
            recipient_id=None,  # Broadcast
            timestamp=current_time,
            payload=update_data,
            ttl=settings.getint('DSOS', 'message_ttl', 5)
        )

        await self.mesh_network.send_message(message)
        self.logger.debug(f"Sent knowledge synchronization with {len(updates_to_send)} updates")

    async def _handle_knowledge_message(self, message: MeshMessage) -> None:
        """Handle incoming knowledge synchronization messages."""
        if message.msg_type != MessageType.COMMAND:
            return

        try:
            payload = message.payload
            if not isinstance(payload, dict) or 'updates' not in payload:
                return

            source_drone = payload.get('source_drone')
            if not source_drone or source_drone == self.drone_id:
                return  # Ignore our own messages

            updates = payload.get('updates', [])
            if not isinstance(updates, list):
                return

            # Process each knowledge update
            for update_data in updates:
                try:
                    knowledge = KnowledgeItem.from_dict(update_data)
                    self.update_knowledge(knowledge)
                except Exception as e:
                    self.logger.debug(f"Failed to process knowledge update: {e}")

            # Update last sync time for this neighbor
            self.last_sync_time[source_drone] = time.time()

        except Exception as e:
            self.logger.error(f"Error handling knowledge message: {e}")

    async def _cleanup_loop(self) -> None:
        """Background loop for cleaning up expired knowledge."""
        while self._running:
            try:
                await self._cleanup_expired_knowledge()
                await asyncio.sleep(self.sync_interval * 2)  # Cleanup less frequently
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Error in knowledge cleanup loop: {e}")
                await asyncio.sleep(self.sync_interval * 2)

    async def _cleanup_expired_knowledge(self) -> None:
        """Remove expired knowledge items."""
        current_time = time.time()
        expired_ids = []

        for knowledge_id, knowledge in self.knowledge_items.items():
            if knowledge.is_expired(current_time):
                expired_ids.append(knowledge_id)

        for knowledge_id in expired_ids:
            self.remove_knowledge(knowledge_id)

        if expired_ids:
            self.logger.debug(f"Cleaned up {len(expired_ids)} expired knowledge items")

    def _remove_oldest_knowledge(self) -> None:
        """Remove the oldest knowledge item to make space."""
        if not self.knowledge_items:
            return

        # Find oldest knowledge item
        oldest_id = min(
            self.knowledge_items.keys(),
            key=lambda kid: self.knowledge_items[kid].timestamp
        )
        self.remove_knowledge(oldest_id)

    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about the knowledge base."""
        current_time = time.time()
        stats = {
            'drone_id': self.drone_id,
            'total_knowledge_items': len(self.knowledge_items),
            'knowledge_by_type': {
                ktype.value: len(ids)
                for ktype, ids in self.knowledge_by_type.items()
            },
            'oldest_knowledge_age': 0.0,
            'newest_knowledge_age': 0.0,
            'avg_confidence': 0.0,
            'neighbors_synced_with': len(self.last_sync_time),
            'pending_updates': len(self.pending_updates)
        }

        if self.knowledge_items:
            timestamps = [k.timestamp for k in self.knowledge_items.values()]
            confidences = [k.confidence for k in self.knowledge_items.values()]

            stats['oldest_knowledge_age'] = current_time - min(timestamps)
            stats['newest_knowledge_age'] = current_time - max(timestamps)
            stats['avg_confidence'] = sum(confidences) / len(confidences)

        return stats