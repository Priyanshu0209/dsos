"""
Base agent class for all drone agents in the swarm.
Defines the common interface and functionality.
"""
import asyncio
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Any
import logging

from dsos.config.settings import settings


class AgentState(Enum):
    """Possible states of an agent."""
    INITIALIZING = auto()
    IDLE = auto()
    TAKEOFF = auto()
    HOVERING = auto()
    NAVIGATING = auto()
    EXECUTING_MISSION = auto()
    RETURNING_HOME = auto()
    LANDING = auto()
    EMERGENCY = auto()
    FAULT = auto()
    DISCONNECTED = auto()


@dataclass
class AgentStatus:
    """Current status of an agent."""
    agent_id: str
    state: AgentState
    battery_level: float  # Percentage 0-100
    health: float  # 0-1, where 1 is perfect health
    position: List[float]  # [x, y, z] in meters
    velocity: List[float]  # [vx, vy, vz] in m/s
    orientation: List[float]  # [roll, pitch, yaw] in radians
    timestamp: float  # Unix timestamp


class BaseAgent(ABC):
    """Abstract base class for all drone agents."""

    def __init__(self, agent_id: Optional[str] = None):
        self.agent_id = agent_id or str(uuid.uuid4())
        self.status = AgentStatus(
            agent_id=self.agent_id,
            state=AgentState.INITIALIZING,
            battery_level=100.0,
            health=1.0,
            position=[0.0, 0.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            timestamp=asyncio.get_event_loop().time()
        )
        self._neighbors: Dict[str, AgentStatus] = {}
        self._is_running = False
        self._main_task: Optional[asyncio.Task] = None
        self.logger = logging.getLogger(f"agent.{self.agent_id}")
        self._load_configuration()

    def _load_configuration(self) -> None:
        """Load agent-specific configuration from settings."""
        self.swarm_size = settings.getint('DSOS', 'swarm_size', 3)
        self.simulation_mode = settings.getboolean('DSOS', 'simulation_mode', True)
        self.max_velocity = settings.getfloat('DSOS', 'max_velocity_mps', 5.0)
        self.max_acceleration = settings.getfloat('DSOS', 'max_acceleration_mps2', 2.0)
        self.min_safe_distance = settings.getfloat('DSOS', 'min_safe_distance_m', 2.0)
        self.obstacle_threshold = settings.getfloat('DSOS', 'obstacle_threshold_m', 1.0)
        self.battery_capacity = settings.getint('DSOS', 'battery_capacity_mah', 5000)
        self.low_battery_threshold = settings.getint('DSOS', 'low_battery_threshold_percent', 20)
        self.critical_battery_threshold = settings.getint('DSOS', 'critical_battery_threshold_percent', 5)
        self.update_rate_hz = settings.getint('DSOS', 'update_rate_hz', 50)
        self.dt = 1.0 / self.update_rate_hz

    @abstractmethod
    async def initialize(self) -> None:
        """Initialize the agent and its components."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown the agent and release resources."""
        pass

    @abstractmethod
    async def update_state(self) -> None:
        """Update the agent's internal state based on sensor data and commands."""
        pass

    @abstractmethod
    async def communicate(self) -> None:
        """Handle communication with neighbors and the swarm."""
        pass

    @abstractmethod
    async def make_decision(self) -> None:
        """Make autonomous decisions based on current state and mission."""
        pass

    @abstractmethod
    async def execute_commands(self) -> None:
        """Execute low-level commands (e.g., motor controls)."""
        pass

    async def start(self) -> None:
        """Start the agent's main loop."""
        if self._is_running:
            return
        self._is_running = True
        await self.initialize()
        self._main_task = asyncio.create_task(self._main_loop())
        self.logger.info(f"Agent {self.agent_id} started")

    async def stop(self) -> None:
        """Stop the agent's main loop."""
        if not self._is_running:
            return
        self._is_running = False
        if self._main_task:
            self._main_task.cancel()
            try:
                await self._main_task
            except asyncio.CancelledError:
                pass
        await self.shutdown()
        self.logger.info(f"Agent {self.agent_id} stopped")

    async def _main_loop(self) -> None:
        """Main agent loop."""
        while self._is_running:
            start_time = asyncio.get_event_loop().time()

            try:
                await self.update_state()
                await self.communicate()
                await self.make_decision()
                await self.execute_commands()
            except Exception as e:
                self.logger.error(f"Error in main loop: {e}", exc_info=True)
                self._handle_fault()

            # Maintain update rate
            elapsed = asyncio.get_event_loop().time() - start_time
            sleep_time = max(0, self.dt - elapsed)
            await asyncio.sleep(sleep_time)

    def _handle_fault(self) -> None:
        """Handle a fault condition."""
        self.status.state = AgentState.FAULT
        self.logger.warning(f"Agent {self.agent_id} entered FAULT state")

    def update_neighbor_status(self, agent_id: str, status: AgentStatus) -> None:
        """Update the status of a neighboring agent."""
        self._neighbors[agent_id] = status

    def get_neighbors(self) -> Dict[str, AgentStatus]:
        """Get the current neighbor statuses."""
        return self._neighbors.copy()

    def is_ready(self) -> bool:
        """Check if the agent is ready for operations."""
        return self.status.state in [AgentState.IDLE, AgentState.HOVERING, AgentState.NAVIGATING]

    def get_status(self) -> AgentStatus:
        """Get a copy of the current agent status."""
        return AgentStatus(**self.status.__dict__)