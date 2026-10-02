"""
Drone Swarm Operating System (DSOS)
A research-grade autonomous drone swarm operating system.
"""

__version__ = "1.0.0"
__author__ = "DSOS Development Team"

# Import core components for easy access
from .core.base_agent import BaseAgent, AgentState, AgentStatus
from .config.settings import settings

# Import control components
from .control import (
    MissionManager, Mission, MissionType, Waypoint,
    FormationManager, FormationType,
    NavigationManager,
    BackendInterface, BackendType, BackendCapabilities, VehicleState,
    AirSimBackend, create_backend
)

# Import agents
from .agents.drone_agent import DroneAgent

# Import communication
from .communication.mesh_network import MeshNetwork

# Import sensors
from .sensors.sensor_manager import SensorManager

# Import AI
from .ai.decision_maker import AIDecisionMaker

# Import telemetry
from .telemetry.telemetry_manager import TelemetryManager

# Import UI
from .ui.gcs import GCSCommand

# Import swarm intelligence
from .swarm import SwarmBaseAgent, SwarmRole, SwarmState, SwarmMetrics
from .swarm.knowledge_base import (
    DistributedKnowledgeBase, KnowledgeType, KnowledgeItem,
    ObjectKnowledge, ObstacleKnowledge, RegionKnowledge,
    CommunicationKnowledge, NeighborKnowledge, MissionKnowledge,
    OccupancyKnowledge, SemanticKnowledge, TargetKnowledge,
    HazardKnowledge, EnvironmentKnowledge
)

__all__ = [
    # Core
    'BaseAgent',
    'AgentState',
    'AgentStatus',
    'settings',

    # Control
    'MissionManager',
    'Mission',
    'MissionType',
    'Waypoint',
    'FormationManager',
    'FormationType',
    'NavigationManager',
    'BackendInterface',
    'BackendType',
    'BackendCapabilities',
    'VehicleState',
    'AirSimBackend',
    'create_backend',

    # Agents
    'DroneAgent',

    # Communication
    'MeshNetwork',

    # Sensors
    'SensorManager',

    # AI
    'AIDecisionMaker',

    # Telemetry
    'TelemetryManager',

    # UI
    'GCSCommand',

    # Swarm Intelligence
    'SwarmBaseAgent',
    'SwarmRole',
    'SwarmState',
    'SwarmMetrics',
    'DistributedKnowledgeBase',
    'KnowledgeType',
    'KnowledgeItem',
    'ObjectKnowledge',
    'ObstacleKnowledge',
    'RegionKnowledge',
    'CommunicationKnowledge',
    'NeighborKnowledge',
    'MissionKnowledge',
    'OccupancyKnowledge',
    'SemanticKnowledge',
    'TargetKnowledge',
    'HazardKnowledge',
    'EnvironmentKnowledge'
]