"""
Control module for DSOS.
Contains mission planning, formation control, navigation, and hardware abstraction.
"""
from ..missions.mission_manager import MissionManager, Mission, MissionType, Waypoint
from ..formations.formation_engine import FormationManager, FormationType
from ..navigation.navigation_manager import NavigationManager
from .backends import BackendInterface, BackendType, BackendCapabilities, VehicleState, AirSimBackend, create_backend

__all__ = [
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
    'create_backend'
]