"""
Mission management for drone swarm operations.
Handles mission planning, waypoint management, and mission execution.
"""
import asyncio
import json
from typing import List, Dict, Optional, Any, Callable
from enum import Enum
import logging

from dsos.config.settings import settings


class MissionType(Enum):
    """Types of missions that can be executed."""
    WAYPOINT = "waypoint"
    PATROL = "patrol"
    SURVEY = "survey"
    SEARCH = "search"
    MAPPING = "mapping"
    ESCORT = "escort"
    FORMATION = "formation"
    RETURN_TO_HOME = "return_to_home"
    LAND = "land"
    TAKEOFF = "takeoff"
    HOVER = "hover"
    CUSTOM = "custom"


class MissionStatus(Enum):
    """Status of a mission."""
    NOT_STARTED = "not_started"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class Waypoint:
    """Represents a single waypoint in a mission."""

    def __init__(self, x: float, y: float, z: float,
                 yaw: float = 0.0,
                 action: str = "fly_to",
                 params: Optional[Dict[str, Any]] = None):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.action = action
        self.params = params or {}
        # Support for mission graphs (conditional branching)
        self.conditions: List[Dict[str, Any]] = [] # list of conditions to evaluate
        self.on_success: Optional[int] = None # Next waypoint index if success (None = sequential next)
        self.on_failure: Optional[int] = None # Next waypoint index if failure

    def add_condition(self, condition_type: str, threshold: float, variable: str = "battery"):
        self.conditions.append({
            'type': condition_type, # e.g. "greater_than", "less_than"
            'threshold': threshold,
            'variable': variable
        })

    def to_dict(self) -> Dict[str, Any]:
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'yaw': self.yaw,
            'action': self.action,
            'params': self.params,
            'conditions': self.conditions,
            'on_success': self.on_success,
            'on_failure': self.on_failure
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Waypoint':
        wp = cls(
            x=data['x'],
            y=data['y'],
            z=data['z'],
            yaw=data.get('yaw', 0.0),
            action=data.get('action', 'fly_to'),
            params=data.get('params', {})
        )
        wp.conditions = data.get('conditions', [])
        wp.on_success = data.get('on_success')
        wp.on_failure = data.get('on_failure')
        return wp


class Mission:
    """Represents a complete mission with waypoints and metadata."""

    def __init__(self, mission_id: str, mission_type: MissionType,
                 name: str = "", description: str = ""):
        self.mission_id = mission_id
        self.mission_type = mission_type
        self.name = name
        self.description = description
        self.waypoints: List[Waypoint] = []
        self.status = MissionStatus.NOT_STARTED
        self.current_waypoint_index = 0
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.metadata: Dict[str, Any] = {}

    def add_waypoint(self, waypoint: Waypoint) -> None:
        """Add a waypoint to the mission."""
        self.waypoints.append(waypoint)

    def remove_waypoint(self, index: int) -> bool:
        """Remove a waypoint by index."""
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]
            return True
        return False

    def get_waypoint(self, index: int) -> Optional[Waypoint]:
        """Get a waypoint by index."""
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None

    def clear_waypoints(self) -> None:
        """Remove all waypoints."""
        self.waypoints.clear()

    def set_waypoints(self, waypoints: List[Waypoint]) -> None:
        """Set the waypoints for this mission."""
        self.waypoints = waypoints.copy()

    def to_dict(self) -> Dict[str, Any]:
        return {
            'mission_id': self.mission_id,
            'mission_type': self.mission_type.value,
            'name': self.name,
            'description': self.description,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'status': self.status.value,
            'current_waypoint_index': self.current_waypoint_index,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'metadata': self.metadata
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Mission':
        mission = cls(
            mission_id=data['mission_id'],
            mission_type=MissionType(data['mission_type']),
            name=data.get('name', ''),
            description=data.get('description', '')
        )
        mission.waypoints = [Waypoint.from_dict(wp) for wp in data.get('waypoints', [])]
        mission.status = MissionStatus(data.get('status', 'not_started'))
        mission.current_waypoint_index = data.get('current_waypoint_index', 0)
        mission.start_time = data.get('start_time')
        mission.end_time = data.get('end_time')
        mission.metadata = data.get('metadata', {})
        return mission


class MissionManager:
    """Manages missions for a drone agent."""

    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.logger = logging.getLogger(f"mission.{agent_id}")
        self.current_mission: Optional[Mission] = None
        self.mission_history: List[Mission] = []
        self._mission_callbacks: Dict[str, List[Callable]] = {
            'start': [],
            'complete': [],
            'fail': [],
            'waypoint_reached': []
        }

    def load_mission(self, mission: Mission) -> None:
        """Load a mission for execution."""
        self.current_mission = mission
        self.current_mission.status = MissionStatus.NOT_STARTED
        self.current_mission.current_waypoint_index = 0
        self.logger.info(f"Loaded mission {mission.mission_id}: {mission.name}")

    def create_waypoint_mission(self, name: str, waypoints: List[List[float]],
                                yaw_angles: Optional[List[float]] = None) -> Mission:
        """Create a simple waypoint mission."""
        mission_id = f"wp_{int(time.time())}_{len(self.mission_history)}"
        mission = Mission(mission_id, MissionType.WAYPOINT, name=name)

        if yaw_angles is None:
            yaw_angles = [0.0] * len(waypoints)
        elif len(yaw_angles) != len(waypoints):
            # If yaw angles don't match waypoints, pad or truncate
            if len(yaw_angles) < len(waypoints):
                yaw_angles.extend([0.0] * (len(waypoints) - len(yaw_angles)))
            else:
                yaw_angles = yaw_angles[:len(waypoints)]

        for i, (x, y, z) in enumerate(waypoints):
            yaw = yaw_angles[i] if i < len(yaw_angles) else 0.0
            waypoint = Waypoint(x, y, z, yaw)
            mission.add_waypoint(waypoint)

        return mission

    def create_patrol_mission(self, name: str, waypoints: List[List[float]],
                              patrol_time: float = 30.0) -> Mission:
        """Create a patrol mission that loops through waypoints."""
        mission_id = f"patrol_{int(time.time())}_{len(self.mission_history)}"
        mission = Mission(mission_id, MissionType.PATROL, name=name)

        for i, (x, y, z) in enumerate(waypoints):
            waypoint = Waypoint(x, y, z, action="patrol",
                                params={'dwell_time': patrol_time})
            mission.add_waypoint(waypoint)

        return mission

    def create_survey_mission(self, name: str, area: List[List[float]],
                              altitude: float, spacing: float) -> Mission:
        """Create a grid survey mission over a rectangular area."""
        # area = [[min_x, min_y], [max_x, max_y]]
        min_x, min_y = area[0]
        max_x, max_y = area[1]

        # Generate grid pattern
        waypoints = []
        x = min_x
        forward = True
        while x <= max_x:
            if forward:
                y = min_y
                while y <= max_y:
                    waypoints.append([x, y, altitude])
                    y += spacing
            else:
                y = max_y
                while y >= min_y:
                    waypoints.append([x, y, altitude])
                    y -= spacing
            x += spacing
            forward = not forward

        mission_id = f"survey_{int(time.time())}_{len(self.mission_history)}"
        mission = Mission(mission_id, MissionType.SURVEY, name=name)
        for wp in waypoints:
            waypoint = Waypoint(wp[0], wp[1], wp[2])
            mission.add_waypoint(waypoint)

        return mission

    def start_mission(self) -> bool:
        """Start the currently loaded mission."""
        if not self.current_mission:
            self.logger.warning("No mission loaded to start")
            return False

        if self.current_mission.status == MissionStatus.RUNNING:
            self.logger.warning("Mission is already running")
            return False

        self.current_mission.status = MissionStatus.RUNNING
        self.current_mission.start_time = asyncio.get_event_loop().time()
        self.current_mission.current_waypoint_index = 0
        self.logger.info(f"Started mission {self.current_mission.mission_id}")

        # Trigger start callbacks
        for callback in self._mission_callbacks['start']:
            try:
                callback(self.current_mission)
            except Exception as e:
                self.logger.error(f"Error in mission start callback: {e}")

        return True

    def pause_mission(self) -> bool:
        """Pause the currently running mission."""
        if not self.current_mission or self.current_mission.status != MissionStatus.RUNNING:
            return False

        self.current_mission.status = MissionStatus.PAUSED
        self.logger.info(f"Paused mission {self.current_mission.mission_id}")
        return True

    def resume_mission(self) -> bool:
        """Resume a paused mission."""
        if not self.current_mission or self.current_mission.status != MissionStatus.PAUSED:
            return False

        self.current_mission.status = MissionStatus.RUNNING
        self.logger.info(f"Resumed mission {self.current_mission.mission_id}")
        return True

    def stop_mission(self) -> bool:
        """Stop the currently running mission."""
        if not self.current_mission or self.current_mission.status not in [
                MissionStatus.RUNNING, MissionStatus.PAUSED]:
            return False

        self.current_mission.status = MissionStatus.CANCELLED
        self.current_mission.end_time = asyncio.get_event_loop().time()
        self.logger.info(f"Stopped mission {self.current_mission.mission_id}")
        return True

    def update_mission_progress(self) -> Optional[Waypoint]:
        """Update mission progress and return the current waypoint."""
        if not self.current_mission or self.current_mission.status != MissionStatus.RUNNING:
            return None

        # Check if we've completed all waypoints
        if self.current_mission.current_waypoint_index >= len(self.current_mission.waypoints):
            self._complete_mission()
            return None

        return self.current_mission.waypoints[self.current_mission.current_waypoint_index]

    def evaluate_waypoint_conditions(self, waypoint: Waypoint, state: Any) -> bool:
        """Evaluate if the waypoint conditions are met."""
        if not waypoint.conditions:
            return True
            
        for cond in waypoint.conditions:
            var = cond.get('variable', 'battery')
            val = getattr(state, 'battery_remaining', 100) if var == 'battery' else 0
            thresh = cond.get('threshold', 0)
            op = cond.get('type', 'greater_than')
            
            if op == 'greater_than' and not (val > thresh): return False
            if op == 'less_than' and not (val < thresh): return False
            if op == 'equals' and not (val == thresh): return False
            
        return True

    def waypoint_reached(self, state: Any = None) -> None:
        """Called when the current waypoint has been reached."""
        if not self.current_mission or self.current_mission.status != MissionStatus.RUNNING:
            return

        current_wp = self.current_mission.waypoints[self.current_mission.current_waypoint_index]
        success = True
        
        # Evaluate conditional branching
        if state is not None and current_wp.conditions:
            success = self.evaluate_waypoint_conditions(current_wp, state)

        # Move to next waypoint based on graph logic
        if success and current_wp.on_success is not None:
            self.current_mission.current_waypoint_index = current_wp.on_success
        elif not success and current_wp.on_failure is not None:
            self.current_mission.current_waypoint_index = current_wp.on_failure
        else:
            self.current_mission.current_waypoint_index += 1

        # Check if mission is complete
        if self.current_mission.current_waypoint_index >= len(self.current_mission.waypoints) or self.current_mission.current_waypoint_index < 0:
            self._complete_mission()
        else:
            # Notify waypoint reached
            for callback in self._mission_callbacks['waypoint_reached']:
                try:
                    callback(self.current_mission,
                             self.current_mission.current_waypoint_index - 1)
                except Exception as e:
                    self.logger.error(f"Error in waypoint reached callback: {e}")

    def _complete_mission(self) -> None:
        """Mark the current mission as complete."""
        if not self.current_mission:
            return

        self.current_mission.status = MissionStatus.COMPLETED
        self.current_mission.end_time = asyncio.get_event_loop().time()
        self.logger.info(f"Completed mission {self.current_mission.mission_id}")

        # Move to history
        if self.current_mission not in self.mission_history:
            self.mission_history.append(self.current_mission)

        # Trigger completion callbacks
        for callback in self._mission_callbacks['complete']:
            try:
                callback(self.current_mission)
            except Exception as e:
                self.logger.error(f"Error in mission complete callback: {e}")

        self.current_mission = None

    def fail_mission(self, reason: str = "") -> None:
        """Mark the current mission as failed."""
        if not self.current_mission:
            return

        self.current_mission.status = MissionStatus.FAILED
        self.current_mission.end_time = asyncio.get_event_loop().time()
        if reason:
            self.current_mission.metadata['failure_reason'] = reason
        self.logger.error(f"Failed mission {self.current_mission.mission_id}: {reason}")

        # Move to history
        if self.current_mission not in self.mission_history:
            self.mission_history.append(self.current_mission)

        # Trigger failure callbacks
        for callback in self._mission_callbacks['fail']:
            try:
                callback(self.current_mission, reason)
            except Exception as e:
                self.logger.error(f"Error in mission fail callback: {e}")

        self.current_mission = None

    def get_mission_status(self) -> Optional[Dict[str, Any]]:
        """Get the status of the current mission."""
        if not self.current_mission:
            return None
        return self.current_mission.to_dict()

    def get_mission_history(self) -> List[Dict[str, Any]]:
        """Get the mission history."""
        return [mission.to_dict() for mission in self.mission_history]

    def register_callback(self, event: str, callback: Callable) -> None:
        """Register a callback for mission events."""
        if event in self._mission_callbacks:
            self._mission_callbacks[event].append(callback)
        else:
            self.logger.warning(f"Unknown mission event: {e}")

    def save_mission_to_file(self, mission: Mission, filename: str) -> bool:
        """Save a mission to a JSON file."""
        try:
            with open(filename, 'w') as f:
                json.dump(mission.to_dict(), f, indent=2)
            self.logger.info(f"Saved mission {mission.mission_id} to {filename}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to save mission to {filename}: {e}")
            return False

    def load_mission_from_file(self, filename: str) -> Optional[Mission]:
        """Load a mission from a JSON file."""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            mission = Mission.from_dict(data)
            self.logger.info(f"Loaded mission {mission.mission_id} from {filename}")
            return mission
        except Exception as e:
            self.logger.error(f"Failed to load mission from {filename}: {e}")
            return None