"""
SwarmBaseAgent - Extension of DroneAgent with swarm intelligence capabilities.
This class adds distributed knowledge sharing, cooperative decision making,
and autonomous swarm behaviors while maintaining backward compatibility.
"""
import asyncio
import time
import numpy as np
from typing import Dict, List, Optional, Any, Tuple, Set
import logging
from collections import defaultdict, deque
from enum import Enum
from dataclasses import dataclass

from dsos.agents.drone_agent import DroneAgent, AgentState, AgentStatus
from dsos.config.settings import settings
from dsos.communication.mesh_network import MeshNetwork, MeshMessage, MessageType
from dsos.swarm.knowledge_base import DistributedKnowledgeBase, KnowledgeType, KnowledgeItem
from dsos.swarm.swarm_manager import ConsensusProtocol, TaskAllocator
from dsos.swarm.swarm_placeholders import (
    SimpleDecisionEngine, SimpleBehaviorTree, SimplePlanner, 
    SimpleResourceManager, SimpleEnergyManager, SimpleCoordinatorElection
)


class SwarmRole(Enum):
    """Roles that a drone can take in the swarm."""
    EXPLORER = "explorer"      # Focused on exploration and mapping
    COMMUNICATOR = "communicator"  # Focused on maintaining network connectivity
    DEFENDER = "defender"      # Focused on security and threat detection
    SUPPORTER = "supporter"    # Focused on helping other drones
    LEADER = "leader"          # Temporary coordination role
    WORKER = "worker"          # General purpose task execution


class SwarmState(Enum):
    """High-level swarm behavioral states."""
    EXPLORING = "exploring"        # Exploring unknown territory
    PATROLLING = "patrolling"      # Following a patrol route
    FORMING = "forming"            # Forming or maintaining formation
    SEARCHING = "searching"        # Searching for specific targets
    TRACKING = "tracking"          # Tracking a target
    RETURNING = "returning"        # Returning to base/home
    EMERGENCY = "emergency"        # Emergency situation
    IDLE = "idle"                  # Waiting for instructions


@dataclass
class SwarmMetrics:
    """Metrics for swarm performance and health."""
    cohesion: float = 0.0          # How well the swarm stays together (0-1)
    alignment: float = 0.0         # How aligned velocities are (0-1)
    separation: float = 0.0        # Proper spacing maintenance (0-1)
    coverage: float = 0.0          # Area coverage percentage (0-1)
    connectivity: float = 0.0      # Network connectivity percentage (0-1)
    efficiency: float = 0.0        # Mission efficiency (0-1)
    robustness: float = 0.0        # Fault tolerance (0-1)


class SwarmBaseAgent(DroneAgent):
    """
    Extended DroneAgent with full swarm intelligence capabilities.
    Maintains backward compatibility with existing DroneAgent API.
    """

    def __init__(self, agent_id: Optional[str] = None, backend=None):
        super().__init__(agent_id, backend)
        self.logger = logging.getLogger(f"swarm.{self.agent_id}")

        # Swarm-specific components
        self.knowledge_base: Optional[DistributedKnowledgeBase] = None
        self.mesh_network: Optional[MeshNetwork] = None

        # Swarm state and role
        self.swarm_role = self._determine_initial_role()
        self.swarm_state = SwarmState.IDLE
        self.swarm_metrics = SwarmMetrics()

        # Local world model and maps
        self.local_world_model: Dict[str, Any] = {}
        self.local_occupancy_grid: Optional[np.ndarray] = None
        self.semantic_map: Dict[str, Any] = {}
        self.explored_areas: Set[Tuple[int, int, int]] = set()  # Grid-based tracking

        # Neighbor tracking
        self.neighbors: Dict[str, Dict[str, Any]] = {}  # neighbor_id -> info
        self.neighbor_knowledge: Dict[str, Dict[str, Any]] = {}  # What we know about each neighbor
        self.communication_history: deque = deque(maxlen=1000)  # Recent communications

        # Decision making and planning
        self.decision_engine: Optional[Any] = None  # Will be set to DecisionEngine instance
        self.behavior_tree: Optional[Any] = None    # Will be set to BehaviorTree instance
        self.planner: Optional[Any] = None          # Will be set to Planner instance
        self.task_queue: deque = deque()            # Pending tasks
        self.completed_tasks: List[Dict[str, Any]] = []  # Completed tasks
        self.current_task: Optional[Dict[str, Any]] = None

        # Resource and energy management
        self.resource_manager: Optional[Any] = None  # Will be set to ResourceManager
        self.energy_manager: Optional[Any] = None    # Will be set to EnergyManager

        # Swarm coordination
        self.consensus_protocol: Optional[Any] = None  # Consensus mechanism
        self.task_allocator: Optional[Any] = None      # Task allocation system
        self.coordinator_election: Optional[Any] = None # Coordinator election mechanism

        # Configuration
        self.update_rate_hz = settings.getint('DSOS', 'swarm_update_rate_hz', 10)
        self.dt = 1.0 / self.update_rate_hz if self.update_rate_hz > 0 else 0.1
        self.neighbor_timeout = settings.getfloat('DSOS', 'neighbor_timeout_sec', 5.0)
        self.consensus_interval = settings.getfloat('DSOS', 'consensus_interval_sec', 2.0)
        self.task_allocation_interval = settings.getfloat('DSOS', 'task_allocation_interval_sec', 5.0)

        # State tracking
        self._last_update_time = 0.0
        self._last_consensus_time = 0.0
        self._last_task_allocation_time = 0.0
        self._is_swarm_initialized = False

    async def _handle_mesh_message(self, message: MeshMessage) -> None:
        """Handle incoming mesh network messages."""
        # This method can be extended to handle specific message types
        # For now, we just process it as needed
        pass

    async def initialize(self) -> None:
        """Initialize the swarm agent with all swarm intelligence components."""
        await super().initialize()  # Initialize base drone agent

        self.logger.info(f"Initializing swarm intelligence for drone {self.agent_id}")

        # Initialize swarm-specific components
        await self._initialize_swarm_components()

        # Mark as initialized
        self._is_swarm_initialized = True
        self.logger.info(f"Swarm agent {self.agent_id} initialized successfully")

    async def _initialize_swarm_components(self) -> None:
        """Initialize all swarm intelligence components."""
        # Initialize knowledge base
        if self.mesh_network is None:
            # Create or get mesh network from existing setup
            self.mesh_network = MeshNetwork(self.agent_id, self._handle_mesh_message)
            await self.mesh_network.start()

        self.knowledge_base = DistributedKnowledgeBase(self.agent_id, self.mesh_network)
        await self.knowledge_base.start()

        # Initialize local world model
        self._initialize_world_model()

        # Initialize subsystems (these would be implemented in separate modules)
        await self._initialize_decision_engine()
        await self._initialize_behavior_tree()
        await self._initialize_planner()
        await self._initialize_resource_managers()
        await self._initialize_coordination_mechanisms()

        # Set initial swarm state
        self.swarm_state = SwarmState.IDLE
        self.swarm_role = self._determine_initial_role()

    def _initialize_world_model(self) -> None:
        """Initialize the local world model and mapping systems."""
        # Initialize occupancy grid (3D)
        grid_size = [
            int(settings.getfloat('DSOS', 'world_size_x_m', 100.0) / settings.getfloat('DSOS', 'grid_resolution_m', 1.0)),
            int(settings.getfloat('DSOS', 'world_size_y_m', 100.0) / settings.getfloat('DSOS', 'grid_resolution_m', 1.0)),
            int(settings.getfloat('DSOS', 'world_size_z_m', 50.0) / settings.getfloat('DSOS', 'grid_resolution_m', 1.0))
        ]
        self.local_occupancy_grid = np.zeros((grid_size[0], grid_size[1], grid_size[2]), dtype=np.int8)

        # Initialize world model
        self.local_world_model = {
            'map_origin': [0.0, 0.0, 0.0],  # World coordinates of grid[0,0,0]
            'grid_resolution': settings.getfloat('DSOS', 'grid_resolution_m', 1.0),
            'last_updated': time.time(),
            'explored_volume': 0.0,
            'unknown_volume': float(np.prod(self.local_occupancy_grid.shape)),
            'dynamic_objects': {},
            'static_objects': {},
            'hazards': [],
            'points_of_interest': []
        }

        # Initialize semantic map
        self.semantic_map = {
            'landmarks': {},
            'regions': {},
            'paths': [],
            'areas_of_interest': {}
        }

    async def _initialize_decision_engine(self) -> None:
        """Initialize the decision-making engine."""
        # This would be implemented with actual decision-making logic
        # For now, we'll create a placeholder
        self.decision_engine = DecisionEngine = SimpleDecisionEngine()

    async def _initialize_behavior_tree(self) -> None:
        """Initialize the behavior tree for autonomous behavior."""
        # This would be implemented with actual behavior tree logic
        # For now, we'll create a placeholder
        self.behavior_tree = SimpleBehaviorTree()

    async def _initialize_planner(self) -> None:
        """Initialize the motion and mission planner."""
        # This would be implemented with actual planning algorithms
        # For now, we'll create a placeholder
        self.planner = SimplePlanner()

    async def _initialize_resource_managers(self) -> None:
        """Initialize resource and energy management systems."""
        # These would be implemented with actual resource tracking
        # For now, we'll create placeholders
        self.resource_manager = SimpleResourceManager()
        self.energy_manager = SimpleEnergyManager()

    async def _initialize_coordination_mechanisms(self) -> None:
        """Initialize swarm coordination mechanisms."""
        # Use real implementations for consensus and task allocation
        self.consensus_protocol = ConsensusProtocol()
        self.task_allocator = TaskAllocator()
        self.coordinator_election = SimpleCoordinatorElection()

    def _determine_initial_role(self) -> SwarmRole:
        """Determine the initial role for this drone based on capabilities and context."""
        # Simple heuristic based on drone ID - in practice this would be more sophisticated
        drone_num = int(self.agent_id.split('_')[-1]) if '_' in self.agent_id else 0
        if drone_num == 1:
            return SwarmRole.EXPLORER
        elif drone_num == 2:
            return SwarmRole.COMMUNICATOR
        elif drone_num == 3:
            return SwarmRole.DEFENDER
        else:
            return SwarmRole.WORKER

    async def shutdown(self) -> None:
        """Shutdown the swarm agent and all its components."""
        self.logger.info(f"Shutting down swarm agent {self.agent_id}")

        # Shutdown swarm components in reverse order
        if self.knowledge_base:
            await self.knowledge_base.stop()

        if self.mesh_network:
            await self.mesh_network.stop()

        # Shutdown subsystems
        if self.decision_engine:
            await self.decision_engine.shutdown()
        if self.behavior_tree:
            await self.behavior_tree.shutdown()
        if self.planner:
            await self.planner.shutdown()
        if self.resource_manager:
            await self.resource_manager.shutdown()
        if self.energy_manager:
            await self.energy_manager.shutdown()
        if self.consensus_protocol:
            await self.consensus_protocol.shutdown()
        if self.task_allocator:
            await self.task_allocator.shutdown()
        if self.coordinator_election:
            await self.coordinator_election.shutdown()

        # Call parent shutdown
        await super().shutdown()

        self.logger.info(f"Swarm agent {self.agent_id} shutdown complete")

    async def update_state(self) -> None:
        """Update the agent's state with swarm intelligence enhancements."""
        current_time = time.time()

        # Update base state (position, velocity, etc.)
        await super().update_state()

        # Update swarm-specific components at the configured rate
        if current_time - self._last_update_time >= self.dt:
            await self._update_swarm_state()
            self._last_update_time = current_time

    async def _update_swarm_state(self) -> None:
        """Update all swarm-specific state components."""
        try:
            # Update neighbor information
            await self._update_neighbor_information()

            # Update local world model based on sensor data
            await self._update_world_model()

            # Update swarm metrics
            await self._update_swarm_metrics()

            # Update decision making
            await self._update_decision_making()

            # Update behavior tree
            await self._update_behavior_tree()

            # Update planning
            await self._update_planning()

            # Update resource management
            await self._update_resource_management()

            # Update coordination mechanisms
            await self._update_coordination()

        except Exception as e:
            self.logger.error(f"Error updating swarm state: {e}")

    async def _update_neighbor_information(self) -> None:
        """Update information about neighboring drones."""
        if not self.mesh_network:
            return

        # Get current neighbors from mesh network
        neighbor_ids = self.mesh_network.get_neighbors()
        current_time = time.time()

        # Update each neighbor's information
        for neighbor_id in neighbor_ids:
            if neighbor_id not in self.neighbors:
                self.neighbors[neighbor_id] = {
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'message_count': 0
                }
            else:
                self.neighbors[neighbor_id]['last_seen'] = current_time

            # Increment message count (we'd get actual count from mesh network in practice)
            self.neighbors[neighbor_id]['message_count'] += 1

        # Remove neighbors that haven't been seen recently
        timeout_time = current_time - self.neighbor_timeout
        expired_neighbors = [
            nid for nid, info in self.neighbors.items()
            if info['last_seen'] < timeout_time
        ]
        for neighbor_id in expired_neighbors:
            del self.neighbors[neighbor_id]
            # Also remove their knowledge from our knowledge base
            # (In a full implementation, we might keep it for a while as historical data)

    async def _update_world_model(self) -> None:
        """Update the local world model based on sensor data and knowledge."""
        # This would integrate data from:
        # 1. Local sensor readings
        # 2. Knowledge from neighbors
        # 3. Previous map updates
        # 4. SLAM algorithms (if implemented)

        # For now, we'll update the explored areas based on current position
        if self.status.position:
            grid_pos = self._world_to_grid(self.status.position)
            self.explored_areas.add(grid_pos)

            # Mark surrounding cells as explored (sensor footprint)
            sensor_range_cells = int(
                settings.getfloat('DSOS', 'sensor_range_m', 10.0) /
                settings.getfloat('DSOS', 'grid_resolution_m', 1.0)
            )

            for dx in range(-sensor_range_cells, sensor_range_cells + 1):
                for dy in range(-sensor_range_cells, sensor_range_cells + 1):
                    for dz in range(-sensor_range_cells, sensor_range_cells + 1):
                        if dx*dx + dy*dy + dz*dz <= sensor_range_cells*sensor_range_cells:
                            check_pos = (
                                grid_pos[0] + dx,
                                grid_pos[1] + dy,
                                grid_pos[2] + dz
                            )
                            # Check bounds
                            if (0 <= check_pos[0] < self.local_occupancy_grid.shape[0] and
                                0 <= check_pos[1] < self.local_occupancy_grid.shape[1] and
                                0 <= check_pos[2] < self.local_occupancy_grid.shape[2]):
                                self.explored_areas.add(check_pos)

        # Update the occupancy grid based on explored areas
        self._update_occupancy_grid_from_explored()

    def _world_to_grid(self, world_pos: List[float]) -> Tuple[int, int, int]:
        """Convert world coordinates to grid coordinates."""
        wx, wy, wz = world_pos
        ox, oy, oz = self.local_world_model['map_origin']
        resolution = self.local_world_model['grid_resolution']

        gx = int((wx - ox) / resolution)
        gy = int((wy - oy) / resolution)
        gz = int((wz - oz) / resolution)

        # Clamp to grid bounds
        gx = max(0, min(gx, self.local_occupancy_grid.shape[0] - 1))
        gy = max(0, min(gy, self.local_occupancy_grid.shape[1] - 1))
        gz = max(0, min(gz, self.local_occupancy_grid.shape[2] - 1))

        return (gx, gy, gz)

    def _grid_to_world(self, grid_pos: Tuple[int, int, int]) -> List[float]:
        """Convert grid coordinates to world coordinates."""
        gx, gy, gz = grid_pos
        ox, oy, oz = self.local_world_model['map_origin']
        resolution = self.local_world_model['grid_resolution']

        wx = ox + gx * resolution
        wy = oy + gy * resolution
        wz = oz + gz * resolution

        return [wx, wy, wz]

    def _update_occupancy_grid_from_explored(self) -> None:
        """Update the occupancy grid based on explored areas."""
        # Mark explored cells as free space (0 = unknown, 1 = occupied, 2 = free)
        for gx, gy, gz in self.explored_areas:
            if (0 <= gx < self.local_occupancy_grid.shape[0] and
                0 <= gy < self.local_occupancy_grid.shape[1] and
                0 <= gz < self.local_occupancy_grid.shape[2]):
                self.local_occupancy_grid[gx, gy, gz] = 2  # Free space

        # Update explored volume in world model
        total_cells = np.prod(self.local_occupancy_grid.shape)
        explored_cells = np.count_nonzero(self.local_occupancy_grid == 2)
        self.local_world_model['explored_volume'] = explored_cells * (
            self.local_world_model['grid_resolution'] ** 3
        )
        self.local_world_model['unknown_volume'] = (
            total_cells - np.count_nonzero(self.local_occupancy_grid == 0)
        ) * (self.local_world_model['grid_resolution'] ** 3)

    async def _update_swarm_metrics(self) -> None:
        """Update swarm performance metrics."""
        # Calculate cohesion (how close drones are to each other)
        if len(self.neighbors) > 0:
            distances = []
            for neighbor_id in self.neighbors.keys():
                # In a real implementation, we'd get actual positions
                # For now, we'll use placeholder values
                distances.append(10.0)  # Placeholder distance

            if distances:
                avg_distance = sum(distances) / len(distances)
                # Normalize: closer = higher cohesion (up to a point)
                self.swarm_metrics.cohesion = max(0.0, min(1.0, 1.0 - (avg_distance / 50.0)))
        else:
            self.swarm_metrics.cohesion = 0.0

        # Calculate alignment (velocity alignment)
        # This would require actual velocity data from neighbors
        self.swarm_metrics.alignment = 0.8  # Placeholder

        # Calculate separation (proper spacing)
        self.swarm_metrics.separation = 0.9  # Placeholder

        # Calculate coverage (based on explored area)
        total_volume = (
            self.local_world_model['grid_resolution'] ** 3 *
            np.prod(self.local_occupancy_grid.shape)
        )
        if total_volume > 0:
            explored_volume = self.local_world_model['explored_volume']
            self.swarm_metrics.coverage = min(1.0, explored_volume / total_volume)
        else:
            self.swarm_metrics.coverage = 0.0

        # Calculate connectivity (based on neighbor count)
        max_neighbors = settings.getint('DSOS', 'max_neighbors', 6)
        neighbor_count = len(self.neighbors)
        self.swarm_metrics.connectivity = min(1.0, neighbor_count / max_neighbors) if max_neighbors > 0 else 0.0

        # Calculate efficiency (task completion rate)
        total_tasks = len(self.completed_tasks) + len(self.task_queue) + (1 if self.current_task else 0)
        if total_tasks > 0:
            self.swarm_metrics.efficiency = len(self.completed_tasks) / total_tasks
        else:
            self.swarm_metrics.efficiency = 0.0

        # Calculate robustness (fault tolerance)
        # This would be based on redundancy, communication reliability, etc.
        self.swarm_metrics.robustness = 0.7  # Placeholder

    async def _handle_mesh_message(self, message: MeshMessage) -> None:
        """Handle incoming mesh network messages."""
        # This method can be extended to handle specific message types
        # For now, we just process it as needed
        pass

    async def _update_decision_making(self) -> None:
        """Update the decision-making process."""
        if self.decision_engine:
            # Prepare context for decision making
            context = {
                'drone_id': self.agent_id,
                'state': self.status.state.name,
                'swarm_state': self.swarm_state.name,
                'swarm_role': self.swarm_role.name,
                'position': self.status.position,
                'velocity': self.status.velocity,
                'battery_level': self.get_battery_level(),
                'health': self.status.health,
                'neighbors': list(self.neighbors.keys()),
                'knowledge_base_stats': self.knowledge_base.get_statistics() if self.knowledge_base else {},
                'swarm_metrics': self._get_swarm_metrics_dict(),
                'timestamp': time.time()
            }

            # Get decision from engine
            decision = await self.decision_engine.make_decision(context)

            # Apply decision if it differs from current state
            if decision:
                await self._apply_decision(decision)

    async def _apply_decision(self, decision: Dict[str, Any]) -> None:
        """Apply a decision from the decision engine."""
        decision_type = decision.get('type')

        if decision_type == 'state_change':
            new_state = decision.get('state')
            if new_state and hasattr(SwarmState, new_state.upper()):
                self.swarm_state = SwarmState[new_state.upper()]
                self.logger.debug(f"Changed swarm state to {new_state}")

        elif decision_type == 'role_change':
            new_role = decision.get('role')
            if new_role and hasattr(SwarmRole, new_role.upper()):
                self.swarm_role = SwarmRole[new_role.upper()]
                self.logger.debug(f"Changed swarm role to {new_role}")

        elif decision_type == 'movement':
            target_position = decision.get('position')
            if target_position:
                await self.goto_position(target_position)

        elif decision_type == 'task':
            task = decision.get('task')
            if task:
                await self._assign_task(task)

        # Add more decision types as needed

    async def _update_behavior_tree(self) -> None:
        """Update the behavior tree execution."""
        if self.behavior_tree:
            # Update blackboard with current state
            blackboard_update = {
                'drone_id': self.agent_id,
                'state': self.status.state.name,
                'swarm_state': self.swarm_state.name,
                'swarm_role': self.swarm_role.name,
                'position': self.status.position,
                'velocity': self.status.velocity,
                'battery_level': self.get_battery_level(),
                'health': self.status.health,
                'neighbors': list(self.neighbors.keys()),
                'timestamp': time.time()
            }
            await self.behavior_tree.update_blackboard(blackboard_update)

            # Execute one tick of the behavior tree
            result = await self.behavior_tree.tick()
            # The behavior tree might issue commands that we need to execute
            # This would be handled by the behavior tree returning actions to take

    async def _update_planning(self) -> None:
        """Update planning and trajectory generation."""
        if self.planner and self.current_task:
            # Check if we need to replan
            if await self.planner.needs_replan(self.current_task, self.status):
                new_plan = await self.planner.plan(
                    self.current_task,
                    self.status,
                    self.knowledge_base,
                    self.local_world_model
                )
                if new_plan:
                    # Execute the plan (this would involve sending commands to follow the path)
                    pass  # Implementation would go here

    async def _update_resource_management(self) -> None:
        """Update resource and energy management."""
        if self.resource_manager:
            await self.resource_manager.update_resources(
                self.status,
                self.knowledge_base,
                self.local_world_model
            )

        if self.energy_manager:
            await self.energy_manager.update_energy(
                self.status,
                self.knowledge_base,
                self.local_world_model
            )

    async def _update_coordination(self) -> None:
        """Update swarm coordination mechanisms."""
        current_time = time.time()

        # Update consensus
        if self.consensus_protocol and current_time - self._last_consensus_time >= self.consensus_interval:
            await self.consensus_protocol.update_consensus(
                self.agent_id,
                self.knowledge_base,
                self.neighbors,
                self.mesh_network
            )
            self._last_consensus_time = current_time

        # Update task allocation
        if self.task_allocator and current_time - self._last_task_allocation_time >= self.task_allocation_interval:
            await self.task_allocator.allocate_tasks(
                self.agent_id,
                self.knowledge_base,
                self.neighbors,
                self.mesh_network,
                self.task_queue
            )
            self._last_task_allocation_time = current_time

        # Check for coordinator election
        if self.coordinator_election:
            await self.coordinator_election.check_election(
                self.agent_id,
                self.knowledge_base,
                self.neighbors,
                self.mesh_network
            )

    async def communicate(self) -> None:
        """Enhanced communication that includes knowledge sharing."""
        # Call the parent communicate method (handles mesh network communication)
        await super().communicate()

        # Additional swarm-specific communication can go here
        # For example, broadcasting important discoveries immediately

    async def execute_commands(self) -> None:
        """Execute commands with swarm intelligence enhancements."""
        # Execute base commands (motor control, basic navigation)
        await super().execute_commands()

        # Execute swarm-specific commands
        await self._execute_swarm_commands()

    async def _execute_swarm_commands(self) -> None:
        """Execute swarm-specific commands from behavior tree, planner, etc."""
        # Get commands from behavior tree
        if self.behavior_tree:
            actions = await self.behavior_tree.get_pending_actions()
            for action in actions:
                await self._execute_action(action)

        # Get commands from planner
        if self.planner and self.current_task:
            waypoints = await self.planner.get_next_waypoints(self.status)
            for waypoint in waypoints:
                await self.goto_position(waypoint)

        # Execute any pending tasks from the task queue
        await self._process_task_queue()

    async def _execute_action(self, action: Dict[str, Any]) -> None:
        """Execute a single action from the behavior tree or planner."""
        action_type = action.get('type')

        if action_type == 'move_to':
            position = action.get('position')
            if position:
                await self.goto_position(position)

        elif action_type == 'hover':
            # Just maintain current position (already handled by base class)
            pass

        elif action_type == 'takeoff':
            altitude = action.get('altitude', 10.0)
            await self.takeoff(altitude)

        elif action_type == 'land':
            await self.land()

        elif action_type == 'communicate_knowledge':
            # Share important knowledge with neighbors
            await self._broadcast_important_knowledge()

        # Add more action types as needed

    async def _broadcast_important_knowledge(self) -> None:
        """Broadcast important discoveries to neighbors immediately."""
        if not self.knowledge_base or not self.mesh_network:
            return

        # Find high-confidence, recent knowledge that's worth sharing
        recent_knowledge = self.knowledge_base.get_knowledge_by_type(KnowledgeType.OBJECT)
        recent_knowledge.extend(self.knowledge_base.get_knowledge_by_type(KnowledgeType.HAZARD))
        recent_knowledge.extend(self.knowledge_base.get_knowledge_by_type(KnowledgeType.TARGET))

        # Filter for high confidence and recent
        important_knowledge = [
            k for k in recent_knowledge
            if k.confidence > 0.8 and (time.time() - k.timestamp) < 5.0
        ]

        if important_knowledge:
            # Create a knowledge update message
            update_data = {
                'source_drone': self.agent_id,
                'timestamp': time.time(),
                'urgent': True,
                'updates': [k.to_dict() for k in important_knowledge[:10]]  # Limit to 10 items
            }

            message = MeshMessage(
                msg_id=f"urgent_knowledge_{int(time.time() * 1000)}",
                msg_type=MessageType.COMMAND,
                sender_id=self.agent_id,
                recipient_id=None,  # Broadcast
                timestamp=time.time(),
                payload=update_data,
                ttl=settings.getint('DSOS', 'message_ttl', 5)
            )

            await self.mesh_network.send_message(message)
            self.logger.debug(f"Broadcasted {len(important_knowledge)} important knowledge items")

    async def _assign_task(self, task: Dict[str, Any]) -> None:
        """Assign a task to this drone."""
        self.current_task = task
        self.task_queue.append(task)
        self.logger.info(f"Assigned task: {task.get('description', 'Unknown task')}")

    async def _process_task_queue(self) -> None:
        """Process tasks from the queue."""
        if not self.task_queue:
            return

        # Simple FIFO processing - in reality this would be priority-based
        if not self.current_task and self.task_queue:
            self.current_task = self.task_queue.popleft()
            self.logger.info(f"Starting task: {self.current_task.get('description', 'Unknown task')}")

        # Check if current task is complete
        if self.current_task and await self._is_task_complete(self.current_task):
            self.completed_tasks.append(self.current_task)
            self.logger.info(f"Completed task: {self.current_task.get('description', 'Unknown task')}")
            self.current_task = None

    async def _is_task_complete(self, task: Dict[str, Any]) -> bool:
        """Check if a task is complete."""
        # This would be implemented with actual task completion logic
        # For now, we'll use a simple placeholder
        task_type = task.get('type', '')
        if task_type == 'waypoint_navigation':
            # Check if we're close to the target waypoint
            target = task.get('target_position', [])
            if self.status.position and target:
                distance = np.linalg.norm(
                    np.array(self.status.position) - np.array(target)
                )
                return distance < 2.0  # Within 2 meters
        elif task_type == 'formation':
            # Check if we're in formation
            return await self._is_in_formation()
        elif task_type == 'exploration':
            # Check exploration progress
            return self.swarm_metrics.coverage > 0.95

        return False  # Default to not complete

    async def _is_in_formation(self) -> bool:
        """Check if the drone is maintaining its formation position."""
        # This would check actual formation position vs desired position
        return True  # Placeholder

    def _get_swarm_metrics_dict(self) -> Dict[str, float]:
        """Get swarm metrics as a dictionary."""
        return {
            'cohesion': self.swarm_metrics.cohesion,
            'alignment': self.swarm_metrics.alignment,
            'separation': self.swarm_metrics.separation,
            'coverage': self.swarm_metrics.coverage,
            'connectivity': self.swarm_metrics.connectivity,
            'efficiency': self.swarm_metrics.efficiency,
            'robustness': self.swarm_metrics.robustness
        }

    # Override or extend existing methods to integrate swarm intelligence

    async def takeoff(self, target_altitude: float = None) -> None:
        """Enhanced takeoff that considers swarm formation."""
        # Check if we should wait for swarm coordination
        if self.swarm_state == SwarmState.FORMING:
            # Wait for formation to be established before taking off
            pass  # Would implement formation waiting logic

        await super().takeoff(target_altitude)

        # Update swarm state after takeoff
        if self.get_battery_level() > 20:  # Not critical battery
            self.swarm_state = SwarmState.IDLE  # Ready for next command

    async def land(self) -> None:
        """Enhanced landing that considers swarm coordination."""
        # Notify swarm of impending landing
        await self._broadcast_land_intent()

        await super().land()

        # Update swarm state after landing
        self.swarm_state = SwarmState.IDLE
        self.swarm_role = self._determine_post_landing_role()

    async def _broadcast_land_intent(self) -> None:
        """Broadcast intention to land to neighbors."""
        if not self.mesh_network:
            return

        message = MeshMessage(
            msg_id=f"land_intent_{int(time.time() * 1000)}",
            msg_type=MessageType.COMMAND,
            sender_id=self.agent_id,
            recipient_id=None,  # Broadcast
            timestamp=time.time(),
            payload={
                'action': 'landing',
                'drone_id': self.agent_id,
                'timestamp': time.time()
            },
            ttl=settings.getint('DSOS', 'message_ttl', 5)
        )

        await self.mesh_network.send_message(message)

    def _determine_post_landing_role(self) -> SwarmRole:
        """Determine role after landing (e.g., charging, maintenance)."""
        # Landed drones might take on support roles
        return SwarmRole.SUPPORTER

    # Additional swarm intelligence methods that can be called by GCS or other systems

    def get_swarm_status(self) -> Dict[str, Any]:
        """Get comprehensive swarm status information."""
        return {
            'drone_id': self.agent_id,
            'timestamp': time.time(),
            'basic_status': {
                'state': self.status.state.name.lower(),
                'position': self.status.position,
                'velocity': self.status.velocity,
                'battery_level': self.get_battery_level(),
                'health': self.status.health,
                'armed': self.is_armed()
            },
            'swarm_status': {
                'swarm_state': self.swarm_state.value,
                'swarm_role': self.swarm_role.value,
                'swarm_metrics': self._get_swarm_metrics_dict()
            },
            'knowledge_base': self.knowledge_base.get_statistics() if self.knowledge_base else {},
            'neighbors': {
                'count': len(self.neighbors),
                'list': list(self.neighbors.keys())
            },
            'task_queue': {
                'pending': len(self.task_queue),
                'current': self.current_task.get('description') if self.current_task else None,
                'completed': len(self.completed_tasks)
            }
        }

    def get_digital_twin(self) -> Dict[str, Any]:
        """Get a digital twin representation of this drone."""
        return {
            'entity_id': self.agent_id,
            'entity_type': 'drone',
            'timestamp': time.time(),
            'state': {
                'position': self.status.position.tolist() if hasattr(self.status.position, 'tolist') else self.status.position,
                'velocity': self.status.velocity.tolist() if hasattr(self.status.velocity, 'tolist') else self.status.velocity,
                'orientation': self.status.orientation.tolist() if hasattr(self.status.orientation, 'tolist') else self.status.orientation,
                'battery_level': self.get_battery_level(),
                'battery_voltage': getattr(self.status, 'battery_voltage', None),
                'health': self.status.health,
                'is_armed': self.is_armed(),
                'is_in_air': self.status.state not in [AgentState.LANDING, AgentState.IDLE, AgentState.EMERGENCY],
                'land_state': getattr(self.status, 'land_state', 'unknown')
            },
            'capabilities': {
                'can_fly': self.get_battery_level() > 20,
                'can_communicate': self.mesh_network is not None and self.mesh_network.is_connected(),
                'can_sense': True,  # Simplified
                'can_process': True
            },
            'mission': {
                'current_task': self.current_task.get('description') if self.current_task else None,
                'progress': 0.0 if not self.current_task else 0.5,  # Simplified
                'waypoints_completed': len([t for t in self.completed_tasks if t.get('type') == 'waypoint_navigation']),
                'total_waypoints': len([t for t in list(self.task_queue) + [self.current_task] if t and t.get('type') == 'waypoint_navigation']) + len([t for t in self.completed_tasks if t.get('type') == 'waypoint_navigation'])
            },
            'swarm': {
                'role': self.swarm_role.value,
                'state': self.swarm_state.value,
                'neighbors_count': len(self.neighbors),
                'knowledge_items': len(self.knowledge_base.knowledge_items) if self.knowledge_base else 0
            },
            'health_prediction': {
                'estimated_fault_time': None,  # Would be predictions from health monitoring
                'maintenance_needed': False
            }
        }