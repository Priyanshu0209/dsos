"""
Navigation module for path planning, obstacle avoidance, and flight control.
"""
import asyncio
import math
import heapq
from typing import List, Tuple, Optional, Dict, Any
import logging

import numpy as np

from dsos.config.settings import settings


class NavigationError(Exception):
    """Exception raised for navigation-related errors."""
    pass


class Node:
    """A node in the pathfinding grid."""

    def __init__(self, x: int, y: int, z: int, walkable: bool = True):
        self.x = x
        self.y = y
        self.z = z
        self.walkable = walkable
        self.g_cost = float('inf')  # Cost from start to this node
        self.h_cost = 0  # Heuristic cost to end
        self.f_cost = float('inf')  # Total cost
        self.parent: Optional['Node'] = None

    @property
    def position(self) -> Tuple[int, int, int]:
        return (self.x, self.y, self.z)

    def __lt__(self, other):
        return self.f_cost < other.f_cost


class Grid3D:
    """3D grid for pathfinding."""

    def __init__(self, width: int, height: int, depth: int,
                 resolution: float = 1.0, origin: List[float] = [0, 0, 0]):
        """
        Initialize a 3D grid.

        Args:
            width, height, depth: Grid dimensions in cells
            resolution: Size of each cell in meters
            origin: [x, y, z] offset of the grid origin in world coordinates
        """
        self.width = width
        self.height = height
        self.depth = depth
        self.resolution = resolution
        self.origin = np.array(origin, dtype=float)
        self.grid: List[List[List[Node]]] = []
        self._initialize_grid()

    def _initialize_grid(self) -> None:
        """Initialize the grid with walkable nodes."""
        self.grid = [[[Node(x, y, z) for z in range(self.depth)]
                      for y in range(self.height)]
                     for x in range(self.width)]

    def set_obstacle(self, x: int, y: int, z: int) -> None:
        """Mark a cell as an obstacle."""
        if self._is_valid_cell(x, y, z):
            self.grid[x][y][z].walkable = False

    def clear_obstacle(self, x: int, y: int, z: int) -> None:
        """Mark a cell as walkable."""
        if self._is_valid_cell(x, y, z):
            self.grid[x][y][z].walkable = True

    def _is_valid_cell(self, x: int, y: int, z: int) -> bool:
        """Check if cell coordinates are within grid bounds."""
        return (0 <= x < self.width and
                0 <= y < self.height and
                0 <= z < self.depth)

    def world_to_grid(self, world_pos: List[float]) -> Tuple[int, int, int]:
        """Convert world coordinates to grid coordinates."""
        wx, wy, wz = world_pos
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int((wy - self.origin[1]) / self.resolution)
        gz = int((wz - self.origin[2]) / self.resolution)
        return (gx, gy, gz)

    def grid_to_world(self, grid_pos: Tuple[int, int, int]) -> List[float]:
        """Convert grid coordinates to world coordinates."""
        gx, gy, gz = grid_pos
        wx = gx * self.resolution + self.origin[0]
        wy = gy * self.resolution + self.origin[1]
        wz = gz * self.resolution + self.origin[2]
        return [wx, wy, wz]

    def is_walkable(self, x: int, y: int, z: int) -> bool:
        """Check if a cell is walkable."""
        if not self._is_valid_cell(x, y, z):
            return False
        return self.grid[x][y][z].walkable

    def get_neighbors(self, node: Node) -> List[Node]:
        """Get walkable neighbors of a node."""
        neighbors = []
        # 26-connected neighborhood (including diagonals)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue  # Skip self
                    nx, ny, nz = node.x + dx, node.y + dy, node.z + dz
                    if self._is_valid_cell(nx, ny, nz) and self.grid[nx][ny][nz].walkable:
                        neighbors.append(self.grid[nx][ny][nz])
        return neighbors


class AStarPathfinder:
    """A* pathfinding algorithm for 3D grids."""

    def __init__(self, grid: Grid3D):
        self.grid = grid

    def find_path(self, start: List[float], goal: List[float]) -> Optional[List[List[float]]]:
        """
        Find a path from start to goal using A*.

        Args:
            start: [x, y, z] start position in world coordinates
            goal: [x, y, z] goal position in world coordinates

        Returns:
            List of [x, y, z] waypoints representing the path, or None if no path found
        """
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(start)
        goal_grid = self.grid.world_to_grid(goal)

        # Check if start and goal are valid
        if not self.grid.is_walkable(*start_grid):
            raise NavigationError(f"Start position {start} is not walkable")
        if not self.grid.is_walkable(*goal_grid):
            raise NavigationError(f"Goal position {goal} is not walkable")

        # Initialize start and goal nodes
        start_node = self.grid.grid[start_grid[0]][start_grid[1]][start_grid[2]]
        goal_node = self.grid.grid[goal_grid[0]][goal_grid[1]][goal_grid[2]]

        # Reset node costs
        self._reset_node_costs()

        # A* algorithm
        open_set: List[Node] = []
        closed_set: set = set()

        heapq.heappush(open_set, start_node)
        start_node.g_cost = 0
        start_node.h_cost = self._heuristic(start_node, goal_node)
        start_node.f_cost = start_node.g_cost + start_node.h_cost

        while open_set:
            current_node = heapq.heappop(open_set)

            if current_node.position == goal_node.position:
                # Found the goal, reconstruct path
                return self._reconstruct_path(current_node)

            closed_set.add(current_node.position)

            for neighbor in self.grid.get_neighbors(current_node):
                if neighbor.position in closed_set:
                    continue

                # Calculate tentative g_score
                tentative_g_cost = current_node.g_cost + self._distance(current_node, neighbor)

                if tentative_g_cost < neighbor.g_cost:
                    neighbor.parent = current_node
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self._heuristic(neighbor, goal_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost

                    if neighbor not in open_set:
                        heapq.heappush(open_set, neighbor)

        # No path found
        return None

    def _reset_node_costs(self) -> None:
        """Reset all node costs to infinity."""
        for x in range(self.grid.width):
            for y in range(self.grid.height):
                for z in range(self.grid.depth):
                    node = self.grid.grid[x][y][z]
                    node.g_cost = float('inf')
                    node.h_cost = 0
                    node.f_cost = float('inf')
                    node.parent = None

    def _heuristic(self, node: Node, goal: Node) -> float:
        """Heuristic function (Euclidean distance)."""
        return self._distance(node, goal)

    def _distance(self, node1: Node, node2: Node) -> float:
        """Calculate distance between two nodes."""
        dx = (node1.x - node2.x) * self.grid.resolution
        dy = (node1.y - node2.y) * self.grid.resolution
        dz = (node1.z - node2.z) * self.grid.resolution
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _reconstruct_path(self, current_node: Node) -> List[List[float]]:
        """Reconstruct the path from start to goal."""
        path: List[List[float]] = []
        while current_node is not None:
            world_pos = self.grid.grid_to_world(current_node.position)
            path.append(world_pos)
            current_node = current_node.parent
        path.reverse()  # Reverse to get start -> goal
        return path


class RRTNode:
    """A node in the RRT* tree."""
    def __init__(self, pos: np.ndarray):
        self.position = pos
        self.parent = None
        self.cost = 0.0

class RRTStarPathfinder:
    """RRT* pathfinding algorithm for continuous spaces."""
    
    def __init__(self, grid: Grid3D, max_iter: int = 500, step_size: float = 2.0, search_radius: float = 4.0):
        self.grid = grid
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        
    def find_path(self, start: List[float], goal: List[float]) -> Optional[List[List[float]]]:
        start_node = RRTNode(np.array(start))
        goal_node = RRTNode(np.array(goal))
        nodes = [start_node]
        
        for i in range(self.max_iter):
            # Sample random point or goal
            if np.random.rand() < 0.1:
                rand_pos = goal_node.position
            else:
                x = np.random.uniform(0, self.grid.width * self.grid.resolution) + self.grid.origin[0]
                y = np.random.uniform(0, self.grid.height * self.grid.resolution) + self.grid.origin[1]
                z = np.random.uniform(0, self.grid.depth * self.grid.resolution) + self.grid.origin[2]
                rand_pos = np.array([x, y, z])
                
            # Find nearest node
            nearest_node = min(nodes, key=lambda n: np.linalg.norm(n.position - rand_pos))
            
            # Steer
            direction = rand_pos - nearest_node.position
            dist = np.linalg.norm(direction)
            if dist > self.step_size:
                rand_pos = nearest_node.position + (direction / dist) * self.step_size
                
            # Check collision (simple check)
            if not self._is_collision_free(nearest_node.position, rand_pos):
                continue
                
            new_node = RRTNode(rand_pos)
            
            # Find near nodes for RRT* rewiring
            near_nodes = [n for n in nodes if np.linalg.norm(n.position - new_node.position) <= self.search_radius]
            
            # Choose best parent
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + np.linalg.norm(new_node.position - nearest_node.position)
            
            for near_node in near_nodes:
                if self._is_collision_free(near_node.position, new_node.position):
                    cost = near_node.cost + np.linalg.norm(new_node.position - near_node.position)
                    if cost < new_node.cost:
                        new_node.parent = near_node
                        new_node.cost = cost
            
            nodes.append(new_node)
            
            # Rewire tree
            for near_node in near_nodes:
                if self._is_collision_free(new_node.position, near_node.position):
                    cost = new_node.cost + np.linalg.norm(near_node.position - new_node.position)
                    if cost < near_node.cost:
                        near_node.parent = new_node
                        near_node.cost = cost
                        
            # Check if reached goal
            if np.linalg.norm(new_node.position - goal_node.position) <= self.step_size:
                if self._is_collision_free(new_node.position, goal_node.position):
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + np.linalg.norm(goal_node.position - new_node.position)
                    nodes.append(goal_node)
                    return self._reconstruct_path(goal_node)
                    
        return None
        
    def _is_collision_free(self, pos1: np.ndarray, pos2: np.ndarray) -> bool:
        # Check along the line segment
        steps = int(np.linalg.norm(pos2 - pos1) / (self.grid.resolution / 2))
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0
            p = pos1 + (pos2 - pos1) * t
            gx, gy, gz = self.grid.world_to_grid(p.tolist())
            if not self.grid.is_walkable(gx, gy, gz):
                return False
        return True
        
    def _reconstruct_path(self, node: RRTNode) -> List[List[float]]:
        path = []
        while node is not None:
            path.append(node.position.tolist())
            node = node.parent
        path.reverse()
        return path


class NavigationManager:
    """Manages navigation for a drone agent."""

    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.logger = logging.getLogger(f"navigation.{agent_id}")
        self.current_path: List[List[float]] = []
        self.current_waypoint_index = 0
        self.pathfinder: Optional[AStarPathfinder] = None
        self.rrt_pathfinder: Optional[RRTStarPathfinder] = None
        self.grid: Optional[Grid3D] = None
        self.waypoint_tolerance = settings.getfloat('DSOS', 'waypoint_acceptance_radius_m', 1.0)
        self.replan_threshold = settings.getfloat('DSOS', 'replan_distance_threshold_m', 5.0)
        self.last_replan_position: Optional[List[float]] = None
        
        # Multi-drone collision avoidance
        self.neighbors_positions: Dict[str, np.ndarray] = {}
        self.safety_radius = settings.getfloat('DSOS', 'min_safe_distance_m', 2.0)

    def initialize_navigation(self, world_bounds: List[List[float]],
                              resolution: float = 1.0) -> None:
        """
        Initialize the navigation system with a 3D grid.

        Args:
            world_bounds: [[min_x, min_y, min_z], [max_x, max_y, max_z]] in meters
            resolution: Grid cell size in meters
        """
        min_bound, max_bound = world_bounds[0], world_bounds[1]
        width = int((max_bound[0] - min_bound[0]) / resolution) + 1
        height = int((max_bound[1] - min_bound[1]) / resolution) + 1
        depth = int((max_bound[2] - min_bound[2]) / resolution) + 1
        origin = [min_bound[0], min_bound[1], min_bound[2]]

        self.grid = Grid3D(width, height, depth, resolution, origin)
        self.pathfinder = AStarPathfinder(self.grid)
        self.rrt_pathfinder = RRTStarPathfinder(self.grid)
        self.logger.info(
            f"Initialized navigation grid: {width}x{height}x{depth} cells "
            f"at {resolution}m resolution"
        )

    def add_obstacle(self, position: List[float], radius: float = 1.0) -> None:
        """
        Add a spherical obstacle to the navigation grid.

        Args:
            position: [x, y, z] center of obstacle
            radius: Radius of obstacle in meters
        """
        if not self.grid:
            self.logger.warning("Navigation grid not initialized")
            return

        # Convert to grid coordinates and mark cells within radius as obstacles
        center_gx, center_gy, center_gz = self.grid.world_to_grid(position)
        radius_cells = int(radius / self.grid.resolution) + 1

        for x in range(max(0, center_gx - radius_cells),
                       min(self.grid.width, center_gx + radius_cells + 1)):
            for y in range(max(0, center_gy - radius_cells),
                           min(self.grid.height, center_gy + radius_cells + 1)):
                for z in range(max(0, center_gz - radius_cells),
                               min(self.grid.depth, center_gz + radius_cells + 1)):
                    # Check if point is within sphere
                    dx = (x - center_gx) * self.grid.resolution
                    dy = (y - center_gy) * self.grid.resolution
                    dz = (z - center_gz) * self.grid.resolution
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    if distance <= radius:
                        self.grid.set_obstacle(x, y, z)

    def clear_obstacles(self) -> None:
        """Clear all obstacles from the navigation grid."""
        if not self.grid:
            return
        self.grid = Grid3D(
            self.grid.width, self.grid.height, self.grid.depth,
            self.grid.resolution, self.grid.origin.tolist()
        )
        if self.pathfinder:
            self.pathfinder.grid = self.grid
        self.logger.info("Cleared all obstacles from navigation grid")

    def update_neighbor_position(self, neighbor_id: str, position: List[float]) -> None:
        """Update neighbor position for collision avoidance."""
        self.neighbors_positions[neighbor_id] = np.array(position)

    def remove_neighbor(self, neighbor_id: str) -> None:
        """Remove a neighbor that is no longer relevant."""
        if neighbor_id in self.neighbors_positions:
            del self.neighbors_positions[neighbor_id]

    def compute_avoidance_velocity(self, current_pos: List[float], desired_velocity: List[float]) -> List[float]:
        """Compute avoidance velocity using artificial potential fields for multi-drone avoidance."""
        pos = np.array(current_pos)
        vel = np.array(desired_velocity)
        repulsion = np.zeros(3)
        
        for nid, npos in self.neighbors_positions.items():
            diff = pos - npos
            dist = np.linalg.norm(diff)
            if dist < self.safety_radius and dist > 0.01:
                # Repulsive force inversely proportional to distance squared
                force = (1.0 / dist - 1.0 / self.safety_radius) * (1.0 / (dist ** 2))
                repulsion += force * (diff / dist)
                
        # Combine desired velocity with repulsion
        k_rep = 2.0  # Repulsion gain
        adjusted_vel = vel + k_rep * repulsion
        
        # Cap velocity magnitude
        max_v = settings.getfloat('DSOS', 'max_velocity_mps', 5.0)
        v_mag = np.linalg.norm(adjusted_vel)
        if v_mag > max_v:
            adjusted_vel = adjusted_vel / v_mag * max_v
            
        return adjusted_vel.tolist()

    def plan_path(self, start: List[float], goal: List[float], use_rrt: bool = False) -> bool:
        """
        Plan a path from start to goal.

        Args:
            start: [x, y, z] start position
            goal: [x, y, z] goal position

        Returns:
            True if path found, False otherwise
        """
        if not self.pathfinder:
            self.logger.warning("Pathfinder not initialized")
            return False

        try:
            if use_rrt and self.rrt_pathfinder:
                path = self.rrt_pathfinder.find_path(start, goal)
            else:
                path = self.pathfinder.find_path(start, goal)
                
            if path is None:
                self.logger.warning(f"No path found from {start} to {goal}")
                self.current_path = []
                return False

            self.current_path = path
            self.current_waypoint_index = 0
            self.last_replan_position = start.copy()
            self.logger.info(
                f"Planned path with {len(path)} waypoints from {start} to {goal}"
            )
            return True
        except NavigationError as e:
            self.logger.error(f"Navigation error: {e}")
            self.current_path = []
            return False

    def get_next_waypoint(self) -> Optional[List[float]]:
        """Get the next waypoint in the current path."""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            return None
        return self.current_path[self.current_waypoint_index]

    def advance_waypoint(self) -> bool:
        """Advance to the next waypoint if close enough to current."""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            return False

        # This would typically be called by the agent when it reaches a waypoint
        # For now, we just increment the index
        if self.current_waypoint_index < len(self.current_path) - 1:
            self.current_waypoint_index += 1
            return True
        else:
            # At final waypoint
            return False

    def is_path_complete(self) -> bool:
        """Check if the current path has been completed."""
        return (not self.current_path or
                self.current_waypoint_index >= len(self.current_path))

    def should_replan(self, current_position: List[float]) -> bool:
        """Check if we should replan the path based on current position."""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            return True  # No path or path complete

        if self.last_replan_position is None:
            return True

        # Check if we've drifted significantly from the planned path
        dx = current_position[0] - self.last_replan_position[0]
        dy = current_position[1] - self.last_replan_position[1]
        dz = current_position[2] - self.last_replan_position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        return distance > self.replan_threshold

    def replan_path(self, current_position: List[float],
                    goal: List[float]) -> bool:
        """
        Replan the path from current position to goal.

        Args:
            current_position: Current [x, y, z] position
            goal: Target [x, y, z] position

        Returns:
            True if replanning succeeded, False otherwise
        """
        self.last_replan_position = current_position.copy()
        return self.plan_path(current_position, goal)

    def smooth_path(self, path: List[List[float]],
                    smoothing_factor: float = 0.1) -> List[List[float]]:
        """
        Smooth a path using simple averaging.

        Args:
            path: List of waypoints to smooth
            smoothing_factor: Amount of smoothing (0.0 = no smoothing, 1.0 = max)

        Returns:
            Smoothed path
        """
        if len(path) < 3:
            return path

        smoothed = [path[0]]  # Keep first point
        for i in range(1, len(path) - 1):
            prev = np.array(path[i-1])
            curr = np.array(path[i])
            next_pt = np.array(path[i+1])
            # Simple smoothing: move current point towards average of neighbors
            smoothed_pt = curr + smoothing_factor * ((prev + next_pt) / 2 - curr)
            smoothed.append(smoothed_pt.tolist())
        smoothed.append(path[-1])  # Keep last point
        return smoothed

    def get_path_length(self) -> float:
        """Get the total length of the current path."""
        if not self.current_path or len(self.current_path) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(self.current_path)):
            p1 = np.array(self.current_path[i-1])
            p2 = np.array(self.current_path[i])
            length += np.linalg.norm(p2 - p1)
        return length

    def get_remaining_distance(self, current_position: List[float]) -> float:
        """Get the remaining distance to goal along the current path."""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            return 0.0

        # Distance from current position to next waypoint
        if self.current_waypoint_index < len(self.current_path):
            next_wp = np.array(self.current_path[self.current_waypoint_index])
            curr_pos = np.array(current_position)
            dist_to_next = np.linalg.norm(next_wp - curr_pos)
        else:
            dist_to_next = 0.0

        # Distance from next waypoint to end of path
        path_remaining = 0.0
        for i in range(self.current_waypoint_index + 1, len(self.current_path)):
            p1 = np.array(self.current_path[i-1])
            p2 = np.array(self.current_path[i])
            path_remaining += np.linalg.norm(p2 - p1)

        return dist_to_next + path_remaining