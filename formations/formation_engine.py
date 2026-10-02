"""
Formation engine for generating and maintaining drone formations.
Supports various formation types and dynamic transformations.
"""
import math
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
import logging

import time
import numpy as np

from dsos.config.settings import settings


class FormationType(Enum):
    """Types of formations that can be generated."""
    NONE = "none"
    LINE = "line"
    HORIZONTAL_LINE = "horizontal_line"
    VERTICAL_LINE = "vertical_line"
    DIAGONAL_LINE = "diagonal_line"
    REVERSE_DIAGONAL_LINE = "reverse_diagonal_line"
    EQUILATERAL_TRIANGLE = "equilateral_triangle"
    INVERTED_TRIANGLE = "inverted_triangle"
    DIAMOND = "diamond"
    COMPACT_DIAMOND = "compact_diamond"
    EXPANDED_DIAMOND = "expanded_diamond"
    ARROW = "arrow"
    REVERSE_ARROW = "reverse_arrow"
    V_FORMATION = "v_formation"
    REVERSE_V = "reverse_v"
    COLUMN = "column"
    STAGGERED_COLUMN = "staggered_column"
    ECHELON_LEFT = "echelon_left"
    ECHELON_RIGHT = "echelon_right"
    CIRCLE = "circle"
    ORBIT = "orbit"
    ARC = "arc"
    SEMI_CIRCLE = "semi_circle"
    SPIRAL = "spiral"
    PATROL = "patrol"
    RECONNAISSANCE = "reconnaissance"
    ESCORT = "escort"
    ADAPTIVE = "adaptive"
    RANDOM = "random"
    SEARCH = "search"
    EXPLORATION = "exploration"
    MAPPING = "mapping"
    COVERAGE = "coverage"
    GRID = "grid"
    CUSTOM = "custom"


class FormationError(Exception):
    """Exception raised for formation-related errors."""
    pass


class Formation:
    """Represents a formation with a set of relative positions."""

    def __init__(self, formation_type: FormationType, center: List[float] = [0, 0, 0],
                 scale: float = 1.0, rotation: float = 0.0):
        """
        Initialize a formation.

        Args:
            formation_type: Type of formation
            center: Center point [x, y, z] in meters
            scale: Scale factor (1.0 = default size)
            rotation: Rotation angle in radians (counter-clockwise around Z-axis)
        """
        self.formation_type = formation_type
        self.center = np.array(center, dtype=float)
        self.scale = scale
        self.rotation = rotation
        self.positions: List[np.ndarray] = []  # Relative positions
        self._generate_positions()

    def _generate_positions(self) -> None:
        """Generate the relative positions for this formation."""
        self.positions = []

        if self.formation_type == FormationType.NONE:
            self.positions = [np.array([0.0, 0.0, 0.0])]

        elif self.formation_type == FormationType.LINE:
            # Line along X-axis
            count = max(2, int(self.scale * 5))  # Default 5 points scaled
            for i in range(count):
                x = (i - (count - 1) / 2) * 2.0  # 2m spacing
                self.positions.append(np.array([x, 0.0, 0.0]))

        elif self.formation_type == FormationType.HORIZONTAL_LINE:
            # Line along Y-axis
            count = max(2, int(self.scale * 5))
            for i in range(count):
                y = (i - (count - 1) / 2) * 2.0
                self.positions.append(np.array([0.0, y, 0.0]))

        elif self.formation_type == FormationType.VERTICAL_LINE:
            # Line along Z-axis
            count = max(2, int(self.scale * 5))
            for i in range(count):
                z = (i - (count - 1) / 2) * 2.0
                self.positions.append(np.array([0.0, 0.0, z]))

        elif self.formation_type == FormationType.DIAGONAL_LINE:
            # Line at 45 degrees in XY plane
            count = max(2, int(self.scale * 5))
            for i in range(count):
                offset = (i - (count - 1) / 2) * 2.0
                self.positions.append(np.array([offset, offset, 0.0]))

        elif self.formation_type == FormationType.REVERSE_DIAGONAL_LINE:
            # Line at -45 degrees in XY plane
            count = max(2, int(self.scale * 5))
            for i in range(count):
                offset = (i - (count - 1) / 2) * 2.0
                self.positions.append(np.array([offset, -offset, 0.0]))

        elif self.formation_type == FormationType.EQUILATERAL_TRIANGLE:
            # Equilateral triangle pointing up
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 3)
            if points < 3:
                points = 3
            # Points at corners of equilateral triangle
            radius = 2.0 * self.scale
            angles = [0, 2 * math.pi / 3, 4 * math.pi / 3]
            for angle in angles:
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.INVERTED_TRIANGLE:
            # Equilateral triangle pointing down
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 3)
            if points < 3:
                points = 3
            radius = 2.0 * self.scale
            angles = [math.pi, 5 * math.pi / 3, 7 * math.pi / 3]  # Shifted by pi
            for angle in angles:
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.DIAMOND:
            # Diamond shape
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 4:
                points = 4
            # Points: up, right, down, left
            radius = 2.0 * self.scale
            self.positions = [
                np.array([0.0, radius, 0.0]),   # up
                np.array([radius, 0.0, 0.0]),   # right
                np.array([0.0, -radius, 0.0]),  # down
                np.array([-radius, 0.0, 0.0])   # left
            ]

        elif self.formation_type == FormationType.COMPACT_DIAMOND:
            # Smaller diamond
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 4:
                points = 4
            radius = 1.0 * self.scale
            self.positions = [
                np.array([0.0, radius, 0.0]),
                np.array([radius, 0.0, 0.0]),
                np.array([0.0, -radius, 0.0]),
                np.array([-radius, 0.0, 0.0])
            ]

        elif self.formation_type == FormationType.EXPANDED_DIAMOND:
            # Larger diamond
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 4:
                points = 4
            radius = 3.0 * self.scale
            self.positions = [
                np.array([0.0, radius, 0.0]),
                np.array([radius, 0.0, 0.0]),
                np.array([0.0, -radius, 0.0]),
                np.array([-radius, 0.0, 0.0])
            ]

        elif self.formation_type == FormationType.ARROW:
            # Arrow shape pointing in +X direction
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 5:
                points = 5
            # Tip, two wings, two tail points
            length = 3.0 * self.scale
            width = 2.0 * self.scale
            self.positions = [
                np.array([length, 0.0, 0.0]),           # tip
                np.array([length * 0.5, width * 0.5, 0.0]),  # upper wing
                np.array([length * 0.5, -width * 0.5, 0.0]), # lower wing
                np.array([-length * 0.5, width * 0.25, 0.0]), # rear left
                np.array([-length * 0.5, -width * 0.25, 0.0]) # rear right
            ]

        elif self.formation_type == FormationType.REVERSE_ARROW:
            # Arrow shape pointing in -X direction
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 5:
                points = 5
            length = 3.0 * self.scale
            width = 2.0 * self.scale
            self.positions = [
                np.array([-length, 0.0, 0.0]),          # tip
                np.array([-length * 0.5, width * 0.5, 0.0]), # upper wing
                np.array([-length * 0.5, -width * 0.5, 0.0]), # lower wing
                np.array([length * 0.5, width * 0.25, 0.0]),  # rear left
                np.array([length * 0.5, -width * 0.25, 0.0])  # rear right
            ]

        elif self.formation_type == FormationType.V_FORMATION:
            # V formation pointing in +X direction
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 3:
                points = 3
            angle = math.radians(30)  # 30 degrees from centerline
            length = 3.0 * self.scale
            self.positions = [np.array([0.0, 0.0, 0.0])]  # leader at origin
            for i in range(1, points):
                side = 1 if i % 2 == 1 else -1
                ring = (i + 1) // 2
                x = length * (ring / points)
                y = side * math.tan(angle) * x
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.REVERSE_V:
            # V formation pointing in -X direction
            if self.scale < 1:
                self.scale = 1
            points = int(self.scale * 5)
            if points < 3:
                points = 3
            angle = math.radians(30)
            length = 3.0 * self.scale
            self.positions = [np.array([0.0, 0.0, 0.0])]  # leader at origin
            for i in range(1, points):
                side = 1 if i % 2 == 1 else -1
                ring = (i + 1) // 2
                x = -length * (ring / points)
                y = side * math.tan(angle) * abs(x)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.COLUMN:
            # Vertical column (along Z-axis)
            count = max(2, int(self.scale * 5))
            for i in range(count):
                z = (i - (count - 1) / 2) * 2.0
                self.positions.append(np.array([0.0, 0.0, z]))

        elif self.formation_type == FormationType.STAGGERED_COLUMN:
            # Staggered column
            count = max(2, int(self.scale * 5))
            for i in range(count):
                z = (i - (count - 1) / 2) * 2.0
                x = (i % 2) * 1.0  # Alternate left/right
                y = 0.0
                self.positions.append(np.array([x, 0.0, z]))

        elif self.formation_type == FormationType.ECHELON_LEFT:
            # Echelon formation slanted left and back
            count = max(2, int(self.scale * 5))
            for i in range(count):
                x = -i * 2.0 * self.scale  # backward
                y = i * 2.0 * self.scale   # left
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.ECHELON_RIGHT:
            # Echelon formation slanted right and back
            count = max(2, int(self.scale * 5))
            for i in range(count):
                x = -i * 2.0 * self.scale  # backward
                y = -i * 2.0 * self.scale  # right
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.CIRCLE:
            # Circle formation
            count = max(3, int(self.scale * 8))
            radius = 2.0 * self.scale
            for i in range(count):
                angle = 2 * math.pi * i / count
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.ORBIT:
            # Same as circle for now
            count = max(3, int(self.scale * 8))
            radius = 2.0 * self.scale
            for i in range(count):
                angle = 2 * math.pi * i / count
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.ARC:
            # Arc (half circle)
            arc_angle = math.pi * 0.5  # 90 degrees
            for i in range(count):
                angle = -arc_angle / 2 + (arc_angle * i / (count - 1)) if count > 1 else 0
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.SPIRAL:
            # Archimedean spiral
            count = max(3, int(self.scale * 10))
            max_radius = 3.0 * self.scale
            for i in range(count):
                # Parameter t from 0 to 1
                t = i / (count - 1) if count > 1 else 0
                radius = max_radius * t
                angle = 2 * math.pi * 3 * t  # 3 turns
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.GRID:
            # Rectangular grid
            cols = max(2, int(math.sqrt(self.scale * 9)))
            rows = max(2, int(math.ceil((self.scale * 9) / cols)))
            spacing = 2.0
            start_x = -(cols - 1) * spacing / 2
            start_y = -(rows - 1) * spacing / 2
            for row in range(rows):
                for col in range(cols):
                    x = start_x + col * spacing
                    y = start_y + row * spacing
                    self.positions.append(np.array([x, y, 0.0]))

        elif self.formation_type == FormationType.ADAPTIVE:
            # Adaptive formation: density depends on scale, spacing depends on context
            count = max(3, int(self.scale * 6))
            # Example heuristic: a staggered formation that expands radially
            radius_step = 1.5
            angle_step = math.pi / 2.5
            self.positions = [np.array([0.0, 0.0, 0.0])]
            for i in range(1, count):
                r = radius_step * math.sqrt(i)
                theta = i * angle_step
                self.positions.append(np.array([r * math.cos(theta), r * math.sin(theta), 0.0]))

        else:
            # For other formations, default to a simple line
            count = max(2, int(self.scale * 3))
            for i in range(count):
                x = (i - (count - 1) / 2) * 2.0
                self.positions.append(np.array([x, 0.0, 0.0]))

        # Apply scaling (already applied in calculations above, but keep for consistency)
        # Apply rotation around Z-axis
        if self.rotation != 0:
            cos_r = math.cos(self.rotation)
            sin_r = math.sin(self.rotation)
            for i in range(len(self.positions)):
                x = self.positions[i][0]
                y = self.positions[i][1]
                self.positions[i][0] = x * cos_r - y * sin_r
                self.positions[i][1] = x * sin_r + y * cos_r

    def get_absolute_positions(self) -> List[List[float]]:
        """
        Get the absolute positions for each agent in the formation.

        Returns:
            List of [x, y, z] positions in global coordinates
        """
        absolute_positions = []
        for pos in self.positions:
            absolute_pos = self.center + pos * self.scale
            absolute_positions.append(absolute_pos.tolist())
        return absolute_positions

    def get_relative_positions(self) -> List[List[float]]:
        """
        Get the relative positions (centered at origin).

        Returns:
            List of [x, y, z] positions relative to formation center
        """
        return [pos.tolist() for pos in self.positions]

    def set_center(self, center: List[float]) -> None:
        """Set the center of the formation."""
        self.center = np.array(center, dtype=float)

    def set_scale(self, scale: float) -> None:
        """Set the scale of the formation and regenerate positions."""
        self.scale = scale
        self._generate_positions()

    def set_rotation(self, rotation: float) -> None:
        """Set the rotation of the formation in radians and regenerate positions."""
        self.rotation = rotation
        self._generate_positions()

    def get_formation_type(self) -> FormationType:
        """Get the formation type."""
        return self.formation_type

    def get_size(self) -> int:
        """Get the number of positions in the formation."""
        return len(self.positions)


class FormationManager:
    """Manages formation assignments for a swarm of drones."""

    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.logger = logging.getLogger(f"formation.{agent_id}")
        self.current_formation: Optional[Formation] = None
        self.previous_formation: Optional[Formation] = None
        self.transition_start_time = 0.0
        self.transition_duration = 0.0
        self.formation_assignments: Dict[str, int] = {}  # agent_id -> position_index
        self.neighbor_positions: Dict[str, List[float]] = {}  # agent_id -> [x, y, z]

    def set_formation(self, formation_type: FormationType, center: List[float] = [0, 0, 0],
                      scale: float = 1.0, rotation: float = 0.0, transition_duration: float = 0.0) -> None:
        """Set the formation for the swarm with optional smooth transition."""
        new_formation = Formation(formation_type, center, scale, rotation)
        
        # Simple transition logic: store previous formation to interpolate if needed
        self.previous_formation = self.current_formation
        self.current_formation = new_formation
        self.transition_start_time = time.time() if transition_duration > 0 else 0
        self.transition_duration = transition_duration
        
        self.logger.info(
            f"Set formation to {formation_type.value} with {len(self.current_formation.positions)} positions"
        )

    def update_agent_position(self, agent_id: str, position: List[float]) -> None:
        """Update the known position of an agent in the swarm."""
        self.neighbor_positions[agent_id] = position

    def get_formation_positions(self) -> List[List[float]]:
        """Get the absolute positions for the current formation."""
        if not self.current_formation:
            return []
        return self.current_formation.get_absolute_positions()

    def assign_positions(self, agent_ids: List[str]) -> Dict[str, List[float]]:
        """
        Assign formation positions to agents.

        Args:
            agent_ids: List of agent IDs in the swarm (including self)

        Returns:
            Dictionary mapping agent_id to assigned position [x, y, z]
        """
        if not self.current_formation:
            # No formation, hold current positions
            return {agent_id: self.neighbor_positions.get(agent_id, [0, 0, 0])
                    for agent_id in agent_ids}

        positions = self.get_formation_positions()
        assignments = {}

        # Sort agent IDs for consistent assignment
        sorted_agents = sorted(agent_ids)

        # Assign positions in order
        for i, agent_id in enumerate(sorted_agents):
            if i < len(positions):
                assignments[agent_id] = positions[i]
                self.formation_assignments[agent_id] = i
            else:
                # More agents than positions, cycle through positions
                pos_index = i % len(positions)
                assignments[agent_id] = positions[pos_index]
                self.formation_assignments[agent_id] = pos_index

        return assignments

    def get_assigned_position(self, agent_id: str) -> Optional[List[float]]:
        """Get the assigned position for a specific agent."""
        if agent_id not in self.formation_assignments:
            return None
        if not self.current_formation:
            return None
        pos_index = self.formation_assignments[agent_id]
        if pos_index >= len(self.current_formation.positions):
            return None
            
        current_pos = self.current_formation.positions[pos_index]
        absolute_target_pos = self._apply_transform(current_pos, self.current_formation)
        
        # Interpolate for smooth transitions
        if self.transition_duration > 0 and self.previous_formation is not None:
            elapsed = time.time() - self.transition_start_time
            if elapsed < self.transition_duration:
                progress = elapsed / self.transition_duration
                # Smoothstep interpolation
                progress = progress * progress * (3 - 2 * progress)
                
                # Get previous position
                prev_pos_index = min(pos_index, len(self.previous_formation.positions) - 1)
                prev_pos = self.previous_formation.positions[prev_pos_index]
                absolute_prev_pos = self._apply_transform(prev_pos, self.previous_formation)
                
                # Interpolate
                x = absolute_prev_pos[0] + (absolute_target_pos[0] - absolute_prev_pos[0]) * progress
                y = absolute_prev_pos[1] + (absolute_target_pos[1] - absolute_prev_pos[1]) * progress
                z = absolute_prev_pos[2] + (absolute_target_pos[2] - absolute_prev_pos[2]) * progress
                return [x, y, z]
            else:
                self.previous_formation = None
                self.transition_duration = 0.0

        return absolute_target_pos.tolist()

    def get_neighbor_assigned_position(self, neighbor_id: str) -> Optional[List[float]]:
        """Get the assigned position for a neighbor agent."""
        return self.get_assigned_position(neighbor_id)

    def get_formation_center(self) -> List[float]:
        """Get the center of the current formation."""
        if not self.current_formation:
            return [0, 0, 0]
        return self.current_formation.center.tolist()

    def get_formation_size(self) -> int:
        """Get the number of positions in the current formation."""
        if not self.current_formation:
            return 0
        return self.current_formation.get_size()

    def is_formation_valid(self) -> bool:
        """Check if the current formation is valid."""
        return self.current_formation is not None

    def _apply_transform(self, position: List[float], formation: Optional[Formation] = None) -> List[float]:
        """Apply scale and rotation to a position relative to center."""
        if formation is None:
            formation = self.current_formation
            
        # Apply rotation around Z-axis
        cos_r = math.cos(formation.rotation)
        sin_r = math.sin(formation.rotation)
        x = position[0] * cos_r - position[1] * sin_r
        y = position[0] * sin_r + position[1] * cos_r
        z = position[2]

        # Apply scale
        x *= formation.scale
        y *= formation.scale
        z *= formation.scale

        # Add center offset
        x += formation.center[0]
        y += formation.center[1]
        z += formation.center[2]

        return [x, y, z]


# Predefined formation patterns for easy access
FORMATION_PATTERNS = {
    'line': FormationType.LINE,
    'horizontal_line': FormationType.HORIZONTAL_LINE,
    'vertical_line': FormationType.VERTICAL_LINE,
    'diagonal_line': FormationType.DIAGONAL_LINE,
    'reverse_diagonal_line': FormationType.REVERSE_DIAGONAL_LINE,
    'equilateral_triangle': FormationType.EQUILATERAL_TRIANGLE,
    'inverted_triangle': FormationType.INVERTED_TRIANGLE,
    'diamond': FormationType.DIAMOND,
    'compact_diamond': FormationType.COMPACT_DIAMOND,
    'expanded_diamond': FormationType.EXPANDED_DIAMOND,
    'arrow': FormationType.ARROW,
    'reverse_arrow': FormationType.REVERSE_ARROW,
    'v_formation': FormationType.V_FORMATION,
    'reverse_v': FormationType.REVERSE_V,
    'column': FormationType.COLUMN,
    'staggered_column': FormationType.STAGGERED_COLUMN,
    'echelon_left': FormationType.ECHELON_LEFT,
    'echelon_right': FormationType.ECHELON_RIGHT,
    'circle': FormationType.CIRCLE,
    'orbit': FormationType.ORBIT,
    'arc': FormationType.ARC,
    'semi_circle': FormationType.SEMI_CIRCLE,
    'spiral': FormationType.SPIRAL,
    'patrol': FormationType.PATROL,
    'reconnaissance': FormationType.RECONNAISSANCE,
    'escort': FormationType.ESCORT,
    'adaptive': FormationType.ADAPTIVE,
    'random': FormationType.RANDOM,
    'search': FormationType.SEARCH,
    'exploration': FormationType.EXPLORATION,
    'mapping': FormationType.MAPPING,
    'coverage': FormationType.COVERAGE,
    'grid': FormationType.GRID,
    'none': FormationType.NONE
}