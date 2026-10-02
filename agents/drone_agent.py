"""
Drone agent implementation for DSOS.
Now uses the Hardware Abstraction Layer (HAL) to work with multiple backends
including AirSim, PX4, Gazebo, etc.
"""
import asyncio
import math
import time
from typing import List, Dict, Optional, Any
import logging

from dsos.core.base_agent import BaseAgent, AgentState, AgentStatus
from dsos.config.settings import settings
from dsos.control.backends import BackendInterface, VehicleState


class DroneAgent(BaseAgent):
    """
    Concrete implementation of a drone agent that uses the Hardware Abstraction Layer.
    Works with any backend (AirSim, PX4, Gazebo, etc.) through the common interface.
    """

    def __init__(self, agent_id: Optional[str] = None, backend: Optional[BackendInterface] = None):
        super().__init__(agent_id)
        self.backend = backend
        self._backend_connected = False

        # State tracking for smooth operation
        self._last_known_state: Optional[VehicleState] = None
        self._state_update_time = 0.0

        # Control parameters
        self._target_position: List[float] = [0.0, 0.0, 0.0]
        self._target_velocity: List[float] = [0.0, 0.0, 0.0]
        self._max_velocity = settings.getfloat('DSOS', 'max_velocity_mps', 5.0)
        self._max_acceleration = settings.getfloat('DSOS', 'max_acceleration_mps2', 2.0)
        self._dt = settings.getint('DSOS', 'update_rate_hz', 50)
        self._dt = 1.0 / self._dt if self._dt > 0 else 0.02

        # Simple PID controllers for position and velocity control
        self._pos_kp = 1.0
        self._pos_ki = 0.1
        self._pos_kd = 0.05
        self._pos_integral = [0.0, 0.0, 0.0]
        self._pos_last_error = [0.0, 0.0, 0.0]

        self._vel_kp = 0.5
        self._vel_ki = 0.05
        self._vel_kd = 0.02
        self._vel_integral = [0.0, 0.0, 0.0]
        self._vel_last_error = [0.0, 0.0, 0.0]

        # Battery simulation (fallback if backend doesn't provide)
        self._battery_drain_rate = 0.01  # % per second in simulation
        self._motors_armed = False

        # Mission and navigation
        self._current_waypoint_index = 0
        self._waypoints: List[List[float]] = []
        self._home_position = [0.0, 0.0, 0.0]

        # Formation keeping
        self._formation_offset: List[float] = [0.0, 0.0, 0.0]
        self._formation_leader_id: Optional[str] = None

    async def initialize(self) -> None:
        """Initialize the drone agent and connect to backend if provided."""
        self.logger.info(f"Initializing drone {self.agent_id}")

        # Connect to backend if provided
        if self.backend and not self._backend_connected:
            # Check if the backend is already connected (main.py calls backend.connect() first)
            if getattr(self.backend, 'is_connected', False):
                self._backend_connected = True
                self.logger.info(f"Backend already connected: {type(self.backend).__name__}")
            else:
                try:
                    connected = await self.backend.connect()
                    if connected:
                        self._backend_connected = True
                        self.logger.info(f"Connected to backend: {type(self.backend).__name__}")
                    else:
                        self.logger.warning("Failed to connect to backend")
                except Exception as e:
                    self.logger.error(f"Error connecting to backend: {e}")

        # Initialize state
        if self._backend_connected and self.backend:
            # Get initial state from backend
            await self._update_state_from_backend()
        else:
            # Initialize simulated state (backward compatibility)
            self.logger.info("Running in simulation mode")
            import random
            self.status.position = [
                float(hash(self.agent_id) % 20 - 10),  # Deterministic but spread out
                float(hash(self.agent_id + "y") % 20 - 10),
                float(-5 - (hash(self.agent_id + "z") % 5))  # Negative Z for altitude
            ]
            self._home_position = self.status.position.copy()
            self._target_position = self.status.position.copy()

        self.status.state = AgentState.IDLE
        self.logger.info(f"Drone {self.agent_id} initialized")

    async def shutdown(self) -> None:
        """Shutdown the drone agent and disconnect from backend."""
        self.logger.info(f"Shutting down drone {self.agent_id}")

        # Disarm motors if armed
        if self._backend_connected and self.backend and self._motors_armed:
            try:
                await self.backend.disarm(self.agent_id)
            except Exception as e:
                self.logger.error(f"Error disarming via backend: {e}")

        await self.disarm_motors()

        # Disconnect from backend
        if self._backend_connected and self.backend:
            try:
                await self.backend.disconnect()
                self._backend_connected = False
                self.logger.info("Disconnected from backend")
            except Exception as e:
                self.logger.error(f"Error disconnecting from backend: {e}")

        self.logger.info(f"Drone {self.agent_id} shut down")

    async def update_state(self) -> None:
        """Update the drone's internal state from backend or simulation."""
        # Update timestamp
        self.status.timestamp = time.time()

        # Get state from backend if connected
        if self._backend_connected and self.backend:
            await self._update_state_from_backend()
        else:
            # Fall back to simulation (original behavior)
            await self._update_state_simulation()

        # Update health based on battery and other factors
        await self._update_health()

    async def _update_state_from_backend(self) -> None:
        """Update state from the backend hardware/simulator."""
        try:
            vehicle_state = await self.backend.get_vehicle_state(self.agent_id)
            if vehicle_state:
                self._last_known_state = vehicle_state
                self._state_update_time = time.time()

                # Update our status object from vehicle state
                self.status.position = vehicle_state.position
                self.status.velocity = vehicle_state.velocity
                self.status.orientation = vehicle_state.orientation

                # Dynamically set telemetry attributes expected by the UI
                setattr(self.status, 'is_connected', vehicle_state.is_connected)
                setattr(self.status, 'flight_mode', vehicle_state.flight_mode)
                setattr(self.status, 'health_status', vehicle_state.health_status)
                setattr(self.status, 'is_armed', vehicle_state.is_armed)
                if hasattr(vehicle_state, 'gps'):
                    setattr(self.status, 'gps', vehicle_state.gps)

                # Update battery if available from backend
                if vehicle_state.battery_remaining is not None:
                    self.status.battery_level = vehicle_state.battery_remaining

                # Update armed state
                self._motors_armed = vehicle_state.is_armed

                # Update state based on flight status
                if vehicle_state.land_state == "landed":
                    if self.status.state not in [AgentState.LANDING, AgentState.EMERGENCY]:
                        self.status.state = AgentState.IDLE
                elif vehicle_state.land_state == "taking_off":
                    self.status.state = AgentState.TAKEOFF
                elif vehicle_state.land_state == "landing":
                    self.status.state = AgentState.LANDING
                elif vehicle_state.position[2] < -0.1:  # In air (NED coordinates)
                    if self.status.state == AgentState.TAKEOFF:
                        # Check if we've reached target altitude
                        altitude_error = self._target_position[2] - self.status.position[2]
                        if abs(altitude_error) < 0.5:  # Within 0.5m of target
                            self.status.state = AgentState.HOVERING
                    elif self.status.state != AgentState.LANDING:
                        self.status.state = AgentState.HOVERING

        except Exception as e:
            self.logger.error(f"Error updating state from backend: {e}")
            # Fall back to simulation if backend fails
            await self._update_state_simulation()

    async def _update_state_simulation(self) -> None:
        """Update state using the original simulation logic (backward compatibility)."""
        # This is the original update_state logic from the simple drone agent
        # Update timestamp
        self.status.timestamp = time.time()

        # Simulate battery drain if motors are armed
        if self._motors_armed:
            self.status.battery_level = max(
                0,
                self.status.battery_level - (self._battery_drain_rate * self._dt)
            )
            # Check battery levels
            if self.status.battery_level <= self.critical_battery_threshold:
                self.status.state = AgentState.EMERGENCY
                self.logger.warning(f"Drone {self.agent_id} CRITICAL BATTERY: {self.status.battery_level:.1f}%")
            elif self.status.battery_level <= self.low_battery_threshold:
                if self.status.state not in [AgentState.RETURNING_HOME, AgentState.LANDING]:
                    self.status.state = AgentState.RETURNING_HOME
                    self.logger.warning(f"Drone {self.agent_id} LOW BATTERY: {self.status.battery_level:.1f}%")
                    await self.return_to_home()

        # Update state based on current conditions
        if self.status.state == AgentState.TAKEOFF:
            # Simple takeoff logic: ascend until reaching target altitude
            altitude_error = self._target_position[2] - self.status.position[2]  # Z is negative in NED for altitude
            if abs(altitude_error) < 0.5:  # Within 0.5m of target altitude
                self.status.state = AgentState.HOVERING
                self.logger.info(f"Drone {self.agent_id} reached takeoff altitude")
        elif self.status.state == AgentState.LANDING:
            # Simple landing logic: descend until reaching ground
            altitude_error = self._target_position[2] - self.status.position[2]
            if abs(altitude_error) < 0.1:  # Within 0.1m of ground
                self.status.state = AgentState.IDLE
                await self.disarm_motors()
                self.logger.info(f"Drone {self.agent_id} has landed")
        elif self.status.state == AgentState.NAVIGATING:
            # Check if we've reached the target position
            pos_error = [
                self._target_position[0] - self.status.position[0],
                self._target_position[1] - self.status.position[1],
                self._target_position[2] - self.status.position[2]
            ]
            distance = sum(e**2 for e in pos_error) ** 0.5
            if distance < 0.5:  # Within 0.5m of target
                if self._current_waypoint_index < len(self._waypoints) - 1:
                    # Move to next waypoint
                    self._current_waypoint_index += 1
                    await self.goto_waypoint(self._current_waypoint_index)
                else:
                    # Mission complete
                    self.status.state = AgentState.HOVERING
                    self.logger.info(f"Drone {self.agent_id} completed mission")

    async def _update_health(self) -> None:
        """Update health status based on battery and other factors."""
        # Base health on battery level primarily
        health_factor = min(1.0, self.status.battery_level / 100.0)
        self.status.health = health_factor * 0.9 + 0.1  # Base health of 0.1

        # Reduce health if we haven't gotten updates from backend recently
        if self._backend_connected and self.backend:
            time_since_update = time.time() - self._state_update_time
            if time_since_update > 2.0:  # No updates for 2 seconds
                self.status.health *= 0.8  # Reduce health due to stale data
                if time_since_update > 5.0:
                    self.status.health *= 0.5  # No updates for 5 seconds
                    self.status.health *= 0.5  # Significant health reduction

    async def communicate(self) -> None:
        """Handle communication with neighbors and the swarm."""
        # If we have a backend, we might use it for communication
        # For now, we'll rely on the mesh network layer for inter-drone communication
        # This method can be extended to use backend-specific communication if needed
        pass

    async def make_decision(self) -> None:
        """Make autonomous decisions based on current state and mission."""
        # When backend is connected, decisions are driven by the UI/SwarmManager.
        # Do NOT issue autonomous commands from the background loop.
        if self._backend_connected and self.backend:
            return
            
        # Handle emergency states (simulation only)
        if self.status.state == AgentState.EMERGENCY:
            await self.land()
            return

        if self.status.state == AgentState.RETURNING_HOME:
            await self.goto_position(self._home_position)
            return

    async def execute_commands(self) -> None:
        """Execute low-level commands (e.g., motor controls) based on current state."""
        # When backend is connected, arming and movement are driven by explicit
        # UI commands. Do NOT auto-arm from the background loop.
        if self._backend_connected and self.backend:
            return
            
        # Simulation mode: Arm motors if needed for takeoff/movement
        if not self._motors_armed and self.status.state not in [AgentState.INITIALIZING, AgentState.IDLE]:
            await self.arm_motors()

        # Execute movement commands based on state
        await self._execute_movement_commands()

    async def _execute_movement_commands(self) -> None:
        """Execute movement commands based on current target and state."""
        # When backend is connected, all commands are routed EXPLICITLY through
        # the SwarmManager -> DroneAgent method chain (takeoff(), move_velocity(), etc.).
        # This background loop must NOT call backend methods, because it OVERWRITES
        # the offboard targets set by those explicit commands.
        # For example: user sends move_velocity -> sets VelocityNedYaw target
        #              background loop runs -> calls goto_position(_target_position)
        #              -> OVERWRITES velocity target with position hold -> drone stays in HOLD
        if self._backend_connected and self.backend:
            return  # Do NOT interfere with explicit commands
            
        # Simulation mode only: run PID control loop
        if self.status.state in [AgentState.TAKEOFF, AgentState.LANDING, AgentState.NAVIGATING,
                                AgentState.RETURNING_HOME, AgentState.HOVERING]:
            await self._execute_via_simulation()

    async def _execute_via_backend(self) -> None:
        """DISABLED: Was conflicting with explicit command dispatch.
        All backend commands now go through the explicit method calls
        (takeoff(), land(), move_velocity(), goto_position(), etc.)
        routed by SwarmManager."""
        pass

    async def _execute_via_simulation(self) -> None:
        """Execute movement commands via simulation (original PID logic)."""
        # Calculate control commands based on current and target states
        # This is a simplified PID controller for position control
        pos_error = [
            self._target_position[0] - self.status.position[0],
            self._target_position[1] - self.status.position[1],
            self._target_position[2] - self.status.position[2]
        ]

        # Update integral and derivative terms
        for i in range(3):
            self._pos_integral[i] += pos_error[i] * self._dt
            # Anti-windup: clamp integral term
            self._pos_integral[i] = max(-1.0, min(1.0, self._pos_integral[i]))
            derivative = (pos_error[i] - self._pos_last_error[i]) / self._dt if self._dt > 0 else 0.0
            self._pos_last_error[i] = pos_error[i]

        # Calculate PID output (desired velocity)
        desired_velocity = [
            self._pos_kp * pos_error[0] + self._pos_ki * self._pos_integral[0] + self._pos_kd * derivative,
            self._pos_kp * pos_error[1] + self._pos_ki * self._pos_integral[1] + self._pos_kd * derivative,
            self._pos_kp * pos_error[2] + self._pos_ki * self._pos_integral[2] + self._pos_kd * derivative
        ]

        # Limit velocity to maximum
        speed = sum(v**2 for v in desired_velocity) ** 0.5
        if speed > self._max_velocity:
            scale = self._max_velocity / speed
            desired_velocity = [v * scale for v in desired_velocity]

        # Now convert desired velocity to motor commands (simulated)
        # In a real implementation with backend, this would go to the flight controller
        # For simulation, we'll just update velocity directly with some inertia
        velocity_error = [
            desired_velocity[0] - self.status.velocity[0],
            desired_velocity[1] - self.status.velocity[1],
            desired_velocity[2] - self.status.velocity[2]
        ]

        # Update integral and derivative terms for velocity control
        for i in range(3):
            self._vel_integral[i] += velocity_error[i] * self._dt
            # Anti-windup
            self._vel_integral[i] = max(-0.5, min(0.5, self._vel_integral[i]))
            derivative = (velocity_error[i] - self._vel_last_error[i]) / self._dt if self._dt > 0 else 0.0
            self._vel_last_error[i] = velocity_error[i]

        # Calculate acceleration command
        acceleration = [
            self._vel_kp * velocity_error[0] + self._vel_ki * self._vel_integral[0] + self._vel_kd * derivative,
            self._vel_kp * velocity_error[1] + self._vel_ki * self._vel_integral[1] + self._vel_kd * derivative,
            self._vel_kp * velocity_error[2] + self._vel_ki * self._vel_integral[2] + self._vel_kd * derivative
        ]

        # Limit acceleration
        acc_magnitude = sum(a**2 for a in acceleration) ** 0.5
        if acc_magnitude > self._max_acceleration:
            scale = self._max_acceleration / acc_magnitude
            acceleration = [a * scale for a in acceleration]

        # Update velocity and position (simple Euler integration)
        for i in range(3):
            self.status.velocity[i] += acceleration[i] * self._dt
            # Limit velocity
            speed = sum(v**2 for v in self.status.velocity) ** 0.5
            if speed > self._max_velocity:
                scale = self._max_velocity / speed
                self.status.velocity = [v * scale for v in self.status.velocity]
            self.status.position[i] += self.status.velocity[i] * self._dt

    async def arm_motors(self) -> str:
        """Arm the drone's motors via backend or simulation."""
        if self._motors_armed:
            return f"{self.agent_id} already armed"

        if self._backend_connected and self.backend:
            try:
                success = await self.backend.arm(self.agent_id)
                if success:
                    self._motors_armed = True
                    self.logger.info(f"Armed motors for drone {self.agent_id} via backend")
                    return f"{self.agent_id} Armed"
                return f"{self.agent_id} Failed: Unknown backend error"
            except Exception as e:
                self.logger.warning(f"Failed to arm via backend for {self.agent_id}: {e}")
                return f"{self.agent_id} Failed: {str(e)}"
        else:
            await self._arm_motors_simulation()
            return f"{self.agent_id} Armed (Sim)"

    async def _arm_motors_simulation(self) -> None:
        """Arm motors in simulation mode."""
        if not self._motors_armed:
            self.logger.info(f"Arming motors for drone {self.agent_id} (simulation)")
            self._motors_armed = True
            # In simulation, we just set a flag

    async def disarm_motors(self) -> str:
        """Disarm the drone's motors via backend or simulation."""
        if not self._motors_armed:
            return f"{self.agent_id} already disarmed"

        if self._backend_connected and self.backend:
            try:
                success = await self.backend.disarm(self.agent_id)
                if success:
                    self._motors_armed = False
                    self.logger.info(f"Disarmed motors for drone {self.agent_id} via backend")
                    return f"{self.agent_id} Disarmed"
                return f"{self.agent_id} Failed: Unknown backend error"
            except Exception as e:
                self.logger.warning(f"Failed to disarm via backend for {self.agent_id}: {e}")
                return f"{self.agent_id} Failed: {str(e)}"
        else:
            await self._disarm_motors_simulation()
            return f"{self.agent_id} Disarmed (Sim)"

    async def _disarm_motors_simulation(self) -> None:
        """Disarm motors in simulation mode."""
        if self._motors_armed:
            self.logger.info(f"Disarming motors for drone {self.agent_id} (simulation)")
            self._motors_armed = False
            # In reality, we would send a command to disarm the motors

    async def takeoff(self, target_altitude: float = None) -> str:
        """Command the drone to takeoff to a target altitude."""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.logger.info(f"[{ts}] [magenta][DRONEAGENT RECEIVED][/magenta] takeoff for {self.agent_id}")
        
        if target_altitude is None:
            target_altitude = settings.getfloat('DSOS', 'default_mission_altitude_m', 10.0)
        self.status.state = AgentState.TAKEOFF

        # Set target position (negative Z for altitude in our coordinate system)
        current_pos = self.status.position
        self._target_position = [current_pos[0], current_pos[1], -target_altitude]  # NED coordinates, negative is up

        # Execute via appropriate method
        if self._backend_connected and self.backend:
            try:
                await self.backend.takeoff(self.agent_id, target_altitude)
                return f"[{ts}] [green][COMPLETED][/green] {self.agent_id} Takeoff"
            except Exception as e:
                return f"[{ts}] [red][FAILED][/red] {self.agent_id}: {str(e)}"
        else:
            return f"[{ts}] [red][FAILED][/red] {self.agent_id} No Backend"

    async def land(self) -> str:
        """Command the drone to land."""
        self.logger.info(f"Drone {self.agent_id} landing")
        self.status.state = AgentState.LANDING

        if self._backend_connected and self.backend:
            try:
                await self.backend.land(self.agent_id)
                return f"{self.agent_id} Landed"
            except Exception as e:
                return f"{self.agent_id} Failed: {str(e)}"
        else:
            # Set target position to current x,y but ground level (0 altitude in NED)
            current_pos = self.status.position
            self._target_position = [current_pos[0], current_pos[1], 0.0]
            return f"{self.agent_id} Landing (Sim)"

    async def goto_position(self, position: List[float]) -> str:
        """Command the drone to fly to a specific position."""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.logger.info(f"[{ts}] [magenta][DRONEAGENT RECEIVED][/magenta] goto_position for {self.agent_id}")
        self.status.state = AgentState.NAVIGATING
        self._target_position = position

        if self._backend_connected and self.backend:
            try:
                await self.backend.goto_position(self.agent_id, position)
                return f"[{ts}] [green][MOVEMENT EXECUTING][/green] {self.agent_id}"
            except Exception as e:
                return f"[{ts}] [red][FAILED][/red] {self.agent_id}: {str(e)}"
        return f"[{ts}] [red][FAILED][/red] {self.agent_id} No Backend"

    async def goto_waypoint(self, waypoint_index: int) -> None:
        """Command the drone to fly to a specific waypoint in the mission."""
        if 0 <= waypoint_index < len(self._waypoints):
            self._current_waypoint_index = waypoint_index
            await self.goto_position(self._waypoints[waypoint_index])
        else:
            self.logger.warning(f"Invalid waypoint index: {waypoint_index}")

    async def move_velocity(self, vx: float, vy: float, vz: float, duration: float = 0.5) -> str:
        """Send a continuous velocity setpoint (body frame)."""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.logger.info(f"[{ts}] [magenta][DRONEAGENT RECEIVED][/magenta] move_velocity for {self.agent_id}")
        
        yaw = self.status.orientation[2] if self.status.orientation else 0.0
        
        # Body to NED rotation
        ned_vx = vx * math.cos(yaw) - vy * math.sin(yaw)
        ned_vy = vx * math.sin(yaw) + vy * math.cos(yaw)
        
        if self._backend_connected and self.backend and hasattr(self.backend, 'set_velocity'):
            try:
                # Backend now expects pure NED [North, East, Down]
                ned_v = [ned_vx, ned_vy, vz]
                await self.backend.set_velocity(self.agent_id, ned_v, duration)
                return f"[{ts}] [green][MOVEMENT EXECUTING][/green] {self.agent_id}"
            except Exception as e:
                return f"[{ts}] [red][FAILED][/red] {self.agent_id} Backend Error: {str(e)}"
                
        return f"[{ts}] [red][FAILED][/red] {self.agent_id} No Backend Connected"

    async def move_relative(self, dx: float, dy: float, dz: float) -> str:
        """Move relative to the current position (body frame)."""
        current = self.status.position
        yaw = self.status.orientation[2] if self.status.orientation else 0.0
        
        # Body to NED rotation
        ned_x = dx * math.cos(yaw) - dy * math.sin(yaw)
        ned_y = dx * math.sin(yaw) + dy * math.cos(yaw)
        
        target = [current[0] + ned_x, current[1] + ned_y, current[2] + dz]
        return await self.goto_position(target)

    async def move_forward(self, distance: float) -> str:
        return await self.move_relative(distance, 0, 0)

    async def move_backward(self, distance: float) -> str:
        return await self.move_relative(-distance, 0, 0)

    async def move_left(self, distance: float) -> str:
        return await self.move_relative(0, -distance, 0)

    async def move_right(self, distance: float) -> str:
        return await self.move_relative(0, distance, 0)

    async def move_up(self, distance: float) -> str:
        return await self.move_relative(0, 0, -distance)  # Z is negative up in NED

    async def move_down(self, distance: float) -> str:
        return await self.move_relative(0, 0, distance)

    async def rotate(self, degrees: float) -> str:
        yaw = (self.status.orientation[2] if self.status.orientation else 0.0) + math.radians(degrees)
        if self._backend_connected and self.backend and hasattr(self.backend, 'set_yaw'):
            try:
                # Convert radians back to degrees for MAVSDK which expects degrees
                await self.backend.set_yaw(self.agent_id, math.degrees(yaw), False)
                return f"{self.agent_id} Rotated"
            except Exception as e:
                return f"{self.agent_id} Failed: {str(e)}"
        self.status.orientation[2] = yaw
        return f"{self.agent_id} Rotated (Sim)"
        
    async def rotate_left(self, degrees: float = 45.0) -> str:
        return await self.rotate(-degrees)
        
    async def rotate_right(self, degrees: float = 45.0) -> str:
        return await self.rotate(degrees)

    async def hover(self) -> str:
        """Command the drone to hover."""
        self.status.state = AgentState.HOVERING
        if self._backend_connected and self.backend:
            state = await self.backend.get_vehicle_state(self.agent_id)
            if state:
                try:
                    await self.backend.goto_position(self.agent_id, state.position)
                    return f"{self.agent_id} Hovering"
                except Exception as e:
                    return f"{self.agent_id} Failed: {str(e)}"
        return f"{self.agent_id} Hovering (Sim)"

    async def load_mission(self, waypoints: List[List[float]]) -> None:
        """Load a mission (list of waypoints) for the drone."""
        self._waypoints = waypoints.copy()
        self._current_waypoint_index = 0
        self.logger.info(f"Drone {self.agent_id} loaded mission with {len(waypoints)} waypoints")
        if self._waypoints:
            await self.goto_waypoint(0)

    async def return_to_home(self) -> str:
        """Command the drone to return to its home position."""
        self.logger.info(f"Drone {self.agent_id} returning home")
        self.status.state = AgentState.RETURNING_HOME
        
        if self._backend_connected and self.backend and hasattr(self.backend, 'return_to_home'):
            try:
                await self.backend.return_to_home(self.agent_id)
                return f"{self.agent_id} Returning Home"
            except Exception as e:
                return f"{self.agent_id} Failed: {str(e)}"
                
        return await self.goto_position(self._home_position)

    def set_formation_offset(self, offset: List[float], leader_id: str) -> None:
        """Set the formation offset relative to a leader."""
        self._formation_offset = offset
        self._formation_leader_id = leader_id

    def get_battery_level(self) -> float:
        """Get the current battery level percentage."""
        return self.status.battery_level

    def is_armed(self) -> bool:
        """Check if the motors are armed."""
        return self._motors_armed

    def is_connected_to_backend(self) -> bool:
        """Check if the drone is connected to a backend."""
        return self._backend_connected and self.backend is not None