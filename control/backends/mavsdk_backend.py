"""
MAVSDK Backend for DSOS.
Implements the Hardware Abstraction Layer for PX4-based drones using MAVSDK.
Supports both SITL and real hardware depending on the connection string.
"""
import asyncio
import logging
from typing import Dict, List, Optional, Any

try:
    from mavsdk.mavlink_direct import MavlinkDirect, MavlinkMessage
except ImportError:  # pragma: no cover - optional dependency path
    MavlinkDirect = None
    MavlinkMessage = None

from dsos.control.backends.base_backend import BackendInterface, BackendCapabilities, VehicleState

try:
    from mavsdk import System
    from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
    from mavsdk.mission import MissionItem, MissionPlan
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False
    System = Any


class MAVSDKBackend(BackendInterface):
    """Backend implementation using MAVSDK for PX4/MAVLink based systems."""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.system_address = config.get("system_address", "udp://:14540")
        self.vehicle_id = config.get("vehicle_id", "drone_1")
        self.grpc_port = config.get("grpc_port", 50051)
        self.systems: Dict[str, System] = {}
        self.state_tasks: Dict[str, asyncio.Task] = {}
        self.logger = logging.getLogger("MAVSDKBackend")

        # Define capabilities
        self.capabilities.can_takeoff = True
        self.capabilities.can_land = True
        self.capabilities.can_hover = True
        self.capabilities.can_move = True
        self.capabilities.can_set_speed = True
        self.capabilities.can_set_yaw = True
        self.capabilities.has_gps = True
        self.capabilities.has_imu = True
        self.capabilities.can_get_battery_info = True
        self.capabilities.can_get_health_status = True

    async def connect(self) -> bool:
        """Connect to the MAVSDK systems (e.g., PX4 SITL or real drone)."""
        if not MAVSDK_AVAILABLE:
            self.logger.error("MAVSDK is not installed. Please run `pip install mavsdk`")
            return False

        try:
            vehicle_id = self.vehicle_id 
            preconnected = self.config.get("preconnected_system")
            
            if preconnected is not None:
                self.logger.info(f"Using pre-connected MAVSDK System for {vehicle_id}")
                drone = preconnected
            else:
                self.logger.info(f"Connecting to MAVSDK at {self.system_address} via gRPC port {self.grpc_port}")
                drone = System(port=self.grpc_port)
                await drone.connect(system_address=self.system_address)
                
                # Wait for connection
                self.logger.info("Waiting for drone to connect...")
                async for state in drone.core.connection_state():
                    if state.is_connected:
                        self.logger.info("Drone discovered and connected!")
                        break

            self.systems[vehicle_id] = drone
            
            # Initialize state tracking task
            self.state_tasks[vehicle_id] = asyncio.create_task(self._track_telemetry(vehicle_id))

            if not await self._wait_for_vehicle_ready(vehicle_id):
                self.logger.warning(f"Vehicle {vehicle_id} connected but telemetry not fully active yet; continuing anyway (will populate async)")
            
            self.is_connected = True
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect via MAVSDK: {e}")
            return False

    async def _wait_for_vehicle_ready(self, vehicle_id: str, timeout_sec: float = 10.0) -> bool:
        """Wait until the backend has a heartbeat, telemetry, and non-empty health state."""
        deadline = asyncio.get_running_loop().time() + timeout_sec
        while asyncio.get_running_loop().time() < deadline:
            state = self.vehicles.get(vehicle_id)
            if state and state.is_connected and state.flight_mode != "UNKNOWN" and state.battery_remaining is not None:
                self.logger.info(f"Vehicle {vehicle_id} is ready: heartbeat + telemetry + health available")
                return True
            await asyncio.sleep(0.2)
        return False

    async def disconnect(self) -> None:
        """Disconnect and cleanup."""
        self.logger.info("Disconnecting MAVSDK backend...")
        for task in self.state_tasks.values():
            task.cancel()
        self.is_connected = False
        self.systems.clear()

    async def _track_telemetry(self, vehicle_id: str):
        """Continuously track telemetry and update VehicleState."""
        system = self.systems[vehicle_id]
        
        # Initialize vehicle state
        self.vehicles[vehicle_id] = VehicleState(
            vehicle_id=vehicle_id,
            timestamp=0.0,
            position=[0.0, 0.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            acceleration=[0.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 0.0],
            is_connected=True
        )
        state = self.vehicles[vehicle_id]

        async def track_position():
            async for position in system.telemetry.position():
                # Down is negative relative altitude
                state.position[2] = -position.relative_altitude_m
                state.gps = [position.latitude_deg, position.longitude_deg, position.absolute_altitude_m]

        async def track_battery():
            async for battery in system.telemetry.battery():
                state.battery_remaining = battery.remaining_percent * 100
                state.battery_voltage = battery.voltage_v

        async def track_flight_mode():
            async for flight_mode in system.telemetry.flight_mode():
                state.flight_mode = str(flight_mode)

        async def track_armed_state():
            async for is_armed in system.telemetry.armed():
                state.is_armed = is_armed

        async def track_in_air():
            async for in_air in system.telemetry.in_air():
                state.is_in_air = in_air

        async def track_health():
            async for health in system.telemetry.health():
                if health.is_global_position_ok and health.is_local_position_ok:
                    state.health_status = "OK"
                else:
                    state.health_status = "WAIT"

        async def track_local_position():
            async for pos_vel in system.telemetry.position_velocity_ned():
                # Store natively in pure NED format
                state.position[0] = pos_vel.position.north_m
                state.position[1] = pos_vel.position.east_m
                
                state.velocity[0] = pos_vel.velocity.north_m_s
                state.velocity[1] = pos_vel.velocity.east_m_s
                state.velocity[2] = pos_vel.velocity.down_m_s

        import math
        async def track_attitude():
            async for attitude in system.telemetry.attitude_euler():
                # Store Euler angles, converting yaw to radians for geometric calculations
                state.orientation = [attitude.roll_deg, attitude.pitch_deg, math.radians(attitude.yaw_deg)]

        async def track_connection():
            async for conn in system.core.connection_state():
                state.is_connected = conn.is_connected

        try:
            # Run telemetry tasks concurrently
            await asyncio.gather(
                track_position(),
                track_local_position(),
                track_attitude(),
                track_battery(),
                track_armed_state(),
                track_flight_mode(),
                track_in_air(),
                track_health(),
                track_connection()
            )
        except asyncio.CancelledError:
            self.logger.info(f"Telemetry tracking cancelled for {vehicle_id}")
        except Exception as e:
            self.logger.error(f"Telemetry tracking error for {vehicle_id}: {e}")

    async def get_vehicles(self) -> List[str]:
        return list(self.systems.keys())

    async def get_vehicle_state(self, vehicle_id: str) -> Optional[VehicleState]:
        return self.vehicles.get(vehicle_id)

    async def _set_vehicle_mode(self, vehicle_id: str) -> bool:
        """Best-effort pre-arm step to place the vehicle in a commandable mode."""
        system = self.systems.get(vehicle_id)
        if not system:
            return False

        try:
            await self._send_guided_mode(vehicle_id)
            return True
        except Exception as exc:
            self.logger.debug("Guided-mode pre-arm step skipped for %s: %s", vehicle_id, exc)
            return False

    async def _send_guided_mode(self, vehicle_id: str) -> None:
        """Try to switch the vehicle into a guided mode via MAVLink direct messages."""
        if MavlinkDirect is None or MavlinkMessage is None:
            return

        system = self.systems.get(vehicle_id)
        if not system:
            return

        message = MavlinkMessage(
            "SET_MODE",
            255,
            190,
            1,
            1,
            '{"target_system":1,"target_component":1,"base_mode":81,"custom_mode":4}',
        )
        await system.mavlink_direct.send_message(message)

    async def arm(self, vehicle_id: str) -> bool:
        if vehicle_id not in self.systems: raise Exception("Vehicle not connected")
        system = self.systems[vehicle_id]
        try:
            await system.action.arm()
            self.vehicles[vehicle_id].is_armed = True
            return True
        except Exception as arm_error:
            self.logger.warning("Initial arm attempt was denied for %s: %s", vehicle_id, arm_error)
            try:
                await self._set_vehicle_mode(vehicle_id)
                await system.action.arm_force()
                self.vehicles[vehicle_id].is_armed = True
                return True
            except Exception as force_error:
                raise Exception(f"MAVSDK arm failed: {arm_error}; force-arm failed: {force_error}")

    async def disarm(self, vehicle_id: str) -> bool:
        if vehicle_id not in self.systems: raise Exception("Vehicle not connected")
        try:
            await self.systems[vehicle_id].action.disarm()
            for _ in range(50):
                if not self.vehicles[vehicle_id].is_armed:
                    return True
                await asyncio.sleep(0.1)
            raise Exception("Timeout waiting for Disarmed state")
        except Exception as e:
            raise Exception(f"MAVSDK disarm failed: {e}")

    async def takeoff(self, vehicle_id: str, altitude: float) -> bool:
        if vehicle_id not in self.systems: raise Exception("Vehicle not connected")
        system = self.systems[vehicle_id]
        try:
            await system.action.set_takeoff_altitude(altitude)
            await self.arm(vehicle_id)
            await system.action.takeoff()
            return True
        except Exception as e:
            raise Exception(f"MAVSDK takeoff failed: {e}")

    async def land(self, vehicle_id: str) -> bool:
        if vehicle_id not in self.systems: raise Exception("Vehicle not connected")
        try:
            await self.systems[vehicle_id].action.land()
            return True
        except Exception as e:
            raise Exception(f"MAVSDK land failed: {e}")

    async def goto_position(self, vehicle_id: str, position: List[float]) -> bool:
        if vehicle_id not in self.systems: return False
        try:
            # Get current yaw to prevent NaN rejection
            state = self.vehicles.get(vehicle_id)
            current_yaw = math.degrees(state.orientation[2]) if (state and state.orientation) else 0.0
            
            pos_ned = PositionNedYaw(position[0], position[1], position[2], current_yaw)
            
            if not hasattr(self, '_offboard_targets'):
                self._offboard_targets = {}
            if not hasattr(self, '_offboard_tasks'):
                self._offboard_tasks = {}
                
            self._offboard_targets[vehicle_id] = pos_ned
            
            if vehicle_id not in self._offboard_tasks or self._offboard_tasks[vehicle_id].done():
                self._offboard_tasks[vehicle_id] = asyncio.create_task(self._run_offboard(vehicle_id, self.systems[vehicle_id]))
            return True
        except Exception as e:
            raise Exception(f"MAVSDK goto_position failed: {e}")

    def _apply_collision_avoidance(self, vehicle_id: str, target):
        """Apply artificial potential fields to repel drones that get too close."""
        safe_distance = 2.0  # 2.0 meters minimum safe distance
        repel_strength = 2.0 # Velocity to add when breaching safe distance
        
        my_state = self.vehicles.get(vehicle_id)
        if not my_state or not hasattr(my_state, 'position'):
            return target
            
        my_pos = my_state.position
        repel_vector = [0.0, 0.0, 0.0]
        
        for other_id, other_state in self.vehicles.items():
            if other_id == vehicle_id or not hasattr(other_state, 'position'):
                continue
                
            other_pos = other_state.position
            # Calculate distance in NED
            dx = my_pos[0] - other_pos[0]
            dy = my_pos[1] - other_pos[1]
            dz = my_pos[2] - other_pos[2]
            
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if 0.01 < dist < safe_distance:
                # We are too close! Calculate repulsive vector.
                force = (safe_distance - dist) / safe_distance * repel_strength
                repel_vector[0] += (dx / dist) * force
                repel_vector[1] += (dy / dist) * force
                repel_vector[2] += (dz / dist) * force
                
        # If there's no collision risk, return original target
        if repel_vector[0] == 0 and repel_vector[1] == 0 and repel_vector[2] == 0:
            return target
            
        self.logger.warning(f"[COLLISION AVOIDANCE] {vehicle_id} deflecting by {repel_vector}")
            
        # Modify the target based on its type
        if isinstance(target, VelocityNedYaw):
            return VelocityNedYaw(
                target.north_m_s + repel_vector[0],
                target.east_m_s + repel_vector[1],
                target.down_m_s + repel_vector[2],
                target.yaw_deg
            )
        elif isinstance(target, PositionNedYaw):
            return PositionNedYaw(
                target.north_m + repel_vector[0] * 0.5,
                target.east_m + repel_vector[1] * 0.5,
                target.down_m + repel_vector[2] * 0.5,
                target.yaw_deg
            )
            
        return target

    async def _run_offboard(self, vehicle_id: str, system):
        """Continuously send the current target setpoint at 20Hz"""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.logger.info(f"[{ts}] [yellow][SENDING MAVSDK COMMAND][/yellow] Initializing offboard loop for {vehicle_id}")
        
        print("Offboard Started")
        
        # We need to send some initial setpoints before starting offboard mode
        initial_target = self._offboard_targets[vehicle_id]
        for _ in range(10):
            if isinstance(initial_target, PositionNedYaw):
                await system.offboard.set_position_ned(initial_target)
            else:
                await system.offboard.set_velocity_ned(initial_target)
            await asyncio.sleep(0.05)
            
        try:
            await system.offboard.start()
            print("PX4 ACK")
            ts = datetime.datetime.now().strftime("%H:%M:%S")
            self.logger.info(f"[{ts}] [green][PX4 ACK][/green] Offboard Started for {vehicle_id}")
        except Exception as e:
            ts = datetime.datetime.now().strftime("%H:%M:%S")
            self.logger.error(f"[{ts}] [red][FAILED][/red] PX4 rejected offboard for {vehicle_id}: {e}")
            return
            
        print("Velocity Stream Started")
        print("Drone Moving")
        
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.logger.info(f"[{ts}] [cyan][VELOCITY STREAM STARTED][/cyan] 20Hz loop active for {vehicle_id}")
        try:
            while True:
                target = self._offboard_targets.get(vehicle_id)
                if not target:
                    break
                    
                # Inject Collision Avoidance Layer
                safe_target = self._apply_collision_avoidance(vehicle_id, target)
                    
                if isinstance(safe_target, PositionNedYaw):
                    await system.offboard.set_position_ned(safe_target)
                else:
                    await system.offboard.set_velocity_ned(safe_target)
                    
                await asyncio.sleep(0.05) # 20 Hz
        except asyncio.CancelledError:
            pass
        finally:
            try:
                await system.offboard.stop()
            except:
                pass
            print("Target Reached")
            print("Movement Complete")
            ts = datetime.datetime.now().strftime("%H:%M:%S")
            self.logger.info(f"[{ts}] [magenta][MOVEMENT COMPLETED][/magenta] Stopped offboard loop for {vehicle_id}")

    async def goto_waypoint(self, vehicle_id: str, waypoint_index: int) -> bool:
        if vehicle_id not in self.systems: return False
        try:
            await self.systems[vehicle_id].mission.set_current_mission_item(waypoint_index)
            return True
        except Exception as e:
            self.logger.error(f"Error goto waypoint {vehicle_id}: {e}")
            return False

    async def load_mission(self, vehicle_id: str, waypoints: List[List[float]]) -> bool:
        if vehicle_id not in self.systems: return False
        try:
            mission_items = []
            for wp in waypoints:
                # Assuming waypoints are provided as [lat, lon, alt] for mission in MAVSDK
                item = MissionItem(
                    wp[0], wp[1], wp[2],
                    10.0, True, float('nan'), float('nan'),
                    MissionItem.CameraAction.NONE,
                    float('nan'), float('nan'), float('nan'), float('nan'), float('nan')
                )
                mission_items.append(item)
            
            plan = MissionPlan(mission_items)
            await self.systems[vehicle_id].mission.upload_mission(plan)
            await self.systems[vehicle_id].mission.start_mission()
            return True
        except Exception as e:
            self.logger.error(f"Error loading mission {vehicle_id}: {e}")
            return False

    async def get_image(self, vehicle_id: str, camera_name: str, image_type: str = "Scene") -> Optional[bytes]:
        # MAVSDK has a camera plugin, but fetching raw bytes often goes through a separate stream/RTSP.
        self.logger.warning("get_image not fully implemented for MAVSDK backend.")
        return None

    async def get_lidar_data(self, vehicle_id: str, lidar_name: str) -> Optional[List[float]]:
        return None

    async def get_imu_data(self, vehicle_id: str, imu_name: str) -> Optional[List[float]]:
        return None

    async def return_to_home(self, vehicle_id: str) -> bool:
        if vehicle_id not in self.systems: raise Exception("Vehicle not connected")
        try:
            await self.systems[vehicle_id].action.return_to_launch()
            return True
        except Exception as e:
            raise Exception(f"MAVSDK RTL failed: {e}")

    async def set_velocity(self, vehicle_id: str, velocity: List[float], duration: float) -> bool:
        if vehicle_id not in self.systems: return False
        try:
            system = self.systems[vehicle_id]
            # Get current yaw to prevent NaN rejection
            state = self.vehicles.get(vehicle_id)
            current_yaw = math.degrees(state.orientation[2]) if (state and state.orientation) else 0.0
            
            # velocity is already in NED format [North, East, Down]
            vel_ned = VelocityNedYaw(velocity[0], velocity[1], velocity[2], current_yaw)

            if not hasattr(self, '_offboard_targets'):
                self._offboard_targets = {}
            if not hasattr(self, '_offboard_tasks'):
                self._offboard_tasks = {}
                
            self._offboard_targets[vehicle_id] = vel_ned
            
            if vehicle_id not in self._offboard_tasks or self._offboard_tasks[vehicle_id].done():
                self._offboard_tasks[vehicle_id] = asyncio.create_task(self._run_offboard(vehicle_id, system))
                
            return True
        except Exception as e:
            self.logger.error(f"Error setting velocity {vehicle_id}: {e}")
            return False

    async def set_yaw(self, vehicle_id: str, angle: float, is_relative: bool = False) -> bool:
        if vehicle_id not in self.systems: return False
        try:
            target_yaw = angle
            if is_relative:
                # Assuming state.orientation[2] is yaw in radians
                state = self.vehicles[vehicle_id]
                current_yaw = math.degrees(state.orientation[2]) if hasattr(state, 'orientation') else 0.0
                target_yaw = current_yaw + angle
                
            # If we are in an offboard loop, update the current target's yaw
            if hasattr(self, '_offboard_targets') and vehicle_id in self._offboard_targets:
                target = self._offboard_targets[vehicle_id]
                if isinstance(target, PositionNedYaw):
                    new_target = PositionNedYaw(target.north_m, target.east_m, target.down_m, target_yaw)
                    self._offboard_targets[vehicle_id] = new_target
                elif isinstance(target, VelocityNedYaw):
                    new_target = VelocityNedYaw(target.north_m_s, target.east_m_s, target.down_m_s, target_yaw)
                    self._offboard_targets[vehicle_id] = new_target
            else:
                # If not in offboard loop, start one holding current position but with new yaw
                state = self.vehicles.get(vehicle_id)
                if state:
                    pos_ned = PositionNedYaw(state.position[0], state.position[1], state.position[2], target_yaw)
                    if not hasattr(self, '_offboard_targets'): self._offboard_targets = {}
                    if not hasattr(self, '_offboard_tasks'): self._offboard_tasks = {}
                    self._offboard_targets[vehicle_id] = pos_ned
                    if vehicle_id not in self._offboard_tasks or self._offboard_tasks[vehicle_id].done():
                        self._offboard_tasks[vehicle_id] = asyncio.create_task(self._run_offboard(vehicle_id, self.systems[vehicle_id]))

            return True
        except Exception as e:
            self.logger.error(f"Error setting yaw {vehicle_id}: {e}")
            return False
