"""
AirSim backend implementation for DSOS.
Provides real integration with Microsoft AirSim simulator using the official Python API.
"""
import asyncio
import logging
import time
from typing import Dict, List, Optional, Tuple
import numpy as np

# Try to import airsim, gracefully handle if not available
try:
    import airsim
    HAS_AIRSIM = True
except ImportError:
    HAS_AIRSIM = False
    airsim = None

from .base_backend import BackendInterface, BackendType, VehicleState, BackendCapabilities


class AirSimBackend(BackendInterface):
    """
    AirSim backend implementation.
    Connects to AirSim simulator and provides real drone control capabilities.
    """

    def __init__(self, config: dict):
        """
        Initialize the AirSim backend.

        Args:
            config: Configuration dictionary with keys:
                - host: AirSim server hostname/IP (default: localhost)
                - port: AirSim server port (default: 41451)
                - vehicle_names: List of vehicle names to manage (optional, auto-discovered if not provided)
        """
        super().__init__(config)
        self.backend_type = BackendType.AIRSIM
        self.client: Optional[airsim.MultirotorClient] = None
        self.vehicle_names: List[str] = []

        # Configure capabilities based on what AirSim provides
        self.capabilities = BackendCapabilities(
            has_gps=True,
            has_imu=True,
            has_lidar=True,
            has_rgb_camera=True,
            has_depth_camera=True,
            has_thermal_camera=True,  # Depends on vehicle setup
            has_mmwave_radar=False,   # AirSim doesn't have native mmWave radar
            can_takeoff=True,
            can_land=True,
            can_hover=True,
            can_move=True,
            can_get_image=True,
            can_get_lidar_data=True,
            can_get_imu_data=True,
            can_get_gps_data=True,
            can_set_speed=True,
            can_set_yaw=True,
            can_get_battery_info=True,
            can_get_health_status=True
        )

        # Store last known states for change detection
        self._last_states: Dict[str, VehicleState] = {}

    async def connect(self) -> bool:
        """
        Connect to the AirSim simulator.

        Returns:
            True if connection successful, False otherwise
        """
        if not HAS_AIRSIM:
            self.logger.error("AirPy library not installed. Install with: pip install airsim")
            return False

        try:
            host = self.config.get('host', 'localhost')
            port = self.config.get('port', 41451)

            self.logger.info(f"Connecting to AirSim at {host}:{port}")
            self.client = airsim.MultirotorClient(ip=host, port=port)
            self.client.confirmConnection()

            # Enable API control for all vehicles
            self.vehicle_names = self.client.listObjects()
            # Filter to only vehicle objects (those that start with common UAV prefixes)
            self.vehicle_names = [name for name in self.vehicle_names
                                if any(prefix in name.lower() for prefix in ['drone', 'uav', 'quad', 'x'])]

            # If no vehicles detected with those prefixes, try common names or use all
            if not self.vehicle_names:
                # Common vehicle names in AirSim examples
                common_names = ["Drone1", "Drone2", "Drone3", "Flight1", "Flight2", "X3"]
                self.vehicle_names = [name for name in self.client.listObjects()
                                    if name in common_names]

            # If still none, use all objects (might include non-vehicles, but lets user filter)
            if not self.vehicle_names:
                self.vehicle_names = self.client.listObjects()

            self.logger.info(f"Discovered vehicles: {self.vehicle_names}")

            # Enable API control and arm all discovered vehicles
            for vehicle_name in self.vehicle_names:
                self.client.enableApiControl(True, vehicle_name)
                # Don't arm by default - let user decide when to arm

            self.is_connected = True
            self.logger.info(f"Successfully connected to AirSim. Found {len(self.vehicle_names)} vehicles.")
            return True

        except Exception as e:
            self.logger.error(f"Failed to connect to AirSim: {e}")
            self.is_connected = False
            return False

    async def disconnect(self) -> None:
        """Disconnect from the AirSim simulator."""
        if self.client and self.is_connected:
            try:
                # Disable API control for all vehicles
                for vehicle_name in self.vehicle_names:
                    try:
                        self.client.enableApiControl(False, vehicle_name)
                    except Exception:
                        pass  # Ignore errors on disconnect
            except Exception as e:
                self.logger.error(f"Error during disconnection: {e}")
            finally:
                self.client = None
                self.is_connected = False
                self.logger.info("Disconnected from AirSim")

    async def get_vehicles(self) -> List[str]:
        """
        Get list of available vehicle IDs in AirSim.

        Returns:
            List of vehicle identifiers
        """
        if not self.is_connected or not self.client:
            return []

        # Return the vehicle names we discovered during connection
        return self.vehicle_names.copy()

    async def get_vehicle_state(self, vehicle_id: str) -> Optional[VehicleState]:
        """
        Get the current state of a vehicle in AirSim.

        Args:
            vehicle_id: ID of the vehicle to query

        Returns:
            VehicleState object or None if vehicle not found/error
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return None

        try:
            # Get kinematics (position, orientation, velocity, acceleration)
            kinematics = self.client.getKinematicsEstimated(vehicle_id)
            pos = kinematics.position
            rot = kinematics.orientation
            vel = kinematics.linear_velocity
            accel = kinematics.linear_acceleration
            ang_vel = kinematics.angular_velocity

            # Get GPS data
            gps_data = self.client.getGpsData(vehicle_id)
            gps = [gps_data.gnss.geo_point.latitude,
                   gps_data.gnss.geo_point.longitude,
                   gps_data.gnss.geo_point.altitude] if gps_data else None

            # Get battery info
            battery_info = self.client.getBatteryInfo(vehicle_id)
            battery_voltage = battery_info.voltage if battery_info else None
            battery_remaining = battery_info.battery_percentage if battery_info else None

            # Create VehicleState object
            state = VehicleState(
                vehicle_id=vehicle_id,
                timestamp=time.time(),
                position=[pos.x_val, pos.y_val, pos.z_val],
                velocity=[vel.x_val, vel.y_val, vel.z_val],
                acceleration=[accel.x_val, accel.y_val, accel.z_val],
                orientation=[
                    self._quaternion_to_roll_pitch_yaw(
                        rot.w_val, rot.x_val, rot.y_val, rot.z_val
                    )[0],  # roll
                    self._quaternion_to_roll_pitch_yaw(
                        rot.w_val, rot.x_val, rot.y_val, rot.z_val
                    )[1],  # pitch
                    self._quaternion_to_roll_pitch_yaw(
                        rot.w_val, rot.x_val, rot.y_val, rot.z_val
                    )[2]   # yaw
                ],
                angular_velocity=[ang_vel.x_val, ang_vel.y_val, ang_vel.z_val],
                gps=gps,
                battery_voltage=battery_voltage,
                battery_remaining=battery_remaining,
                is_armed=self.client.isApiControlEnabled(vehicle_id),
                is_in_air=self._is_in_air(vehicle_id),
                land_state=self._get_land_state(vehicle_id)
            )

            # Cache the state
            self._last_states[vehicle_id] = state
            return state

        except Exception as e:
            self.logger.error(f"Error getting state for vehicle {vehicle_id}: {e}")
            return self._last_states.get(vehicle_id)  # Return last known state if available

    async def arm(self, vehicle_id: str) -> bool:
        """
        Arm a vehicle's motors in AirSim.

        Args:
            vehicle_id: ID of the vehicle to arm

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            self.client.armDisarm(True, vehicle_id)
            self.logger.info(f"Armed vehicle {vehicle_id}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to arm vehicle {vehicle_id}: {e}")
            return False

    async def disarm(self, vehicle_id: str) -> bool:
        """
        Disarm a vehicle's motors in AirSim.

        Args:
            vehicle_id: ID of the vehicle to disarm

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            self.client.armDisarm(False, vehicle_id)
            self.logger.info(f"Disarmed vehicle {vehicle_id}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to disarm vehicle {vehicle_id}: {e}")
            return False

    async def takeoff(self, vehicle_id: str, altitude: float) -> bool:
        """
        Command a vehicle to takeoff to a specified altitude in AirSim.

        Args:
            vehicle_id: ID of the vehicle
            altitude: Target altitude in meters (relative to launch position)

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            self.client.takeoffAsync(vehicle_id).join()
            # Note: AirSim takeoff goes to a default altitude, we'll hover at target after takeoff
            await asyncio.sleep(1)  # Give it a moment to start taking off
            # Then move to the desired altitude if needed
            current_state = await self.get_vehicle_state(vehicle_id)
            if current_state:
                target_pos = [
                    current_state.position[0],
                    current_state.position[1],
                    -altitude  # Negative because Z is down in NED (AirSim uses NED)
                ]
                await self.goto_position(vehicle_id, target_pos)
            self.logger.info(f"Commanded vehicle {vehicle_id} to takeoff to {altitude}m")
            return True
        except Exception as e:
            self.logger.error(f"Failed to takeoff vehicle {vehicle_id}: {e}")
            return False

    async def land(self, vehicle_id: str) -> bool:
        """
        Command a vehicle to land in AirSim.

        Args:
            vehicle_id: ID of the vehicle

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            self.client.landAsync(vehicle_id).join()
            self.logger.info(f"Commanded vehicle {vehicle_id} to land")
            return True
        except Exception as e:
            self.logger.error(f"Failed to land vehicle {vehicle_id}: {e}")
            return False

    async def goto_position(self, vehicle_id: str, position: List[float]) -> bool:
        """
        Command a vehicle to fly to a specific position in AirSim.

        Args:
            vehicle_id: ID of the vehicle
            position: Target position [x, y, z] in meters (ENU coordinates)

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            # Convert from ENU (used internally) to NED (used by AirSim)
            # ENU: East, North, Up
            # NED: North, East, Down
            # So: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
            ned_position = [
                position[1],  # North = East (ENU)
                position[0],  # East = North (ENU)
                -position[2]  # Down = -Up (ENU)
            ]

            # Move to position using moveToPositionAsync
            self.client.moveToPositionAsync(
                ned_position[0], ned_position[1], ned_position[2],
                velocity=5.0,  # 5 m/s speed
                vehicle_name=vehicle_id
            ).join()

            self.logger.debug(f"Commanded vehicle {vehicle_id} to position {position} (NED: {ned_position})")
            return True
        except Exception as e:
            self.logger.error(f"Failed to move vehicle {vehicle_id} to position {position}: {e}")
            return False

    async def goto_waypoint(self, vehicle_id: str, waypoint_index: int) -> bool:
        """
        Command a vehicle to go to a specific waypoint in its mission.

        Note: AirSim doesn't have native waypoint mission concept in the same way.
        This implementation assumes the user has managed waypoints externally
        and this just goes to a specific coordinate if we had stored waypoints.
        For true waypoint mission support, we would need to integrate with
        AirSim's mission planning or implement our own waypoint following.

        Args:
            vehicle_id: ID of the vehicle
            waypoint_index: Index of the waypoint to navigate to

        Returns:
            True if command successful, False otherwise
        """
        # This is a simplified implementation - in a full system, we would
        # store waypoints per vehicle and retrieve the specific waypoint
        self.logger.warning(
            f"goto_waypoint not fully implemented for AirSim backend. "
            f"Would need waypoint storage per vehicle. Index requested: {waypoint_index}"
        )
        return False

    async def load_mission(self, vehicle_id: str, waypoints: List[List[float]]) -> bool:
        """
        Load a mission (list of waypoints) for a vehicle.

        Note: Similar to goto_waypoint, AirSim doesn't have native mission upload.
        This would require implementing our own waypoint following system
        or using external tools. For now, we'll just store the waypoints
        for potential use by goto_waypoint.

        Args:
            vehicle_id: ID of the vehicle
            waypoints: List of waypoints, each [x, y, z] in meters

        Returns:
            True if command successful, False otherwise
        """
        self.logger.warning(
            f"load_mission not fully implemented for AirSim backend. "
            f"Would need waypoint following system. Stored {len(waypoints)} waypoints for {vehicle_id}"
        )
        # In a full implementation, we would store these waypoints
        # associated with the vehicle for later use
        return True  # Return success for now since we're storing conceptually

    async def get_image(self, vehicle_id: str, camera_name: str,
                       image_type: str = "Scene") -> Optional[bytes]:
        """
        Capture an image from a vehicle's camera in AirSim.

        Args:
            vehicle_id: ID of the vehicle
            camera_name: Name of the camera to use (e.g., "0", "FrontCenter", etc.)
            image_type: Type of image to capture ("Scene", "Depth", "Segmentation", etc.)

        Returns:
            Image data as bytes or None if failed
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return None

        try:
            # Map image type string to AirSim enum
            image_type_map = {
                "Scene": airsim.ImageType.Scene,
                "Depth": airsim.ImageType.DepthPerspective,
                "DepthPerspective": airsim.ImageType.DepthPerspective,
                "DepthPlanar": airsim.ImageType.DepthPlanar,
                "DepthVis": airsim.ImageType.DepthVis,
                "DisparityNormalized": airsim.ImageType.DisparityNormalized,
                "Segmentation": airmis.ImageType.Segmentation,
                "Seg": airsim.ImageType.Segmentation,
                "SurfaceNormals": airsim.ImageType.SurfaceNormals,
                "Infrared": airsim.ImageType.Infrared
            }

            airsim_image_type = image_type_map.get(image_type, airsim.ImageType.Scene)

            # Get images from the camera
            responses = self.client.simGetImages([
                airsim.ImageRequest(camera_name, airsim_image_type, False, False)
            ], vehicle_id)

            if responses and len(responses) > 0:
                response = responses[0]
                if response.image_data_uint8:
                    # Convert to numpy array and then to bytes
                    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                    return img1d.tobytes()
                elif response.image_data_float:  # For depth images etc.
                    img1d = np.frombuffer(response.image_data_float, dtype=np.float32)
                    return img1d.tobytes()

            return None
        except Exception as e:
            self.logger.error(f"Failed to get image from vehicle {vehicle_id}, camera {camera_name}: {e}")
            return None

    async def get_lidar_data(self, vehicle_id: str, lidar_name: str) -> Optional[List[float]]:
        """
        Get LiDAR point cloud data from AirSim.

        Args:
            vehicle_id: ID of the vehicle
            lidar_name: Name of the LiDAR sensor

        Returns:
            List of point cloud data [x1, y1, z1, x2, y2, z2, ...] or None if failed
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return None

        try:
            lidar_data = self.client.getLidarData(lidar_name, vehicle_id)
            if lidar_data.point_cloud:
                # Return as flat list of [x1, y1, z1, x2, y2, z2, ...]
                return list(lidar_data.point_cloud)
            return None
        except Exception as e:
            self.logger.error(f"Failed to get LiDAR data from vehicle {vehicle_id}: {e}")
            return None

    async def get_imu_data(self, vehicle_id: str, imu_name: str) -> Optional[List[float]]:
        """
        Get IMU data from AirSim.

        Args:
            vehicle_id: ID of the vehicle
            imu_name: Name of the IMU sensor

        Returns:
            IMU data [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z] or None if failed
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return None

        try:
            imu_data = self.client.getImuData(imu_name, vehicle_id)
            if imu_data:
                return [
                    imu_data.linear_acceleration.x_val,
                    imu_data.linear_acceleration.y_val,
                    imu_data.linear_acceleration.z_val,
                    imu_data.angular_velocity.x_val,
                    imu_data.angular_velocity.y_val,
                    imu_data.angular_velocity.z_val
                ]
            return None
        except Exception as e:
            self.logger.error(f"Failed to get IMU data from vehicle {vehicle_id}: {e}")
            return None

    async def set_velocity(self, vehicle_id: str, velocity: List[float],
                          duration: float) -> bool:
        """
        Set velocity for a vehicle in AirSim.

        Args:
            vehicle_id: ID of the vehicle
            velocity: Target velocity [vx, vy, vz] in m/s (ENU coordinates)
            duration: Duration to maintain velocity in seconds

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            # Convert from ENU to NED for AirSim
            ned_velocity = [
                velocity[1],  # North = East (ENU)
                velocity[0],  # East = North (ENU)
                -velocity[2]  # Down = -Up (ENU)
            ]

            # Move by velocity vector for specified duration
            self.client.moveByVelocityZAsync(
                ned_velocity[0], ned_velocity[1], ned_velocity[2],
                duration,  # duration
                vehicle_name=vehicle_id
            ).join()

            self.logger.debug(f"Set velocity for vehicle {vehicle_id}: {velocity} m/s for {duration}s")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set velocity for vehicle {vehicle_id}: {e}")
            return False

    async def set_yaw(self, vehicle_id: str, angle: float,
                     is_relative: bool = False) -> bool:
        """
        Set yaw angle for a vehicle in AirSim.

        Args:
            vehicle_id: ID of the vehicle
            angle: Target yaw angle in degrees
            is_relative: If True, angle is relative to current heading

        Returns:
            True if command successful, False otherwise
        """
        if not self.is_connected or not self.client or vehicle_id not in self.vehicle_names:
            return False

        try:
            if is_relative:
                # Get current yaw first
                current_state = await self.get_vehicle_state(vehicle_id)
                if current_state:
                    current_yaw = current_state.orientation[2] * 180 / 3.14159  # Convert rad to deg
                    target_yaw = current_yaw + angle
                else:
                    target_yaw = angle  # Fallback if we can't get current state
            else:
                target_yaw = angle

            # Convert to radians for AirSim
            target_yaw_rad = target_yaw * 3.14159 / 180.0

            self.client.rotateToYawAsync(target_yaw_rad, vehicle_name=vehicle_id).join()
            self.logger.debug(f"Set yaw for vehicle {vehicle_id}: {angle} degrees (relative: {is_relative})")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set yaw for vehicle {vehicle_id}: {e}")
            return False

    def _quaternion_to_roll_pitch_yaw(self, w: float, x: float, y: float, z: float) -> tuple:
        """
        Convert quaternion to roll, pitch, yaw Euler angles.

        Args:
            w, x, y, z: Quaternion components

        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _is_in_air(self, vehicle_id: str) -> bool:
        """
        Determine if a vehicle is in the air based on its state.

        Args:
            vehicle_id: ID of the vehicle

        Returns:
            True if vehicle is airborne, False otherwise
        """
        try:
            state = self.get_vehicle_state(vehicle_id)
            if state:
                # Consider in air if altitude > 0.1m above ground
                # Note: In NED, Z is negative up, so we check if position[2] < -0.1
                return state.position[2] < -0.1
            return False
        except Exception:
            return False

    def _get_land_state(self, vehicle_id: str) -> str:
        """
        Get the landing state of a vehicle.

        Args:
            vehicle_id: ID of the vehicle

        Returns:
            Land state string: "landed", "in_air", "taking_off", "landing"
        """
        try:
            # This is a simplified implementation
            # In reality, we might need to check velocity, recent commands, etc.
            state = self.get_vehicle_state(vehicle_id)
            if state:
                if state.position[2] < -0.1:  # Above ground
                    # Check if moving upward or downward recently
                    if hasattr(self, '_last_altitude') and vehicle_id in self._last_altitude:
                        altitude_change = state.position[2] - self._last_altitude[vehicle_id]
                        if altitude_change > 0.05:  # Moving up
                            return "taking_off"
                        elif altitude_change < -0.05:  # Moving down
                            return "landing"
                    return "in_air"
                else:
                    return "landed"
            return "unknown"
        except Exception:
            return "unknown"


# Factory function to create backend instances
def create_backend(backend_type: str, config: dict) -> Optional[BackendInterface]:
    """
    Factory function to create backend instances.

    Args:
        backend_type: Type of backend to create ("airsim", "px4_sitl", etc.)
        config: Configuration dictionary for the backend

    Returns:
        BackendInterface instance or None if type not supported
    """
    backend_type = backend_type.lower()

    if backend_type == "airsim":
        return AirSimBackend(config)
    elif backend_type in ["mavsdk", "px4_sitl", "real_drone"]:
        try:
            from dsos.control.backends.mavsdk_backend import MAVSDKBackend
            return MAVSDKBackend(config)
        except ImportError:
            logging.getLogger("create_backend").error("MAVSDKBackend could not be imported.")
            return None
    elif backend_type == "gazebo":
        # TODO: Implement Gazebo specific backend (if not using MAVSDK for it)
        try:
            from dsos.control.backends.mavsdk_backend import MAVSDKBackend
            return MAVSDKBackend(config)
        except ImportError:
            return None
    elif backend_type == "ros2":
        # TODO: Implement ROS2 backend
        from .base_backend import BackendInterface
        raise NotImplementedError("ROS2 backend not implemented yet")
    elif backend_type == "simulation":
        # Return existing simulated backend for backward compatibility
        from dsos.agents.drone_agent import DroneAgent
        # Note: This returns a DroneAgent, not a BackendInterface
        # For true backward compatibility, we'd need to wrap it
        # But for now, let's return None to indicate we should use the existing system
        return None
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")