"""
Hardware Abstraction Layer (HAL) for DSOS.
Defines a common interface for all drone backends (AirSim, PX4, Gazebo, etc.)
so the same commands work regardless of the underlying platform.
"""
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any, Tuple
import asyncio
import logging
from dataclasses import dataclass
from enum import Enum


class BackendType(Enum):
    """Types of backends supported by DSOS."""
    AIRSIM = "airsim"
    PX4_SITL = "px4_sitl"
    GAZEBO = "gazebo"
    ROS2 = "ros2"
    MAVSDK = "mavsdk"
    REAL_DRONE = "real_drone"
    SIMULATION = "simulation"  # Existing simulated backend


@dataclass
class BackendCapabilities:
    """Capabilities that a backend can provide."""
    has_gps: bool = False
    has_imu: bool = False
    has_lidar: bool = False
    has_rgb_camera: bool = False
    has_depth_camera: bool = False
    has_thermal_camera: bool = False
    has_mmag_radar: bool = False
    can_takeoff: bool = False
    can_land: bool = False
    can_hover: bool = False
    can_move: bool = False
    can_get_image: bool = False
    can_get_lidar_data: bool = False
    can_get_imu_data: bool = False
    can_get_gps_data: bool = False
    can_set_speed: bool = False
    can_set_yaw: bool = False
    can_get_battery_info: bool = False
    can_get_health_status: bool = False


@dataclass
class VehicleState:
    """Standardized vehicle state across all backends."""
    vehicle_id: str
    timestamp: float
    position: List[float]  # [x, y, z] in meters (ENU coordinates)
    velocity: List[float]  # [vx, vy, vz] in m/s
    acceleration: List[float]  # [ax, ay, az] in m/s^2
    orientation: List[float]  # [roll, pitch, yaw] in radians
    angular_velocity: List[float]  # [wx, wy, wz] in rad/s

    # Sensor data
    gps: Optional[List[float]] = None  # [latitude, longitude, altitude]
    battery_voltage: Optional[float] = None  # Volts
    battery_remaining: Optional[float] = None  # Percentage (0-100)
    temperature: Optional[float] = None  # Celsius

    # Status
    is_connected: bool = False
    is_armed: bool = False
    is_in_air: bool = False
    land_state: str = "unknown"  # "landed", "in_air", "taking_off", "landing"
    flight_mode: str = "UNKNOWN"
    health_status: str = "WAIT"
    rc_signal_strength: Optional[float] = None  # Percentage (0-100)


class BackendInterface(ABC):
    """
    Abstract base class for all drone backends.
    Each backend must implement this interface to work with DSOS.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the backend with configuration.

        Args:
            config: Backend-specific configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        self.vehicles: Dict[str, VehicleState] = {}
        self.is_connected = False
        self.capabilities = BackendCapabilities()

    @abstractmethod
    async def connect(self) -> bool:
        """
        Connect to the backend (simulator or hardware).

        Returns:
            True if connection successful, False otherwise
        """
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from the backend."""
        pass

    @abstractmethod
    async def get_vehicles(self) -> List[str]:
        """
        Get list of available vehicle IDs.

        Returns:
            List of vehicle identifiers
        """
        pass

    @abstractmethod
    async def get_vehicle_state(self, vehicle_id: str) -> Optional[VehicleState]:
        """
        Get the current state of a vehicle.

        Args:
            vehicle_id: ID of the vehicle to query

        Returns:
            VehicleState object or None if vehicle not found/error
        """
        pass

    @abstractmethod
    async def arm(self, vehicle_id: str) -> bool:
        """
        Arm a vehicle's motors.

        Args:
            vehicle_id: ID of the vehicle to arm

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def disarm(self, vehicle_id: str) -> bool:
        """
        Disarm a vehicle's motors.

        Args:
            vehicle_id: ID of the vehicle to disarm

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def takeoff(self, vehicle_id: str, altitude: float) -> bool:
        """
        Command a vehicle to takeoff to a specified altitude.

        Args:
            vehicle_id: ID of the vehicle
            altitude: Target altitude in meters (relative to home)

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def land(self, vehicle_id: str) -> bool:
        """
        Command a vehicle to land.

        Args:
            vehicle_id: ID of the vehicle

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def goto_position(self, vehicle_id: str, position: List[float]) -> bool:
        """
        Command a vehicle to fly to a specific position.

        Args:
            vehicle_id: ID of the vehicle
            position: Target position [x, y, z] in meters (ENU coordinates)

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def goto_waypoint(self, vehicle_id: str, waypoint_index: int) -> bool:
        """
        Command a vehicle to go to a specific waypoint in its mission.

        Args:
            vehicle_id: ID of the vehicle
            waypoint_index: Index of the waypoint to navigate to

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def load_mission(self, vehicle_id: str, waypoints: List[List[float]]) -> bool:
        """
        Load a mission (list of waypoints) for a vehicle.

        Args:
            vehicle_id: ID of the vehicle
            waypoints: List of waypoints, each [x, y, z] in meters

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def get_image(self, vehicle_id: str, camera_name: str,
                       image_type: str = "Scene") -> Optional[bytes]:
        """
        Capture an image from a vehicle's camera.

        Args:
            vehicle_id: ID of the vehicle
            camera_name: Name of the camera to use
            image_type: Type of image to capture (Scene, Depth, Segmentation, etc.)

        Returns:
            Image data as bytes or None if failed
        """
        pass

    @abstractmethod
    async def get_lidar_data(self, vehicle_id: str, lidar_name: str) -> Optional[List[float]]:
        """
        Get LiDAR point cloud data.

        Args:
            vehicle_id: ID of the vehicle
            lidar_name: Name of the LiDAR sensor

        Returns:
            List of point cloud data or None if failed
        """
        pass

    @abstractmethod
    async def get_imu_data(self, vehicle_id: str, imu_name: str) -> Optional[List[float]]:
        """
        Get IMU (Inertial Measurement Unit) data.

        Args:
            vehicle_id: ID of the vehicle
            imu_name: Name of the IMU sensor

        Returns:
            IMU data [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z] or None
        """
        pass

    @abstractmethod
    async def set_velocity(self, vehicle_id: str, velocity: List[float],
                          duration: float) -> bool:
        """
        Set velocity for a vehicle.

        Args:
            vehicle_id: ID of the vehicle
            velocity: Target velocity [vx, vy, vz] in m/s
            duration: Duration to maintain velocity in seconds

        Returns:
            True if command successful, False otherwise
        """
        pass

    @abstractmethod
    async def set_yaw(self, vehicle_id: str, angle: float,
                     is_relative: bool = False) -> bool:
        """
        Set yaw angle for a vehicle.

        Args:
            vehicle_id: ID of the vehicle
            angle: Target yaw angle in degrees
            is_relative: If True, angle is relative to current heading

        Returns:
            True if command successful, False otherwise
        """
        pass

    def get_capabilities(self) -> BackendCapabilities:
        """
        Get the capabilities of this backend.

        Returns:
            BackendCapabilities object describing what this backend can do
        """
        return self.capabilities

    def is_connected(self) -> bool:
        """
        Check if the backend is connected.

        Returns:
            True if connected, False otherwise
        """
        return self.is_connected