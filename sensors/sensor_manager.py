"""
Sensor abstraction layer for drone agents.
Provides a unified interface for various sensors.
"""
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
import time
import math
import random
from enum import Enum

from dsos.config.settings import settings


class SensorType(Enum):
    """Types of sensors available."""
    GPS = "gps"
    IMU = "imu"
    LIDAR = "lidar"
    MMWAVE_RADAR = "mmwave_radar"
    RGB_CAMERA = "rgb_camera"
    DEPTH_CAMERA = "depth_camera"
    THERMAL_CAMERA = "thermal_camera"
    ULTRASONIC = "ultrasonic"
    BAROMETER = "barometer"
    MAGNETOMETER = "magnetometer"
    AIRSPEED = "airspeed"
    BATTERY = "battery"


@dataclass
class SensorData:
    """Container for sensor data."""
    sensor_type: SensorType
    timestamp: float
    value: Any
    accuracy: float = 1.0  # 0.0 to 1.0
    raw_data: Optional[Any] = None


class SensorBase(ABC):
    """Base class for all sensors."""

    def __init__(self, sensor_type: SensorType, update_rate_hz: float = 10.0):
        self.sensor_type = sensor_type
        self.update_rate_hz = update_rate_hz
        self.last_update_time = 0.0
        self.is_enabled = True
        self.calibration_offset = 0.0
        self.calibration_scale = 1.0
        self.noise_level = 0.01
        self.logger = None  # Will be set by sensor manager

    @abstractmethod
    def read_raw(self) -> Any:
        """Read raw data from the sensor."""
        pass

    def read(self) -> SensorData:
        """Read and process sensor data."""
        if not self.is_enabled:
            return SensorData(
                sensor_type=self.sensor_type,
                timestamp=time.time(),
                value=None,
                accuracy=0.0
            )

        current_time = time.time()
        if current_time - self.last_update_time < (1.0 / self.update_rate_hz):
            # Return last reading if not enough time has passed
            # In a real implementation, we would cache the last reading
            pass

        try:
            raw_value = self.read_raw()
            # Apply calibration
            calibrated_value = (raw_value + self.calibration_offset) * self.calibration_scale
            # Add noise for simulation
            if isinstance(calibrated_value, (int, float)):
                noisy_value = calibrated_value + random.gauss(0, self.noise_level)
            elif isinstance(calibrated_value, list):
                noisy_value = [v + random.gauss(0, self.noise_level) for v in calibrated_value]
            else:
                noisy_value = calibrated_value

            self.last_update_time = current_time
            return SensorData(
                sensor_type=self.sensor_type,
                timestamp=current_time,
                value=noisy_value,
                accuracy=1.0 - self.noise_level,  # Simplified accuracy model
                raw_data=raw_value
            )
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error reading {self.sensor_type.value} sensor: {e}")
            return SensorData(
                sensor_type=self.sensor_type,
                timestamp=time.time(),
                value=None,
                accuracy=0.0
            )

    def enable(self) -> None:
        """Enable the sensor."""
        self.is_enabled = True

    def disable(self) -> None:
        """Disable the sensor."""
        self.is_enabled = False

    def calibrate(self, offset: float = 0.0, scale: float = 1.0) -> None:
        """Calibrate the sensor."""
        self.calibration_offset = offset
        self.calibration_scale = scale


class MockGPS(SensorBase):
    """Mock GPS sensor."""

    def __init__(self, update_rate_hz: float = 5.0):
        super().__init__(SensorType.GPS, update_rate_hz)
        self.base_position = [0.0, 0.0, 0.0]  # [lat, lon, alt] or [x, y, z]
        self.noise_level = 0.5  # meters

    def read_raw(self) -> List[float]:
        # Simulate GPS reading with some noise
        noise = [random.gauss(0, self.noise_level) for _ in range(3)]
        return [
            self.base_position[0] + noise[0],
            self.base_position[1] + noise[1],
            self.base_position[2] + noise[2]
        ]


class MockIMU(SensorBase):
    """Mock IMU (Inertial Measurement Unit) sensor."""

    def __init__(self, update_rate_hz: float = 100.0):
        super().__init__(SensorType.IMU, update_rate_hz)
        self.orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] in radians
        self.acceleration = [0.0, 0.0, 9.81]  # [ax, ay, az] in m/s^2
        self.angular_velocity = [0.0, 0.0, 0.0]  # [wx, wy, wz] in rad/s
        self.noise_level = 0.01

    def read_raw(self) -> dict:
        # Simulate IMU readings
        accel_noise = [random.gauss(0, self.noise_level) for _ in range(3)]
        gyro_noise = [random.gauss(0, self.noise_level * 0.1) for _ in range(3)]
        return {
            'acceleration': [
                self.acceleration[i] + accel_noise[i] for i in range(3)
            ],
            'angular_velocity': [
                self.angular_velocity[i] + gyro_noise[i] for i in range(3)
            ],
            'orientation': self.orientation.copy()  # Orientation typically doesn't have noise in this simple model
        }


class MockLidar(SensorBase):
    """Mock LIDAR sensor."""

    def __init__(self, update_rate_hz: float = 10.0, fov_degrees: float = 360.0,
                 range_min: float = 0.1, range_max: float = 30.0):
        super().__init__(SensorType.LIDAR, update_rate_hz)
        self.fov_degrees = fov_degrees
        self.range_min = range_min
        self.range_max = range_max
        self.num_points = 360  # Points per scan
        self.noise_level = 0.02  # meters

    def read_raw(self) -> List[float]:
        # Simulate a 360-degree LIDAR scan
        ranges = []
        for i in range(self.num_points):
            angle = math.radians(i * 360.0 / self.num_points)
            # Simulate some obstacles
            base_range = self.range_max
            # Add some random obstacles
            if random.random() < 0.1:  # 10% chance of obstacle
                base_range = random.uniform(self.range_min, self.range_max * 0.5)
            # Add noise
            noise = random.gauss(0, self.noise_level)
            range_val = max(self.range_min, min(self.range_max, base_range + noise))
            ranges.append(range_val)
        return ranges


class MockCamera(SensorBase):
    """Mock camera sensor (simplified)."""

    def __init__(self, sensor_type: SensorType, update_rate_hz: float = 5.0,
                 resolution: tuple = (640, 480)):
        super().__init__(sensor_type, update_rate_hz)
        self.resolution = resolution
        self.noise_level = 0.05

    def read_raw(self) -> dict:
        # Simulate camera image (just return metadata for simplicity)
        return {
            'resolution': self.resolution,
            'timestamp': time.time(),
            # In a real implementation, this would be image data
            'data': f"{self.sensor_type.value}_image_data_placeholder"
        }


class SensorManager:
    """Manages sensors for a drone agent."""

    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.logger = f"sensor.{agent_id}"
        self.sensors: Dict[SensorType, SensorBase] = {}
        self.last_readings: Dict[SensorType, SensorData] = {}
        self.enabled_sensors: set = set()

    def add_sensor(self, sensor: SensorBase) -> None:
        """Add a sensor to the manager."""
        self.sensors[sensor.sensor_type] = sensor
        sensor.logger = self.logger
        self.enabled_sensors.add(sensor.sensor_type)

    def remove_sensor(self, sensor_type: SensorType) -> None:
        """Remove a sensor from the manager."""
        if sensor_type in self.sensors:
            del self.sensors[sensor_type]
            self.enabled_sensors.discard(sensor_type)
            if sensor_type in self.last_readings:
                del self.last_readings[sensor_type]

    def get_sensor(self, sensor_type: SensorType) -> Optional[SensorBase]:
        """Get a sensor by type."""
        return self.sensors.get(sensor_type)

    def enable_sensor(self, sensor_type: SensorType) -> None:
        """Enable a sensor."""
        if sensor_type in self.sensors:
            self.sensors[sensor_type].enable()
            self.enabled_sensors.add(sensor_type)

    def disable_sensor(self, sensor_type: SensorType) -> None:
        """Disable a sensor."""
        if sensor_type in self.sensors:
            self.sensors[sensor_type].disable()
            self.enabled_sensors.discard(sensor_type)

    def read_sensor(self, sensor_type: SensorType) -> Optional[SensorData]:
        """Read data from a specific sensor."""
        sensor = self.get_sensor(sensor_type)
        if sensor is None:
            return None
        data = sensor.read()
        self.last_readings[sensor_type] = data
        return data

    def read_all_sensors(self) -> Dict[SensorType, SensorData]:
        """Read data from all enabled sensors."""
        readings = {}
        for sensor_type in self.enabled_sensors:
            if sensor_type in self.sensors:
                data = self.sensors[sensor_type].read()
                self.last_readings[sensor_type] = data
                readings[sensor_type] = data
        return readings

    def get_last_reading(self, sensor_type: SensorType) -> Optional[SensorData]:
        """Get the last reading from a sensor."""
        return self.last_readings.get(sensor_type)

    def initialize_default_sensors(self) -> None:
        """Initialize a default set of sensors for simulation."""
        # Add common sensors
        self.add_sensor(MockGPS(update_rate_hz=5.0))
        self.add_sensor(MockIMU(update_rate_hz=100.0))
        self.add_sensor(MockLidar(update_rate_hz=10.0))
        self.add_sensor(MockCamera(SensorType.RGB_CAMERA, update_rate_hz=5.0))
        self.add_sensor(MockCamera(SensorType.DEPTH_CAMERA, update_rate_hz=5.0))
        self.add_sensor(MockCamera(SensorType.THERMAL_CAMERA, update_rate_hz=2.0))

        # For simulation, we'll add a simple battery sensor
        class MockBattery(SensorBase):
            def __init__(self):
                super().__init__(SensorType.BATTERY, update_rate_hz=1.0)
                self.charge_level = 100.0  # percent
                self.drain_rate = 0.01  # percent per second

            def read_raw(self) -> float:
                # Simulate battery drain
                self.charge_level = max(0, self.charge_level - self.drain_rate / self.update_rate_hz)
                return self.charge_level

        self.add_sensor(MockBattery())

        self.logger.info("Initialized default sensor suite")


# Make enums available
from enum import Enum