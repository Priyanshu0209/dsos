"""
Configuration management module for DSOS.
Loads and provides access to system configuration.
"""
import configparser
from pathlib import Path
from typing import Any, Optional


class Settings:
    """Singleton configuration manager."""
    _instance = None
    _config: configparser.ConfigParser

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Settings, cls).__new__(cls)
            cls._instance._config = configparser.ConfigParser()
            # Load default configuration
            cls._instance._load_configuration()
        return cls._instance

    def _load_configuration(self) -> None:
        """Load configuration from file."""
        config_path = Path(__file__).parent.parent / "config" / "settings.ini"
        if config_path.exists():
            self._config.read(config_path)
        else:
            # Create default configuration if file doesn't exist
            self._create_default_config(config_path)

    def _create_default_config(self, config_path: Path) -> None:
        """Create a default configuration file."""
        config_path.parent.mkdir(parents=True, exist_ok=True)
        self._config['DSOS'] = {
            'swarm_size': '3',
            'simulation_mode': 'true',
            'airsim_host': 'localhost',
            'airsim_port': '41451',
            'update_rate_hz': '50',
            'heartbeat_interval_sec': '1.0',
            'message_timeout_sec': '5.0',
            'mesh_network_port': '9000',
            'broadcast_prefix': 'DSOS_',
            'max_velocity_mps': '5.0',
            'max_acceleration_mps2': '2.0',
            'min_safe_distance_m': '2.0',
            'obstacle_threshold_m': '1.0',
            'battery_capacity_mah': '5000',
            'low_battery_threshold_percent': '20',
            'critical_battery_threshold_percent': '5',
            'default_mission_altitude_m': '10.0',
            'waypoint_acceptance_radius_m': '1.0',
            'log_level': 'INFO',
            'log_file': 'logs/dsos.log',
            'telemetry_update_rate_hz': '10',
            'telemetry_history_size': '1000',
            'decision_update_rate_hz': '10',
            'behavior_tree_tick_rate_hz': '5',
            'default_formation': 'V_FORMATION',
            'formation_update_rate_hz': '5'
        }
        with open(config_path, 'w') as configfile:
            self._config.write(configfile)

    def get(self, section: str, key: str, fallback: Any = None) -> Any:
        """Get a configuration value."""
        return self._config.get(section, key, fallback=fallback)

    def getint(self, section: str, key: str, fallback: int = 0) -> int:
        """Get an integer configuration value."""
        return self._config.getint(section, key, fallback=fallback)

    def getfloat(self, section: str, key: str, fallback: float = 0.0) -> float:
        """Get a float configuration value."""
        return self._config.getfloat(section, key, fallback=fallback)

    def getboolean(self, section: str, key: str, fallback: bool = False) -> bool:
        """Get a boolean configuration value."""
        return self._config.getboolean(section, key, fallback=fallback)

    def set(self, section: str, key: str, value: Any) -> None:
        """Set a configuration value."""
        if not self._config.has_section(section):
            self._config.add_section(section)
        self._config.set(section, key, str(value))

    def save(self) -> None:
        """Save configuration to file."""
        config_path = Path(__file__).parent.parent / "config" / "settings.ini"
        with open(config_path, 'w') as configfile:
            self._config.write(configfile)


# Global settings instance
settings = Settings()