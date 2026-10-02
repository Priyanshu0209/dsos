#!/usr/bin/env python3
"""
Initialization script for DSOS.
Sets up the directory structure and default configuration.
"""

import os
import sys
from pathlib import Path


def create_directories():
    """Create the necessary directory structure."""
    base_dir = Path(__file__).parent
    directories = [
        "core",
        "agents",
        "communication",
        "control",
        "missions",
        "formations",
        "navigation",
        "sensors",
        "ai",
        "telemetry",
        "ui",
        "utils",
        "plugins",
        "config",
        "logs",
        "tests",
        "docs",
        "deployment"
    ]

    for directory in directories:
        dir_path = base_dir / directory
        dir_path.mkdir(exist_ok=True)
        # Create __init__.py in each directory
        init_file = dir_path / "__init__.py"
        if not init_file.exists():
            init_file.write_text('"""{} package."""\n'.format(directory))
        print(f"Created directory: {directory}")


def create_init_files():
    """Create __init__.py files in directories that might be missed."""
    base_dir = Path(__file__).parent
    # Already handled in create_directories, but ensure all packages have __init__.py
    for root, dirs, files in os.walk(base_dir):
        # Skip hidden directories and __pycache__
        dirs[:] = [d for d in dirs if not d.startswith('.') and d != '__pycache__']
        for dir_name in dirs:
            dir_path = Path(root) / dir_name
            init_file = dir_path / "__init__.py"
            if not init_file.exists():
                init_file.write_text('"""{} package."""\n'.format(dir_name))
                print(f"Created {init_file}")


def create_default_config():
    """Create default configuration files."""
    config_dir = Path(__file__).parent / "config"
    config_dir.mkdir(exist_ok=True)

    # Create settings.ini if it doesn't exist
    settings_file = config_dir / "settings.ini"
    if not settings_file.exists():
        default_config = """[DSOS]
# Configuration file for Autonomous Drone Swarm Operating System

# General settings
swarm_size = 3
simulation_mode = true
airsim_host = localhost
airsim_port = 41451
update_rate_hz = 50

# Communication settings
heartbeat_interval_sec = 1.0
message_timeout_sec = 5.0
mesh_network_port = 9000
broadcast_prefix = "DSOS_"

# Safety settings
max_velocity_mps = 5.0
max_acceleration_mps2 = 2.0
min_safe_distance_m = 2.0
obstacle_threshold_m = 1.0

# Battery settings
battery_capacity_mah = 5000
low_battery_threshold_percent = 20
critical_battery_threshold_percent = 5

# Mission settings
default_mission_altitude_m = 10.0
waypoint_acceptance_radius_m = 1.0

# Logging
log_level = INFO
log_file = logs/dsos.log

# Telemetry
telemetry_update_rate_hz = 10
telemetry_history_size = 1000

# AI/Agent settings
decision_update_rate_hz = 10
behavior_tree_tick_rate_hz = 5

# Formation settings
default_formation = V_FORMATION
formation_update_rate_hz = 5
"""
        settings_file.write_text(default_config)
        print(f"Created default configuration: {settings_file}")


def create_readme():
    """Create a more detailed README file."""
    readme_path = Path(__file__).parent / "README.md"
    if not readme_path.exists():
        readme_content = """# Drone Swarm Operating System (DSOS)

A research-grade autonomous drone swarm operating system designed for scalability, robustness, and flexibility.

## Features

- Modular, plugin-based architecture
- Leaderless swarm intelligence with decentralized communication
- Mesh networking with heartbeat protocol and neighbor discovery
- Distributed consensus and autonomous decision making
- Dynamic mission planning and adaptive swarm behavior
- Collision and obstacle avoidance
- Intelligent path planning and AI-based task allocation
- Autonomous recovery, fault tolerance, and automatic reconnection
- Health monitoring, battery management, and emergency management
- Mission recording, replay, and telemetry visualization
- Professional terminal-based command interface with real-time keyboard control
- Over 100 executable commands for comprehensive swarm control
- Support for AirSim simulation, with extensibility to Unreal Engine, Gazebo, PX4, MAVSDK, MAVLink, ROS 2, and more
- Comprehensive sensor abstraction layer (GPS, IMU, LiDAR, cameras, etc.)
- Advanced AI layer (finite state machines, behavior trees, rule engines, planning algorithms)
- Professional telemetry system with real-time dashboards
- Security, authentication, logging, diagnostics, and configuration management
- Testing framework, benchmarking tools, documentation, and deployment support

## Getting Started

### Prerequisites

- Python 3.12+
- (Optional) AirSim installed and running for simulation

### Installation

1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt`
3. Configure the system in `config/`
4. Run the Ground Control Station: `python -m ui.gcs`

### Simulation Mode

To run in AirSim simulation mode, ensure AirSim is running and configured correctly.

## Architecture

The system follows a clean, modular architecture:

- **core**: Fundamental classes and interfaces
- **agents**: Individual drone agent implementations
- **communication**: Mesh networking, messaging protocols
- **control**: Ground Control Station and command interface
- **missions**: Mission definitions and planning
- **formations**: Formation generation and management
- **navigation**: Path planning and obstacle avoidance
- **sensors**: Sensor abstraction and fusion
- **ai**: Decision making, behavior trees, AI modules
- **telemetry**: Data collection, visualization, and logging
- **ui**: Terminal-based interface and keyboard controls
- **utils**: Utility functions and helpers
- **plugins**: Plugin interface and management
- **config**: Configuration files and management
- **logs**: Logging output
- **tests**: Unit and integration tests
- **docs**: Documentation
- **deployment**: Deployment scripts and configurations

## Contributing

Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
"""
        readme_path.write_text(readme_content)
        print(f"Created README: {readme_path}")


def main():
    """Main initialization function."""
    print("Initializing DSOS project structure...")
    create_directories()
    create_init_files()
    create_default_config()
    create_readme()
    print("Initialization complete!")


if __name__ == '__main__':
    main()