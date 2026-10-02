# Autonomous Drone Swarm Operating System (DSOS)

A research-grade autonomous drone swarm operating system designed for scalability, robustness, and flexibility. Built with Python, AirSim, and a modular architecture resembling a professional Ground Control Station (GCS).

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
- AirSim installed and running (or Unreal Engine with AirSim plugin)
- Required Python packages (see requirements.txt)

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