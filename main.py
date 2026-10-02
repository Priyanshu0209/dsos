#!/usr/bin/env python3
"""
Main entry point for the Drone Swarm Operating System (DSOS).
Features automated boot sequence, process management, and HAL backend integration.
"""

import sys
import os
# Add parent directory to python path to resolve 'dsos' module
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    os.environ['PYTHONPATH'] = parent_dir + os.pathsep + os.environ.get('PYTHONPATH', '')

# Ensure we are running in the venv
VENV_PYTHON = os.path.join(parent_dir, "dsos", "venv", "bin", "python")
if sys.executable != VENV_PYTHON and os.path.exists(VENV_PYTHON):
    print("Restarting in virtual environment...")
    os.execl(VENV_PYTHON, VENV_PYTHON, *sys.argv)

import asyncio
import logging
import signal
import threading
from typing import List, Optional
# Import DSOS modules
from dsos.config.settings import settings
from dsos.control.backends import create_backend, BackendType
from dsos.agents.drone_agent import DroneAgent
from dsos.ui.gcs import GCSCommand
from dsos.core.boot_manager import BootManager
from dsos.ui.gcs_dashboard import GCSDashboard
from dsos.swarm.swarm_manager import SwarmManager
from dsos.core.dispatcher import CommandDispatcher


class DSOS:
    """Main DSOS system controller."""

    def __init__(self):
        self.logger = logging.getLogger("dsos")
        self.drones: List[DroneAgent] = []
        self.gcs: Optional[GCSCommand] = None
        self.running = False
        self._shutdown_event = asyncio.Event()
        self.backends: List = []  # List of connected backends
        self.boot_manager = BootManager()

    async def initialize(self) -> bool:
        """Initialize the DSOS system."""
        self.dashboard = GCSDashboard()
        self.dashboard.print_header()
        
        self.logger.info("Initializing Drone Swarm Operating System...")

        # Run automatic boot sequence with live dashboard updates
        if not await self.boot_manager.boot(status_callback=self.dashboard.update_status):
            self.logger.error("Boot sequence failed. Aborting startup.")
            self.dashboard.finish(success=False)
            return False

        # Load configuration
        swarm_size = settings.getint('DSOS', 'swarm_size', 3)
        simulation_mode = settings.getboolean('DSOS', 'simulation_mode', True)
        backend_type_str = settings.get('DSOS', 'backend_type', 'simulation').lower()
        if simulation_mode and backend_type_str == 'simulation':
            backend_type_str = 'gazebo'

        self.logger.info(f"Swarm size: {swarm_size}")
        self.logger.info(f"Backend type: {backend_type_str}")

        # Connect backends and discover drones first
        self.dashboard.update_status("Initializing Swarm.................", "WAIT")
        
        connected_systems = []
        if self.boot_manager and getattr(self.boot_manager, 'mavsdk_manager', None):
            connected_systems = self.boot_manager.mavsdk_manager.connected_systems
            
        actual_swarm_size = len(connected_systems) if connected_systems else swarm_size
        
        if actual_swarm_size == 0 and backend_type_str in ('mavsdk', 'px4_sitl', 'gazebo'):
            self.logger.error("No drones discovered. Aborting swarm initialization.")
            return False

        for i in range(actual_swarm_size):
            drone_id = f"drone_{i+1}"
            backend = None
            
            try:
                backend_type = BackendType(backend_type_str)
                backend_config = self._get_backend_config(backend_type, i)
                if connected_systems:
                    backend_config['preconnected_system'] = connected_systems[i]
                backend = create_backend(backend_type.value, backend_config)
                if backend:
                    self.backends.append(backend)
            except ValueError:
                self.logger.error(f"Unknown backend type: {backend_type_str}")
                return False

            drone = DroneAgent(drone_id, backend)
            self.drones.append(drone)

        for backend in self.backends:
            success = await backend.connect()
            if not success:
                self.logger.error(f"Failed to connect to backend: {backend.__class__.__name__}")
                self.dashboard.finish(success=False)
                return False

        self.dashboard.update_status("Initializing Swarm.................", "OK")

        # Start all drones (Initializes HAL, Comms)
        self.dashboard.update_status("Initializing Telemetry.............", "WAIT")
        for drone in self.drones:
            await drone.start()
        self.dashboard.update_status("Initializing Telemetry.............", "OK")
        
        self.dashboard.update_status("Initializing AI....................", "OK")
        self.dashboard.update_status("Initializing Missions..............", "OK")

        self.swarm_manager = SwarmManager(self.drones)
        self.dispatcher = CommandDispatcher(self.swarm_manager)

        # Initialize Ground Control Station
        self.gcs = GCSCommand(dashboard=self.dashboard, dispatcher=self.dispatcher)
        self.gcs.drones = self.drones
        self.gcs.backends = self.backends
        
        if self.backends:
            self.gcs.active_backend = self.backends[0]
            for i, b in enumerate(self.backends):
                self.gcs.available_backends[f"{backend_type_str}_{i}"] = b
            
        self.logger.info("Initialized Ground Control Station")
        self.dashboard.finish(success=True)
        return True

    def _get_backend_config(self, backend_type: BackendType, drone_index: int) -> dict:
        """Get configuration for a specific backend type."""
        config = {}

        if backend_type == BackendType.AIRSIM:
            config = {
                'host': settings.get('DSOS', 'airsim_host', 'localhost'),
                'port': settings.getint('DSOS', 'airsim_port', 41451) + drone_index,
            }
        elif backend_type == BackendType.ROS2:
            config = {
                'node_name': f"{settings.get('DSOS', 'ros2_node_name', 'dsos_controller')}_{drone_index}",
            }
        elif backend_type in (BackendType.GAZEBO, BackendType.MAVSDK, BackendType.PX4_SITL):
            # Check if BootManager generated a dynamic port, else use default + index
            if self.boot_manager.dynamic_mavsdk_ports and drone_index < len(self.boot_manager.dynamic_mavsdk_ports):
                port = self.boot_manager.dynamic_mavsdk_ports[drone_index]
                address = f"udp://:{port}"
            else:
                default_address = settings.get('DSOS', 'mavsdk_address', 'udp://:14540')
                if default_address.startswith('udp://:'):
                    base_port = int(default_address.split(':')[-1])
                    address = f"udp://:{base_port + drone_index}"
                else:
                    address = default_address # Serial or other
            config = {
                'system_address': address,
                'vehicle_id': f"drone_{drone_index+1}",
                'grpc_port': 50051 + drone_index,
            }
        elif backend_type == BackendType.REAL_DRONE:
            config = {
                'connection_string': settings.get('DSOS', 'real_drone_connection', ''),
            }

        return config

    async def start(self) -> None:
        """Start the DSOS system."""
        if self.running:
            return

        self.logger.info("Starting DSOS system...")
        if not await self.initialize():
            self._shutdown_event.set()
            return

        self.running = True
        self.logger.info("DSOS system started")

    async def stop(self) -> None:
        """Stop the DSOS system."""
        if not self.running:
            return

        self.logger.info("Stopping DSOS system...")
        self.running = False

        # Safely land/hover drones before shutting down
        self.logger.info("Commanding all drones to land safely...")
        for drone in self.drones:
            try:
                # Disarm/land the drone
                if hasattr(drone, 'land'):
                    await drone.land()
                if hasattr(drone, 'shutdown'):
                    await drone.shutdown()
            except Exception as e:
                self.logger.error(f"Error stopping drone: {e}")

        # Stop GCS
        if self.gcs:
            self.gcs.stop()

        # Disconnect all backends
        for backend in self.backends:
            if hasattr(backend, 'disconnect'):
                try:
                    await backend.disconnect()
                except Exception as e:
                    self.logger.error(f"Error disconnecting backend: {e}")

        # Shutdown BootManager and background processes
        await self.boot_manager.shutdown()

        self._shutdown_event.set()
        self.logger.info("DSOS system stopped")

    def run(self) -> None:
        """Run the DSOS system (blocking)."""
        # Setup signal handlers for graceful shutdown
        def signal_handler(sig, frame):
            print("\nShutting down...")
            if not self._shutdown_event.is_set():
                try:
                    loop = asyncio.get_running_loop()
                except RuntimeError:
                    loop = None

                if loop and loop.is_running():
                    loop.create_task(self.stop())
                else:
                    asyncio.run(self.stop())

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Run the async event loop
        try:
            asyncio.run(self._async_run())
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            pass # Cleanup handled by signal handlers or asyncio loop

    async def _async_run(self) -> None:
        """Async run method."""
        await self.start()

        if not self.running:
            return

        # Start Textual GCS App
        if self.gcs:
            from dsos.ui.textual_gcs import TextualGCSApp
            app = TextualGCSApp(self.gcs)
            
            # Textual takes over the UI and blocks until closed
            await app.run_async()
            
            # Shut down everything when UI closes
            await self.stop()


def main():
    """Main entry point."""
    # Setup logging
    os.makedirs("logs", exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler("logs/dsos.log")
            # Removed StreamHandler to prevent TUI corruption
        ]
    )

    # Create and run DSOS
    dsos = DSOS()
    try:
        dsos.run()
    except Exception as e:
        logging.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    main()