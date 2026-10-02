"""
Boot Manager for DSOS.
Orchestrates the automatic startup sequence of all external dependencies
using dedicated launchers and strict failure policies.
"""
import sys
import os
import asyncio
import logging
import shutil
import signal
from typing import Optional, List, Dict, Callable

from dsos.config.settings import settings
from dsos.core.process_manager import ProcessManager, ProcessConfig
from dsos.core.launchers import GazeboLauncher, PX4Launcher, MAVSDKManager

class BootManager:
    """Manages the startup sequence of the entire DSOS ecosystem."""

    def __init__(self):
        self.logger = logging.getLogger("BootManager")
        self.process_manager = ProcessManager(critical_failure_callback=self._on_critical_failure)
        self.swarm_size = settings.getint('DSOS', 'swarm_size', 3)
        self.backend_type = settings.get('DSOS', 'backend_type', 'simulation').lower()
        self.simulation_mode = settings.getboolean('DSOS', 'simulation_mode', True)
        
        # We force auto_start to True if we are in simulation mode as per the new plan
        self.auto_start = settings.getboolean('DSOS', 'auto_start_simulation', True)
        self.launch_qgc = settings.getboolean('DSOS', 'launch_qgc', True)
        
        self.px4_path = settings.get('DSOS', 'px4_path', 'px4')
        self.gazebo_path = settings.get('DSOS', 'gazebo_path', 'gzserver')
        self.qgc_path = settings.get('DSOS', 'qgc_path', 'QGroundControl')
        
        # When simulation mode is True, force real backend behavior
        if self.simulation_mode and self.backend_type == 'simulation':
            self.backend_type = 'gazebo' # Default to full gazebo sitl for the redesign
            
        self.gazebo_launcher = GazeboLauncher(self.process_manager, self.gazebo_path, self.px4_path)
        self.px4_launcher = PX4Launcher(self.process_manager, self.px4_path, self.swarm_size, self.gazebo_launcher)
        self.mavsdk_manager: Optional[MAVSDKManager] = None
        
        self.dynamic_mavsdk_ports: List[int] = []

    async def boot(self, status_callback: Optional[Callable[[str, str], None]] = None) -> bool:
        """Execute the complete boot sequence."""
        self.logger.info("DSOS Automatic Boot Sequence Started")
        
        def cb(step, status):
            if status_callback:
                status_callback(step, status)
                
        cb("Checking Dependencies..............", "WAIT")
        
        # 1. Verify Python Version
        if sys.version_info < (3, 12):
            self.logger.error("Python 3.12+ is required.")
            cb("Checking Dependencies..............", "FAIL")
            return False
            
        os.makedirs("logs", exist_ok=True)
        cb("Checking Dependencies..............", "OK")

        if not self.auto_start:
            self.logger.info("Auto-start disabled. Skipping external dependencies.")
            return True

        if self.backend_type in ['px4_sitl', 'gazebo']:
            self.logger.info("Starting explicit Gazebo and PX4 SITL boot sequence.")
            
            # 1. Launch Gazebo explicitly
            if not await self.gazebo_launcher.launch(cb):
                return False
            
            # 2. Launch PX4 instances (must be configured to NOT launch Gazebo)
            if not await self.px4_launcher.launch(cb):
                return False
                
            self.dynamic_mavsdk_ports = self.px4_launcher.mavsdk_ports
            self.mavsdk_manager = MAVSDKManager(self.dynamic_mavsdk_ports)
            
        if self.mavsdk_manager:
            if not await self.mavsdk_manager.connect_and_discover(cb):
                return False

        # Launch QGroundControl
        if self.launch_qgc:
            cb("Launching QGroundControl...........", "WAIT")
            if not shutil.which(self.qgc_path) and not os.path.exists(self.qgc_path):
                self.logger.warning(f"QGroundControl executable not found: {self.qgc_path}.")
                cb("Launching QGroundControl...........", "FAIL")
                # We won't strictly fail the whole system if QGC is missing, just mark it failed
            else:
                self.process_manager.add_process(ProcessConfig(
                    name="qgroundcontrol",
                    command=[self.qgc_path],
                    restart_on_failure=False
                ))
                success = await self.process_manager.start_process("qgroundcontrol")
                if success:
                    cb("Launching QGroundControl...........", "OK")
                else:
                    cb("Launching QGroundControl...........", "FAIL")
            
            # End QGC Launch

        # Process Verification
        import psutil
        gz_count = 0
        px4_count = 0
        qgc_count = 0
        
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                name = proc.info['name'] or ""
                cmdline = proc.info['cmdline'] or []
                cmd_str = " ".join(cmdline).lower()
                
                if 'gz' in name or ('ruby' in name and 'sim' in cmd_str):
                    gz_count += 1
                elif 'px4' in name and 'bin/px4' in cmd_str:
                    px4_count += 1
                elif 'qgroundcontrol' in name.lower() or 'qgroundcontrol' in cmd_str:
                    qgc_count += 1
            except Exception:
                pass

                
        mav_count = self.mavsdk_manager.discovered_vehicles if hasattr(self.mavsdk_manager, 'discovered_vehicles') else 0
        
        print("\n" + "="*40)
        print("PROCESS VERIFICATION SUMMARY")
        print("="*40)
        print(f"Gazebo Processes: {gz_count}")
        print(f"PX4 Processes: {px4_count}")
        print(f"QGroundControl Processes: {qgc_count}")
        print(f"MAVSDK Connections: {mav_count}")
        print("="*40 + "\n")
        
        if px4_count != self.swarm_size or mav_count != self.swarm_size:
            self.logger.error("Architecture Verification Failed! Incorrect number of PX4/MAVSDK instances.")
            print(f"Found {px4_count} PX4 instances! Exactly {self.swarm_size} is required.")
            print(f"Found {mav_count} MAVSDK connections! Exactly {self.swarm_size} is required.")
            cb("Architecture Verification..........", "FAIL")
            return False
            
        if gz_count == 0:
            print("Warning: Gazebo process not detected by psutil, but continuing anyway.")
            
        cb("Architecture Verification..........", "OK")
        self.logger.info("Boot sequence completed successfully.")
        return True

    async def shutdown(self):
        """Shutdown all started services."""
        self.logger.info("Initiating BootManager shutdown...")
        await self.process_manager.stop_all()
        self.logger.info("BootManager shutdown complete.")

    def _on_critical_failure(self, process_name: str):
        """Callback for when a critical process dies unexpectedly."""
        self.logger.critical(f"CRITICAL: {process_name} has crashed. Initiating emergency shutdown.")
        # In a real environment we would signal main.py, here we can just stop process manager
        # Main.py should ideally monitor boot_manager state
        asyncio.create_task(self.shutdown())
        # Let the application know to exit
        os.kill(os.getpid(), signal.SIGINT)
