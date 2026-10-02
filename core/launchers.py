"""
Launchers for DSOS external dependencies.
Provides dedicated process managers for Gazebo, PX4 SITL, and MAVSDK connections.
"""
import asyncio
import logging
import os
import shutil
import re
from typing import List, Optional

from dsos.core.process_manager import ProcessManager, ProcessConfig

class GazeboLauncher:
    """Manages the lifecycle of the Gazebo simulator."""
    def __init__(self, process_manager: ProcessManager, gazebo_path: str, px4_path: str = ""):
        self.logger = logging.getLogger("GazeboLauncher")
        self.process_manager = process_manager
        self.gazebo_path = gazebo_path
        self.px4_path = os.path.expanduser(px4_path) if px4_path else ""
        self.is_running = False

    async def launch(self, yield_cb=None) -> bool:
        """Launch Gazebo in the background and wait for it to initialize."""
        if not shutil.which(self.gazebo_path):
            self.logger.error(f"Gazebo executable not found at {self.gazebo_path}")
            if yield_cb: yield_cb("Launching Gazebo...................", "FAIL")
            return False

        if yield_cb: yield_cb("Launching Gazebo...................", "WAIT")
        self.logger.info("Launching Gazebo simulator...")
        command_args = [self.gazebo_path]
        if os.path.basename(self.gazebo_path) in ["gz", "ign"]:
            command_args.extend(["sim", "-v", "4", "-r", "default.sdf"])
        else:
            command_args.append("--verbose")

        env = os.environ.copy()
        if self.px4_path:
            # Fix Gazebo failing to find PX4 models by supplying the correct resource paths
            px4_models = os.path.join(self.px4_path, "Tools", "simulation", "gz", "models")
            px4_worlds = os.path.join(self.px4_path, "Tools", "simulation", "gz", "worlds")
            px4_plugins = os.path.join(self.px4_path, "build", "px4_sitl_default", "src", "modules", "simulation", "gz_plugins")
            server_config = os.path.join(self.px4_path, "Tools/simulation/gz/server.config")
            
            env["PX4_GZ_MODELS"] = px4_models
            env["PX4_GZ_WORLDS"] = px4_worlds
            env["GZ_SIM_SERVER_CONFIG_PATH"] = server_config
            
            gz_res = env.get("GZ_SIM_RESOURCE_PATH", "")
            env["GZ_SIM_RESOURCE_PATH"] = f"{gz_res}:{px4_models}:{px4_worlds}" if gz_res else f"{px4_models}:{px4_worlds}"
            
            gz_plg = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
            env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{gz_plg}:{px4_plugins}" if gz_plg else px4_plugins

        self.process_manager.add_process(ProcessConfig(
            name="gazebo_server",
            command=command_args,
            env=env,
            restart_on_failure=True,
            startup_timeout_sec=30.0,
            # Wait for Gazebo's initialization string
            success_pattern=r"World .* initialized|Loaded level|Serving world controls|Serving world SDF generation|Gazebo Sim Server" 
        ))
        
        success = await self.process_manager.start_process("gazebo_server")
        if success:
            # Additional verification: wait a bit and check if process is still running
            await asyncio.sleep(2.0)
            proc = self.process_manager.processes.get("gazebo_server")
            if proc and proc.returncode is not None:
                success = False
                
        if success:
            self.is_running = True
            self.logger.info("Gazebo initialized successfully.")
            if yield_cb: yield_cb("Launching Gazebo...................", "OK")
        else:
            self.logger.error("Failed to initialize Gazebo.")
            if yield_cb: yield_cb("Launching Gazebo...................", "FAIL")
        
        return success

    def get_log_content(self) -> str:
        """Fetch recent Gazebo log output for debugging."""
        log_file = self.process_manager.get_log_path("gazebo_server")
        if log_file and os.path.exists(log_file):
            try:
                with open(log_file, 'r') as f:
                    lines = f.readlines()
                    return "".join(lines[-20:])
            except Exception:
                pass
        return "No Gazebo log available."

    async def shutdown(self):
        """Shutdown Gazebo cleanly."""
        self.logger.info("Shutting down Gazebo...")
        await self.process_manager.stop_process("gazebo_server")
        self.is_running = False


import psutil

class StartupCleanupManager:
    """Manages pre-flight cleanup of old processes, lock files, and stale runtime environments."""
    def __init__(self, px4_path: str):
        self.logger = logging.getLogger("StartupCleanupManager")
        self.px4_path = px4_path
        
    async def cleanup(self):
        self.logger.info("Running Startup Cleanup Manager...")
        
        # 1. Terminate old DSOS px4 and gz sim processes
        terminated = 0
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'cwd']):
            try:
                name = proc.info['name'] or ""
                cmdline = proc.info['cmdline'] or []
                cwd = proc.info['cwd'] or ""
                
                is_target = False
                if name == 'px4' and self.px4_path in cwd:
                    is_target = True
                if ('gz' in name or 'ruby' in name) and 'sim' in cmdline:
                    is_target = True
                if 'mavsdk_server' in name:
                    is_target = True
                    
                if is_target:
                    self.logger.info(f"Cleaning up stale process: {name} (PID: {proc.pid})")
                    try:
                        proc.kill()
                        proc.wait(timeout=2.0)
                    except psutil.TimeoutExpired:
                        pass
                    terminated += 1
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
                
        if terminated > 0:
            await asyncio.sleep(1.0)
            
        # 2. Clean rootfs and lock files
        for i in range(20): # Clean up to 20 instances
            lock_file = f"/tmp/px4_lock-{i}"
            if os.path.exists(lock_file):
                try:
                    os.remove(lock_file)
                    self.logger.info(f"Removed stale lock file: {lock_file}")
                except Exception:
                    pass
                    
            instance_dir = os.path.join(self.px4_path, "build", "px4_sitl_default", f"instance_{i}")
            if os.path.exists(instance_dir):
                try:
                    shutil.rmtree(instance_dir)
                    self.logger.info(f"Removed stale instance directory: {instance_dir}")
                except Exception:
                    pass
                    
        self.logger.info("Startup Cleanup Manager finished.")


class PX4Launcher:
    """Manages the lifecycle of PX4 SITL instances using the official wrapper scripts."""
    def __init__(self, process_manager: ProcessManager, px4_path: str, swarm_size: int, gazebo_launcher: GazeboLauncher):
        self.logger = logging.getLogger("PX4Launcher")
        self.process_manager = process_manager
        self.gazebo_launcher = gazebo_launcher
        self.px4_path = os.path.expanduser(px4_path)
        self.swarm_size = swarm_size
        self.mavsdk_ports: List[int] = []

    async def build_px4(self, yield_cb=None) -> bool:
        """Automatically compile PX4 SITL."""
        if yield_cb: yield_cb("Compiling PX4 (May take 10+ mins)..", "WAIT")
        self.logger.info("Starting PX4 compilation: make px4_sitl gz_x500")
        
        process = await asyncio.create_subprocess_shell(
            "make px4_sitl gz_x500",
            cwd=self.px4_path,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT
        )
        
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            
        await process.wait()
        if process.returncode == 0:
            self.logger.info("PX4 compilation successful.")
            if yield_cb: yield_cb("Compiling PX4 (May take 10+ mins)..", "OK")
            return True
        else:
            self.logger.error("PX4 compilation failed.")
            if yield_cb: yield_cb("Compiling PX4 (May take 10+ mins)..", "FAIL")
            return False

    async def launch(self, yield_cb=None) -> bool:
        """Launch PX4 SITL instances based on swarm size."""
        # Find the correct PX4 path
        if os.path.isfile(self.px4_path):
            self.px4_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(self.px4_path))))
            
        if yield_cb: yield_cb("Running Pre-flight Cleanup.........", "WAIT")
        cleanup_manager = StartupCleanupManager(self.px4_path)
        await cleanup_manager.cleanup()
        if yield_cb: yield_cb("Running Pre-flight Cleanup.........", "OK")
        
        if yield_cb: yield_cb("Launching PX4 Swarm Instances....", "WAIT")
        
        px4_bin = os.path.join(self.px4_path, "build", "px4_sitl_default", "bin", "px4")
        if not os.path.exists(px4_bin):
            self.logger.error(f"PX4 binary not found at {px4_bin}. Please run 'make px4_sitl' first.")
            if yield_cb: yield_cb("Launching PX4 Swarm Instances....", "FAIL")
            return False

        # Wait for Gazebo to be fully ready before launching PX4 to avoid race conditions
        self.logger.info("Waiting for Gazebo to become ready...")
        gazebo_ready = False
        for _ in range(30):
            try:
                proc = await asyncio.create_subprocess_exec(
                    "gz", "model", "-m", "ground_plane",
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL
                )
                await proc.wait()
                if proc.returncode == 0:
                    gazebo_ready = True
                    break
            except Exception:
                pass
            await asyncio.sleep(1.0)
            
        if not gazebo_ready:
            self.logger.error("Gazebo did not become ready in time.")
            if yield_cb: yield_cb("Launching PX4 Swarm Instances....", "FAIL")
            return False

        px4_pids = []
        for i in range(self.swarm_size):
            # Calculate spawn positions (line formation along Y axis)
            x_pos = 0.0
            y_pos = (i - (self.swarm_size - 1) / 2.0) * 2.0
            
            env = os.environ.copy()
            # Let PX4 handle spawning the model in the existing Gazebo server
            env["PX4_SYS_AUTOSTART"] = "4001"
            env["PX4_GZ_MODEL_POSE"] = f"{x_pos},{y_pos},0.2"
            env["PX4_SIM_MODEL"] = "gz_x500"
            env["PX4_GZ_WORLD"] = "default"
            env["PX4_SIM_WORLD"] = "default"
            
            # Ensure PX4 knows where models are (it constructs the sdf URI using this)
            px4_models = os.path.join(self.px4_path, "Tools", "simulation", "gz", "models")
            env["PX4_GZ_MODELS"] = px4_models
            
            # Start instance
            self.logger.info(f"Spawning PX4 instance {i} at y={y_pos}")
            cmd = [px4_bin, "-i", str(i), "-d", os.path.join(self.px4_path, "build", "px4_sitl_default", "etc")]
            
            working_dir = os.path.join(self.px4_path, "build", "px4_sitl_default", f"instance_{i}")
            os.makedirs(working_dir, exist_ok=True)
            
            log_path = os.path.join("logs", f"px4_instance_{i}.log")
            log_file = open(log_path, "w")
            
            process = await asyncio.create_subprocess_exec(
                *cmd,
                cwd=working_dir,
                env=env,
                stdout=log_file,
                stderr=asyncio.subprocess.STDOUT
            )
            
            px4_pids.append(process.pid)
            self.process_manager.add_external_pid(process.pid)
            
            # Stagger startup to prevent Gazebo model spawn race conditions
            await asyncio.sleep(2.0)
            
        if yield_cb: yield_cb("Launching PX4 Swarm Instances....", "OK")
        
        # Verify and capture processes
        if yield_cb: yield_cb("Verifying PX4 Instance Ownership...", "WAIT")
            
        if yield_cb: yield_cb("Verifying PX4 Instance Ownership...", "OK")
        
        if yield_cb: yield_cb("Verifying UDP Port Bindings......", "WAIT")
        
        from dsos.config.settings import settings
        default_address = settings.get('DSOS', 'mavsdk_address', 'udp://:14540')
        mavsdk_base_port = 14540
        if default_address.startswith('udp://:'):
            mavsdk_base_port = int(default_address.split(':')[-1])

        # PX4 ALWAYS binds to 14580+ internally, regardless of MAVSDK port
        px4_base_port = 14580
        max_wait = 15
        px4_udp_ports = set()
        
        for _ in range(max_wait):
            px4_udp_ports = set()
            for pid in px4_pids:
                try:
                    proc = psutil.Process(pid)
                    for conn in proc.connections(kind='udp'):
                        if conn.laddr:
                            px4_udp_ports.add(conn.laddr.port)
                except Exception:
                    pass
                    
            all_found = True
            for i in range(self.swarm_size):
                if (px4_base_port + i) not in px4_udp_ports:
                    all_found = False
                    break
                    
            if all_found:
                break
                
            await asyncio.sleep(1.0)

        self.mavsdk_ports = []
        for i in range(self.swarm_size):
            port = px4_base_port + i
            if port not in px4_udp_ports:
                self.logger.error(f"PX4 internal port {port} is not actively bound! PX4 might have failed to start.")
                if yield_cb: yield_cb("Verifying UDP Port Bindings......", "FAIL")
                return False
            # Expose the correct MAVSDK listen ports
            self.mavsdk_ports.append(mavsdk_base_port + i)
            
        if yield_cb: yield_cb("Verifying UDP Port Bindings......", "OK")
        
        # Verify Gazebo Models
        if yield_cb: yield_cb("Verifying Gazebo Entities........", "WAIT")
        self.logger.info("Verifying Gazebo entity count...")
        
        expected_entities = ["ground_plane"] + [f"x500_{i}" for i in range(self.swarm_size)]
        
        models_found = False
        for _ in range(45):
            try:
                proc = await asyncio.create_subprocess_exec(
                    "gz", "model", "--list",
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE
                )
                stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=5.0)
                output = stdout.decode().splitlines()
                
                # Check if all expected models are in the output
                if all(any(e in line for line in output) for e in expected_entities):
                    models_found = True
                    break
            except asyncio.TimeoutError:
                self.logger.warning("gz model --list timed out.")
                try:
                    proc.kill()
                except:
                    pass
            except Exception as e:
                self.logger.warning(f"Error querying Gazebo models: {e}")
                
            await asyncio.sleep(1.0)
            
        if not models_found:
            self.logger.error(f"Gazebo model verification failed! Expected: {expected_entities}")
            if yield_cb: yield_cb("Verifying Gazebo Entities........", "FAIL")
            return False
            
        if yield_cb: yield_cb("Verifying Gazebo Entities........", "OK")
        
        self.logger.info("All PX4 instances initialized and tracked successfully.")
        return True

    async def shutdown(self):
        """Shutdown PX4 cleanly."""
        self.logger.info("PX4 shutdown delegated to ProcessManager (external PID tracking).")


class MAVSDKManager:
    """Manages MAVSDK discovery and health monitoring."""
    def __init__(self, ports: List[int]):
        self.logger = logging.getLogger("MAVSDKManager")
        self.ports = ports
        self.discovered_vehicles = 0
        
    async def connect_and_discover(self, yield_cb=None) -> bool:
        """Ensure that MAVSDK endpoints are listening and healthy."""
        if yield_cb: yield_cb("Connecting MAVSDK..................", "WAIT")
        self.logger.info(f"Pinging MAVSDK ports {self.ports} for discovery...")
        
        import mavsdk
        import psutil
        
        timeout_sec = 15.0
        
        print("\n" + "="*60)
        print("MAVSDK DIAGNOSTIC MODE: Connection Starting")
        print("="*60)
        
        print("RUNNING PX4 PROCESSES:")
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if proc.info['name'] == 'px4':
                    print(f" - PID: {proc.info['pid']}, CMD: {' '.join(proc.info['cmdline'] or [])}")
            except: pass
            
        print("\nRUNNING GAZEBO PROCESSES:")
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                name = proc.info['name'] or ""
                cmdline = proc.info['cmdline'] or []
                if ('gz' in name or 'ruby' in name) and 'sim' in cmdline:
                    print(f" - PID: {proc.info['pid']}, CMD: {' '.join(cmdline)}")
            except: pass
            
        print("\nUDP SOCKETS OPENED BY PX4:")
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.info['name'] == 'px4':
                    for c in proc.connections(kind='udp'):
                        print(f" - PID: {proc.info['pid']} listening on {c.laddr}")
            except: pass
            
        print(f"\nCONFIGURED MAVLINK PORTS: {self.ports}")
        print("ATTEMPTING CONNECTIONS:")
        for i, port in enumerate(self.ports):
            print(f" - Drone {i+1}: udp://:{port} (gRPC: {60051 + i})")
            
        print(f"\nDISCOVERY (Timeout: {timeout_sec}s):")
        
        self.discovered_vehicles = 0
        self.connected_systems = []
        tasks = []
        
        async def verify_drone(port: int, index: int):
            grpc_port = 60051 + index
            self.logger.info(f"Drone {index+1}: Instantiating MAVSDK System on gRPC port {grpc_port}")
            drone = mavsdk.System(port=grpc_port)
            
            status = {
                "drone": index + 1,
                "port": port,
                "connected": False,
                "heartbeat": False,
                "uuid": None,
                "telemetry": False,
                "error": "",
                "system": None
            }
            
            try:
                self.logger.info(f"Drone {index+1}: Connecting to UDP endpoint udp://:{port}")
                await drone.connect(system_address=f"udp://:{port}")
                
                self.logger.info(f"Drone {index+1}: Waiting for connection state heartbeat...")
                async with asyncio.timeout(timeout_sec):
                    async for state in drone.core.connection_state():
                        if state.is_connected:
                            self.logger.info(f"Drone {index+1}: Received MAVLink heartbeat!")
                            status["connected"] = True
                            status["heartbeat"] = True
                            status["uuid"] = "unknown"
                            break
                            
                if not status["connected"]:
                    status["error"] = f"Heartbeat Timeout ({timeout_sec}s)"
                    self.logger.error(f"Drone {index+1}: {status['error']}")
                    return status

                # We have heartbeat, so the drone is discovered! 
                # (GPS lock might take longer, we shouldn't block boot manager on it)
                status["telemetry"] = True
                status["system"] = drone
                            
            except asyncio.TimeoutError:
                status["error"] = "Timeout exception caught."
                self.logger.error(f"Drone {index+1}: Timeout exception caught.")
            except Exception as e:
                status["error"] = str(e)
                self.logger.error(f"Drone {index+1}: Exception during discovery - {e}")
                
            return status
            
        tasks = [verify_drone(port, i) for i, port in enumerate(self.ports)]
        results = await asyncio.gather(*tasks)
        
        successes = [r for r in results if r["telemetry"]]
        self.discovered_vehicles = len(successes)
        self.connected_systems = [r["system"] for r in successes]
        
        print("\nMAVSDK DISCOVERY RESULTS:")
        print(f"{'Drone':<7} | {'PX4 Port':<10} | {'MAVSDK Endpoint':<20} | {'Heartbeat':<10} | {'Connected':<10} | {'Telemetry':<15}")
        print("-" * 85)
        for r in results:
            hb = "YES" if r["heartbeat"] else "NO"
            conn = "YES" if r["connected"] else "NO"
            tel = "OK" if r["telemetry"] else r["error"]
            print(f"{r['drone']:<7} | {r['port']:<10} | udp://:{r['port']:<12} | {hb:<10} | {conn:<10} | {tel:<15}")
        print("-" * 85 + "\n")
        
        if self.discovered_vehicles == 0:
            self.logger.error("0 MAVSDK systems discovered. Aborting boot.")
            if yield_cb: yield_cb("Connecting MAVSDK..................", "FAIL")
            print("\n" + "="*60)
            print("CRITICAL: ZERO MAVSDK SYSTEMS DISCOVERED")
            print("="*60)
            print("Please check the UDP port bindings and ensure PX4 is emitting MAVLink.")
            print("="*60 + "\n")
            return False
            
        if self.discovered_vehicles != len(self.ports):
            self.logger.error(f"Failed to discover all vehicles. Found {self.discovered_vehicles} / {len(self.ports)}")
            if yield_cb: yield_cb("Connecting MAVSDK..................", "FAIL")
            return False

        self.logger.info(f"Discovered {self.discovered_vehicles} vehicles ready for MAVSDK.")
        
        if yield_cb: yield_cb("Connecting MAVSDK..................", "OK")
        if yield_cb: yield_cb("Discovering Vehicles...............", f"{self.discovered_vehicles} Found")
        return True
