"""
Ground Control Station (GCS) for DSOS.
Provides a terminal-based command interface for controlling the drone swarm.
Now includes backend management capabilities.
"""
import cmd
import shlex
import threading
import time
import asyncio
from typing import Dict, List, Optional, Any
import logging

from dsos.ui.gcs_dashboard import GCSDashboard, ANSI

# Import DSOS modules
try:
    from dsos.config.settings import settings
    from dsos.agents.drone_agent import DroneAgent
    from dsos.control.backends import BackendType, create_backend
    from dsos.missions.mission_manager import MissionManager, Mission, MissionType, Waypoint
    from dsos.formations.formation_engine import FormationManager, FormationType
    from dsos.navigation.navigation_manager import NavigationManager
    from dsos.sensors.sensor_manager import SensorManager
    from dsos.ai.decision_maker import AIDecisionMaker
    from dsos.telemetry.telemetry_manager import TelemetryManager
    from dsos.core.boot_manager import BootManager
except ImportError as e:
    print(f"Warning: Could not import some DSOS modules: {e}")
    # We'll create mock classes for demonstration
    pass


class GCSCommand(cmd.Cmd):
    """Ground Control Station command interpreter."""

    intro = ''
    prompt = f'{ANSI.GREEN}{ANSI.BOLD}(DSOS-GCS){ANSI.RESET} '
    file = None

    def __init__(self, dashboard: Optional[GCSDashboard] = None, dispatcher: Optional[Any] = None):
        super().__init__()
        self.logger = logging.getLogger("gcs")
        self.drones: List[Any] = []
        self.selected_drones: List[str] = []
        self.running = False
        self._command_thread: Optional[threading.Thread] = None
        self.dashboard = dashboard
        self.dispatcher = dispatcher
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = None

        # Initialize managers (these would be connected to actual drone instances)
        self.mission_manager = MissionManager("gcs")
        self.formation_manager = FormationManager("gcs")
        self.navigation_manager = NavigationManager("gcs")
        self.sensor_manager = SensorManager("gcs")
        self.ai_decision_maker = AIDecisionMaker("gcs")
        self.telemetry_manager = TelemetryManager("gcs")

        # Backend management
        self.available_backends = {}
        self.active_backend: Optional[Any] = None

        # Boot Manager
        self.boot_manager = BootManager()

        # Load configuration
        self.swarm_size = settings.getint('DSOS', 'swarm_size', 3)
        self.simulation_mode = settings.getboolean('DSOS', 'simulation_mode', True)
        self.backend_type_str = settings.get('DSOS', 'backend_type', 'simulation').lower()

    def preloop(self):
        """Called before the command loop starts."""
        if self.dashboard:
            self.dashboard.print_drone_table(self.drones)
        else:
            print(f"{ANSI.CYAN}{ANSI.BOLD}Welcome to the Drone Swarm Operating System Ground Control Station.{ANSI.RESET}")
            print(f"Type {ANSI.YELLOW}help{ANSI.RESET} or {ANSI.YELLOW}?{ANSI.RESET} to list commands.\n")

    def postcmd(self, stop, line):
        """Called after each command to refresh the dashboard."""
        if not stop and self.dashboard and line.strip() and line.strip().lower() not in ['help', '?', 'clear']:
            # Short sleep to let background threads update state before rendering
            time.sleep(0.1)
            self.dashboard.print_drone_table(self.drones)
        return stop

    def emptyline(self):
        """Do nothing on empty line."""
        if self.dashboard:
            self.dashboard.print_drone_table(self.drones)
        pass

    def do_clear(self, arg):
        """Clear the terminal screen and reprint the dashboard."""
        if self.dashboard:
            self.dashboard.clear_screen()
            self.dashboard.print_drone_table(self.drones)
        else:
            print("\033[2J\033[H", end="")

    def do_boot(self, arg):
        """Execute the automatic boot sequence (launch Gazebo, PX4 SITL, etc.)."""
        print("Starting DSOS Boot Sequence...")
        
        def run_boot():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                success = loop.run_until_complete(self.boot_manager.boot())
                if success:
                    print("Boot sequence completed successfully.")
                else:
                    print("Boot sequence failed.")
            except Exception as e:
                print(f"Boot error: {e}")
            finally:
                loop.close()

        threading.Thread(target=run_boot, daemon=True).start()
        
    def do_discover(self, arg):
        """Discover drones via the active backend."""
        if not self.active_backend:
            print("No active backend configured for discovery.")
            return

        print("Discovering drones...")
        
        def run_discover():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                vehicles = loop.run_until_complete(self.active_backend.get_vehicles())
                print(f"Discovered {len(vehicles)} vehicles: {vehicles}")
            except Exception as e:
                print(f"Discovery error: {e}")
            finally:
                loop.close()

        threading.Thread(target=run_discover, daemon=True).start()

    def do_arm(self, arg):
        """Command selected drones to arm motors."""
        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Commanding {len(self.selected_drones)} drones to arm...")
        threading.Thread(target=self._execute_arm, args=(self.selected_drones,), daemon=True).start()
        print("Arm command sent.")

    def _execute_arm(self, drone_ids: List[str]) -> None:
        """Execute arm command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("arm_motors", [], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_disarm(self, arg):
        """Command selected drones to disarm motors."""
        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Commanding {len(self.selected_drones)} drones to disarm...")
        threading.Thread(target=self._execute_disarm, args=(self.selected_drones,), daemon=True).start()
        print("Disarm command sent.")

    def _execute_disarm(self, drone_ids: List[str]) -> None:
        """Execute disarm command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("disarm_motors", [], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_rtl(self, arg):
        """Command selected drones to Return To Launch."""
        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Commanding {len(self.selected_drones)} drones to RTL...")
        threading.Thread(target=self._execute_emergency, args=(self.selected_drones, 'rtl'), daemon=True).start()
        print("RTL command sent.")

    def _get_backend_config(self, backend_type: BackendType) -> dict:
        """Get configuration for a specific backend type."""
        config = {}

        if backend_type == BackendType.AIRSIM:
            config = {
                'host': settings.get('DSOS', 'airsim_host', 'localhost'),
                'port': settings.getint('DSOS', 'airsim_port', 41451),
            }
        elif backend_type == BackendType.PX4_SITL:
            config = {
                'hostname': settings.get('DSOS', 'px4_host', 'localhost'),
                'port': settings.getint('DSOS', 'px4_port', 4560),
            }
        elif backend_type == BackendType.GAZEBO:
            config = {
                'hostname': settings.get('DSOS', 'gazebo_host', 'localhost'),
                'port': settings.getint('DSOS', 'gazebo_port', 11345),
            }
        elif backend_type == BackendType.ROS2:
            config = {
                'node_name': settings.get('DSOS', 'ros2_node_name', 'dsos_controller'),
            }
        elif backend_type == BackendType.MAVSDK:
            config = {
                'system_address': settings.get('DSOS', 'mavsdk_address', 'udp://:14540'),
            }
        elif backend_type == BackendType.REAL_DRONE:
            config = {
                'connection_string': settings.get('DSOS', 'real_drone_connection', ''),
            }

        return config

    def do_status(self, arg):
        """Show status of all drones or specific drone.
        Usage: status [drone_id]"""
        args = shlex.split(arg)
        if args:
            drone_id = args[0]
            drone = self._get_drone_by_id(drone_id)
            if drone:
                self._print_drone_status(drone)
            else:
                print(f"Drone {drone_id} not found")
        else:
            print(f"Swarm Status ({len(self.drones)} drones):")
            print(f"  Simulation Mode: {self.simulation_mode}")
            print(f"  Active Backend: {getattr(self.active_backend, '__class__.__name__', 'None')}")
            print(f"  Selected Drones: {len(self.selected_drones)}")

            for drone in self.drones:
                selected = "*" if drone.agent_id in self.selected_drones else " "
                backend_status = "Connected" if drone.is_connected_to_backend() else "Simulated"
                print(f"  [{selected}] {drone.agent_id}: {drone.status.state.name} | "
                      f"Bat: {drone.get_battery_level():.1f}% | "
                      f"Backend: {backend_status}")

    def _get_drone_by_id(self, drone_id: str) -> Optional[DroneAgent]:
        """Get a drone by its ID."""
        for drone in self.drones:
            if drone.agent_id == drone_id:
                return drone
        return None

    def _print_drone_status(self, drone: DroneAgent) -> None:
        """Print detailed status for a single drone."""
        print(f"Drone {drone.agent_id}:")
        print(f"  State: {drone.status.state.name}")
        print(f"  Battery: {drone.get_battery_level():.1f}%")
        print(f"  Armed: {drone.is_armed()}")
        print(f"  Backend Connected: {drone.is_connected_to_backend()}")
        print(f"  Position: [{drone.status.position[0]:.2f}, {drone.status.position[1]:.2f}, {drone.status.position[2]:.2f}] m")
        print(f"  Velocity: [{drone.status.velocity[0]:.2f}, {drone.status.velocity[1]:.2f}, {drone.status.velocity[2]:.2f}] m/s")
        print(f"  Orientation: [{drone.status.orientation[0]:.2f}, {drone.status.orientation[1]:.2f}, {drone.status.orientation[2]:.2f}] rad")

    def do_select(self, arg):
        """Select drones for subsequent commands.
        Usage: select <drone_id1> [drone_id2] ... or select all or select none"""
        args = shlex.split(arg)
        if not args:
            print("Usage: select <drone_id1> [drone_id2] ... or select all or select none")
            return

        if args[0].lower() == 'all':
            self.selected_drones = [drone.agent_id for drone in self.drones]
            print(f"Selected all {len(self.selected_drones)} drones")
        elif args[0].lower() == 'none':
            self.selected_drones = []
            print("Cleared selection")
        else:
            # Select specific drones
            valid_drones = []
            invalid_drones = []
            for drone_id in args:
                drone = self._get_drone_by_id(drone_id)
                if drone:
                    valid_drones.append(drone_id)
                else:
                    invalid_drones.append(drone_id)

            self.selected_drones = valid_drones
            if invalid_drones:
                print(f"Warning: Ignoring unknown drones: {', '.join(invalid_drones)}")
            print(f"Selected {len(self.selected_drones)} drones: {', '.join(self.selected_drones)}")

    def do_takeoff(self, arg):
        """Command selected drones to takeoff.
        Usage: takeoff [altitude]"""
        args = shlex.split(arg)
        altitude = 10.0  # default altitude in meters
        if args:
            try:
                altitude = float(args[0])
            except ValueError:
                print("Error: Altitude must be a number")
                return

        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Commanding {len(self.selected_drones)} drones to takeoff to {altitude}m...")
        # Use threading to avoid blocking the command interface
        threading.Thread(target=self._execute_takeoff, args=(self.selected_drones, altitude), daemon=True).start()
        print("Takeoff command sent.")

    def _execute_takeoff(self, drone_ids: List[str], altitude: float) -> None:
        """Execute takeoff command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("takeoff", [altitude], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_land(self, arg):
        """Command selected drones to land."""
        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Commanding {len(self.selected_drones)} drones to land...")
        threading.Thread(target=self._execute_land, args=(self.selected_drones,), daemon=True).start()
        print("Landing command sent.")

    def _execute_land(self, drone_ids: List[str]) -> None:
        """Execute land command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("land", [], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_hover(self, arg):
        """Command selected drones to hover at current position."""
        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Commanding {len(self.selected_drones)} drones to hover...")
        threading.Thread(target=self._execute_hover, args=(self.selected_drones,), daemon=True).start()
        print("Hover command sent.")

    def _execute_hover(self, drone_ids: List[str]) -> None:
        """Execute hover command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("hover", [], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_move(self, arg):
        """Command selected drones to move to a position.
        Usage: 
          move <x> <y> [z]
          move forward|backward|left|right|up|down <distance>"""
        args = shlex.split(arg)
        if not args:
            print("Usage: move <x> <y> [z] OR move forward <dist>")
            return

        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        direction_keywords = ['forward', 'backward', 'left', 'right', 'up', 'down']
        if args[0].lower() in direction_keywords:
            if len(args) < 2:
                print(f"Usage: move {args[0]} <distance>")
                return
            try:
                dist = float(args[1])
            except ValueError:
                print("Error: Distance must be a number")
                return
            
            # Since relative movement needs current pos per drone, we handle it in _execute_move
            direction = args[0].lower()
            print(f"Commanding {len(self.selected_drones)} drones to move {direction} by {dist}m...")
            threading.Thread(target=self._execute_relative_move, args=(self.selected_drones, direction, dist), daemon=True).start()
            print("Move command sent.")
            return

        # Absolute move
        if len(args) < 2:
            print("Usage: move <x> <y> [z]")
            return

        try:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2]) if len(args) > 2 else 0.0
        except ValueError:
            print("Error: Coordinates must be numbers")
            return

        position = [x, y, z]
        print(f"Commanding {len(self.selected_drones)} drones to move to [{x}, {y}, {z}]...")
        threading.Thread(target=self._execute_move, args=(self.selected_drones, position), daemon=True).start()
        print("Move command sent.")

    def _execute_move(self, drone_ids: List[str], position: List[float]) -> None:
        """Execute move command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("goto_position", [position], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def _execute_relative_move(self, drone_ids: List[str], direction: str, dist: float) -> None:
        """Execute relative move command in background thread."""
        if not self.dispatcher: return
        method_name = f"move_{direction}"
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch(method_name, [dist], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_rotate(self, arg):
        """Command selected drones to rotate by a specific angle.
        Usage: rotate <degrees>"""
        if not arg:
            print("Usage: rotate <degrees>")
            return
            
        try:
            angle = float(arg)
        except ValueError:
            print("Error: Angle must be a number")
            return
            
        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return
            
        print(f"Commanding {len(self.selected_drones)} drones to rotate by {angle} degrees...")
        threading.Thread(target=self._execute_rotate, args=(self.selected_drones, angle), daemon=True).start()
        print("Rotate command sent.")

    def _execute_rotate(self, drone_ids: List[str], degrees: float) -> None:
        if not self.dispatcher: return
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("rotate", [degrees], drone_ids), self.loop)
            try:
                results = future.result()
                for res in results:
                    print(res)
            except Exception as e:
                print(f"Error: {e}")

    def do_form(self, arg):
        """Command selected drones to form a formation.
        Usage: form <formation_type> [center_x center_y center_z] [scale] [rotation_degrees]"""
        args = shlex.split(arg)
        if not args:
            print("Usage: form <formation_type> [center_x center_y center_z] [scale] [rotation_degrees]")
            print("Available formations: line, column, circle, v_formation, grid, etc.")
            return

        formation_type = args[0].lower()
        # In a real implementation, we would map string to FormationType enum
        # For demo, we'll just use the string

        # Parse optional parameters
        center = [0.0, 0.0, 0.0]
        scale = 1.0
        rotation = 0.0

        if len(args) >= 4:
            try:
                center[0] = float(args[1])
                center[1] = float(args[2])
                center[2] = float(args[3])
            except ValueError:
                print("Error: Center coordinates must be numbers")
                return
            if len(args) >= 5:
                try:
                    scale = float(args[4])
                except ValueError:
                    print("Error: Scale must be a number")
                    return
            if len(args) >= 6:
                try:
                    rotation = float(args[5])  # degrees
                except ValueError:
                    print("Error: Rotation must be a number")
                    return

        if not self.selected_drones:
            print("Error: No drones selected. Use 'select' command first.")
            return

        print(f"Forming {len(self.selected_drones)} drones into {formation_type} formation...")
        threading.Thread(target=self._execute_form,
                        args=(self.selected_drones, formation_type, center, scale, rotation),
                        daemon=True).start()
        print(f"Formation {formation_type} command sent.")

    def _execute_form(self, drone_ids: List[str], formation_type: str,
                     center: List[float], scale: float, rotation: float) -> None:
        """Execute formation command in background thread."""
        self.formation_manager.set_formation(formation_type, center, scale, rotation)
        positions = self.formation_manager.get_formation_positions()
        
        if not self.dispatcher: return

        # Iterate and assign individually via dispatcher goto_position
        if self.loop:
            for i, drone_id in enumerate(drone_ids):
                target_position = positions[i % len(positions)]
                asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("goto_position", [target_position], [drone_id]), self.loop)

    def do_mission(self, arg):
        """Manage missions.
        Usage: mission <load|start|pause|stop|status> [mission_file]"""
        args = shlex.split(arg)
        if not args:
            print("Usage: mission <load|start|pause|stop|status> [mission_file]")
            return

        command = args[0].lower()

        if command == 'load':
            if len(args) < 2:
                print("Usage: mission load <mission_file>")
                return
            # In a real implementation, we would load a mission from file
            print(f"Loading mission from {args[1]}...")
            # Demo: create a simple waypoint mission
            mission = Mission("demo_mission", MissionType.WAYPOINT, "Demo Mission")
            mission.add_waypoint(Waypoint(0, 0, 10))
            mission.add_waypoint(Waypoint(10, 0, 10))
            mission.add_waypoint(Waypoint(10, 10, 10))
            mission.add_waypoint(Waypoint(0, 10, 10))
            mission.add_waypoint(Waypoint(0, 0, 10))
            self.mission_manager.load_mission(mission)
            print("Mission loaded.")

        elif command == 'start':
            if not self.selected_drones:
                print("Error: No drones selected. Use 'select' command first.")
                return
            print(f"Starting mission on {len(self.selected_drones)} selected drones...")
            threading.Thread(target=self._execute_mission_start,
                           args=(self.selected_drones,), daemon=True).start()
            print("Mission start command sent.")

        elif command == 'pause':
            if not self.selected_drones:
                print("Error: No drones selected. Use 'select' command first.")
                return
            print(f"Pausing mission on {len(self.selected_drones)} selected drones...")
            threading.Thread(target=self._execute_mission_pause,
                           args=(self.selected_drones,), daemon=True).start()
            print("Mission pause command sent.")

        elif command == 'stop':
            if not self.selected_drones:
                print("Error: No drones selected. Use 'select' command first.")
                return
            print(f"Stopping mission on {len(self.selected_drones)} selected drones...")
            threading.Thread(target=self._execute_mission_stop,
                           args=(self.selected_drones,), daemon=True).start()
            print("Mission stop command sent.")

        elif command == 'status':
            mission = self.mission_manager.current_mission
            if mission:
                print(f"Current Mission: {mission.name}")
                print(f"  Type: {mission.mission_type.value}")
                print(f"  Status: {mission.status.value}")
                print(f"  Waypoints: {len(mission.waypoints)}")
                print(f"  Current Waypoint: {mission.current_waypoint_index}")
            else:
                print("No mission loaded.")
        else:
            print(f"Unknown command: {command}")

    def _execute_mission_start(self, drone_ids: List[str]) -> None:
        """Execute mission start in background thread."""
        for drone_id in drone_ids:
            drone = self._get_drone_by_id(drone_id)
            if drone:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    # If we have a mission loaded, start it
                    mission = self.mission_manager.current_mission
                    if mission and mission.waypoints:
                        # Convert mission waypoints to the format expected by drone
                        waypoints = [[wp.x, wp.y, wp.z] for wp in mission.waypoints]
                        loop.run_until_complete(drone.load_mission(waypoints))
                    else:
                        # No specific mission, just hover
                        if drone.is_connected_to_backend():
                            # Just maintain current position
                            pass
                        else:
                            drone.status.state = drone.status.state.HOVERING
                except Exception as e:
                    self.logger.error(f"Error starting mission for {drone_id}: {e}")
                finally:
                    loop.close()

    def _execute_mission_pause(self, drone_ids: List[str]) -> None:
        """Execute mission pause in background thread."""
        for drone_id in drone_ids:
            drone = self._get_drone_by_id(drone_id)
            if drone:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    drone.status.state = drone.status.state.PAUSED
                except Exception as e:
                    self.logger.error(f"Error pausing mission for {drone_id}: {e}")
                finally:
                    loop.close()

    def _execute_mission_stop(self, drone_ids: List[str]) -> None:
        """Execute mission stop in background thread."""
        for drone_id in drone_ids:
            drone = self._get_drone_by_id(drone_id)
            if drone:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    drone.status.state = drone.status.state.IDLE
                    # Optionally return to home or just hover
                    # drone.status.state = drone.status.state.HOVERING
                except Exception as e:
                    self.logger.error(f"Error stopping mission for {drone_id}: {e}")
                finally:
                    loop.close()

    def do_formation(self, arg):
        """Manage formations.
        Usage: formation <list|set> [parameters]"""
        args = shlex.split(arg)
        if not args:
            print("Usage: formation <list|set> [parameters]")
            return

        subcommand = args[0].lower()

        if subcommand == 'list':
            print("Available formations:")
            print("  line, horizontal_line, vertical_line, diagonal_line, reverse_diagonal_line")
            print("  equilateral_triangle, inverted_triangle, diamond, compact_diamond, expanded_diamond")
            print("  arrow, reverse_arrow, v_formation, reverse_v, column, staggered_column")
            print("  echelon_left, echelon_right, circle, orbit, arc, semi_circle, spiral")
            print("  patrol, reconnaissance, escort, adaptive, random, search, exploration, mapping, coverage, grid")
        elif subcommand == 'set':
            # This is similar to the 'form' command
            self.onecmd("form " + " ".join(args[1:]))
        else:
            print(f"Unknown formation subcommand: {subcommand}")

    def do_emergency(self, arg):
        """Trigger emergency procedures.
        Usage: emergency <land|rtl|stop> [drone_id]"""
        args = shlex.split(arg)
        if not args:
            print("Usage: emergency <land|rtl|stop> [drone_id]")
            return

        command = args[0].lower()
        target_drone = args[1] if len(args) > 1 else None

        if target_drone and target_drone not in [d.agent_id for d in self.drones]:
            print(f"Error: Drone {target_drone} not found")
            return

        drones_to_affect = [target_drone] if target_drone else self.selected_drones
        if not drones_to_affect:
            drones_to_affect = [d.agent_id for d in self.drones]  # All drones if none selected

        print(f"Executing emergency {command} on {len(drones_to_affect)} drones...")
        threading.Thread(target=self._execute_emergency,
                        args=(drones_to_affect, command), daemon=True).start()
        print(f"Emergency {command} command sent.")

    def _execute_emergency(self, drone_ids: List[str], command: str) -> None:
        """Execute emergency command in background thread."""
        if not self.dispatcher: return
        if self.loop:
            if command == 'land':
                asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("land", [], drone_ids), self.loop)
            elif command == 'rtl':
                asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("return_to_home", [], drone_ids), self.loop)
            elif command == 'stop':
                asyncio.run_coroutine_threadsafe(self.dispatcher.dispatch("disarm_motors", [], drone_ids), self.loop)
            else:
                print(f"Unknown emergency command: {command}")

    def do_param(self, arg):
        """Get or set system parameters.
        Usage: param get <param_name> or param set <param_name> <value>"""
        args = shlex.split(arg)
        if len(args) < 2:
            print("Usage: param get <param_name> or param set <param_name> <value>")
            return

        command = args[0].lower()
        param_name = args[1]

        if command == 'get':
            try:
                value = settings.get('DSOS', param_name)
                print(f"{param_name} = {value}")
            except Exception:
                print(f"Parameter {param_name} not found")
        elif command == 'set':
            if len(args) < 3:
                print("Usage: param set <param_name> <value>")
                return
            value = args[2]
            # Try to convert to appropriate type
            try:
                # Try integer
                value = int(value)
            except ValueError:
                try:
                    # Try float
                    value = float(value)
                except ValueError:
                    # Keep as string
                    pass
            settings.set('DSOS', param_name, str(value))
            settings.save()
            print(f"Set {param_name} = {value}")
        else:
            print(f"Unknown param command: {command}")

    def do_backend(self, arg):
        """Manage backends.
        Usage: backend <list|set|status> [backend_type] [params]"""
        args = shlex.split(arg)
        if not args:
            print("Usage: backend <list|set|status> [backend_type] [params]")
            return

        subcommand = args[0].lower()

        if subcommand == 'list':
            print("Available backends:")
            print("  simulation - Built-in simulation (default)")
            print("  airsim - Microsoft AirSim simulator")
            print("  px4_sitl - PX4 Software In The Loop")
            print("  gazebo - Gazebo simulator")
            print("  ros2 - ROS 2")
            print("  mavsdk - MAVSDK")
            print("  real_drone - Direct hardware connection")
            print(f"\nCurrently configured backend: {self.backend_type_str}")
            print(f"Active backend: {getattr(self.active_backend, '__class__.__name__', 'None')}")

        elif subcommand == 'set':
            if len(args) < 2:
                print("Usage: backend set <backend_type> [params]")
                return

            backend_type_str = args[1].lower()
            try:
                backend_type = BackendType(backend_type_str)
                # Update settings
                settings.set('DSOS', 'backend_type', backend_type.value)
                settings.save()

                # Re-initialize backend
                self._initialize_backend()
                print(f"Backend set to {backend_type.value}")
            except ValueError:
                print(f"Error: Unknown backend type '{backend_type_str}'")
            except Exception as e:
                print(f"Error setting backend: {e}")

        elif subcommand == 'status':
            print(f"Backend configuration:")
            print(f"  Type: {self.backend_type_str}")
            print(f"  Simulation mode: {self.settings.getboolean('DSOS', 'simulation_mode', True)}")
            print(f"  Active backend: {getattr(self.active_backend, '__class__.__name__', 'None')}")
            if self.active_backend:
                print(f"  Backend connected: {self.active_backend.is_connected()}")
                print(f"  Backend capabilities: {self.active_backend.get_capabilities()}")

                # Show available vehicles if applicable
                if hasattr(self.active_backend, 'get_vehicles'):
                    # This would be async, so we'll show a placeholder
                    print(f"  Available vehicles: [use backend command to get list]")
            else:
                print("  No active backend")

    def do_sim(self, arg):
        """Control simulation mode.
        Usage: sim <start|stop|status>"""
        args = shlex.split(arg)
        if not args:
            print("Usage: sim <start|stop|status>")
            return

        command = args[0].lower()
        if command == 'start':
            self.simulation_mode = True
            settings.set('DSOS', 'simulation_mode', 'true')
            settings.save()
            # Reset to simulation backend (None)
            self.active_backend = None
            for drone in self.drones:
                drone.backend = None
            print("Simulation mode started")
        elif command == 'stop':
            self.simulation_mode = False
            settings.set('DSOS', 'simulation_mode', 'false')
            settings.save()
            # Try to restore the configured backend
            self._initialize_backend()
            print("Simulation mode stopped")
        elif command == 'status':
            print(f"Simulation mode: {'ON' if self.simulation_mode else 'OFF'}")
            print(f"Number of simulated drones: {len([d for d in self.drones if not d.is_connected_to_backend()])}")
            print(f"Number of backend-connected drones: {len([d for d in self.drones if d.is_connected_to_backend()])}")
        else:
            print(f"Unknown sim command: {command}")

    def do_help(self, arg):
        """List available commands with help."""
        if arg:
            super().do_help(arg)
        else:
            print("Available commands:")
            print("  drone     - Drone management (select, status, etc.)")
            print("  flight    - Flight control (takeoff, land, hover, move)")
            print("  formation - Formation control")
            print("  mission   - Mission management")
            print("  emergency - Emergency procedures")
            print("  param     - Parameter management")
            print("  backend   - Backend management")
            print("  sim       - Simulation control")
            print("  boot      - Run automated boot sequence")
            print("  discover  - Discover vehicles via backend")
            print("  arm       - Arm selected drones")
            print("  disarm    - Disarm selected drones")
            print("  rtl       - Return to launch")
            print("  help      - Show help")
            print("  exit/quit - Exit GCS")
        # Show some examples
        print("\nExamples:")
        print("  boot                 - Boot simulators and backends")
        print("  select all           - Select all drones")
        print("  arm                  - Arm selected drones")
        print("  takeoff 15           - Takeoff selected drones to 15m")
        print("  form v_formation     - Form selected drones in V formation")
        print("  rtl                  - Return to home")
        print("  mission load mission.txt - Load a mission from file")
        print("  backend set airsim   - Switch to AirSim backend")
        print("  sim status           - Check simulation mode status")

    def do_quit(self, arg):
        """Exit the GCS."""
        print("Exiting Ground Control Station...")
        return True

    def do_exit(self, arg):
        """Exit the GCS."""
        return self.do_quit(arg)
        
    def do_shutdown(self, arg):
        """Gracefully shutdown all drones and exit DSOS."""
        print("Initiating full swarm shutdown...")
        if self.drones:
            threading.Thread(target=self._execute_shutdown, args=(self.drones,), daemon=True).start()
        print("Waiting for graceful shutdown...")
        time.sleep(2)
        return True
        
    def _execute_shutdown(self, drones: List[Any]) -> None:
        for drone in drones:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(drone.shutdown())
            except Exception as e:
                self.logger.error(f"Error shutting down {drone.agent_id}: {e}")
            finally:
                loop.close()

    def start(self) -> None:
        """Start the GCS interface."""
        if self.running:
            return
        self.running = True
        print(self.intro)
        self.cmdloop()

    def stop(self) -> None:
        """Stop the GCS interface."""
        self.running = False
        # Post-loop cleanup would go here

# For backward compatibility and testing
if __name__ == '__main__':
    import sys
    import logging

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler("logs/gcs.log"),
            logging.StreamHandler(sys.stdout)
        ]
    )

    # Start GCS
    gcs = GCSCommand()
    try:
        gcs.start()
    except KeyboardInterrupt:
        print("\nInterrupted...")
    finally:
        gcs.stop()