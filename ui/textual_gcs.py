"""
Textual-based modern Ground Control Station for DSOS.
"""
from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, DataTable, Label, Log, Input, Button
from textual.containers import Horizontal, Vertical, ScrollableContainer
from textual.binding import Binding
from textual import events, on, work
import asyncio
from typing import List, Dict, Any
import logging

class Sidebar(ScrollableContainer):
    """Sidebar for drone selection, help, and status."""
    def compose(self) -> ComposeResult:
        yield Label("AWAITING COMMAND...", id="key-action", classes="action-display")
        
        yield Label("="*30, classes="divider")
        yield Label("DRONE SELECTION", classes="section-title")
        yield Button("[ ] Drone 1 (Ctrl+1)", id="sel_1", classes="btn-menu")
        yield Button("[ ] Drone 2 (Ctrl+2)", id="sel_2", classes="btn-menu")
        yield Button("[ ] Drone 3 (Ctrl+3)", id="sel_3", classes="btn-menu")
        yield Button("Ctrl+A  Select All", id="sel_all", classes="btn-menu")
        yield Button("Ctrl+D  Deselect All", id="sel_none", classes="btn-menu")
        
        yield Label("-" * 30, classes="divider")
        yield Label("FLIGHT COMMANDS", classes="section-title")
        yield Button("F1  Arm", id="cmd_arm", classes="btn-menu")
        yield Button("F2  Disarm", id="cmd_disarm", classes="btn-menu")
        yield Button("F3  Takeoff", id="cmd_takeoff", classes="btn-menu")
        yield Button("F4  Hover", id="cmd_hover", classes="btn-menu")
        yield Button("F5  Land", id="cmd_land", classes="btn-menu")
        yield Button("F6  RTL", id="cmd_rtl", classes="btn-menu")
        yield Button("F7  Emergency Stop", id="cmd_estop", variant="error")
        
        yield Label("-" * 30, classes="divider")
        yield Label("MOVEMENT", classes="section-title")
        yield Button("↑  Forward", id="mv_up", classes="btn-menu")
        yield Button("↓  Backward", id="mv_down", classes="btn-menu")
        yield Button("←  Left", id="mv_left", classes="btn-menu")
        yield Button("→  Right", id="mv_right", classes="btn-menu")
        yield Button("W  Up", id="mv_w", classes="btn-menu")
        yield Button("S  Down", id="mv_s", classes="btn-menu")
        yield Button("A  Rotate Left", id="mv_a", classes="btn-menu")
        yield Button("D  Rotate Right", id="mv_d", classes="btn-menu")
        yield Button("Space  Stop / Hover", id="mv_space", classes="btn-menu")
        yield Button("Shift  Speed +", id="mv_speed_up", classes="btn-menu")
        yield Button("Ctrl   Speed -", id="mv_speed_down", classes="btn-menu")
        
        yield Label("-" * 30, classes="divider")
        yield Label("FORMATIONS", classes="section-title")
        yield Button("1  Line", id="form_1", classes="btn-menu")
        yield Button("2  Column", id="form_2", classes="btn-menu")
        yield Button("3  V", id="form_3", classes="btn-menu")
        yield Button("4  Diamond", id="form_4", classes="btn-menu")
        yield Button("5  Triangle", id="form_5", classes="btn-menu")
        yield Button("6  Square", id="form_6", classes="btn-menu")
        yield Button("7  Circle", id="form_7", classes="btn-menu")
        yield Button("8  Grid", id="form_8", classes="btn-menu")
        yield Button("9  Arrow", id="form_9", classes="btn-menu")
        yield Button("0  Custom", id="form_0", classes="btn-menu")
        yield Button("Q  Echelon Left", id="form_q", classes="btn-menu")
        yield Button("E  Echelon Right", id="form_e", classes="btn-menu")
        yield Button("R  Rectangle", id="form_r", classes="btn-menu")
        yield Button("T  Pentagon", id="form_t", classes="btn-menu")
        yield Button("Y  Hexagon", id="form_y", classes="btn-menu")
        yield Button("U  Spiral", id="form_u", classes="btn-menu")
        yield Button("I  Horizontal Wall", id="form_i", classes="btn-menu")
        yield Button("O  Vertical Wall", id="form_o", classes="btn-menu")
        
        yield Label("-" * 30, classes="divider")
        yield Label("SETTINGS", classes="section-title")
        yield Label("Takeoff Altitude (m)", classes="input-label")
        yield Input(value="10.0", id="takeoff_alt", classes="alt-input")
        yield Label("Formation Altitude (m)", classes="input-label")
        yield Input(value="15.0", id="form_alt", classes="alt-input")
        yield Label("Cruise Speed (m/s)", classes="input-label")
        yield Input(value="5.0", id="cruise_speed", classes="alt-input")
        yield Label("Max Speed (m/s)", classes="input-label")
        yield Input(value="10.0", id="max_speed", classes="alt-input")
        
        yield Label("-" * 30, classes="divider")
        yield Label("STATUS", classes="section-title")
        yield Label("Selected Drone: None", id="stat_sel")
        yield Label("Current Formation: None", id="stat_form")
        yield Label("Current Speed: 0.0 m/s", id="stat_spd")
        yield Label("Altitude: 0.0 m", id="stat_alt")
        yield Label("Mode: UNKNOWN", id="stat_mode")
        yield Label("="*30, classes="divider")

class TextualGCSApp(App):
    """Modern TUI Ground Control Station."""
    
    CSS = """
    Screen {
        layout: horizontal;
    }
    Sidebar {
        width: 35;
        dock: left;
        padding: 1 2;
        background: $panel;
        border-right: vkey $primary;
        overflow-y: auto;
    }
    .action-display {
        color: $warning;
        text-style: bold;
        padding: 1;
        background: $surface;
    }
    .section-title {
        color: $accent;
        text-style: bold;
        margin-top: 1;
        margin-bottom: 1;
    }
    .divider {
        color: $surface;
    }
    .main-area {
        width: 1fr;
        height: 1fr;
        layout: vertical;
    }
    DataTable {
        height: 15;
        border: solid $primary;
    }
    Log {
        border: solid $secondary;
        height: 1fr;
    }
    .btn-menu {
        width: 100%;
        height: 3;
        border: none;
        background: transparent;
        color: $text;
        text-align: left;
    }
    .btn-menu:hover {
        background: $primary-background;
    }
    .alt-input {
        width: 100%;
        margin-top: 1;
        margin-bottom: 1;
        border: solid $primary;
    }
    .input-label {
        color: $text-muted;
    }
    """

    BINDINGS = [
        Binding("ctrl+q", "quit", "Quit", show=True),
    ]

    def __init__(self, gcs_command):
        super().__init__()
        self.gcs_cmd = gcs_command
        self.drones = gcs_command.drones
        self.selected_drones = [False, False, False]
        self.update_task = None
        self.worker_task = None
        self.command_queue = asyncio.Queue()
        self.logger = logging.getLogger("textual_gcs")
        self._last_move_time = 0.0
        self._motion_watchdog_task = None

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header(show_clock=True)
        yield Sidebar()
        with Vertical(classes="main-area"):
            yield DataTable(id="drone_table")
            yield Log(id="gcs_log", highlight=True)
        yield Footer()

    def on_mount(self) -> None:
        """Called when app is mounted."""
        self.title = "DSOS Ground Control Station"
        self.sub_title = "Modern Swarm Command Interface"
        
        table = self.query_one("#drone_table", DataTable)
        table.can_focus = False  # Global keyboard capture
        table.add_column("Drone ID", key="Drone ID")
        table.add_column("Status", key="Status")
        table.add_column("Battery", key="Battery")
        table.add_column("GPS", key="GPS")
        table.add_column("Pos(N,E,D)", key="Pos")
        table.add_column("Vel(N,E,D)", key="Vel")
        table.add_column("Heading", key="Heading")
        table.add_column("Alt", key="Alt")
        table.add_column("Mode", key="Mode")
        table.add_column("Armed", key="Armed")
        table.add_column("Offboard", key="Offboard")
        
        for i in range(3):
            table.add_row(f"Drone {i+1}", "OFFLINE", "0%", "N/A", "0,0,0", "0,0,0", "0°", "0.0m", "UNKNOWN", "NO", "NO", key=str(i))
            
        self.log_widget = self.query_one("#gcs_log", Log)
        self.log_widget.can_focus = False  # Global keyboard capture
        self.log_widget.write_line("GCS Initialized. Welcome to DSOS.")
        
        self.update_selection_ui()
        
        # Start background polling task for UI updates and command worker
        self.update_task = asyncio.create_task(self.update_loop())
        self.worker_task = asyncio.create_task(self._command_worker_loop())
        self._motion_watchdog_task = asyncio.create_task(self._motion_watchdog())
        
    async def _command_worker_loop(self):
        """Background task to process the command queue sequentially."""
        import datetime
        while True:
            cmd_name, args, targets = await self.command_queue.get()
            ts = datetime.datetime.now().strftime("%H:%M:%S")
            self.write_log(f"[{ts}] [cyan][QUEUE ACCEPTED][/cyan] '{cmd_name}' for {targets}")
            
            try:
                results = await self.gcs_cmd.dispatcher.dispatch(cmd_name, args, targets)
                for res in results:
                    ts = datetime.datetime.now().strftime("%H:%M:%S")
                    if "Failed" in res or "Error" in res:
                        self.write_log(f"[{ts}] [red][FAILED][/red] {res}")
                    else:
                        self.write_log(f"[{ts}] [green][COMPLETED][/green] {res}")
            except Exception as e:
                ts = datetime.datetime.now().strftime("%H:%M:%S")
                self.write_log(f"[{ts}] [red][FAILED][/red] Exception: {e}")
            
            self.command_queue.task_done()
        
    async def update_loop(self):
        """Periodically update the table with drone data."""
        table = self.query_one("#drone_table", DataTable)
        while True:
            current_formation = getattr(self.gcs_cmd.dispatcher.swarm_manager, 'current_formation', 'None')
            self.sub_title = f"Modern Swarm Command Interface - Current Formation: {current_formation.upper()}"
            
            # Update Sidebar STATUS section
            selected_ids = []
            for i, sel in enumerate(self.selected_drones):
                if sel and i < len(self.drones):
                    selected_ids.append(f"D{i+1}")
            
            self.query_one("#stat_sel", Label).update(f"Selected: {','.join(selected_ids) or 'None'}")
            self.query_one("#stat_form", Label).update(f"Formation: {current_formation.upper()}")
            
            avg_alt, avg_spd, modes = 0, 0, set()
            active_count = 0
            
            for i, drone in enumerate(self.drones):
                if i >= 3: break
                status = getattr(drone, "status", None)
                if not status: continue
                
                status_str = "ONLINE" if getattr(status, 'is_connected', False) else "OFFLINE"
                batt_val = getattr(status, 'battery_level', 0) or 0
                batt = f"{batt_val:.0f}%"
                mode = getattr(status, 'flight_mode', "UNKNOWN")
                
                is_armed = getattr(status, 'is_armed', False)
                armed_str = "YES" if is_armed else "NO"
                offboard_str = "YES" if mode == "OFFBOARD" else "NO"
                
                pos = status.position if status.position else [0,0,0]
                vel = status.velocity if status.velocity else [0,0,0]
                ori = status.orientation if status.orientation else [0,0,0]
                
                pos_str = f"{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}"
                vel_str = f"{vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}"
                
                import math
                heading = math.degrees(ori[2]) if len(ori) > 2 else 0.0
                head_str = f"{heading:.0f}°"
                
                alt_val = pos[2] * -1 if pos else 0.0
                alt = f"{alt_val:.1f}m"
                speed = (vel[0]**2 + vel[1]**2 + vel[2]**2)**0.5
                
                gps_info = getattr(status, 'gps', None)
                gps_str = f"{gps_info[0]:.5f}, {gps_info[1]:.5f}" if gps_info and len(gps_info) >= 2 else "WAITING"
                
                # Active drone highlighting
                drone_label = f"Drone {i+1}"
                if self.selected_drones[i]:
                    drone_label = f"[bold cyan]=> Drone {i+1} <=[/bold cyan]"
                    status_str = f"[bold cyan]{status_str}[/bold cyan]"
                    
                    active_count += 1
                    avg_alt += alt_val
                    avg_spd += speed
                    modes.add(mode)
                
                # Update row data
                table.update_cell(str(i), "Drone ID", drone_label)
                table.update_cell(str(i), "Status", status_str)
                table.update_cell(str(i), "Battery", batt)
                table.update_cell(str(i), "GPS", gps_str)
                table.update_cell(str(i), "Pos", pos_str)
                table.update_cell(str(i), "Vel", vel_str)
                table.update_cell(str(i), "Heading", head_str)
                table.update_cell(str(i), "Alt", alt)
                table.update_cell(str(i), "Mode", mode)
                table.update_cell(str(i), "Armed", armed_str)
                table.update_cell(str(i), "Offboard", offboard_str)
                
            # Finish sidebar updates
            if active_count > 0:
                self.query_one("#stat_alt", Label).update(f"Altitude: {avg_alt/active_count:.1f} m")
                self.query_one("#stat_spd", Label).update(f"Speed: {avg_spd/active_count:.1f} m/s")
                self.query_one("#stat_mode", Label).update(f"Mode: {','.join(modes)}")
                
            await asyncio.sleep(0.5)

    def write_log(self, msg: str):
        if hasattr(self, 'log_widget'):
            self.log_widget.write_line(msg)

    # --- Actions / Commands ---
    def action_select_drone(self, idx: int):
        self.selected_drones[idx-1] = not self.selected_drones[idx-1]
        self.update_selection_ui()

    def action_select_all(self):
        self.selected_drones = [True, True, True]
        self.update_selection_ui()

    def action_deselect_all(self):
        self.selected_drones = [False, False, False]
        self.update_selection_ui()

    def update_selection_ui(self):
        for i in range(3):
            btn = self.query_one(f"#sel_{i+1}", Button)
            mark = "X" if self.selected_drones[i] else " "
            btn.label = f"[{mark}] Drone {i+1} (Ctrl+{i+1})"
            
        # Update GCS selected drones backend
        self.gcs_cmd.selected_drones = []
        for i, sel in enumerate(self.selected_drones):
            if sel and i < len(self.drones):
                self.gcs_cmd.selected_drones.append(self.drones[i].agent_id)

    async def execute_command_on_selection(self, cmd_name: str, args: List[Any] = None, source: str = "Button"):
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.write_log(f"[{ts}] [blue][UI RECEIVED][/blue] {source} -> {cmd_name}")
        
        if not any(self.selected_drones):
            self.write_log(f"[{ts}] [red][FAILED][/red] No drones selected!")
            return
            
        args = args or []
        targets = self.gcs_cmd.selected_drones.copy()
        
        # Enqueue instead of blocking
        await self.command_queue.put((cmd_name, args, targets))

    async def _motion_watchdog(self):
        import time
        while True:
            await asyncio.sleep(0.05)
            if self._last_move_time > 0 and (time.time() - self._last_move_time) > 0.2:
                self._last_move_time = 0.0
                await self.execute_command_on_selection("hover")
                self.write_log("Key released. Stopping drones smoothly.")
                self._clear_key_action()

    @on(Button.Pressed)
    async def on_button_pressed(self, event: Button.Pressed):
        btn_id = event.button.id
        if not btn_id: return
        
        try: speed = float(self.query_one("#cruise_speed", Input).value)
        except: speed = 5.0
        
        cmd_map = {
            "cmd_arm": ("arm", []), "cmd_disarm": ("disarm", []),
            "cmd_hover": ("hover", []),
            "cmd_land": ("land", []), "cmd_rtl": ("rtl", []),
            "cmd_estop": ("estop", []),
            
            "mv_up": ("move_velocity", [speed, 0.0, 0.0, 0.5]),
            "mv_down": ("move_velocity", [-speed, 0.0, 0.0, 0.5]),
            "mv_left": ("move_velocity", [0.0, -speed, 0.0, 0.5]),
            "mv_right": ("move_velocity", [0.0, speed, 0.0, 0.5]),
            "mv_w": ("move_velocity", [0.0, 0.0, -speed, 0.5]),
            "mv_s": ("move_velocity", [0.0, 0.0, speed, 0.5]),
            "mv_a": ("rotate_left", [15.0]),
            "mv_d": ("rotate_right", [15.0]),
            "mv_space": ("hover", []),
            
            "form_1": ("form", ["line"]), "form_2": ("form", ["column"]),
            "form_3": ("form", ["v_formation"]), "form_4": ("form", ["diamond"]),
            "form_5": ("form", ["triangle"]), "form_6": ("form", ["square"]),
            "form_7": ("form", ["circle"]), "form_8": ("form", ["grid"]),
            "form_9": ("form", ["arrow"]), "form_0": ("form", ["custom"]),
            "form_q": ("form", ["echelon_left"]), "form_e": ("form", ["echelon_right"]),
            "form_r": ("form", ["rectangle"]), "form_t": ("form", ["pentagon"]),
            "form_y": ("form", ["hexagon"]), "form_u": ("form", ["spiral"]),
            "form_i": ("form", ["horizontal_wall"]), "form_o": ("form", ["vertical_wall"])
        }
        
        if btn_id == "sel_1": self.action_select_drone(1)
        elif btn_id == "sel_2": self.action_select_drone(2)
        elif btn_id == "sel_3": self.action_select_drone(3)
        elif btn_id == "sel_all": self.action_select_all()
        elif btn_id == "sel_none": self.action_deselect_all()
        elif btn_id == "mv_speed_up":
            self.query_one("#cruise_speed", Input).value = str(min(20.0, speed + 1.0))
        elif btn_id == "mv_speed_down":
            self.query_one("#cruise_speed", Input).value = str(max(1.0, speed - 1.0))
        elif btn_id == "cmd_takeoff":
            try: alt = float(self.query_one("#takeoff_alt", Input).value)
            except: alt = 10.0
            await self.execute_command_on_selection("takeoff", [alt])
        elif btn_id in cmd_map:
            cmd, args = cmd_map[btn_id]
            if cmd == "form":
                try: alt = float(self.query_one("#form_alt", Input).value)
                except: alt = 15.0
                args = [args[0], alt]
            await self.execute_command_on_selection(cmd, args, source=f"Button({btn_id})")
            
    @on(events.Key)
    async def global_key_intercept(self, event: events.Key):
        # Allow Input boxes to work normally if focused
        if isinstance(self.focused, Input): return
        
        key = event.key
        
        if key == "ctrl+1": self.action_select_drone(1); return
        if key == "ctrl+2": self.action_select_drone(2); return
        if key == "ctrl+3": self.action_select_drone(3); return
        if key == "ctrl+a": self.action_select_all(); return
        if key == "ctrl+d": self.action_deselect_all(); return
        
        speed_input = self.query_one("#cruise_speed", Input)
        try: speed = float(speed_input.value)
        except: speed = 5.0
        
        if "shift+" in key or key.isupper():
            speed += 1.0
            speed_input.value = str(min(20.0, speed))
            key = key.replace("shift+", "").lower()
        elif "ctrl+" in key:
            speed = max(1.0, speed - 1.0)
            speed_input.value = str(speed)
            key = key.replace("ctrl+", "").lower()
        else:
            key = key.lower()

        cmd = None; args = []; action_str = None

        if key == "up": cmd, args, action_str = "move_velocity", [speed, 0.0, 0.0, 0.5], "Moving Forward"
        elif key == "down": cmd, args, action_str = "move_velocity", [-speed, 0.0, 0.0, 0.5], "Moving Backward"
        elif key == "left": cmd, args, action_str = "move_velocity", [0.0, -speed, 0.0, 0.5], "Moving Left"
        elif key == "right": cmd, args, action_str = "move_velocity", [0.0, speed, 0.0, 0.5], "Moving Right"
        elif key == "w": cmd, args, action_str = "move_velocity", [0.0, 0.0, -speed, 0.5], "Moving Up"
        elif key == "s": cmd, args, action_str = "move_velocity", [0.0, 0.0, speed, 0.5], "Moving Down"
        elif key == "a": cmd, args, action_str = "rotate_left", [15.0], "Rotating Left"
        elif key == "d": cmd, args, action_str = "rotate_right", [15.0], "Rotating Right"
        elif key == "space": cmd, args, action_str = "hover", [], "Stopping / Hovering"
        
        elif key in ["1","2","3","4","5","6","7","8","9","0","q","e","r","t","y","u","i","o"]:
            try: alt = float(self.query_one("#form_alt", Input).value)
            except: alt = 15.0
            
            form_map = {
                "1": ("line", "Line"), "2": ("column", "Column"), "3": ("v_formation", "V"), 
                "4": ("diamond", "Diamond"), "5": ("triangle", "Triangle"), "6": ("square", "Square"),
                "7": ("circle", "Circle"), "8": ("grid", "Grid"), "9": ("arrow", "Arrow"),
                "0": ("custom", "Custom"), "q": ("echelon_left", "Echelon Left"), "e": ("echelon_right", "Echelon Right"),
                "r": ("rectangle", "Rectangle"), "t": ("pentagon", "Pentagon"), "y": ("hexagon", "Hexagon"),
                "u": ("spiral", "Spiral"), "i": ("horizontal_wall", "Horiz Wall"), "o": ("vertical_wall", "Vert Wall")
            }
            formation, action_str = form_map[key]
            cmd, args = "form", [formation, alt]
        
        elif key == "f1": cmd, args, action_str = "arm", [], "Arming"
        elif key == "f2": cmd, args, action_str = "disarm", [], "Disarming"
        elif key == "f3": 
            alt_input = self.query_one("#takeoff_alt", Input).value
            alt = float(alt_input) if alt_input.strip() else 10.0
            cmd, args, action_str = "takeoff", [alt], "Takeoff"
        elif key == "f4": cmd, args, action_str = "hover", [], "Hovering"
        elif key == "f5": cmd, args, action_str = "land", [], "Landing"
        elif key == "f6": cmd, args, action_str = "rtl", [], "RTL"
        elif key == "f7": cmd, args, action_str = "estop", [], "E-STOP"
        
        if cmd:
            await self.execute_command_on_selection(cmd, args, source=f"Key({key})")
            if action_str:
                self.query_one("#key-action", Label).update(action_str)
                self._last_move_time = __import__("time").time()
            event.prevent_default()
                
    def flash_key_action(self, key_name: str, action_str: str):
        lbl = self.query_one("#key-action", Label)
        lbl.update(f"Pressed {key_name.upper()}\n>> {action_str}")
        lbl.add_class("action-active")
        
        if hasattr(self, '_flash_task') and self._flash_task:
            self._flash_task.cancel()
        self._flash_task = asyncio.create_task(self._clear_key_action())
        
    async def _clear_key_action(self):
        try:
            await asyncio.sleep(0.5)
            lbl = self.query_one("#key-action", Label)
            lbl.remove_class("action-active")
            lbl.update("AWAITING COMMAND...")
        except asyncio.CancelledError:
            pass
                
    async def _auto_brake_timer(self):
        """Timer to automatically halt the drone if no keys are pressed for 0.2s."""
        try:
            await asyncio.sleep(0.3)
            # If not cancelled by another keypress, send a stop command
            await self.execute_command_on_selection("move_velocity", [0.0, 0.0, 0.0, 1.0])
        except asyncio.CancelledError:
            pass

    async def action_arm(self): await self.execute_command_on_selection("arm_motors")
    async def action_disarm(self): await self.execute_command_on_selection("disarm_motors")
    async def action_takeoff(self): await self.execute_command_on_selection("takeoff", [10.0])
    async def action_hover(self): await self.execute_command_on_selection("hover")
    async def action_land(self): await self.execute_command_on_selection("land")
    async def action_rtl(self): await self.execute_command_on_selection("return_to_home")
    
    async def action_estop(self): 
        self.write_log("[bold red]EMERGENCY STOP INITIATED![/bold red]")
        await self.execute_command_on_selection("disarm_motors")
        
    def action_help(self):
        self.write_log("Help: Select drones with Ctrl+1/2/3 or Ctrl+A, then press F2-F7 to command.")
        self.write_log("Formations (Shift+1 to Shift+7): V-Form, Line, Column, Diamond, Grid, Circle, Arrow")
        
    def action_formation_menu(self):
        self.write_log("Formation menu: Use Shift+1 through Shift+7 to trigger formations directly.")
        
    async def action_formation(self, formation_type: str):
        await self.execute_command_on_selection("form", [formation_type])
        
    def action_mission(self):
        self.write_log("Mission menu: (Not yet fully wired to modal, try legacy CLI)")

    def action_quit(self):
        self.write_log("Shutting down GCS...")
        if self.worker_task:
            self.worker_task.cancel()
        self.exit()
