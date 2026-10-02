"""
GCS Dashboard for DSOS.
Provides a professional startup dashboard and status rendering using standard ANSI escape codes.
"""
import sys
import time
from typing import Dict, Optional

class ANSI:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"

class GCSDashboard:
    """Renders professional startup dashboard and live status panels."""
    
    def __init__(self):
        self.statuses: Dict[str, str] = {}
        self._last_rendered_lines = 0

    def clear_screen(self):
        """Clears the terminal screen."""
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def print_header(self):
        """Prints the DSOS Boot Manager header."""
        self.clear_screen()
        header = f"""
{ANSI.CYAN}{ANSI.BOLD}==========================================
DSOS Boot Manager
================={ANSI.RESET}
"""
        sys.stdout.write(header)
        sys.stdout.flush()
        
    def render_row(self, label: str, status: str):
        """Renders a single row in the dashboard."""
        padding = 35 - len(label)
        dots = "." * max(1, padding)
        
        # Colorize status
        color = ANSI.WHITE
        if status == "OK":
            color = ANSI.GREEN
        elif status == "WAIT":
            color = ANSI.YELLOW
        elif status == "FAIL":
            color = ANSI.RED
        elif "Found" in status:
            color = ANSI.GREEN
            
        row = f"{ANSI.WHITE}{label}{dots} {color}{status}{ANSI.RESET}\n"
        sys.stdout.write(row)
        sys.stdout.flush()

    def update_status(self, step: str, status: str):
        """Updates the dashboard sequentially."""
        if step not in self.statuses or self.statuses[step] != status:
            if status != "WAIT":
                # Clear line and overwrite if it was waiting (simple implementation just prints new lines)
                pass
            self.render_row(step, status)
            self.statuses[step] = status

    def finish(self, success: bool):
        """Prints the final system ready status."""
        if success:
            sys.stdout.write(f"\n{ANSI.GREEN}{ANSI.BOLD}System Ready.{ANSI.RESET}\n\n")
        else:
            sys.stdout.write(f"\n{ANSI.RED}{ANSI.BOLD}System Startup Failed. Check logs.{ANSI.RESET}\n\n")
        sys.stdout.flush()
        time.sleep(1)

    def print_drone_table(self, drones, mission_status="IDLE"):
        """Prints a colored status table for drones during GCS operation."""
        sys.stdout.write(f"\n{ANSI.CYAN}{ANSI.BOLD}--- SWARM STATUS ---{ANSI.RESET}\n")
        header = f"{ANSI.BOLD}{'ID':<10} | {'MODE':<10} | {'BATT':<6} | {'ALT':<6} | {'HEALTH'}{ANSI.RESET}"
        sys.stdout.write(f"{header}\n")
        sys.stdout.write("-" * 50 + "\n")
        
        for drone in drones:
            state = drone.state
            
            mode_color = ANSI.GREEN if state.is_armed else ANSI.YELLOW
            mode_text = "ARMED" if state.is_armed else "DISARMED"
            
            batt_color = ANSI.GREEN if state.battery_remaining > 20 else ANSI.RED
            batt_text = f"{state.battery_remaining:.0f}%"
            
            alt_text = f"{state.position[2]:.1f}m"
            
            sys.stdout.write(f"{drone.agent_id:<10} | {mode_color}{mode_text:<10}{ANSI.RESET} | {batt_color}{batt_text:<6}{ANSI.RESET} | {alt_text:<6} | {ANSI.GREEN}OK{ANSI.RESET}\n")
            
        sys.stdout.write(f"\n{ANSI.BLUE}Mission Status:{ANSI.RESET} {mission_status}\n\n")
        sys.stdout.flush()
