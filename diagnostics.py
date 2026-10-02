#!/usr/bin/env python3
"""
DSOS Diagnostics Tool.
Validates environment, dependencies, and configuration, printing PASS/FAIL
and remediation steps for any failed checks.
"""
import os
import sys
import shutil
import asyncio

# Ensure parent directory is in path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
# Always restart in virtual environment if available
venv_python = os.path.join(parent_dir, "dsos", "venv", "bin", "python")
if os.path.exists(venv_python) and sys.executable != venv_python:
    print("Restarting diagnostics in virtual environment...")
    os.execl(venv_python, venv_python, *sys.argv)

from dsos.config.settings import settings

class Diagnostics:
    def __init__(self):
        self.failed_checks = 0

    def print_result(self, name: str, success: bool, remediation: str = ""):
        status = "\033[92mPASS\033[0m" if success else "\033[91mFAIL\033[0m"
        print(f"[{status}] {name}")
        if not success:
            self.failed_checks += 1
            if remediation:
                print(f"       -> Remediation: {remediation}")

    def run_all(self):
        print("========================================")
        print("DSOS DIAGNOSTICS MODE")
        print("========================================")

        # Python Version Check
        has_python312 = sys.version_info >= (3, 12)
        self.print_result(
            "Python 3.12+ Installed", 
            has_python312, 
            "Install Python 3.12 or higher and ensure the virtual environment uses it."
        )

        # MAVSDK Check
        try:
            import mavsdk
            has_mavsdk = True
        except ImportError:
            has_mavsdk = False
        self.print_result(
            "MAVSDK Python Library Installed", 
            has_mavsdk, 
            "Run 'pip install mavsdk' or 'pip install -r requirements.txt'."
        )

        # PSUtil Check
        try:
            import psutil
            has_psutil = True
        except ImportError:
            has_psutil = False
        self.print_result(
            "psutil Python Library Installed", 
            has_psutil, 
            "Run 'pip install psutil' or 'pip install -r requirements.txt'."
        )

        # Textual Check
        try:
            import textual
            has_textual = True
        except ImportError:
            has_textual = False
        self.print_result(
            "textual Python Library Installed", 
            has_textual, 
            "Run 'pip install textual' or 'pip install -r requirements.txt'."
        )

        # Gazebo Check
        gazebo_path = settings.get('DSOS', 'gazebo_path', 'gzserver')
        # Sometimes 'gz' or 'ign' is used
        if os.path.basename(gazebo_path) in ['gz', 'ign']:
            has_gazebo = shutil.which(gazebo_path) is not None
        else:
            has_gazebo = shutil.which(gazebo_path) is not None
            
        # fallback check if default doesn't match
        if not has_gazebo and shutil.which('gz'):
            has_gazebo = True
            gazebo_path = 'gz'
            
        self.print_result(
            f"Gazebo Simulator Found ({gazebo_path})", 
            has_gazebo, 
            "Install Gazebo Harmonic (gz) or Gazebo Classic (gzserver) and add it to PATH. Update settings.ini if installed elsewhere."
        )

        # PX4 Check
        px4_path = settings.get('DSOS', 'px4_path', 'px4')
        px4_path_expanded = os.path.expanduser(px4_path)
        px4_bin = os.path.join(px4_path_expanded, "build", "px4_sitl_default", "bin", "px4")
        
        has_px4 = os.path.exists(px4_bin) or shutil.which(px4_path) is not None
        self.print_result(
            f"PX4 SITL Built ({px4_bin if os.path.exists(px4_bin) else px4_path})", 
            has_px4, 
            "Clone PX4-Autopilot, run 'make px4_sitl gz_x500', and configure 'px4_path' in dsos/config/settings.ini to point to the PX4-Autopilot directory."
        )

        # QGroundControl Check
        qgc_path = settings.get('DSOS', 'qgc_path', 'QGroundControl')
        has_qgc = shutil.which(qgc_path) is not None or os.path.exists(os.path.expanduser(qgc_path))
        self.print_result(
            f"QGroundControl Found ({qgc_path})", 
            has_qgc, 
            "Download QGroundControl.AppImage, make it executable, and update 'qgc_path' in settings.ini. Optional, but recommended."
        )

        # Port Bindings Check (Check if defaults are already in use)
        # It's a common failure if another instance or stray processes hold the ports.
        import socket
        ports_to_check = [14540, 14541, 14542, 14580, 14581, 14582]
        ports_in_use = []
        for port in ports_to_check:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                try:
                    s.bind(('0.0.0.0', port))
                except OSError:
                    ports_in_use.append(port)
                    
        self.print_result(
            "Required UDP Ports Available", 
            len(ports_in_use) == 0, 
            f"Ports {ports_in_use} are already in use. Run 'killall px4' or check for stale dsos/mavsdk processes."
        )

        print("\n========================================")
        if self.failed_checks == 0:
            print("\033[92mALL DIAGNOSTICS PASSED! System is ready to run.\033[0m")
        else:
            print(f"\033[91m{self.failed_checks} DIAGNOSTICS FAILED! Please apply remediation steps before running DSOS.\033[0m")
        print("========================================\n")

if __name__ == '__main__':
    diag = Diagnostics()
    diag.run_all()
