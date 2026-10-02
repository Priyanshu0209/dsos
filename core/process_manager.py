"""
Process Manager for DSOS.
Manages background processes (like Gazebo, PX4 SITL, MAVSDK server) with asyncio,
handling logging, restarts, and graceful shutdowns.
"""
import asyncio
import logging
import os
import signal
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
from datetime import datetime

@dataclass
class ProcessConfig:
    name: str
    command: List[str]
    cwd: Optional[str] = None
    env: Optional[Dict[str, str]] = None
    restart_on_failure: bool = False
    max_retries: int = 3
    startup_timeout_sec: float = 10.0
    stop_timeout_sec: float = 5.0
    depends_on: List[str] = field(default_factory=list)
    success_pattern: Optional[str] = None  # Regex pattern to detect successful startup in stdout

class ProcessManager:
    """Manages background processes with health monitoring and automatic restarts."""
    
    def __init__(self, log_dir: str = "logs", critical_failure_callback: Optional[Callable[[str], None]] = None):
        self.logger = logging.getLogger("ProcessManager")
        self.log_dir = log_dir
        self.critical_failure_callback = critical_failure_callback
        self.processes: Dict[str, asyncio.subprocess.Process] = {}
        self.configs: Dict[str, ProcessConfig] = {}
        self.retry_counts: Dict[str, int] = {}
        self._running = False
        self._shutdown_event = asyncio.Event()
        self._monitor_task: Optional[asyncio.Task] = None
        self._log_files: Dict[str, Any] = {}

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

    def get_log_path(self, name: str) -> Optional[str]:
        """Get the most recent log file path for a process."""
        import glob
        try:
            files = glob.glob(os.path.join(self.log_dir, f"{name}_*.log"))
            if files:
                return max(files, key=os.path.getmtime)
        except Exception:
            pass
        return None

    def add_process(self, config: ProcessConfig) -> None:
        """Register a new process configuration."""
        self.configs[config.name] = config
        self.retry_counts[config.name] = 0
        self.logger.info(f"Registered process: {config.name}")

    async def start_all(self) -> bool:
        """Start all registered processes considering dependencies."""
        self._running = True
        self._monitor_task = asyncio.create_task(self._monitor_loop())
        
        # Simple dependency resolution (assumes no circular dependencies)
        started: set = set()
        pending = set(self.configs.keys())
        
        while pending:
            ready_to_start = [
                name for name in pending
                if all(dep in started for dep in self.configs[name].depends_on)
            ]
            
            if not ready_to_start:
                self.logger.error("Circular dependency or missing dependency detected!")
                return False
                
            tasks = [self.start_process(name) for name in ready_to_start]
            results = await asyncio.gather(*tasks, return_exceptions=True)
            
            for name, result in zip(ready_to_start, results):
                if isinstance(result, Exception) or not result:
                    self.logger.error(f"Failed to start required process: {name}")
                    return False
                started.add(name)
                pending.remove(name)
                
        return True

    async def start_process(self, name: str) -> bool:
        """Start a specific process."""
        if name not in self.configs:
            self.logger.error(f"Process config not found for {name}")
            return False
            
        config = self.configs[name]
        
        if name in self.processes and self.processes[name].returncode is None:
            self.logger.info(f"Process {name} is already running.")
            return True

        self.logger.info(f"Starting process: {name} (Command: {' '.join(config.command)})")
        
        # Setup log files for stdout/stderr
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(self.log_dir, f"{name}_{timestamp}.log")
        log_file = open(log_path, "w")
        self._log_files[name] = log_file

        env = os.environ.copy()
        if config.env:
            env.update(config.env)

        # We need to capture output to monitor for success_pattern if specified
        # But we also want to log it to file. We can use a pipe and redirect manually.
        try:
            if config.success_pattern:
                process = await asyncio.create_subprocess_exec(
                    *config.command,
                    cwd=config.cwd,
                    env=env,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT,
                    start_new_session=True,
                    limit=1024*1024*5  # 5MB limit to prevent LimitOverrunError
                )
                self.processes[name] = process
                self.logger.info(f"Process {name} started with PID {process.pid}")
                
                # Monitor output
                import re
                success_regex = re.compile(config.success_pattern)
                success_found = False
                
                async def pipe_reader():
                    nonlocal success_found
                    while True:
                        line = await process.stdout.readline()
                        if not line:
                            break
                        line_str = line.decode('utf-8', errors='replace')
                        log_file.write(line_str)
                        log_file.flush()
                        if not success_found and success_regex.search(line_str):
                            success_found = True
                            self.logger.info(f"Process {name} reached success state.")
                            
                # Start reading in background
                asyncio.create_task(pipe_reader())
                
                # Wait for success pattern or timeout. A process that exits early or
                # never reaches the expected startup marker must be treated as failed.
                start_time = asyncio.get_running_loop().time()
                while True:
                    if process.returncode is not None:
                        self.logger.error(f"Process {name} exited before reaching success state.")
                        return False
                    if success_found:
                        return True
                    if asyncio.get_running_loop().time() - start_time > config.startup_timeout_sec:
                        self.logger.warning(f"Process {name} timed out waiting for success pattern.")
                        return False
                    await asyncio.sleep(0.1)
                
            else:
                process = await asyncio.create_subprocess_exec(
                    *config.command,
                    cwd=config.cwd,
                    env=env,
                    stdout=log_file,
                    stderr=asyncio.subprocess.STDOUT,
                    start_new_session=True
                )
                self.processes[name] = process
                self.logger.info(f"Process {name} started with PID {process.pid}")
                
                # Simple wait for process to settle
                await asyncio.sleep(1.0)
                if process.returncode is not None:
                    self.logger.error(f"Process {name} exited immediately with code {process.returncode}")
                    return False
                    
                return True
            
        except Exception as e:
            self.logger.error(f"Error starting process {name}: {e}")
            return False

    async def _monitor_loop(self) -> None:
        """Monitor running processes and restart if needed."""
        while self._running:
            for name, process in list(self.processes.items()):
                if process.returncode is not None:
                    # Process exited
                    config = self.configs[name]
                    self.logger.warning(f"Process {name} (PID {process.pid}) exited with code {process.returncode}")
                    
                    if config.restart_on_failure and self.retry_counts[name] < config.max_retries and self._running:
                        self.retry_counts[name] += 1
                        self.logger.info(f"Restarting process {name} (Retry {self.retry_counts[name]}/{config.max_retries})")
                        await self.start_process(name)
                    else:
                        if self._running:
                            err_msg = f"Process {name} failed critically and cannot be recovered."
                            self.logger.error(err_msg)
                            if self.critical_failure_callback:
                                self.critical_failure_callback(name)
            
            await asyncio.sleep(2.0)

    async def stop_process(self, name: str) -> None:
        """Stop a specific process gracefully."""
        if name not in self.processes:
            return
            
        process = self.processes[name]
        config = self.configs[name]
        
        if process.returncode is None:
            self.logger.info(f"Stopping process {name} (PID {process.pid})")
            try:
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                
                try:
                    await asyncio.wait_for(process.wait(), timeout=config.stop_timeout_sec)
                except asyncio.TimeoutError:
                    self.logger.warning(f"Process {name} did not stop gracefully. Sending SIGKILL.")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    await process.wait()
            except ProcessLookupError:
                pass # Already dead
            except Exception as e:
                self.logger.error(f"Error stopping process {name}: {e}")

        # Close log file
        if name in self._log_files:
            try:
                self._log_files[name].close()
                del self._log_files[name]
            except Exception:
                pass

        if name in self.processes:
            del self.processes[name]
        self.logger.info(f"Process {name} stopped.")

    async def stop_all(self) -> None:
        """Stop all processes."""
        self._running = False
        self._shutdown_event.set()
        
        if self._monitor_task:
            self._monitor_task.cancel()
            try:
                await self._monitor_task
            except asyncio.CancelledError:
                pass

        self.logger.info("Stopping all background processes...")
        # Stop in reverse dependency order (simple implementation: just stop all concurrently for now)
        tasks = [self.stop_process(name) for name in list(self.processes.keys())]
        if tasks:
            await asyncio.gather(*tasks)
            
        # Kill tracked external PIDs
        for pid in getattr(self, 'external_pids', []):
            try:
                self.logger.info(f"Killing tracked external process PID: {pid}")
                import os, signal
                os.kill(pid, signal.SIGTERM)
                # We won't block on waiting for external PIDs here to keep it simple, 
                # but we'll send SIGKILL shortly after if needed in a real scenario.
            except ProcessLookupError:
                pass
            except Exception as e:
                self.logger.error(f"Error killing external PID {pid}: {e}")
        
        if hasattr(self, 'external_pids'):
            self.external_pids.clear()
            
        self.logger.info("All background processes stopped.")

    def add_external_pid(self, pid: int) -> None:
        """Track an external process by PID so it is killed on shutdown."""
        if not hasattr(self, 'external_pids'):
            self.external_pids = []
        if pid not in self.external_pids:
            self.external_pids.append(pid)
            self.logger.info(f"Registered external PID {pid} for cleanup.")
