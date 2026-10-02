"""
Telemetry and logging system for DSOS.
Handles data collection, storage, and visualization preparation.
"""
import asyncio
import json
import time
import logging
from typing import Dict, List, Optional, Any, Callable
from collections import deque
from dataclasses import dataclass, asdict
import threading
import csv

from dsos.config.settings import settings


@dataclass
class TelemetryPacket:
    """Container for telemetry data."""
    timestamp: float
    agent_id: str
    position: List[float]  # [x, y, z]
    velocity: List[float]  # [vx, vy, vz]
    orientation: List[float]  # [roll, pitch, yaw]
    battery_level: float
    health: float
    status: str
    sensor_data: Dict[str, Any] = None


class TelemetryManager:
    """Manages telemetry data collection and storage."""

    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.logger = logging.getLogger(f"telemetry.{agent_id}")
        self.max_history_size = settings.getint('DSOS', 'telemetry_history_size', 1000)
        self.update_rate_hz = settings.getint('DSOS', 'telemetry_update_rate_hz', 10)

        # Telemetry storage
        self.telemetry_history: deque = deque(maxlen=self.max_history_size)
        self.latest_telemetry: Optional[TelemetryPacket] = None

        # Subscription system for real-time updates
        self.subscribers: List[Callable[[TelemetryPacket], None]] = []
        self._lock = threading.Lock()

        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.start_time = time.time()

    def add_telemetry(self, position: List[float], velocity: List[float],
                      orientation: List[float], battery_level: float,
                      health: float, status: str,
                      sensor_data: Optional[Dict[str, Any]] = None) -> None:
        """Add a new telemetry packet."""
        packet = TelemetryPacket(
            timestamp=time.time(),
            agent_id=self.agent_id,
            position=position.copy(),
            velocity=velocity.copy(),
            orientation=orientation.copy(),
            battery_level=battery_level,
            health=health,
            status=status,
            sensor_data=sensor_data.copy() if sensor_data else {}
        )

        with self._lock:
            self.telemetry_history.append(packet)
            self.latest_telemetry = packet
            self.packets_sent += 1

            # Notify subscribers
            for subscriber in self.subscribers:
                try:
                    subscriber(packet)
                except Exception as e:
                    self.logger.error(f"Error in telemetry subscriber: {e}")

    def get_latest_telemetry(self) -> Optional[TelemetryPacket]:
        """Get the most recent telemetry packet."""
        with self._lock:
            return self.latest_telemetry

    def get_telemetry_history(self, limit: Optional[int] = None) -> List[TelemetryPacket]:
        """Get telemetry history."""
        with self._lock:
            if limit is None:
                return list(self.telemetry_history)
            else:
                return list(self.telemetry_history)[-limit:]

    def subscribe(self, callback: Callable[[TelemetryPacket], None]) -> None:
        """Subscribe to telemetry updates."""
        with self._lock:
            self.subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[TelemetryPacket], None]) -> None:
        """Unsubscribe from telemetry updates."""
        with self._lock:
            if callback in self.subscribers:
                self.subscribers.remove(callback)

    def get_statistics(self) -> Dict[str, Any]:
        """Get telemetry statistics."""
        uptime = time.time() - self.start_time
        return {
            'agent_id': self.agent_id,
            'uptime_seconds': uptime,
            'packets_sent': self.packets_sent,
            'packets_received': self.packets_received,
            'send_rate_hz': self.packets_sent / max(uptime, 1.0),
            'history_size': len(self.telemetry_history),
            'max_history_size': self.max_history_size
        }

    def clear_history(self) -> None:
        """Clear telemetry history."""
        with self._lock:
            self.telemetry_history.clear()
            self.latest_telemetry = None

    def export_telemetry(self, filepath: str) -> bool:
        """Export telemetry history to a JSON file."""
        try:
            with self._lock:
                data = [asdict(packet) for packet in self.telemetry_history]
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            self.logger.info(f"Exported telemetry to {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to export telemetry: {e}")
            return False

    def import_telemetry(self, filepath: str) -> bool:
        """Import telemetry history from a JSON file."""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            packets = []
            for item in data:
                packet = TelemetryPacket(**item)
                packets.append(packet)
            with self._lock:
                self.telemetry_history = deque(packets, maxlen=self.max_history_size)
                if packets:
                    self.latest_telemetry = packets[-1]
            self.logger.info(f"Imported telemetry from {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to import telemetry: {e}")
            return False

    def export_telemetry_csv(self, filepath: str) -> bool:
        """Export telemetry history to a CSV file."""
        try:
            with self._lock:
                if not self.telemetry_history:
                    return False
                data = [asdict(packet) for packet in self.telemetry_history]
                
            if not data:
                return False
                
            keys = data[0].keys()
            with open(filepath, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                for row in data:
                    row['position'] = json.dumps(row.get('position', []))
                    row['velocity'] = json.dumps(row.get('velocity', []))
                    row['orientation'] = json.dumps(row.get('orientation', []))
                    row['sensor_data'] = json.dumps(row.get('sensor_data', {}))
                    writer.writerow(row)
                    
            self.logger.info(f"Exported telemetry to {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to export telemetry to CSV: {e}")
            return False


class TelemetryAggregator:
    """Aggregates telemetry from multiple agents for swarm-level analysis."""

    def __init__(self):
        self.logger = logging.getLogger("telemetry.aggregator")
        self.agent_telemetry: Dict[str, TelemetryManager] = {}
        self.swarm_history: deque = deque(maxlen=10000)  # Larger buffer for swarm data
        self._lock = threading.Lock()

    def register_agent(self, agent_id: str, telemetry_manager: TelemetryManager) -> None:
        """Register an agent's telemetry manager."""
        with self._lock:
            self.agent_telemetry[agent_id] = telemetry_manager
            # Subscribe to the agent's telemetry
            telemetry_manager.subscribe(self._on_agent_telemetry)

    def unregister_agent(self, agent_id: str) -> None:
        """Unregister an agent's telemetry manager."""
        with self._lock:
            if agent_id in self.agent_telemetry:
                tm = self.agent_telemetry.pop(agent_id)
                tm.unsubscribe(self._on_agent_telemetry)

    def _on_agent_telemetry(self, packet: TelemetryPacket) -> None:
        """Handle incoming telemetry from an agent."""
        with self._lock:
            self.swarm_history.append(packet)
            # Could perform swarm-level analysis here

    def get_swarm_state(self) -> Dict[str, Any]:
        """Get the current state of the entire swarm."""
        with self._lock:
            agent_states = {}
            for agent_id, tm in self.agent_telemetry.items():
                latest = tm.get_latest_telemetry()
                if latest:
                    agent_states[agent_id] = asdict(latest)

            return {
                'timestamp': time.time(),
                'agents': agent_states,
                'agent_count': len(agent_states),
                'swarm_history_size': len(self.swarm_history)
            }

    def get_agent_telemetry(self, agent_id: str) -> Optional[List[TelemetryPacket]]:
        """Get telemetry history for a specific agent."""
        with self._lock:
            if agent_id in self.agent_telemetry:
                return list(self.agent_telemetry[agent_id].get_telemetry_history())
            return None

    def export_swarm_telemetry(self, filepath: str) -> bool:
        """Export swarm telemetry to a JSON file."""
        try:
            with self._lock:
                swarm_state = self.get_swarm_state()
            with open(filepath, 'w') as f:
                json.dump(swarm_state, f, indent=2)
            self.logger.info(f"Exported swarm telemetry to {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to export swarm telemetry: {e}")
            return False

    def export_swarm_telemetry_csv(self, filepath: str) -> bool:
        """Export swarm telemetry history to a CSV file."""
        try:
            with self._lock:
                if not self.swarm_history:
                    return False
                data = [asdict(packet) for packet in self.swarm_history]
                
            if not data:
                return False
                
            keys = data[0].keys()
            with open(filepath, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                for row in data:
                    row['position'] = json.dumps(row.get('position', []))
                    row['velocity'] = json.dumps(row.get('velocity', []))
                    row['orientation'] = json.dumps(row.get('orientation', []))
                    row['sensor_data'] = json.dumps(row.get('sensor_data', {}))
                    writer.writerow(row)
                    
            self.logger.info(f"Exported swarm telemetry to {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to export swarm telemetry to CSV: {e}")
            return False


class Logger:
    """Enhanced logging system for DSOS."""

    def __init__(self, name: str):
        self.logger = logging.getLogger(name)
        # Set up file handler if not already configured
        if not self.logger.handlers:
            self.logger.setLevel(logging.INFO)
            # Console handler
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)

            # File handler
            try:
                fh = logging.FileHandler('logs/dsos.log')
                fh.setLevel(logging.DEBUG)
                fh.setFormatter(formatter)
                self.logger.addHandler(fh)
            except Exception:
                pass  # If we can't create log file, continue with console only

    def debug(self, msg: str) -> None:
        self.logger.debug(msg)

    def info(self, msg: str) -> None:
        self.logger.info(msg)

    def warning(self, msg: str) -> None:
        self.logger.warning(msg)

    def error(self, msg: str) -> None:
        self.logger.error(msg)

    def critical(self, msg: str) -> None:
        self.logger.critical(msg)


# Make enums available
from enum import Enum