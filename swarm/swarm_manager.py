"""
Swarm Manager implementations for DSOS.
Provides distributed consensus and cooperative task allocation for leaderless swarms.
"""
import logging
import time
import asyncio
import time
import math
import json
from typing import Dict, List, Optional, Any, Callable
import logging

from dsos.agents.drone_agent import DroneAgent, AgentState
from dsos.communication.mesh_network import MeshNetwork, MeshMessage, MessageType
from dsos.swarm.knowledge_base import DistributedKnowledgeBase
from dsos.config.settings import settings

class ConsensusProtocol:
    """Implements distributed consensus for leaderless swarms (e.g., Raft or Paxos simplified)."""
    
    def __init__(self):
        self.logger = logging.getLogger("ConsensusProtocol")
        self.proposals: Dict[str, Dict] = {}  # proposal_id -> {votes: int, timestamp: float, data: Any}
        
    async def update_consensus(self, drone_id: str, knowledge_base: Any,
                             neighbors: Dict[str, Any], mesh_network: Any) -> None:
        """Periodic check for required consensus on global swarm state."""
        # Clean up old proposals
        current_time = time.time()
        expired = [pid for pid, data in self.proposals.items() if current_time - data['timestamp'] > 10.0]
        for pid in expired:
            del self.proposals[pid]

    async def propose(self, drone_id: str, proposal_id: str, data: Any, mesh_network: Any) -> None:
        """Propose a change to the swarm."""
        self.proposals[proposal_id] = {'votes': 1, 'timestamp': time.time(), 'data': data}
        
        # Broadcast proposal
        message = MeshMessage(
            msg_id=f"prop_{proposal_id}_{int(time.time()*1000)}",
            msg_type=MessageType.DATA,
            sender_id=drone_id,
            recipient_id=None,
            timestamp=time.time(),
            payload={'type': 'consensus_proposal', 'id': proposal_id, 'data': data},
            ttl=settings.getint('DSOS', 'message_ttl', 5)
        )
        if mesh_network:
            await mesh_network.send_message(message)

    async def handle_proposal(self, drone_id: str, proposal_id: str, data: Any, mesh_network: Any) -> None:
        """Handle incoming proposal and vote if appropriate."""
        if proposal_id not in self.proposals:
            self.proposals[proposal_id] = {'votes': 1, 'timestamp': time.time(), 'data': data}
            # Cast vote
            message = MeshMessage(
                msg_id=f"vote_{proposal_id}_{int(time.time()*1000)}",
                msg_type=MessageType.DATA,
                sender_id=drone_id,
                recipient_id=None,
                timestamp=time.time(),
                payload={'type': 'consensus_vote', 'id': proposal_id},
                ttl=settings.getint('DSOS', 'message_ttl', 5)
            )
            if mesh_network:
                await mesh_network.send_message(message)

    async def handle_vote(self, proposal_id: str, swarm_size: int) -> bool:
        """Register a vote. Returns True if consensus is reached."""
        if proposal_id in self.proposals:
            self.proposals[proposal_id]['votes'] += 1
            # Majority consensus
            if self.proposals[proposal_id]['votes'] > (swarm_size / 2):
                return True
        return False
        
    async def shutdown(self) -> None:
        pass


class TaskAllocator:
    """Implements Contract Net Protocol (CNP) for cooperative task allocation."""
    
    def __init__(self):
        self.logger = logging.getLogger("TaskAllocator")
        self.bids: Dict[str, Dict[str, float]] = {}  # task_id -> {drone_id: bid_value}
        
    async def allocate_tasks(self, drone_id: str, knowledge_base: Any,
                           neighbors: Dict[str, Any], mesh_network: Any,
                           task_queue: Any) -> None:
        """Periodically evaluate if any tasks need bidding or allocation."""
        pass
        
    async def announce_task(self, drone_id: str, task: Dict[str, Any], mesh_network: Any):
        """Announce a task to the swarm."""
        task_id = task.get('id', f"task_{int(time.time()*1000)}")
        self.bids[task_id] = {}
        
        message = MeshMessage(
            msg_id=f"task_ann_{task_id}",
            msg_type=MessageType.COMMAND,
            sender_id=drone_id,
            recipient_id=None,
            timestamp=time.time(),
            payload={'type': 'task_announcement', 'task': task},
            ttl=settings.getint('DSOS', 'message_ttl', 5)
        )
        if mesh_network:
            await mesh_network.send_message(message)

    async def calculate_bid(self, task: Dict[str, Any], state: Any) -> float:
        """Calculate a bid for a task based on distance and battery."""
        # Lower bid is better
        target = task.get('target_position', [0, 0, 0])
        current_pos = state.position if hasattr(state, 'position') and state.position else [0, 0, 0]
        
        distance = sum((a - b) ** 2 for a, b in zip(target, current_pos)) ** 0.5
        battery = state.battery_remaining if hasattr(state, 'battery_remaining') and state.battery_remaining else 100
        
        # Simple heuristic: distance penalizes, battery rewards
        bid = distance * 10 - battery
        return max(0.1, bid)
        
    async def shutdown(self) -> None:
        pass


import math
import json
import os

class FormationEngine:
    """Calculates relative offsets for various drone formations."""
    
    @staticmethod
    def calculate_offsets(formation_type: str, num_drones: int, spacing: float = 2.5) -> List[List[float]]:
        offsets = [[0.0, 0.0, 0.0]]
        if num_drones <= 1:
            return offsets
            
        for i in range(1, num_drones):
            # Calculate offset for drone i (1-indexed for followers)
            if formation_type == "v_formation":
                row = (i + 1) // 2
                col = spacing * row if i % 2 == 1 else -spacing * row
                offsets.append([-spacing * row, col, 0.0])
                
            elif formation_type == "line":
                pos = (i + 1) // 2
                val = spacing * pos if i % 2 == 1 else -spacing * pos
                offsets.append([0.0, val, 0.0])
                
            elif formation_type == "column":
                offsets.append([-spacing * i, 0.0, 0.0])
                
            elif formation_type == "diamond":
                # Leader at 0,0. Then layer 1: 2 drones. Layer 2: 1 drone.
                if i == 1: offsets.append([-spacing, spacing, 0.0])
                elif i == 2: offsets.append([-spacing, -spacing, 0.0])
                elif i == 3: offsets.append([-2 * spacing, 0.0, 0.0])
                else: offsets.append([-spacing * (i//2), spacing if i%2 else -spacing, 0.0])
                
            elif formation_type == "triangle":
                row = int(math.floor((-1 + math.sqrt(1 + 8 * i)) / 2))
                pos_in_row = i - (row * (row + 1)) // 2
                y_offset = (pos_in_row * spacing) - ((row * spacing) / 2)
                offsets.append([-spacing * row, y_offset, 0.0])
                
            elif formation_type == "square":
                side = int(math.ceil(math.sqrt(num_drones)))
                r, c = divmod(i, side)
                offsets.append([-spacing * r, spacing * c, 0.0])
                
            elif formation_type == "rectangle":
                cols = int(math.ceil(math.sqrt(num_drones) * 1.5))
                r, c = divmod(i, cols)
                offsets.append([-spacing * r, spacing * c, 0.0])
                
            elif formation_type == "circle":
                radius = max(spacing, (num_drones * spacing) / (2 * math.pi))
                angle = (2 * math.pi * i) / (num_drones - 1)
                offsets.append([-radius + radius * math.cos(angle), radius * math.sin(angle), 0.0])
                
            elif formation_type == "semi_circle":
                radius = max(spacing, (num_drones * spacing) / math.pi)
                angle = (math.pi * i) / (num_drones - 1) if num_drones > 1 else 0
                offsets.append([-radius * math.sin(angle), radius * math.cos(angle), 0.0])
                
            elif formation_type == "arrow":
                if i < 3:
                    # Arrow head (V shape)
                    row = (i + 1) // 2
                    col = spacing * row if i % 2 == 1 else -spacing * row
                    offsets.append([-spacing * row, col, 0.0])
                else:
                    # Arrow tail
                    offsets.append([-spacing * (i - 1), 0.0, 0.0])
                    
            elif formation_type == "grid":
                cols = int(math.ceil(math.sqrt(num_drones)))
                r, c = divmod(i, cols)
                offsets.append([-spacing * r, spacing * c, 0.0])
                
            elif formation_type == "staggered":
                offsets.append([-spacing * i, spacing if i % 2 == 0 else -spacing, 0.0])
                
            elif formation_type == "echelon_left":
                offsets.append([-spacing * i, -spacing * i, 0.0])
                
            elif formation_type == "echelon_right":
                offsets.append([-spacing * i, spacing * i, 0.0])
                
            elif formation_type == "horizontal_wall":
                # Same as line but spaced out
                val = spacing * i
                offsets.append([0.0, val, 0.0])
                
            elif formation_type == "vertical_wall":
                cols = int(math.ceil(math.sqrt(num_drones)))
                r, c = divmod(i, cols)
                offsets.append([0.0, spacing * c, -spacing * r]) # -Z is up in NED
                
            elif formation_type == "hexagon":
                radius = spacing
                angle = (2 * math.pi * (i % 6)) / 6
                ring = (i // 6) + 1
                offsets.append([ring * radius * math.cos(angle), ring * radius * math.sin(angle), 0.0])
                
            elif formation_type == "pentagon":
                radius = spacing
                angle = (2 * math.pi * (i % 5)) / 5
                ring = (i // 5) + 1
                offsets.append([ring * radius * math.cos(angle), ring * radius * math.sin(angle), 0.0])
                
            elif formation_type == "spiral":
                radius = spacing * (1 + 0.2 * i)
                angle = i * (math.pi / 4)
                offsets.append([radius * math.cos(angle), radius * math.sin(angle), 0.0])
                
            elif formation_type == "custom":
                # Attempt to load from JSON
                try:
                    with open("dsos/formations/custom.json", "r") as f:
                        custom_data = json.load(f)
                        if i < len(custom_data):
                            offsets.append(custom_data[i])
                        else:
                            offsets.append([-spacing * i, 0.0, 0.0])
                except:
                    offsets.append([-spacing * i, 0.0, 0.0])
            else:
                offsets.append([-spacing * i, 0.0, 0.0]) # fallback to column
                
        return offsets


class SwarmManager:
    """Manages the swarm and routes commands to individual DroneAgents."""

    def __init__(self, drones: List[Any]):
        self.logger = logging.getLogger("SwarmManager")
        self.drones = {drone.agent_id: drone for drone in drones}
        self.current_formation = "column"
        self.current_spacing = 2.5
        self.min_spacing = 1.0  # Safety: absolute minimum distance between drones
        self.min_altitude = 1.0 # Safety: minimum altitude to prevent crash
        
    def _check_collision_safety(self, positions: List[List[float]]) -> bool:
        """Check if any two positions are dangerously close."""
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = sum((a - b)**2 for a, b in zip(positions[i], positions[j])) ** 0.5
                if dist < self.min_spacing:
                    return False
        return True

    async def execute_command(self, command: str, args: List[Any], target_ids: Optional[List[str]] = None) -> List[str]:
        """Route command to target drones and collect results."""
        if command in ["move_forward", "move_backward", "move_left", "move_right", "move_up", "move_down", "move_velocity", "rotate_left", "rotate_right"]:
            print("Movement Command Received")
            
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.logger.info(f"[{ts}] [magenta][SWARMMANAGER RECEIVED][/magenta] '{command}' for {target_ids}")
        
        targets = [self.drones[tid] for tid in (target_ids or self.drones.keys()) if tid in self.drones]
        if not targets:
            self.logger.warning("No valid targets for command")
            return [f"[{ts}] [red][FAILED][/red] No valid targets selected."]

        if command == "form":
            print("Formation Selected")
            self.current_formation = args[0] if args else "v_formation"
            try: form_alt = float(args[1])
            except: form_alt = 15.0
            spacing = self.current_spacing

            leader = targets[0]
            base_pos = list(leader.status.position) if leader.status.position else [0.0, 0.0, 0.0]
            base_pos[2] = -form_alt # Target altitude for the entire formation
            
            print("Target Positions Calculated")
            offsets = FormationEngine.calculate_offsets(self.current_formation, len(targets), spacing)
            
            target_positions = []
            for i in range(len(targets)):
                target_pos = [
                    base_pos[0] + offsets[i][0],
                    base_pos[1] + offsets[i][1],
                    base_pos[2] + offsets[i][2]
                ]
                target_positions.append(target_pos)
                
            async def safe_goto(drone, target_pos, index):
                print(f"Drone {index+1} Moving")
                try:
                    # 1. Stagger altitude (+3m per drone index) to avoid 2D plane collisions
                    stagger_alt = -form_alt - (3.0 * index)
                    curr_pos = list(drone.status.position) if drone.status.position else [0.0, 0.0, 0.0]
                    
                    stagger_climb = [curr_pos[0], curr_pos[1], stagger_alt]
                    await drone.goto_position(stagger_climb)
                    await asyncio.sleep(2.0) # Wait for climb
                    
                    # 2. Move horizontally to target slot at staggered altitude
                    stagger_target = [target_pos[0], target_pos[1], stagger_alt]
                    await drone.goto_position(stagger_target)
                    await asyncio.sleep(4.0) # Wait for horizontal translation
                    
                    # 3. Descend/Climb to final formation altitude
                    res = await drone.goto_position(target_pos)
                    return f"{drone.agent_id} formed {self.current_formation}: {res}"
                except Exception as e:
                    return f"{drone.agent_id} Failed: {e}"

            results = await asyncio.gather(*(safe_goto(drone, target_positions[i], i) for i, drone in enumerate(targets)))
            print("Formation Complete")
            return list(results)
            
        # ===== SWARM ORBIT ROTATION =====
        if command in ["rotate_left", "rotate_right"]:
            angle_deg = float(args[0]) if args else 15.0
            if command == "rotate_left":
                angle_deg = -angle_deg
            
            if not hasattr(self, 'formation_yaw'):
                self.formation_yaw = 0.0
            self.formation_yaw += math.radians(angle_deg)
            
            if not hasattr(self, 'current_formation') or self.current_formation == "None":
                self.current_formation = "v_formation"
            
            # Calculate new rotated positions
            offsets = FormationEngine.calculate_offsets(
                self.current_formation, len(targets), 3.0
            )
            
            base_pos = list(targets[0].status.position) if targets[0].status.position else [0.0, 0.0, 0.0]
            
            import math as m
            cos_yaw = m.cos(self.formation_yaw)
            sin_yaw = m.sin(self.formation_yaw)
            
            results = []
            for i, drone in enumerate(targets):
                if i < len(offsets):
                    # Rotate the offset by the new formation yaw
                    rx = offsets[i][0] * cos_yaw - offsets[i][1] * sin_yaw
                    ry = offsets[i][0] * sin_yaw + offsets[i][1] * cos_yaw
                    
                    target_pos = [
                        base_pos[0] + rx,
                        base_pos[1] + ry,
                        base_pos[2] + offsets[i][2]
                    ]
                    # Also set the drone's individual yaw to match the formation's
                    if hasattr(drone, 'rotate'):
                        asyncio.create_task(drone.rotate(math.degrees(self.formation_yaw)))
                        
                    res = await drone.goto_position(target_pos)
                    results.append(f"{drone.agent_id} orbited to {angle_deg}deg: {res}")
            return results
        
        # ===== COMMAND NAME → METHOD NAME MAPPING =====
        # The UI sends command names that may differ from DroneAgent method names.
        # This table ensures every UI command reaches the correct method.
        COMMAND_MAP = {
            "arm":          "arm_motors",
            "disarm":       "disarm_motors",
            "rtl":          "return_to_home",
            "estop":        "land",            # Emergency stop = land immediately
            "move_forward": "move_forward",
            "move_backward":"move_backward",
            "move_left":    "move_left",
            "move_right":   "move_right",
            "move_up":      "move_up",
            "move_down":    "move_down",
            "move_velocity":"move_velocity",
            "takeoff":      "takeoff",
            "land":         "land",
            "hover":        "hover",
            "rotate":       "rotate",
            "rotate_left":  "rotate_left",
            "rotate_right": "rotate_right",
            "goto_position":"goto_position",
        }
        
        # Resolve the actual method name
        method_name = COMMAND_MAP.get(command, command)
        
        results = []
        for drone in targets:
            try:
                func = getattr(drone, method_name, None)
                if func is None:
                    msg = f"{drone.agent_id} Failed: No method '{method_name}' (from command '{command}')"
                    self.logger.error(msg)
                    results.append(msg)
                    continue
                    
                self.logger.info(f"[DISPATCHING] {drone.agent_id}.{method_name}(*{args})")
                if asyncio.iscoroutinefunction(func):
                    res = await func(*args)
                else:
                    res = func(*args)
                self.logger.info(f"[DISPATCH OK] {drone.agent_id}.{method_name} -> {res}")
                results.append(str(res))
            except Exception as e:
                self.logger.error(f"[DISPATCH ERROR] '{method_name}' on {drone.agent_id}: {e}")
                results.append(f"{drone.agent_id} Error: {str(e)}")

        return results

