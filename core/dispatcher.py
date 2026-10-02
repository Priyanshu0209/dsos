"""
Command Dispatcher for DSOS.
Routes commands from the UI/Parser to the Swarm Manager.
"""
import logging
from typing import List, Any, Optional

class CommandDispatcher:
    def __init__(self, swarm_manager: Any):
        self.logger = logging.getLogger("CommandDispatcher")
        self.swarm_manager = swarm_manager

    async def dispatch(self, command: str, args: List[Any], target_ids: Optional[List[str]] = None) -> List[str]:
        """
        Dispatch a command to the swarm manager.
        """
        self.logger.info(f"Dispatching '{command}' to {target_ids}")
        return await self.swarm_manager.execute_command(command, args, target_ids)
