#!/usr/bin/env python3

import asyncio
import sys
import time
from pathlib import Path

# Add project root to path
current_dir = Path(__file__).parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

from dsos.main import DSOS
from dsos.config.settings import settings

async def test_arm_takeoff():
    print("Starting DSOS Integration Test Pipeline...")
    
    # Initialize DSOS
    app = DSOS()
    
    # Run the boot sequence and backend connections
    print("Initializing DSOS (this starts PX4 instances and connects MAVSDK)...")
    if not await app.initialize():
        print("Initialization failed.")
        return False
        
    print("DSOS Initialized.")
    
    # Ensure there are drones
    if not app.drones:
        print("No drones were initialized!")
        return False
        
    # Wait for drones to fully connect and be ready
    print("Waiting for drones to receive telemetry and heartbeat...")
    for i in range(30):
        all_ready = True
        for drone in app.drones:
            # Check if backend connected and health is OK
            # Note: DroneAgent's status might not update instantly
            if not drone._backend_connected:
                all_ready = False
                break
            
            # Print status periodically
            if i % 5 == 0:
                print(f"  Drone {drone.agent_id} | Connected: {drone._backend_connected} | Armed: {drone.is_armed()} | Health: {drone.status.health}")
                
        if all_ready:
            print("All drones backend connected.")
            break
            
        await asyncio.sleep(1)
        
    print("Attempting ARM via Command Dispatcher...")
    
    # Trigger command dispatcher just like UI does
    drone_ids = [d.agent_id for d in app.drones]
    
    try:
        # Arm
        await app.gcs.dispatcher.dispatch("arm_motors", [], drone_ids)
        print("Dispatched ARM.")
        
        # Wait a bit
        print("Waiting 5 seconds for ARM to process...")
        await asyncio.sleep(5)
        
        # Check if armed
        armed_count = 0
        for drone in app.drones:
            is_armed = drone.is_armed()
            print(f"  Drone {drone.agent_id} Armed: {is_armed}")
            if is_armed:
                armed_count += 1
                
        if armed_count == 0:
            print("Failed to arm any drones. Aborting takeoff.")
            return False
            
        # Takeoff
        print("Attempting TAKEOFF via Command Dispatcher...")
        await app.gcs.dispatcher.dispatch("takeoff", [], drone_ids)
        print("Dispatched TAKEOFF.")
        
        # Wait a bit for takeoff
        print("Waiting 15 seconds for TAKEOFF to process...")
        for _ in range(15):
            await asyncio.sleep(1)
            
        # Check altitude
        for drone in app.drones:
            alt = drone.status.position[2]
            print(f"  Drone {drone.agent_id} Altitude: {alt}m")
            
    finally:
        # Land and stop
        print("Attempting LAND via Command Dispatcher...")
        await app.gcs.dispatcher.dispatch("land", [], drone_ids)
        print("Dispatched LAND.")
        
        print("Waiting 10 seconds for LAND to process...")
        await asyncio.sleep(10)
        
        print("Stopping DSOS...")
        await app.stop()
        print("DSOS stopped.")
        
    return True

if __name__ == '__main__':
    success = asyncio.run(test_arm_takeoff())
    sys.exit(0 if success else 1)
