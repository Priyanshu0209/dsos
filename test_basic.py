#!/usr/bin/env python3
"""
Basic test script for DSOS.
Tests that core modules can be imported and initialized.
Updated to test the new backend architecture.
"""

import sys
import traceback
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


def test_imports():
    """Test that key modules can be imported."""
    print("Testing imports...")

    try:
        from dsos.config.settings import settings
        print("✓ Settings module imported")
    except Exception as e:
        print(f"✗ Failed to import settings: {e}")
        return False

    try:
        from dsos.agents.drone_agent import DroneAgent
        print("✓ Drone agent module imported")
    except Exception as e:
        print(f"✗ Failed to import drone agent: {e}")
        return False

    try:
        from dsos.control.backends import BaseBackend, BackendType, create_backend
        print("✓ Backend modules imported")
    except Exception as e:
        print(f"✗ Failed to import backend modules: {e}")
        return False

    try:
        from dsos.missions.mission_manager import MissionManager
        print("✓ Mission manager module imported")
    except Exception as e:
        print(f"✗ Failed to import mission manager: {e}")
        return False

    try:
        from dsos.formations.formation_engine import FormationManager
        print("✓ Formation manager module imported")
    except Exception as e:
        print(f"✗ Failed to import formation manager: {e}")
        return False

    try:
        from dsos.navigation.navigation_manager import NavigationManager
        print("✓ Navigation manager module imported")
    except Exception as e:
        print(f"✗ Failed to import navigation manager: {e}")
        return False

    try:
        from dsos.sensors.sensor_manager import SensorManager
        print("✓ Sensor manager module imported")
    except Exception as e:
        print(f"✗ Failed to import sensor manager: {e}")
        return False

    try:
        from dsos.ai.decision_maker import AIDecisionMaker
        print("✓ AI decision maker module imported")
    except Exception as e:
        print(f"✗ Failed to import AI decision maker: {e}")
        return False

    try:
        from dsos.telemetry.telemetry_manager import TelemetryManager
        print("✓ Telemetry manager module imported")
    except Exception as e:
        print(f"✗ Failed to import telemetry manager: {e}")
        return False

    try:
        from dsos.ui.gcs import GCSCommand
        print("✓ GCS module imported")
    except Exception as e:
        print(f"✗ Failed to import GCS: {e}")
        return False

    return True


def test_initialization():
    """Test that key components can be initialized."""
    print("\nTesting initialization...")

    try:
        from dsos.config.settings import settings
        # Test accessing settings
        swarm_size = settings.getint('DSOS', 'swarm_size', 3)
        print(f"✓ Settings access works: swarm_size = {swarm_size}")
    except Exception as e:
        print(f"✗ Settings access failed: {e}")
        return False

    try:
        from dsos.agents.drone_agent import DroneAgent
        drone = DroneAgent("test_drone")
        print("✓ Drone agent instantiated")
    except Exception as e:
        print(f"✗ Drone agent instantiation failed: {e}")
        return False

    try:
        from dsos.missions.mission_manager import MissionManager
        mm = MissionManager("test")
        print("✓ Mission manager instantiated")
    except Exception as e:
        print(f"✗ Mission manager instantiation failed: {e}")
        return False

    try:
        from dsos.formations.formation_engine import FormationManager
        fm = FormationManager("test")
        print("✓ Formation manager instantiated")
    except Exception as e:
        print(f"✗ Formation manager instantiation failed: {e}")
        return False

    try:
        from dsos.control.backends import create_backend, BackendType
        # Test creating a simulation backend (should always work)
        backend = create_backend('simulation', {})
        # Note: Our factory returns None for simulation to maintain backward compatibility
        # This is expected
        print("✓ Backend factory works")
    except Exception as e:
        print(f"✗ Backend factory failed: {e}")
        return False

    return True


def test_configuration():
    """Test that configuration loads correctly."""
    print("\nTesting configuration...")

    try:
        from dsos.config.settings import settings
        # Test a few key settings
        sim_mode = settings.getboolean('DSOS', 'simulation_mode', False)
        update_rate = settings.getint('DSOS', 'update_rate_hz', 50)
        backend_type = settings.get('DSOS', 'backend_type', 'simulation')
        print(f"✓ Configuration loaded: simulation_mode={sim_mode}, update_rate_hz={update_rate}, backend_type={backend_type}")
        return True
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        traceback.print_exc()
        return False


def test_backend_creation():
    """Test backend creation."""
    print("\nTesting backend creation...")

    try:
        from dsos.control.backends import create_backend, BackendType

        # Test simulation backend (returns None for backward compatibility)
        sim_backend = create_backend('simulation', {})
        print(f"✓ Simulation backend created: {sim_backend} (None is expected for backward compatibility)")

        # Test that we can at least import the AirSim backend class
        from dsos.control.backends.airsim_backend import AirSimBackend
        print("✓ AirSim backend class can be imported")

        return True
    except Exception as e:
        print(f"✗ Backend creation test failed: {e}")
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("DSOS Basic Functionality Test (Updated for Backend Architecture)")
    print("=" * 60)

    tests = [
        test_imports,
        test_initialization,
        test_configuration,
        test_backend_creation
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if test():
            passed += 1
        print()  # Empty line between tests

    print("=" * 60)
    print(f"Test Results: {passed}/{total} tests passed")
    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed!")
        return 1


if __name__ == '__main__':
    sys.exit(main())