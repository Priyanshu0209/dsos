#!/usr/bin/env python3
"""
Test script for the SwarmBaseAgent implementation.
This demonstrates the basic functionality of the swarm intelligence system.
"""

import asyncio
import sys
import pytest
from pathlib import Path

# Add the parent directory of the current file's parent to the path
# so we can import the dsos package (which is located in /home/priyanshu)
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir.parent))  # Add /home/priyanshu


@pytest.mark.asyncio
async def test_swarm_agent_creation():
    """Test that we can create a SwarmBaseAgent."""
    print("Testing SwarmBaseAgent creation...")

    try:
        from dsos.swarm import SwarmBaseAgent

        # Create a swarm agent
        agent = SwarmBaseAgent("test_drone_1")
        print(f"✓ Created SwarmBaseAgent: {agent.agent_id}")

        # Check initial state
        assert agent.agent_id == "test_drone_1"
        assert agent.swarm_state.value == "idle"
        assert agent.swarm_role.value == "explorer"  # Default role based on ID
        print("✓ Initial state checks passed")

        return True
    except Exception as e:
        print(f"✗ Failed to create SwarmBaseAgent: {e}")
        return False


@pytest.mark.asyncio
async def test_swarm_agent_initialization():
    """Test that we can initialize a SwarmBaseAgent."""
    print("\nTesting SwarmBaseAgent initialization...")

    try:
        from dsos.swarm import SwarmBaseAgent

        # Create and initialize a swarm agent
        agent = SwarmBaseAgent("test_drone_2")
        await agent.initialize()

        print(f"✓ Initialized SwarmBaseAgent: {agent.agent_id}")

        # Check that components were initialized
        assert agent.knowledge_base is not None
        assert agent.mesh_network is not None
        assert agent.decision_engine is not None
        assert agent.behavior_tree is not None
        assert agent.planner is not None
        print("✓ All subsystems initialized")

        # Shutdown
        await agent.shutdown()
        print("✓ Shutdown completed")

        return True
    except Exception as e:
        print(f"✗ Failed to initialize SwarmBaseAgent: {e}")
        import traceback
        traceback.print_exc()
        return False


@pytest.mark.asyncio
async def test_knowledge_base():
    """Test the knowledge base functionality."""
    print("\nTesting KnowledgeBase...")

    try:
        from dsos.swarm import DistributedKnowledgeBase, KnowledgeType
        from dsos.communication.mesh_network import MeshNetwork

        # Create a mesh network and knowledge base
        mesh_network = MeshNetwork("test_drone_3", lambda msg: None)
        await mesh_network.start()

        kb = DistributedKnowledgeBase("test_drone_3", mesh_network)
        await kb.start()

        print("✓ Created and started KnowledgeBase")

        # Test adding knowledge
        from dsos.swarm.knowledge_base import ObjectKnowledge

        obj_knowledge = ObjectKnowledge(
            knowledge_id="obj_001",
            knowledge_type=KnowledgeType.OBJECT,
            timestamp=1234567890.0,
            source_drone="test_drone_3",
            confidence=0.9,
            location=[10.0, 20.0, 30.0],
            object_class="person",
            object_id="person_001"
        )

        result = kb.add_knowledge(obj_knowledge)
        assert result == True
        print("✓ Added knowledge item")

        # Test retrieving knowledge
        retrieved = kb.get_knowledge("obj_001")
        assert retrieved is not None
        assert retrieved.knowledge_id == "obj_001"
        assert retrieved.object_class == "person"
        print("✓ Retrieved knowledge item")

        # Test getting knowledge by type
        objects = kb.get_knowledge_by_type(KnowledgeType.OBJECT)
        assert len(objects) == 1
        assert objects[0].knowledge_id == "obj_001"
        print("✓ Retrieved knowledge by type")

        # Test statistics
        stats = kb.get_statistics()
        assert stats['total_knowledge_items'] == 1
        assert stats['knowledge_by_type']['object'] == 1
        print("✓ Got statistics")

        # Cleanup
        await kb.stop()
        await mesh_network.stop()
        print("✓ Cleaned up resources")

        return True
    except Exception as e:
        print(f"✗ Failed to test KnowledgeBase: {e}")
        import traceback
        traceback.print_exc()
        return False


@pytest.mark.asyncio
async def test_swarm_status():
    """Test getting swarm status."""
    print("\nTesting swarm status...")

    try:
        from dsos.swarm import SwarmBaseAgent

        # Create and initialize a swarm agent
        agent = SwarmBaseAgent("test_drone_4")
        await agent.initialize()

        # Get status
        status = agent.get_swarm_status()

        # Check structure
        assert 'drone_id' in status
        assert 'timestamp' in status
        assert 'basic_status' in status
        assert 'swarm_status' in status
        assert 'knowledge_base' in status
        assert 'neighbors' in status
        assert 'task_queue' in status

        assert status['drone_id'] == "test_drone_4"
        assert status['basic_status']['state'] == 'idle'
        assert status['swarm_status']['swarm_state'] == 'idle'
        assert status['swarm_status']['swarm_role'] == 'worker'

        print("✓ Got valid swarm status")

        # Get digital twin
        twin = agent.get_digital_twin()
        assert twin['entity_id'] == "test_drone_4"
        assert twin['entity_type'] == "drone"
        assert 'state' in twin
        assert 'capabilities' in twin
        assert 'mission' in twin
        assert 'swarm' in twin

        print("✓ Got valid digital twin")

        # Shutdown
        await agent.shutdown()
        print("✓ Shutdown completed")

        return True
    except Exception as e:
        print(f"✗ Failed to test swarm status: {e}")
        import traceback
        traceback.print_exc()
        return False


async def run_all_tests():
    """Run all tests."""
    print("=" * 60)
    print("DSOS Swarm Intelligence Test Suite")
    print("=" * 60)

    tests = [
        test_swarm_agent_creation,
        test_swarm_agent_initialization,
        test_knowledge_base,
        test_swarm_status
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if await test():
            passed += 1

    print("\n" + "=" * 60)
    print(f"Test Results: {passed}/{total} passed")
    if passed == total:
        print("🎉 All tests passed!")
        return True
    else:
        print("❌ Some tests failed!")
        return False


if __name__ == "__main__":
    success = asyncio.run(run_all_tests())
    sys.exit(0 if success else 1)