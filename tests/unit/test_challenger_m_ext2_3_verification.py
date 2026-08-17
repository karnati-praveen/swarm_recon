"""
Challenger Verification Script for Iteration 2 Verification.

Empirically tests:
1. MAVLink SET_POSITION_TARGET_LOCAL_NED bitmask fix (0x0BC0 / 0b0000101111000000).
2. SwarmSDKAdapter.trigger_target_cleared() behavior.
3. Swarm state reversion (explicit clear, peer broadcast clear, target loss timeout).
"""

import math
import sys
import unittest
from pathlib import Path

_project_root = Path(__file__).resolve().parent.parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from swarm_recon.config import DroneState, SimulationConfig, SwarmMode, PacketType, TelemetryPacket
from swarm_recon.agents.drone import SwarmAgent
from swarm_recon.sdk_template import (
    TargetTelemetry,
    SwarmReconROS2Node,
    MAVLinkBridge,
    SwarmSDKAdapter,
)


class EmpiricalVerificationTests(unittest.TestCase):

    def test_1_bitmask_fix_0x0BC0(self):
        """Verify MAVLink type_mask bitmask equals 0x0BC0 (3008) with correct bit configuration."""
        bridge = MAVLinkBridge(drone_id=1, altitude=10.0)
        pos_target_msg = bridge.encode_set_position_target_local_ned(
            x=20.0, y=30.0, vx=1.5, vy=2.5, yaw=0.78
        )
        
        type_mask = pos_target_msg["type_mask"]
        
        # 1. Exact integer and hexadecimal equivalence
        self.assertEqual(type_mask, 0x0BC0, f"type_mask {type_mask:#06x} must equal 0x0BC0")
        self.assertEqual(type_mask, 0b0000101111000000, f"type_mask {bin(type_mask)} must match 0b0000101111000000")
        self.assertEqual(type_mask, 3008, f"type_mask decimal must equal 3008")

        # 2. Detailed bit field verification according to MAVLink standard
        # Bit 0 (Position X): 0 = Enable
        self.assertEqual((type_mask & (1 << 0)), 0, "Bit 0 (Pos X) must be 0 (Enabled)")
        # Bit 1 (Position Y): 0 = Enable
        self.assertEqual((type_mask & (1 << 1)), 0, "Bit 1 (Pos Y) must be 0 (Enabled)")
        # Bit 2 (Position Z): 0 = Enable
        self.assertEqual((type_mask & (1 << 2)), 0, "Bit 2 (Pos Z) must be 0 (Enabled)")

        # Bit 3 (Velocity X): 0 = Enable
        self.assertEqual((type_mask & (1 << 3)), 0, "Bit 3 (Vel X) must be 0 (Enabled)")
        # Bit 4 (Velocity Y): 0 = Enable
        self.assertEqual((type_mask & (1 << 4)), 0, "Bit 4 (Vel Y) must be 0 (Enabled)")
        # Bit 5 (Velocity Z): 0 = Enable
        self.assertEqual((type_mask & (1 << 5)), 0, "Bit 5 (Vel Z) must be 0 (Enabled)")

        # Bit 6 (Accel X): 1 = Ignore
        self.assertNotEqual((type_mask & (1 << 6)), 0, "Bit 6 (Accel X) must be 1 (Ignored)")
        # Bit 7 (Accel Y): 1 = Ignore
        self.assertNotEqual((type_mask & (1 << 7)), 0, "Bit 7 (Accel Y) must be 1 (Ignored)")
        # Bit 8 (Accel Z): 1 = Ignore
        self.assertNotEqual((type_mask & (1 << 8)), 0, "Bit 8 (Accel Z) must be 1 (Ignored)")

        # Bit 9 (Use Force): 1 = Ignore force (use acceleration/velocity)
        self.assertNotEqual((type_mask & (1 << 9)), 0, "Bit 9 (Force) must be 1 (Ignored)")

        # Bit 10 (Ignore Yaw): 0 = Enable Yaw Control
        self.assertEqual((type_mask & (1 << 10)), 0, "Bit 10 (Ignore Yaw) must be 0 (Yaw Enabled)")

        # Bit 11 (Ignore Yaw Rate): 1 = Ignore Yaw Rate
        self.assertNotEqual((type_mask & (1 << 11)), 0, "Bit 11 (Ignore Yaw Rate) must be 1 (Yaw Rate Ignored)")

    def test_2_trigger_target_cleared_behavior(self):
        """Verify SwarmSDKAdapter.trigger_target_cleared() updates state and broadcasts telemetry."""
        config = SimulationConfig(width=100.0, height=100.0, num_drones=4)
        agent = SwarmAgent(drone_id=0, config=config, initial_position=(10.0, 10.0))
        adapter = SwarmSDKAdapter(agent=agent)

        # Setup active target state first
        adapter.trigger_target_found(target_pos=(50.0, 50.0), target_id=1)
        self.assertEqual(adapter.operating_mode, "TARGET_ENCIRCLEMENT")
        self.assertIsNotNone(adapter.active_target)
        self.assertTrue(agent.target_detected)
        self.assertEqual(agent.mode, SwarmMode.TARGET_TRACKING)

        # Now trigger clear
        clear_telemetry = adapter.trigger_target_cleared(target_id=1)

        # Assert returned TargetTelemetry schema
        self.assertTrue(clear_telemetry.cleared)
        self.assertEqual(clear_telemetry.target_id, 1)
        self.assertEqual(clear_telemetry.confidence, 0.0)
        self.assertEqual(clear_telemetry.detecting_drone_id, 0)

        # Assert adapter internal state
        self.assertIsNone(adapter.active_target)
        self.assertEqual(adapter.operating_mode, "VORONOI_SEARCH")

        # Assert agent internal state
        self.assertFalse(agent.target_detected)
        self.assertIsNone(agent.detected_target_position)
        self.assertEqual(agent.mode, SwarmMode.SEARCH)
        self.assertIsNone(agent.target_position)

        # Assert ROS 2 published messages contain cleared payload
        published = adapter.ros_node.published_messages
        self.assertGreater(len(published), 0)
        last_msg = published[-1]
        self.assertEqual(last_msg["topic"], "/drone_0/target_found")
        self.assertTrue(last_msg["payload"]["cleared"])

    def test_3_state_reversion_mechanisms(self):
        """Verify state reversion via peer clear broadcast, explicit clear, and loss timeout."""
        config = SimulationConfig(width=100.0, height=100.0, num_drones=4, target_loss_timeout=2.0)
        agent = SwarmAgent(drone_id=0, config=config, initial_position=(10.0, 10.0))
        adapter = SwarmSDKAdapter(agent=agent)

        # Mechanism A: Peer broadcast cleared event desync resolution via step_and_export_commands
        adapter.trigger_target_found(target_pos=(40.0, 40.0), target_id=1)
        self.assertEqual(adapter.operating_mode, "TARGET_ENCIRCLEMENT")

        peer_clear_telemetry = TargetTelemetry(
            target_id=1, position=(40.0, 40.0), confidence=0.0, timestamp=5.0, detecting_drone_id=2, cleared=True
        )
        adapter.ros_node.receive_target_broadcast({"payload": peer_clear_telemetry.to_dict()})
        
        # Step adapter to sync
        cmd_output = adapter.step_and_export_commands(dt=0.1, peers={}, threats=[])
        self.assertEqual(adapter.operating_mode, "VORONOI_SEARCH")
        self.assertEqual(cmd_output["mode"], "VORONOI_SEARCH")
        self.assertEqual(agent.mode, SwarmMode.SEARCH)

        # Mechanism B: Target loss timeout auto-reversion in agent.update()
        pkt = agent.detect_target((60.0, 60.0), target_id=2)
        self.assertEqual(agent.mode, SwarmMode.TARGET_TRACKING)

        # Advance time past target_loss_timeout (2.0s) without receiving target updates
        from swarm_recon.core.grid import GridSearchSpace
        grid = GridSearchSpace(100.0, 100.0)
        
        # Run step 1 (t = 0.1s)
        agent.update(dt=0.1, peers={}, threats=[], grid=grid)
        self.assertEqual(agent.mode, SwarmMode.TARGET_TRACKING)

        # Advance past 2.0s (e.g. 25 steps of 0.1s = 2.5s)
        for _ in range(25):
            agent.update(dt=0.1, peers={}, threats=[], grid=grid)

        # Agent should have auto-reverted to SEARCH mode due to packet loss timeout
        self.assertEqual(agent.mode, SwarmMode.SEARCH)
        self.assertIsNone(agent.target_position)


if __name__ == "__main__":
    unittest.main(verbosity=2)
