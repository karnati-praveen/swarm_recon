"""
Empirical Challenge Test Suite for swarm_recon/sdk_template.py
"""

import math
import sys
import unittest
from pathlib import Path

# Ensure project root is in sys.path
_project_root = Path(__file__).resolve().parent.parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from swarm_recon.config import DroneState, SimulationConfig
from swarm_recon.agents.drone import SwarmAgent
from swarm_recon.sdk_template import (
    TargetTelemetry,
    SwarmReconROS2Node,
    MAVLinkBridge,
    SwarmSDKAdapter,
)


class TestTargetTelemetry(unittest.TestCase):

    def test_target_telemetry_roundtrip(self):
        t1 = TargetTelemetry(
            target_id=42,
            position=(12.3, 45.6),
            confidence=0.95,
            timestamp=100.0,
            detecting_drone_id=2,
            standoff_radius=20.0,
            cleared=False,
        )
        d = t1.to_dict()
        self.assertEqual(d["target_id"], 42)
        self.assertEqual(d["position"], [12.3, 45.6])
        self.assertEqual(d["confidence"], 0.95)
        self.assertEqual(d["timestamp"], 100.0)
        self.assertEqual(d["detecting_drone_id"], 2)
        self.assertEqual(d["standoff_radius"], 20.0)
        self.assertFalse(d["cleared"])

        t2 = TargetTelemetry.from_dict(d)
        self.assertEqual(t2.target_id, 42)
        self.assertEqual(t2.position, (12.3, 45.6))
        self.assertEqual(t2.confidence, 0.95)
        self.assertEqual(t2.timestamp, 100.0)
        self.assertEqual(t2.detecting_drone_id, 2)
        self.assertEqual(t2.standoff_radius, 20.0)
        self.assertFalse(t2.cleared)

    def test_target_telemetry_missing_optional_fields(self):
        d_missing = {
            "target_id": 1,
            "position": [10.0, 20.0],
            "confidence": 0.9,
            "timestamp": 5.0,
            "detecting_drone_id": 3,
        }
        t = TargetTelemetry.from_dict(d_missing)
        self.assertEqual(t.standoff_radius, 15.0)  # Default value check
        self.assertFalse(t.cleared)  # Default value check

    def test_target_telemetry_missing_required_fields(self):
        d_invalid = {"target_id": 1, "confidence": 0.9}
        with self.assertRaises(KeyError):
            TargetTelemetry.from_dict(d_invalid)

    def test_target_telemetry_type_casting(self):
        d_strings = {
            "target_id": "10",
            "position": ["1.5", "2.5"],
            "confidence": "0.85",
            "timestamp": "123.4",
            "detecting_drone_id": "5",
            "standoff_radius": "18.0",
            "cleared": "True",
        }
        t = TargetTelemetry.from_dict(d_strings)
        self.assertEqual(t.target_id, 10)
        self.assertIsInstance(t.target_id, int)
        self.assertEqual(t.position, (1.5, 2.5))
        self.assertIsInstance(t.position[0], float)
        self.assertEqual(t.confidence, 0.85)
        self.assertEqual(t.detecting_drone_id, 5)
        self.assertEqual(t.standoff_radius, 18.0)
        self.assertTrue(t.cleared)


class TestSwarmReconROS2Node(unittest.TestCase):

    def test_publish_telemetry(self):
        node = SwarmReconROS2Node(drone_id=3)
        state = DroneState(
            id=3,
            position=(10.0, 20.0),
            velocity=(1.0, -1.0),
            heading=0.5,
            active=True,
            sector_id=3,
            last_heartbeat=12.0,
        )
        msg = node.publish_telemetry(state)
        self.assertEqual(msg["topic"], "/drone_3/telemetry")
        self.assertEqual(msg["drone_id"], 3)
        self.assertEqual(msg["position"], {"x": 10.0, "y": 20.0, "z": 10.0})
        self.assertEqual(msg["velocity"], {"vx": 1.0, "vy": -1.0, "vz": 0.0})
        self.assertEqual(len(node.published_messages), 1)

    def test_publish_target_found(self):
        node = SwarmReconROS2Node(drone_id=3)
        t = TargetTelemetry(
            target_id=1,
            position=(30.0, 40.0),
            confidence=0.9,
            timestamp=1.0,
            detecting_drone_id=3,
        )
        msg = node.publish_target_found(t)
        self.assertEqual(msg["topic"], "/drone_3/target_found")
        self.assertEqual(msg["payload"]["target_id"], 1)
        self.assertEqual(node.active_target, t)

    def test_receive_peer_telemetry(self):
        node = SwarmReconROS2Node(drone_id=3)
        peer_msg = {
            "drone_id": 5,
            "position": {"x": 50.0, "y": 60.0},
            "velocity": {"vx": 0.5, "vy": 0.5},
            "heading": 1.0,
            "active": True,
            "sector_id": 1,
            "last_heartbeat": 100.0,
        }
        node.receive_peer_telemetry(peer_msg)
        self.assertIn(5, node.received_peers)
        peer_state = node.received_peers[5]
        self.assertEqual(peer_state.id, 5)
        self.assertEqual(peer_state.position, (50.0, 60.0))

    def test_receive_target_broadcast_cleared(self):
        node = SwarmReconROS2Node(drone_id=3)
        t = TargetTelemetry(
            target_id=1, position=(30.0, 40.0), confidence=0.9, timestamp=1.0, detecting_drone_id=3
        )
        node.receive_target_broadcast({"payload": t.to_dict()})
        self.assertIsNotNone(node.active_target)

        t_cleared = TargetTelemetry(
            target_id=1, position=(30.0, 40.0), confidence=0.0, timestamp=2.0, detecting_drone_id=3, cleared=True
        )
        node.receive_target_broadcast({"payload": t_cleared.to_dict()})
        self.assertIsNone(node.active_target)


class TestMAVLinkBridge(unittest.TestCase):

    def setUp(self):
        self.bridge = MAVLinkBridge(drone_id=2, altitude=15.0)

    def test_convert_swarm_to_ned(self):
        # Swarm 2D Cartesian: x_recon=10.0, y_recon=20.0, z_recon=15.0
        x_ned, y_ned, z_ned = self.bridge.convert_swarm_to_ned(10.0, 20.0, 15.0)
        self.assertEqual(x_ned, 20.0)   # North = y_recon
        self.assertEqual(y_ned, 10.0)   # East = x_recon
        self.assertEqual(z_ned, -15.0)  # Down = -z_recon

    def test_parse_local_position_ned(self):
        ned_msg = {"x": 20.0, "y": 10.0, "vx": 2.0, "vy": 1.0}
        x_recon, y_recon, vx_recon, vy_recon = self.bridge.parse_local_position_ned(ned_msg)
        self.assertEqual(x_recon, 10.0)
        self.assertEqual(y_recon, 20.0)
        self.assertEqual(vx_recon, 1.0)
        self.assertEqual(vy_recon, 2.0)

    def test_encode_heartbeat(self):
        hb = self.bridge.encode_heartbeat()
        self.assertEqual(hb["msgid"], 0)
        self.assertEqual(hb["system_id"], 3) # drone_id + 1
        self.assertEqual(hb["type"], 18)

    def test_encode_nav_waypoint(self):
        wp = self.bridge.encode_nav_waypoint(target_x=15.0, target_y=25.0, yaw=0.78, hold_time=5.0)
        self.assertEqual(wp["msgid"], 76)
        self.assertEqual(wp["command"], 16)
        self.assertEqual(wp["param1"], 5.0)
        self.assertEqual(wp["param4"], 0.78)
        self.assertEqual(wp["param5"], 25.0) # North
        self.assertEqual(wp["param6"], 15.0) # East
        self.assertEqual(wp["param7"], -15.0) # Down

    def test_encode_set_position_target_local_ned_bitmask(self):
        pos_cmd = self.bridge.encode_set_position_target_local_ned(x=15.0, y=25.0, vx=2.0, vy=3.0, yaw=0.5)
        self.assertEqual(pos_cmd["msgid"], 84)
        type_mask = pos_cmd["type_mask"]
        
        # Check Bit 10 (0x0400 = ignore yaw bit)
        ignore_yaw_bit = (type_mask & 0x0400) != 0
        ignore_yaw_rate_bit = (type_mask & 0x0800) != 0
        
        print(f"\n[MAVLink Bitmask Analysis] type_mask: {bin(type_mask)} ({type_mask})")
        print(f"[MAVLink Bitmask Analysis] Bit 10 (Ignore Yaw): {ignore_yaw_bit}")
        print(f"[MAVLink Bitmask Analysis] Bit 11 (Ignore Yaw Rate): {ignore_yaw_rate_bit}")
        
        # Bit 10 must be 0 (False) so that MAVLink flight controllers enable yaw control
        self.assertFalse(ignore_yaw_bit, "Bit 10 must be 0 to enable yaw control in MAVLink.")
        self.assertTrue(ignore_yaw_rate_bit, "Bit 11 must be 1 to ignore yaw rate in MAVLink.")


class TestSwarmSDKAdapter(unittest.TestCase):

    def setUp(self):
        self.config = SimulationConfig(width=100.0, height=100.0, num_drones=4)
        self.agent = SwarmAgent(drone_id=0, config=self.config, initial_position=(10.0, 10.0))
        self.adapter = SwarmSDKAdapter(agent=self.agent)

    def test_adapter_initialization(self):
        self.assertEqual(self.adapter.operating_mode, "VORONOI_SEARCH")
        self.assertIsNone(self.adapter.active_target)

    def test_trigger_target_found(self):
        t = self.adapter.trigger_target_found(target_pos=(50.0, 50.0), target_id=1)
        self.assertEqual(self.adapter.operating_mode, "TARGET_ENCIRCLEMENT")
        self.assertEqual(self.adapter.active_target, t)

    def test_calculate_encirclement_waypoint(self):
        self.adapter.trigger_target_found(target_pos=(50.0, 50.0), target_id=1)
        wx, wy = self.adapter.calculate_encirclement_waypoint((65.0, 50.0)) # Pos at 15m right of target
        dist = math.hypot(wx - 50.0, wy - 50.0)
        self.assertAlmostEqual(dist, 15.0, places=4)

    def test_target_cleared_desynchronization_vulnerability(self):
        """
        Tests whether receiving a TARGET_CLEARED broadcast on ROS 2 updates
        the SwarmSDKAdapter's active_target and operating_mode upon stepping.
        """
        # Step 1: Trigger target found
        self.adapter.trigger_target_found(target_pos=(50.0, 50.0), target_id=1)
        self.assertEqual(self.adapter.operating_mode, "TARGET_ENCIRCLEMENT")
        self.assertIsNotNone(self.adapter.active_target)

        # Step 2: Peer sends TARGET_CLEARED broadcast to ROS 2 node
        cleared_telemetry = TargetTelemetry(
            target_id=1, position=(50.0, 50.0), confidence=0.0, timestamp=10.0, detecting_drone_id=1, cleared=True
        )
        cleared_msg = {"payload": cleared_telemetry.to_dict()}
        self.adapter.ros_node.receive_target_broadcast(cleared_msg)

        # Step 3: Step adapter to synchronize ROS 2 node state with SDK Adapter & SwarmAgent
        self.adapter.step_and_export_commands(0.1, {}, [])

        # Check ROS 2 node state vs Adapter state
        ros_node_target = self.adapter.ros_node.active_target
        adapter_target = self.adapter.active_target
        adapter_mode = self.adapter.operating_mode

        print(f"\n[Desync Test] ros_node.active_target: {ros_node_target}")
        print(f"[Desync Test] adapter.active_target: {adapter_target}")
        print(f"[Desync Test] adapter.operating_mode: {adapter_mode}")

        self.assertIsNone(ros_node_target)
        self.assertIsNone(adapter_target, "Adapter active_target correctly reset to None after clearance.")
        self.assertEqual(adapter_mode, "VORONOI_SEARCH", "Adapter operating_mode correctly reverted to VORONOI_SEARCH.")

    def test_zero_and_negative_standoff_radius(self):
        """Tests encirclement waypoint when standoff radius is zero or negative (clamped to [10m, 20m])."""
        self.adapter.trigger_target_found(target_pos=(50.0, 50.0), target_id=1)
        
        # Test zero radius -> clamped to 10.0m min standoff
        self.adapter.active_target.standoff_radius = 0.0
        wx0, wy0 = self.adapter.calculate_encirclement_waypoint((65.0, 50.0))
        dist0 = math.hypot(wx0 - 50.0, wy0 - 50.0)
        self.assertAlmostEqual(dist0, 10.0, places=4, msg="Zero standoff radius clamped to 10.0m minimum standoff.")

        # Test negative radius -> clamped to 10.0m min standoff
        self.adapter.active_target.standoff_radius = -10.0
        wx_neg, wy_neg = self.adapter.calculate_encirclement_waypoint((65.0, 50.0))
        dist_neg = math.hypot(wx_neg - 50.0, wy_neg - 50.0)
        self.assertAlmostEqual(dist_neg, 10.0, places=4, msg="Negative standoff radius clamped to 10.0m minimum standoff.")


if __name__ == "__main__":
    unittest.main(verbosity=2)
