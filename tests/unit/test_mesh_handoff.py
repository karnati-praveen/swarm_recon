"""
Unit tests for Milestone M-EXT-MESH1: Multi-Hop Mesh Routing & Data Mule Protocol Core.

Tests:
1. TelemetryPacket extended schema, packet_id property, and serialization/deserialization.
2. SimulationConfig RF mesh parameters (comm_range, packet_drop_rate, jamming_center, jamming_radius, mule_cache_ttl) and serialization/deserialization.
3. SwarmAgent deduplication (_seen_packet_ids), Data Mule store-and-forward caching (_mule_cache), get_mule_packets, cache TTL pruning, and sequence counter increment.
4. SimulationEngine._is_rf_connected line-of-sight raycasting and range checks.
5. Multi-hop telemetry mesh relay (Drone A -> Drone B -> Drone C) with 50% packet drop rate resilience.
"""

import math
import random
import pytest

from swarm_recon.config import (
    SimulationConfig,
    SwarmMode,
    PacketType,
    TargetState,
    TelemetryPacket,
)
from swarm_recon.agents.drone import SwarmAgent
from swarm_recon.simulation.engine import SimulationEngine


def test_telemetry_packet_mesh_fields():
    """Verify TelemetryPacket extended fields, packet_id property, and serialization."""
    ts = TargetState(target_id=1, position=(20.0, 30.0), timestamp=5.0)
    pkt = TelemetryPacket(
        sender_id=1,
        packet_type=PacketType.TARGET_FOUND,
        target_state=ts,
        timestamp=5.0,
        source_id=0,
        destination_id=2,
        sequence_id=42,
        hop_count=1,
        ttl=10,
        relayed_by=[1],
    )

    assert pkt.packet_id == "0_42"
    assert pkt.source_id == 0
    assert pkt.destination_id == 2
    assert pkt.sequence_id == 42
    assert pkt.hop_count == 1
    assert pkt.relayed_by == [1]

    d = pkt.to_dict()
    assert d["source_id"] == 0
    assert d["destination_id"] == 2
    assert d["sequence_id"] == 42
    assert d["hop_count"] == 1
    assert d["relayed_by"] == [1]

    restored = TelemetryPacket.from_dict(d)
    assert restored.packet_id == "0_42"
    assert restored.source_id == 0
    assert restored.destination_id == 2
    assert restored.sequence_id == 42
    assert restored.hop_count == 1
    assert restored.relayed_by == [1]


def test_simulation_config_mesh_fields():
    """Verify SimulationConfig extended RF mesh parameters and serialization."""
    cfg = SimulationConfig(
        comm_range=40.0,
        packet_drop_rate=0.25,
        jamming_center=(50.0, 50.0),
        jamming_radius=15.0,
        mule_cache_ttl=20.0,
    )

    assert cfg.comm_range == 40.0
    assert cfg.packet_drop_rate == 0.25
    assert cfg.jamming_center == (50.0, 50.0)
    assert cfg.jamming_radius == 15.0
    assert cfg.mule_cache_ttl == 20.0

    d = cfg.to_dict()
    assert d["comm_range"] == 40.0
    assert d["packet_drop_rate"] == 0.25
    assert d["jamming_center"] == [50.0, 50.0]
    assert d["jamming_radius"] == 15.0
    assert d["mule_cache_ttl"] == 20.0

    restored = SimulationConfig.from_dict(d)
    assert restored.comm_range == 40.0
    assert restored.packet_drop_rate == 0.25
    assert restored.jamming_center == (50.0, 50.0)
    assert restored.jamming_radius == 15.0
    assert restored.mule_cache_ttl == 20.0


def test_agent_deduplication_and_mule_caching():
    """Verify SwarmAgent packet deduplication, _mule_cache, and get_mule_packets."""
    cfg = SimulationConfig(mule_cache_ttl=10.0)
    agent = SwarmAgent(drone_id=1, config=cfg, initial_position=(10.0, 10.0))

    ts = TargetState(target_id=1, position=(50.0, 50.0), timestamp=2.0)
    pkt = TelemetryPacket(
        sender_id=0,
        packet_type=PacketType.TARGET_FOUND,
        target_state=ts,
        timestamp=2.0,
        source_id=0,
        destination_id=-1,
        sequence_id=1,
        hop_count=0,
        ttl=5,
        relayed_by=[],
    )

    # First reception -> accepted and cached
    accepted1 = agent.receive_telemetry_packet(pkt)
    assert accepted1 is True
    assert agent.mode == SwarmMode.TARGET_TRACKING
    assert "0_1" in agent._seen_packet_ids

    # Second reception -> duplicate rejected
    accepted2 = agent.receive_telemetry_packet(pkt)
    assert accepted2 is False

    # Check cached mule packet
    mule_pkts = agent.get_mule_packets(current_time=3.0)
    assert len(mule_pkts) == 1
    relayed_pkt = mule_pkts[0]
    assert relayed_pkt.sender_id == 1
    assert relayed_pkt.hop_count == 1
    assert relayed_pkt.relayed_by == [1]

    # Exceed cache TTL -> pruned
    mule_pkts_expired = agent.get_mule_packets(current_time=15.0)
    assert len(mule_pkts_expired) == 0
    assert "0_1" not in agent._mule_cache


def test_agent_detect_target_sequence_id():
    """Verify detect_target increments sequence counter and sets source_id."""
    cfg = SimulationConfig()
    agent = SwarmAgent(drone_id=0, config=cfg, initial_position=(10.0, 10.0))

    pkt1 = agent.detect_target(target_pos=(50.0, 50.0), target_id=1)
    assert pkt1.source_id == 0
    assert pkt1.sequence_id == 1
    assert pkt1.packet_id == "0_1"

    agent.clear_target(target_id=1)
    pkt2 = agent.detect_target(target_pos=(60.0, 60.0), target_id=2)
    assert pkt2.source_id == 0
    assert pkt2.sequence_id == 3  # clear_target incremented to 2, detect_target incremented to 3
    assert pkt2.packet_id == "0_3"


def test_simulation_engine_is_rf_connected():
    """Verify SimulationEngine._is_rf_connected range and jamming circle checks."""
    cfg = SimulationConfig(
        comm_range=30.0,
        jamming_center=(50.0, 50.0),
        jamming_radius=10.0,
    )
    engine = SimulationEngine(config=cfg)

    # 1. Out of range (dist = 40 > 30)
    assert not engine._is_rf_connected((0.0, 0.0), (40.0, 0.0))

    # 2. In range, un-jammed (dist = 20 <= 30, away from (50,50))
    assert engine._is_rf_connected((10.0, 10.0), (25.0, 10.0))

    # 3. Endpoint inside jamming circle
    assert not engine._is_rf_connected((50.0, 50.0), (50.0, 60.0))

    # 4. Line segment intersects jamming circle (10,50) to (90,50) passing through center (50,50)
    assert not engine._is_rf_connected((25.0, 50.0), (75.0, 50.0))

    # 5. Line segment around jamming circle (10,35 to 80,35: dist to center (50,50) is 15 > 10)
    assert engine._is_rf_connected((30.0, 35.0), (50.0, 35.0))


def test_multihop_mesh_relay_end_to_end():
    """Verify 2-hop mesh relay (A -> B -> C) under 50% packet drop rate."""
    random.seed(42)
    cfg = SimulationConfig(
        width=100.0,
        height=100.0,
        num_drones=3,
        dt=0.1,
        total_time=10.0,
        comm_range=45.0,
        packet_drop_rate=0.50,
        jamming_center=(45.0, 50.0),
        jamming_radius=15.0,
        random_seed=42,
    )
    engine = SimulationEngine(config=cfg)

    # Drone 0 (A) at (10, 50), Drone 1 (B) at (45, 25), Drone 2 (C) at (80, 50)
    engine._agents[0]._x, engine._agents[0]._y = 10.0, 50.0
    engine._agents[0]._vx, engine._agents[0]._vy = 0.0, 0.0

    engine._agents[1]._x, engine._agents[1]._y = 45.0, 25.0
    engine._agents[1]._vx, engine._agents[1]._vy = 0.0, 0.0

    engine._agents[2]._x, engine._agents[2]._y = 80.0, 50.0
    engine._agents[2]._vx, engine._agents[2]._vy = 0.0, 0.0

    # Confirm topology: A->C is blocked; A->B and B->C are connected
    assert not engine._is_rf_connected((10.0, 50.0), (80.0, 50.0))
    assert engine._is_rf_connected((10.0, 50.0), (45.0, 25.0))
    assert engine._is_rf_connected((45.0, 25.0), (80.0, 50.0))

    engine._target_schedule = {
        0.5: {"event": "TARGET_FOUND", "target_id": 1, "position": (50.0, 50.0), "detecting_drone": 0}
    }

    log = engine.run()

    # Drone 2 must eventually transition to TARGET_TRACKING
    d2_mode_tracking = False
    for frame in log.frames:
        d2 = frame.drone_states.get(2)
        if d2 and d2.mode == SwarmMode.TARGET_TRACKING:
            d2_mode_tracking = True
            break

    assert d2_mode_tracking is True, "Drone 2 failed to receive multi-hop telemetry and enter TARGET_TRACKING mode"
    assert len(engine._agents[2]._seen_packet_ids) > 0
