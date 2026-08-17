"""
Unit tests for Milestone M-EXT1: Target-Triggered Voronoi Collapse Core ("Hunter-Killer" Target Handoff).

Tests:
1. Configuration schemas, enums, dataclasses, serialization/deserialization.
2. EvaderForces.target_encirclement force calculations.
3. SwarmAgent mode state machine, target detection, target clearance, telemetry packet processing, and auto-reversion timeout.
4. SimulationEngine telemetry message bus and target schedule processing.
5. SwarmMetrics standoff distances, standoff maintenance checks, and mode transition logging.
"""

import math
import pytest
from typing import Dict, Any

from swarm_recon.config import (
    SwarmMode,
    PacketType,
    TargetState,
    TelemetryPacket,
    DroneState,
    SimulationConfig,
    TrajectoryFrame,
    TrajectoryLog,
    ThreatZone,
)
from swarm_recon.core.grid import GridSearchSpace
from swarm_recon.evasion.forces import EvaderForces
from swarm_recon.agents.drone import SwarmAgent
from swarm_recon.simulation.engine import SimulationEngine
from swarm_recon.analysis.metrics import SwarmMetrics


# ============================================================================
# 1. Configuration & Data Schemas Unit Tests
# ============================================================================

def test_config_enums_and_dataclasses():
    """Verify SwarmMode and PacketType enums, TargetState, TelemetryPacket serialization."""
    assert SwarmMode.SEARCH == "SEARCH"
    assert SwarmMode.TARGET_TRACKING == "TARGET_TRACKING"
    assert PacketType.HEARTBEAT == "HEARTBEAT"
    assert PacketType.TARGET_FOUND == "TARGET_FOUND"
    assert PacketType.TARGET_CLEARED == "TARGET_CLEARED"

    ts = TargetState(target_id=1, position=(50.0, 50.0), timestamp=10.0, status="active", detected_by=0)
    ts_dict = ts.to_dict()
    assert ts_dict["target_id"] == 1
    assert ts_dict["position"] == [50.0, 50.0]
    assert ts_dict["timestamp"] == 10.0
    assert ts_dict["status"] == "active"
    assert ts_dict["detected_by"] == 0

    ts_restored = TargetState.from_dict(ts_dict)
    assert ts_restored.target_id == 1
    assert ts_restored.position == (50.0, 50.0)

    pkt = TelemetryPacket(sender_id=0, packet_type=PacketType.TARGET_FOUND, target_state=ts, timestamp=10.0)
    pkt_dict = pkt.to_dict()
    assert pkt_dict["sender_id"] == 0
    assert pkt_dict["packet_type"] == "TARGET_FOUND"
    assert pkt_dict["target_state"]["position"] == [50.0, 50.0]

    pkt_restored = TelemetryPacket.from_dict(pkt_dict)
    assert pkt_restored.sender_id == 0
    assert pkt_restored.packet_type == PacketType.TARGET_FOUND
    assert pkt_restored.target_state.position == (50.0, 50.0)


def test_drone_state_mode_extension():
    """Verify DroneState mode and target_position fields and serialization."""
    state = DroneState(
        id=2,
        position=(10.0, 20.0),
        velocity=(1.0, -1.0),
        mode=SwarmMode.TARGET_TRACKING,
        target_position=(50.0, 50.0),
    )
    assert state.mode == SwarmMode.TARGET_TRACKING
    assert state.target_position == (50.0, 50.0)

    d = state.to_dict()
    assert d["mode"] == "TARGET_TRACKING"
    assert d["target_position"] == [50.0, 50.0]

    restored = DroneState.from_dict(d)
    assert restored.mode == SwarmMode.TARGET_TRACKING
    assert restored.target_position == (50.0, 50.0)


def test_simulation_config_standoff_params():
    """Verify SimulationConfig default standoff parameters and validation."""
    cfg = SimulationConfig()
    assert cfg.standoff_radius_min == 10.0
    assert cfg.standoff_radius_max == 20.0
    assert cfg.standoff_radius_nominal == 15.0
    assert cfg.target_loss_timeout == 5.0

    d = cfg.to_dict()
    assert d["standoff_radius_nominal"] == 15.0
    restored = SimulationConfig.from_dict(d)
    assert restored.target_loss_timeout == 5.0


def test_trajectory_frame_target_tracking():
    """Verify TrajectoryFrame target_state and mode_counts fields."""
    dstate = DroneState(id=0, mode=SwarmMode.TARGET_TRACKING, target_position=(50.0, 50.0))
    frame = TrajectoryFrame(
        timestamp=5.0,
        drone_states={0: dstate},
        active_drone_count=1,
        coverage_ratio=0.25,
        target_state={"target_id": 1, "position": [50.0, 50.0]},
        mode_counts={"SEARCH": 0, "TARGET_TRACKING": 1},
    )
    f_dict = frame.to_dict()
    assert f_dict["mode_counts"]["TARGET_TRACKING"] == 1
    assert f_dict["target_state"]["position"] == [50.0, 50.0]

    restored = TrajectoryFrame.from_dict(f_dict)
    assert restored.mode_counts["TARGET_TRACKING"] == 1
    assert restored.target_state["position"] == [50.0, 50.0]


# ============================================================================
# 2. EvaderForces Encirclement Unit Tests
# ============================================================================

def test_evader_forces_target_encirclement():
    """Verify EvaderForces.target_encirclement calculation."""
    target_pos = (50.0, 50.0)
    # Drone outside nominal radius (r = 30.0m > 15.0m)
    px, py = 80.0, 50.0
    peers = [
        DroneState(id=0, position=(px, py), active=True),
        DroneState(id=1, position=(50.0, 80.0), active=True),
    ]

    fx, fy = EvaderForces.target_encirclement(
        px, py, target_pos, peers, standoff_radius=15.0
    )

    # Displacement from target = (30, 0). Outward unit vector rx = (1, 0).
    # Inward unit vector ux = (-1, 0). Tangential unit vector tau = (0, 1).
    # dr = 30 - 15 = 15 => f_radial_mag = 4.0 * 15 = 60 capped to 35.
    # Inward force component should be negative in x direction.
    assert fx < 0.0, f"Expected inward x-force (fx < 0), got {fx}"
    # Tangential orbital drive (2.0 m/s^2 CCW) should produce positive y-force.
    assert fy > 0.0, f"Expected orbital y-force (fy > 0), got {fy}"


def test_evader_forces_target_encirclement_inside_radius():
    """Verify EvaderForces.target_encirclement when drone is inside standoff radius."""
    target_pos = (50.0, 50.0)
    # Drone inside nominal radius (r = 5.0m < 15.0m)
    px, py = 55.0, 50.0
    peers = [DroneState(id=0, position=(px, py), active=True)]

    fx, fy = EvaderForces.target_encirclement(
        px, py, target_pos, peers, standoff_radius=15.0
    )

    # Displacement from target = (5, 0). Outward unit vector rx = (1, 0).
    # Inward unit vector ux = (-1, 0).
    # dr = 5 - 15 = -10 => f_radial_mag = 4.0 * (-10) = -40 capped to -35.
    # Force should push outward (fx > 0).
    assert fx > 0.0, f"Expected repulsive x-force (fx > 0), got {fx}"


# ============================================================================
# 3. SwarmAgent Multi-Mode State Machine Unit Tests
# ============================================================================

def test_swarm_agent_target_detection_and_clear():
    """Verify SwarmAgent detect_target and clear_target methods."""
    cfg = SimulationConfig(num_drones=3)
    agent = SwarmAgent(drone_id=0, config=cfg, initial_position=(10.0, 10.0))

    assert agent.mode == SwarmMode.SEARCH

    pkt_found = agent.detect_target(target_pos=(50.0, 50.0), target_id=1)
    assert agent.mode == SwarmMode.TARGET_TRACKING
    assert agent.target_position == (50.0, 50.0)
    assert pkt_found.packet_type == PacketType.TARGET_FOUND

    pkt_clear = agent.clear_target(target_id=1)
    assert agent.mode == SwarmMode.SEARCH
    assert agent.target_position is None
    assert pkt_clear.packet_type == PacketType.TARGET_CLEARED


def test_swarm_agent_telemetry_packet_reception():
    """Verify SwarmAgent receive_telemetry_packet state transitions."""
    cfg = SimulationConfig(num_drones=3)
    agent1 = SwarmAgent(drone_id=1, config=cfg, initial_position=(20.0, 20.0))

    ts = TargetState(target_id=1, position=(60.0, 60.0), timestamp=5.0)
    pkt_found = TelemetryPacket(sender_id=0, packet_type=PacketType.TARGET_FOUND, target_state=ts, timestamp=5.0)

    agent1.receive_telemetry_packet(pkt_found)
    assert agent1.mode == SwarmMode.TARGET_TRACKING
    assert agent1.target_position == (60.0, 60.0)

    pkt_clear = TelemetryPacket(sender_id=0, packet_type=PacketType.TARGET_CLEARED, timestamp=15.0)
    agent1.receive_telemetry_packet(pkt_clear)
    assert agent1.mode == SwarmMode.SEARCH
    assert agent1.target_position is None


def test_swarm_agent_target_loss_timeout():
    """Verify SwarmAgent automatic mode reversion after target_loss_timeout."""
    cfg = SimulationConfig(num_drones=2, target_loss_timeout=3.0, dt=0.1)
    agent = SwarmAgent(drone_id=0, config=cfg, initial_position=(10.0, 10.0))
    grid = GridSearchSpace(100.0, 100.0, 1.0)

    # Detect target at t = 0
    agent.detect_target(target_pos=(50.0, 50.0), target_id=1)
    assert agent.mode == SwarmMode.TARGET_TRACKING

    # Step forward 2.0s (less than 3.0s timeout) without new packets
    for _ in range(20):
        agent.update(dt=0.1, peers={}, threats=[], grid=grid)
    assert agent.mode == SwarmMode.TARGET_TRACKING

    # Step forward another 1.5s (total 3.5s > 3.0s timeout)
    for _ in range(15):
        agent.update(dt=0.1, peers={}, threats=[], grid=grid)
    assert agent.mode == SwarmMode.SEARCH, "Agent failed to revert to SEARCH after target loss timeout"


# ============================================================================
# 4. SimulationEngine & Telemetry Bus Unit Tests
# ============================================================================

def test_simulation_engine_target_schedule():
    """Verify SimulationEngine processes target_schedule events correctly."""
    cfg = SimulationConfig(num_drones=4, total_time=20.0, dt=0.1, comm_range=100.0)
    target_schedule = {
        5.0: {"event": "TARGET_FOUND", "target_id": 1, "position": (50.0, 50.0)},
        15.0: {"event": "TARGET_CLEARED", "target_id": 1},
    }

    engine = SimulationEngine(config=cfg, target_schedule=target_schedule)
    log = engine.run()

    # Before t=5.0s: mode is SEARCH
    frame_pre = log.frames[20]  # t = 2.0s
    assert frame_pre.mode_counts.get("SEARCH", 0) == 4

    # Mid-run t=10.0s: mode is TARGET_TRACKING
    frame_mid = log.frames[100]  # t = 10.0s
    assert frame_mid.mode_counts.get("TARGET_TRACKING", 0) == 4

    # Post clear t=18.0s: mode is SEARCH
    frame_post = log.frames[180]  # t = 18.0s
    assert frame_post.mode_counts.get("SEARCH", 0) == 4


# ============================================================================
# 5. SwarmMetrics M-EXT1 Unit Tests
# ============================================================================

def test_swarm_metrics_standoff_and_transitions():
    """Verify SwarmMetrics standoff and transition event calculations."""
    cfg = SimulationConfig(num_drones=4, total_time=20.0, dt=0.1, comm_range=100.0)
    target_schedule = {
        5.0: {"event": "TARGET_FOUND", "target_id": 1, "position": (50.0, 50.0)},
        15.0: {"event": "TARGET_CLEARED", "target_id": 1},
    }

    engine = SimulationEngine(config=cfg, target_schedule=target_schedule)
    log = engine.run()

    # Test mode_transition_events
    events = SwarmMetrics.mode_transition_events(log)
    assert len(events) >= 8  # 4 drones SEARCH->TRACKING + 4 drones TRACKING->SEARCH
    search_to_track = [e for e in events if e["to_mode"] == "TARGET_TRACKING"]
    track_to_search = [e for e in events if e["to_mode"] == "SEARCH"]
    assert len(search_to_track) == 4
    assert len(track_to_search) == 4

    # Test target_standoff_distances
    standoff = SwarmMetrics.target_standoff_distances(log)
    assert "time_series" in standoff
    assert "summary" in standoff
    assert standoff["summary"]["sample_count"] > 0
    assert standoff["summary"]["mean"] > 0.0

    # Test is_standoff_maintained
    maintained = SwarmMetrics.is_standoff_maintained(
        log, min_r=10.0, max_r=20.0, tolerance_ratio=0.90, settling_window=2.0
    )
    assert "maintained" in maintained
    assert "in_range_ratio" in maintained
