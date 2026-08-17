import pytest
import math
from swarm_recon.config import SimulationConfig, SwarmMode
from swarm_recon.simulation.engine import SimulationEngine

def test_mesh_handoff_under_jamming():
    """
    Test Tier 6: Verify RF-Denied Mesh Handoff
    Drone 0 and Drone 2 are separated by a jamming zone and a distance greater than comm_range.
    Drone 1 acts as a relay outside the jamming zone.
    Drone 0 detects a target and transmits TARGET_FOUND.
    Drone 2 should receive it via Drone 1 and transition to TARGET_TRACKING.
    """
    config = SimulationConfig(
        width=100.0,
        height=100.0,
        num_drones=3,
        dt=0.1,
        total_time=20.0,
        comm_range=45.0,
        packet_drop_rate=0.50,
        jamming_center=(45.0, 50.0),
        jamming_radius=15.0,
        mule_cache_ttl=30.0,
        random_seed=42,
    )

    engine = SimulationEngine(config=config)

    # Topology setup:
    # 0 (A): (10, 50)
    # 1 (B): (45, 25) - clear path to A and C
    # 2 (C): (80, 50) - direct path to A is jammed and out of range (70m)
    engine._agents[0]._x, engine._agents[0]._y = 10.0, 50.0
    engine._agents[0]._vx, engine._agents[0]._vy = 0.0, 0.0

    engine._agents[1]._x, engine._agents[1]._y = 45.0, 25.0
    engine._agents[1]._vx, engine._agents[1]._vy = 0.0, 0.0

    engine._agents[2]._x, engine._agents[2]._y = 80.0, 50.0
    engine._agents[2]._vx, engine._agents[2]._vy = 0.0, 0.0

    assert not engine._is_rf_connected((10.0, 50.0), (80.0, 50.0)), "Direct link A->C should be disconnected"
    assert engine._is_rf_connected((10.0, 50.0), (45.0, 25.0)), "Link A->B should be connected"
    assert engine._is_rf_connected((45.0, 25.0), (80.0, 50.0)), "Link B->C should be connected"

    # Schedule target detection by Drone 0
    engine._target_schedule = {
        0.5: {"event": "TARGET_FOUND", "target_id": 1, "position": (50.0, 50.0), "detecting_drone": 0}
    }

    log = engine.run()

    # Verify Drone 2 received packet and transitioned to TARGET_TRACKING mode
    d2_mode_tracking = False
    for frame in log.frames:
        d2_state = frame.drone_states.get(2)
        if d2_state and d2_state.mode == SwarmMode.TARGET_TRACKING:
            d2_mode_tracking = True
            break

    seen_by_d2 = engine._agents[2]._seen_packet_ids
    assert len(seen_by_d2) > 0, "Drone 2 did not record packet reception"
    assert d2_mode_tracking, "Drone 2 failed to transition to TARGET_TRACKING mode via multi-hop mesh"
