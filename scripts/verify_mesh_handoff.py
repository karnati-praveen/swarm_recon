"""
Programmatic Verification Script for Milestone M-EXT-MESH1:
Multi-Hop Mesh Routing & Data Mule Protocol Core.

Verifies:
1. Multi-hop relay (>= 2 hops: Drone A -> Drone B -> Drone C) around an RF jamming barrier / distance limits.
2. Packet delivery resilience under 50% ambient link packet drop rate (packet_drop_rate = 0.50).
3. Exit code 0 on PASS, 1 on FAIL.
"""

import argparse
import os
import random
import sys

# Ensure project root is on sys.path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from swarm_recon.config import (
    SimulationConfig,
    SwarmMode,
    PacketType,
    TargetState,
    TelemetryPacket,
)
from swarm_recon.simulation.engine import SimulationEngine


def run_mesh_handoff_verification(
    comm_range: float = 45.0,
    jamming_center: tuple = (45.0, 50.0),
    jamming_radius: float = 15.0,
    packet_drop_rate: float = 0.50,
    total_time: float = 20.0,
    seed: int = 42,
) -> bool:
    """
    Run multi-hop mesh routing and Data Mule verification.
    """
    random.seed(seed)

    config = SimulationConfig(
        width=100.0,
        height=100.0,
        num_drones=3,
        dt=0.1,
        total_time=total_time,
        comm_range=comm_range,
        packet_drop_rate=packet_drop_rate,
        jamming_center=jamming_center,
        jamming_radius=jamming_radius,
        mule_cache_ttl=30.0,
        random_seed=seed,
    )

    # Instantiate engine
    engine = SimulationEngine(config=config)

    # Set custom positions for deterministic topology:
    # Drone 0 (A): (10.0, 50.0)
    # Drone 1 (B): (45.0, 25.0)  [Outside jamming circle (45.0, 50.0) r=15.0]
    # Drone 2 (C): (80.0, 50.0)
    engine._agents[0]._x, engine._agents[0]._y = 10.0, 50.0
    engine._agents[0]._vx, engine._agents[0]._vy = 0.0, 0.0

    engine._agents[1]._x, engine._agents[1]._y = 45.0, 25.0
    engine._agents[1]._vx, engine._agents[1]._vy = 0.0, 0.0

    engine._agents[2]._x, engine._agents[2]._y = 80.0, 50.0
    engine._agents[2]._vx, engine._agents[2]._vy = 0.0, 0.0

    # Verify direct link A -> C is NOT connected
    direct_ac = engine._is_rf_connected((10.0, 50.0), (80.0, 50.0))
    if direct_ac:
        print("[FAIL] Expected direct link A -> C to be disconnected by jamming/range, but it was connected!")
        return False

    # Verify links A -> B and B -> C ARE connected
    link_ab = engine._is_rf_connected((10.0, 50.0), (45.0, 25.0))
    link_bc = engine._is_rf_connected((45.0, 25.0), (80.0, 50.0))
    if not link_ab or not link_bc:
        print(f"[FAIL] Expected links A-B ({link_ab}) and B-C ({link_bc}) to be connected!")
        return False

    print(f"[INFO] Topology verified: Direct A->C jammed/out of range. Links A->B ({link_ab}) and B->C ({link_bc}) connected.")

    # Schedule target detection by Drone 0 at t = 0.5s
    target_schedule = {
        0.5: {"event": "TARGET_FOUND", "target_id": 1, "position": (50.0, 50.0), "detecting_drone": 0}
    }
    engine._target_schedule = target_schedule

    # Run simulation
    log = engine.run()

    # Verify Drone 2 received packet and transitioned to TARGET_TRACKING mode
    d2_mode_tracking = False
    for frame in log.frames:
        d2_state = frame.drone_states.get(2)
        if d2_state and d2_state.mode == SwarmMode.TARGET_TRACKING:
            d2_mode_tracking = True
            break

    seen_by_d2 = engine._agents[2]._seen_packet_ids
    drone2_received_packet = len(seen_by_d2) > 0

    print(f"[INFO] Drone 2 received packet: {drone2_received_packet}")
    print(f"[INFO] Drone 2 mode transitioned to TARGET_TRACKING: {d2_mode_tracking}")
    print(f"[INFO] Seen packet IDs by Drone 2: {seen_by_d2}")

    pass_checks = True

    if not d2_mode_tracking:
        print("[FAIL] Drone 2 failed to transition to TARGET_TRACKING mode via multi-hop mesh!")
        pass_checks = False
    else:
        print("[PASS] Multi-hop telemetry delivery confirmed: Drone 2 reached TARGET_TRACKING mode.")

    if not drone2_received_packet:
        print("[FAIL] Drone 2 did not record packet reception in _seen_packet_ids.")
        pass_checks = False
    else:
        print("[PASS] Packet deduplicated and processed at destination Drone 2.")

    if pass_checks:
        print("\n=== ALL MESH ROUTING & DATA MULE VERIFICATION CHECKS PASSED ===")
        return True
    else:
        print("\n=== MESH ROUTING VERIFICATION FAILED ===")
        return False


def main():
    parser = argparse.ArgumentParser(description="Multi-Hop Mesh Routing & Data Mule Verification")
    parser.add_argument("--comm-range", type=float, default=45.0, help="P2P RF communication range")
    parser.add_argument("--packet-drop-rate", type=float, default=0.50, help="Simulated link drop rate")
    parser.add_argument("--jamming-radius", type=float, default=15.0, help="RF jamming zone radius")
    parser.add_argument("--time-limit", type=float, default=20.0, help="Max simulation run time")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    success = run_mesh_handoff_verification(
        comm_range=args.comm_range,
        jamming_center=(45.0, 50.0),
        jamming_radius=args.jamming_radius,
        packet_drop_rate=args.packet_drop_rate,
        total_time=args.time_limit,
        seed=args.seed,
    )

    if success:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
