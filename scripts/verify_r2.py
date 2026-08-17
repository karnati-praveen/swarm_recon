#!/usr/bin/env python3
"""
verify_r2.py — Requirement R2 Verification Script

Runs a simulation with randomly-placed threat zones and verifies that:
1. All drones maintained positive clearance from all threats (no penetrations).
2. Mean heading entropy >= 1.5 bits (fluid, unpredictable trajectories).

Usage:
    python scripts/verify_r2.py [--trajectory-file logs/traj.json] [--threats-file config/threats.json]

If --trajectory-file is provided, loads an existing log instead of running a new simulation.

Exit codes:
    0 — Both threat clearance and entropy criteria met (PASS)
    1 — One or more criteria failed (FAIL)
"""

import argparse
import json
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from swarm_recon.config import SimulationConfig, ThreatZone, TrajectoryLog
from swarm_recon.simulation.engine import SimulationEngine
from swarm_recon.analysis.metrics import SwarmMetrics


def parse_args():
    parser = argparse.ArgumentParser(description="R2: Emergent threat evasion & trajectory fluidity verification.")
    parser.add_argument("--trajectory-file", type=str, default=None,
                        help="Path to existing trajectory JSON log. If not provided, runs a new simulation.")
    parser.add_argument("--threats-file", type=str, default=None,
                        help="Path to JSON file containing threat zone definitions.")
    parser.add_argument("--drones", type=int, default=8, help="Number of drones (used only if running new sim)")
    parser.add_argument("--time-limit", type=float, default=90.0, help="Simulation time limit in seconds")
    parser.add_argument("--seed", type=int, default=99, help="Random seed for new simulations")
    parser.add_argument("--entropy-threshold", type=float, default=1.5, help="Minimum required mean heading entropy (bits)")
    parser.add_argument("--output", type=str, default=None, help="Optional path to save trajectory log JSON")
    parser.add_argument("--quiet", action="store_true", help="Suppress verbose output")
    return parser.parse_args()


def _default_threats():
    """Generate a set of representative test threat zones."""
    return [
        ThreatZone(id=0, center=(25.0, 25.0), radius=8.0, severity=1.5),
        ThreatZone(id=1, center=(75.0, 25.0), radius=6.0, severity=1.0),
        ThreatZone(id=2, center=(50.0, 65.0), radius=10.0, severity=2.0),
        ThreatZone(id=3, center=(20.0, 80.0), radius=5.0, severity=1.0),
        ThreatZone(id=4, center=(80.0, 75.0), radius=7.0, severity=1.2),
    ]


def main():
    args = parse_args()

    threats = []
    if args.threats_file:
        with open(args.threats_file, "r", encoding="utf-8") as f:
            raw = json.load(f)
        threats = [ThreatZone.from_dict(t) for t in raw]
    else:
        threats = _default_threats()

    if not args.quiet:
        print(f"[R2] Verification: {len(threats)} threat zones, entropy threshold={args.entropy_threshold:.2f} bits")

    if args.trajectory_file:
        if not args.quiet:
            print(f"[R2] Loading existing trajectory: {args.trajectory_file}")
        log = TrajectoryLog.load_json(args.trajectory_file)
    else:
        if not args.quiet:
            print(f"[R2] Running new simulation: {args.drones} drones, {args.time_limit}s, seed={args.seed}")
        config = SimulationConfig(
            width=100.0,
            height=100.0,
            resolution=1.0,
            dt=0.1,
            total_time=args.time_limit,
            num_drones=args.drones,
            sensor_radius=5.0,
            heartbeat_timeout=3.0,
            max_drone_speed=5.0,
            random_seed=args.seed,
        )
        engine = SimulationEngine(config=config, threats=threats, kill_at={})
        log = engine.run()

        if args.output:
            os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
            log.save_json(args.output)
            if not args.quiet:
                print(f"[R2] Trajectory saved: {args.output}")

    # Compute metrics
    min_clearance = SwarmMetrics.min_threat_clearance(log)
    entropy = SwarmMetrics.heading_entropy(log)

    if not args.quiet:
        print(f"\n[R2] Results:")
        print(f"  Minimum threat clearance : {min_clearance:.3f} m  (must be > 0.0)")
        print(f"  Mean heading entropy     : {entropy:.4f} bits  (must be >= {args.entropy_threshold:.2f})")

    clearance_ok = min_clearance > 0.0
    entropy_ok = entropy >= args.entropy_threshold

    if clearance_ok and entropy_ok:
        print(f"\n[R2] PASS — Clearance={min_clearance:.3f}m > 0, Entropy={entropy:.4f} >= {args.entropy_threshold}")
        sys.exit(0)
    else:
        reasons = []
        if not clearance_ok:
            reasons.append(f"min_clearance={min_clearance:.3f}m <= 0.0 (drone penetrated threat zone!)")
        if not entropy_ok:
            reasons.append(f"entropy={entropy:.4f} < {args.entropy_threshold:.2f} bits (trajectories too predictable)")
        print(f"\n[R2] FAIL — " + "; ".join(reasons))
        sys.exit(1)


if __name__ == "__main__":
    main()
