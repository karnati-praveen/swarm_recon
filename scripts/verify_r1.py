#!/usr/bin/env python3
"""
verify_r1.py — Requirement R1 Verification Script

Spawns N drones, kills K of them at mid-simulation, and verifies that
the remaining swarm achieves >95% search area coverage within a time limit.

Usage:
    python scripts/verify_r1.py --drones 10 --killed 3 --time-limit 120 --seed 42

Exit codes:
    0 — Coverage > 95% (PASS)
    1 — Coverage <= 95% (FAIL)
"""

import argparse
import json
import math
import os
import sys

# Ensure project root is on sys.path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from swarm_recon.config import SimulationConfig, ThreatZone
from swarm_recon.simulation.engine import SimulationEngine
from swarm_recon.analysis.metrics import SwarmMetrics


def parse_args():
    parser = argparse.ArgumentParser(description="R1: Decentralized search & dynamic reassignment verification.")
    parser.add_argument("--drones", type=int, default=10, help="Initial number of drones (N)")
    parser.add_argument("--killed", type=int, default=3, help="Number of drones to kill mid-simulation (K)")
    parser.add_argument("--time-limit", type=float, default=180.0, help="Simulation time limit in seconds")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    parser.add_argument("--coverage-threshold", type=float, default=0.95, help="Minimum required coverage ratio")
    parser.add_argument("--output", type=str, default=None, help="Optional path to save trajectory log JSON")
    parser.add_argument("--quiet", action="store_true", help="Suppress verbose output")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.killed >= args.drones:
        print(f"[ERROR] killed ({args.killed}) must be less than drones ({args.drones})")
        sys.exit(1)

    if not args.quiet:
        print(f"[R1] Verification: N={args.drones} drones, K={args.killed} killed, "
              f"time_limit={args.time_limit}s, seed={args.seed}")
        print(f"[R1] Coverage threshold: {args.coverage_threshold * 100:.1f}%")

    # Build config
    config = SimulationConfig(
        width=100.0,
        height=100.0,
        resolution=1.0,
        dt=0.1,
        total_time=args.time_limit,
        num_drones=args.drones,
        sensor_radius=6.0,
        heartbeat_timeout=3.0,
        max_drone_speed=5.0,
        random_seed=args.seed,
    )

    # Schedule K drone kills at mid-simulation
    kill_time = args.time_limit / 2.0
    kill_ids = list(range(args.killed))
    kill_at = {kill_time: kill_ids}

    if not args.quiet:
        print(f"[R1] Kill schedule: drones {kill_ids} will be killed at t={kill_time:.1f}s")

    # Run simulation
    engine = SimulationEngine(config=config, threats=[], kill_at=kill_at)
    log = engine.run()

    # Calculate coverage
    coverage = SwarmMetrics.coverage_ratio(log)
    active_at_end = sum(1 for s in log.frames[-1].drone_states.values() if s.active) if log.frames else 0

    if not args.quiet:
        print(f"\n[R1] Results:")
        print(f"  Total frames recorded : {len(log.frames)}")
        print(f"  Active drones at end  : {active_at_end} (of {args.drones - args.killed} survivors)")
        print(f"  Final coverage ratio  : {coverage * 100:.2f}%")
        print(f"  Threshold             : {args.coverage_threshold * 100:.1f}%")

    # Optionally save log
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        log.save_json(args.output)
        if not args.quiet:
            print(f"  Trajectory log saved  : {args.output}")

    # Verdict
    passed = coverage > args.coverage_threshold
    if passed:
        print(f"\n[R1] PASS — Coverage {coverage * 100:.2f}% > {args.coverage_threshold * 100:.1f}%")
        sys.exit(0)
    else:
        print(f"\n[R1] FAIL — Coverage {coverage * 100:.2f}% <= {args.coverage_threshold * 100:.1f}%")
        sys.exit(1)


if __name__ == "__main__":
    main()
