#!/usr/bin/env python3
"""
Verification Script — Milestone M-EXT1: Target-Triggered Voronoi Collapse ("Hunter-Killer" Target Handoff)

Simulates target detection, Voronoi collapse & encirclement, standoff maintenance,
threat evasion, target clearing, and automatic mode reversion.

Exit codes:
  0: All verification checks passed.
  1: Verification failure.
"""

import argparse
import json
import sys
from pathlib import Path

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from swarm_recon.config import SimulationConfig, ThreatZone, SwarmMode
from swarm_recon.simulation.engine import SimulationEngine
from swarm_recon.analysis.metrics import SwarmMetrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify M-EXT1 Target-Triggered Voronoi Collapse & Handoff."
    )
    parser.add_argument("--drones", type=int, default=10, help="Number of drones")
    parser.add_argument("--time-limit", type=float, default=120.0, help="Total simulation duration (s)")
    parser.add_argument("--detect-time", "--target-time", type=float, default=30.0, help="Target detection time (s)")
    parser.add_argument("--clear-time", type=float, default=90.0, help="Target clear time (s)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--output", type=str, default=None, help="Optional output JSON path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("=" * 70)
    print("M-EXT1 VERIFICATION: Target-Triggered Voronoi Collapse & Target Handoff")
    print("=" * 70)
    print(f"Config: Drones={args.drones}, TimeLimit={args.time_limit}s, "
          f"DetectTime={args.detect_time}s, ClearTime={args.clear_time}s, Seed={args.seed}")

    config = SimulationConfig(
        width=100.0,
        height=100.0,
        num_drones=args.drones,
        total_time=args.time_limit,
        random_seed=args.seed,
    )

    threats = [
        ThreatZone(id=1, center=(25.0, 75.0), radius=12.0, severity=1.0),
        ThreatZone(id=2, center=(75.0, 25.0), radius=10.0, severity=1.0),
    ]

    target_pos = (50.0, 50.0)
    target_schedule = {
        args.detect_time: {
            "event": "TARGET_FOUND",
            "target_id": 1,
            "position": target_pos,
            "detecting_drone": 0,
        },
        args.clear_time: {
            "event": "TARGET_CLEARED",
            "target_id": 1,
            "drone_id": 0,
        },
    }

    engine = SimulationEngine(
        config=config,
        threats=threats,
        target_schedule=target_schedule,
    )

    print("\nRunning simulation engine...")
    log = engine.run()
    print(f"Simulation completed. Total frames logged: {len(log.frames)}")

    # 1. Mode transition verification
    events = SwarmMetrics.mode_transition_events(log)
    print(f"\n[Check 1] Swarm Mode Transitions: Logged {len(events)} mode change events")
    search_to_tracking = [e for e in events if e["to_mode"] == "TARGET_TRACKING"]
    tracking_to_search = [e for e in events if e["to_mode"] == "SEARCH"]
    print(f"  SEARCH -> TARGET_TRACKING transitions: {len(search_to_tracking)}")
    print(f"  TARGET_TRACKING -> SEARCH transitions: {len(tracking_to_search)}")

    # 2. Standoff distance evaluation
    standoff_metrics = SwarmMetrics.target_standoff_distances(log)
    standoff_check = SwarmMetrics.is_standoff_maintained(
        log,
        min_r=config.standoff_radius_min,
        max_r=config.standoff_radius_max,
        tolerance_ratio=0.95,
    )

    summary = standoff_metrics["summary"]
    print("\n[Check 2] Target Standoff Distance Statistics:")
    print(f"  Mean Distance: {summary['mean']:.2f} m (Nominal Target: {config.standoff_radius_nominal:.1f} m)")
    print(f"  Std Dev: {summary['std']:.2f} m")
    print(f"  Min / Max: {summary['min']:.2f} m / {summary['max']:.2f} m")
    print(f"  Median: {summary['median']:.2f} m")
    print(f"  Total Samples Evaluated: {summary['sample_count']}")
    print(f"  Standoff Maintained ([10.0m, 20.0m]): {standoff_check['maintained']} "
          f"(In-range ratio: {standoff_check['in_range_ratio'] * 100:.1f}%)")

    # 3. Threat evasion clearance evaluation
    min_clearance = SwarmMetrics.min_threat_clearance(log)
    print(f"\n[Check 3] Threat Evasion Clearance:")
    print(f"  Minimum Signed Clearance to Threat Boundary: {min_clearance:.2f} m")

    # 4. Coverage ratio evaluation
    coverage = SwarmMetrics.coverage_ratio(log)
    print(f"\n[Check 4] Final Search Space Coverage Ratio:")
    print(f"  Area Coverage: {coverage * 100:.2f}%")

    # Pass / Fail criteria checks
    passed = True
    reasons = []

    if len(search_to_tracking) == 0:
        passed = False
        reasons.append("FAIL: No SEARCH -> TARGET_TRACKING mode transitions detected.")

    if len(tracking_to_search) == 0:
        passed = False
        reasons.append("FAIL: No TARGET_TRACKING -> SEARCH mode reversions detected.")

    if not standoff_check["maintained"]:
        passed = False
        reasons.append(f"FAIL: Target standoff not maintained within [10.0m, 20.0m]. Ratio = {standoff_check['in_range_ratio'] * 100:.1f}%")

    if min_clearance <= 0.0:
        passed = False
        reasons.append(f"FAIL: Threat zone collision detected! Min clearance = {min_clearance:.2f} m")

    print("\n" + "=" * 70)
    if passed:
        print("VERIFICATION RESULT: PASSED ALL CHECKS")
        print("=" * 70)
        exit_code = 0
    else:
        print("VERIFICATION RESULT: FAILED")
        for r in reasons:
            print(f"  - {r}")
        print("=" * 70)
        exit_code = 1

    if args.output:
        out_data = {
            "passed": passed,
            "exit_code": exit_code,
            "reasons": reasons,
            "coverage_ratio": coverage,
            "min_threat_clearance": min_clearance,
            "standoff_summary": summary,
            "standoff_maintained": standoff_check,
            "mode_transitions": len(events),
        }
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(out_data, f, indent=2)
        print(f"Results written to {args.output}")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
