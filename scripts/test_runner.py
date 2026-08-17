#!/usr/bin/env python3
"""
test_runner.py — Unified E2E Test Runner

Executes all three requirement verification scripts (R1, R2, R3) and
produces a structured JSON summary report.

Usage:
    python scripts/test_runner.py [--output results.json] [--quiet]

Exit codes:
    0 — All verifications passed
    1 — One or more verifications failed
"""

import argparse
import json
import os
import subprocess
import sys
import time


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    parser = argparse.ArgumentParser(description="Unified E2E verification runner for swarm_recon.")
    parser.add_argument("--output", type=str, default=None, help="Path to write JSON results report")
    parser.add_argument("--quiet", action="store_true", help="Suppress subprocess output")
    parser.add_argument("--r1-drones", type=int, default=10, help="R1: number of initial drones")
    parser.add_argument("--r1-killed", type=int, default=3, help="R1: number of drones to kill")
    parser.add_argument("--r1-time-limit", type=float, default=180.0, help="R1: simulation time limit")
    parser.add_argument("--r1-seed", type=int, default=42, help="R1: random seed")
    parser.add_argument("--r2-drones", type=int, default=8, help="R2: number of drones")
    parser.add_argument("--r2-time-limit", type=float, default=90.0, help="R2: simulation time limit")
    parser.add_argument("--r2-seed", type=int, default=99, help="R2: random seed")
    parser.add_argument("--dep-dir", type=str, default=".venv", help="R3: dependency directory to check")
    parser.add_argument("--max-dep-mb", type=float, default=500.0, help="R3: maximum dependency size in MB")
    parser.add_argument("--r4-time-limit", type=float, default=120.0, help="R4: target handoff time limit")
    parser.add_argument("--r5-comm-range", type=float, default=45.0, help="R5: comm range")
    parser.add_argument("--r5-jamming-radius", type=float, default=15.0, help="R5: jamming radius")
    parser.add_argument("--r5-drop-rate", type=float, default=0.50, help="R5: packet drop rate")
    return parser.parse_args()


def run_verification(label: str, cmd: list, quiet: bool) -> dict:
    """Run a single verification script and capture result."""
    print(f"\n{'=' * 60}")
    print(f"  Running: {label}")
    print(f"  Command: {' '.join(cmd)}")
    print(f"{'=' * 60}")

    start = time.time()
    result = subprocess.run(
        cmd,
        capture_output=quiet,
        text=True,
        cwd=os.path.abspath(os.path.join(_SCRIPT_DIR, "..")),
    )
    elapsed = time.time() - start

    if not quiet:
        pass  # Output already streamed to stdout
    else:
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)

    passed = result.returncode == 0
    status = "PASS" if passed else "FAIL"
    print(f"\n  [{label}] {status} (exit={result.returncode}, elapsed={elapsed:.1f}s)")

    return {
        "label": label,
        "command": cmd,
        "exit_code": result.returncode,
        "passed": passed,
        "elapsed_seconds": round(elapsed, 2),
        "stdout": result.stdout if quiet else "",
        "stderr": result.stderr if quiet else "",
    }


def main():
    args = parse_args()
    python = sys.executable
    scripts = _SCRIPT_DIR

    verifications = [
        {
            "label": "R1 — Decentralized Search & Dynamic Reassignment",
            "cmd": [
                python, os.path.join(scripts, "verify_r1.py"),
                "--drones", str(args.r1_drones),
                "--killed", str(args.r1_killed),
                "--time-limit", str(args.r1_time_limit),
                "--seed", str(args.r1_seed),
            ],
        },
        {
            "label": "R2 — Emergent Evasion Behaviors",
            "cmd": [
                python, os.path.join(scripts, "verify_r2.py"),
                "--drones", str(args.r2_drones),
                "--time-limit", str(args.r2_time_limit),
                "--seed", str(args.r2_seed),
            ],
        },
        {
            "label": "R3 — Dependency Size Limit",
            "cmd": [
                python, os.path.join(scripts, "verify_r3.py"),
                "--target-dir", args.dep_dir,
                "--max-size-mb", str(args.max_dep_mb),
            ],
        },
        {
            "label": "R4 — Target Handoff & Encirclement",
            "cmd": [
                python, os.path.join(scripts, "verify_target_handoff.py"),
                "--time-limit", str(args.r4_time_limit),
            ],
        },
        {
            "label": "R5 — RF-Denied Mesh Routing",
            "cmd": [
                python, os.path.join(scripts, "verify_mesh_handoff.py"),
                "--comm-range", str(args.r5_comm_range),
                "--jamming-radius", str(args.r5_jamming_radius),
                "--packet-drop-rate", str(args.r5_drop_rate),
            ],
        },
    ]

    print(f"\n{'#' * 60}")
    print(f"  SWARM RECON — E2E Verification Suite")
    print(f"{'#' * 60}")

    results = []
    for v in verifications:
        res = run_verification(v["label"], v["cmd"], args.quiet)
        results.append(res)

    # Summary
    total = len(results)
    passed = sum(1 for r in results if r["passed"])
    failed = total - passed
    all_passed = failed == 0

    print(f"\n{'#' * 60}")
    print(f"  SUMMARY: {passed}/{total} verifications PASSED")
    for r in results:
        status = "[PASS]" if r["passed"] else "[FAIL]"
        print(f"    {status}  {r['label']}  ({r['elapsed_seconds']}s)")
    print(f"{'#' * 60}\n")

    report = {
        "summary": {
            "total": total,
            "passed": passed,
            "failed": failed,
            "all_passed": all_passed,
        },
        "verifications": results,
    }

    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        print(f"Results written to: {args.output}")

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
