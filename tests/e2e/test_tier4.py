"""
Tier 4 E2E Test Suite: Real-World Stress Scenarios

Validates full system performance:
1. Full Swarm Recon Benchmark: N=20 drones in 200m x 200m area, 5 dynamic kills staggered mid-simulation,
   multiple circular threat zones, asserting >95% coverage within time limit T, zero threat collisions,
   mean jerk <= 2.0 m/s^3, and 36-bin Shannon heading entropy >= 1.5 bits.
2. System-wide Scalability & Moving Threat Stress Benchmark.
"""

import os
import json
import pytest
from tests.e2e.conftest import StandaloneSimulationEngine, TrajectoryLogParser


def test_full_swarm_recon_stress_benchmark(default_sim_config, temp_log_dir):
    """
    E2E-T4-01: Primary Real-World Reconnaissance & Dynamic Kill Stress Benchmark.
    Simulates N=20 drones in 200m x 200m area with 5 staggered kills and 5 circular threats.
    """
    N = 20
    width = 200.0
    height = 200.0
    time_limit = 120.0

    # 5 dynamic kills staggered mid-simulation
    kill_schedule = {
        15.0: [3],
        30.0: [7],
        45.0: [12],
        60.0: [15],
        75.0: [18]
    }

    # 5 circular threat zones
    threats = [
        {"center": (40.0, 50.0), "radius": 12.0},
        {"center": (150.0, 60.0), "radius": 15.0},
        {"center": (100.0, 120.0), "radius": 10.0},
        {"center": (60.0, 160.0), "radius": 14.0},
        {"center": (160.0, 150.0), "radius": 11.0}
    ]

    config = dict(default_sim_config)
    config["num_drones"] = N
    config["width"] = width
    config["height"] = height
    config["time_limit"] = time_limit
    config["dt"] = 0.1
    config["seed"] = 1337
    config["kill_schedule"] = kill_schedule
    config["threats"] = threats

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Export trajectory log to temp directory and re-verify with TrajectoryLogParser
    log_file = os.path.join(temp_log_dir, "tier4_benchmark_traj.json")
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump(metrics["log_data"], f)

    parser = TrajectoryLogParser.from_file(log_file)
    parsed_metrics = parser.evaluate_metrics(threats)

    coverage = parsed_metrics["coverage_ratio"]
    min_clearance = parsed_metrics["min_threat_clearance"]
    entropy = parsed_metrics["mean_heading_entropy"]
    jerk = parsed_metrics["mean_jerk"]

    print(f"\n=======================================================")
    print(f"Tier 4 Benchmark Results (N=20, K=5 dynamic kills):")
    print(f"  • Reachable Area Coverage: {coverage * 100.0:.2f}% (Target: >95.0%)")
    print(f"  • Minimum Threat Clearance: {min_clearance:.2f} m (Target: >0.0 m)")
    print(f"  • Shannon Heading Entropy: {entropy:.2f} bits (Target: >=1.50 bits)")
    print(f"  • Mean Kinematic Jerk:     {jerk:.2f} m/s^3 (Target: <=2.0 m/s^3)")
    print(f"=======================================================\n")

    # Primary Quantitative Assertions (R1 & R2)
    assert coverage >= 0.95, f"R1 Assertion Failed: Reachable area coverage {coverage * 100.0:.2f}% < 95.0% target!"
    assert min_clearance > 0.0, f"R2 Assertion Failed: Threat collision detected (min clearance {min_clearance:.2f} m <= 0.0 m)!"
    assert entropy >= 1.50, f"R2 Assertion Failed: Shannon heading entropy {entropy:.2f} < 1.50 bits!"
    assert jerk <= 2.00, f"R2 Assertion Failed: Trajectory mean jerk {jerk:.2f} exceeded 2.00 m/s^3 limit!"


def test_scalability_moving_threat_stress(default_sim_config):
    """
    E2E-T4-02: Scalability and long-duration moving threat stress test.
    Evaluates 20 drones in 250m x 250m grid with moving threats over 180s simulation time.
    """
    N = 20
    width = 250.0
    height = 250.0
    time_limit = 180.0

    kill_schedule = {
        20.0: [1, 2],
        50.0: [5],
        80.0: [9],
        110.0: [13],
        140.0: [17]
    }

    threats = [
        {"center": (50.0, 50.0), "radius": 15.0, "velocity": (0.1, 0.05)},
        {"center": (200.0, 200.0), "radius": 18.0, "velocity": (-0.08, -0.05)},
        {"center": (120.0, 120.0), "radius": 12.0}
    ]

    config = dict(default_sim_config)
    config["num_drones"] = N
    config["width"] = width
    config["height"] = height
    config["time_limit"] = time_limit
    config["dt"] = 0.1
    config["kill_schedule"] = kill_schedule
    config["threats"] = threats

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Assert scalable coverage completion > 95%
    assert metrics["coverage_ratio"] >= 0.95, f"Scalability coverage {metrics['coverage_ratio']:.3f} < 0.95 threshold"

    # Assert zero moving threat collisions
    assert metrics["min_threat_clearance"] > 0.0, "Threat collision detected under moving threat fields!"

    # Assert log frame count (180s / 0.1s = 1800 frames)
    assert len(engine.trajectory_frames) == 1800, f"Expected 1800 trajectory frames, got {len(engine.trajectory_frames)}"
