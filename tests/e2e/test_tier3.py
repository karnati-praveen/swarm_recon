"""
Tier 3 E2E Test Suite: Cross-Feature Interactions

Tests cross-feature interactions:
1. Simultaneous drone failure mid-simulation + dynamic Rotational APF threat evasion.
2. Combined APF repulsion + Boids flocking separation + dynamic re-partitioning under moving/stationary threats.
"""

import pytest
from tests.e2e.conftest import StandaloneSimulationEngine


def test_simultaneous_failure_and_evasion(default_sim_config):
    """E2E-T3-01: Simultaneous drone failure mid-simulation + dynamic Rotational APF threat evasion."""
    N = 10
    kill_schedule = {30.0: [2, 5, 8]}  # 3 drones killed simultaneously at t=30s
    threats = [
        {"center": (30.0, 40.0), "radius": 8.0},
        {"center": (70.0, 60.0), "radius": 10.0},
        {"center": (50.0, 80.0), "radius": 7.0}
    ]

    config = dict(default_sim_config)
    config["num_drones"] = N
    config["width"] = 100.0
    config["height"] = 100.0
    config["time_limit"] = 120.0
    config["dt"] = 0.1
    config["kill_schedule"] = kill_schedule
    config["threats"] = threats

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Assert 3 killed drones, 7 active survivors
    active_count = sum(1 for active in engine.active_drones.values() if active)
    assert active_count == 7, f"Expected 7 active survivors post-kill, found {active_count}"

    # Assert threat clearance safety (0 threat penetrations)
    min_clearance = metrics["min_threat_clearance"]
    assert min_clearance > 0.0, f"Threat collision during simultaneous failure evasion! Clearance: {min_clearance:.2f}m"

    # Assert reachable area coverage > 95%
    coverage = metrics["coverage_ratio"]
    assert coverage >= 0.95, f"Reachable area coverage ratio {coverage:.3f} failed 0.95 target"


def test_apf_boids_repartition_moving_threats(default_sim_config):
    """E2E-T3-02: Combined APF repulsion + Boids flocking separation + dynamic re-partitioning under moving threats."""
    N = 12
    threats = [
        {"center": (20.0, 30.0), "radius": 10.0, "velocity": (0.2, 0.1)},   # Moving threat 1
        {"center": (80.0, 90.0), "radius": 12.0, "velocity": (-0.1, -0.2)}, # Moving threat 2
        {"center": (60.0, 40.0), "radius": 8.0}                             # Stationary threat 3
    ]

    config = dict(default_sim_config)
    config["num_drones"] = N
    config["width"] = 120.0
    config["height"] = 120.0
    config["time_limit"] = 120.0
    config["dt"] = 0.1
    config["threats"] = threats

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Assert zero threat clearance violations
    min_clearance = metrics["min_threat_clearance"]
    assert min_clearance > 0.0, f"Moving threat collision detected! Clearance: {min_clearance:.2f}m <= 0.0m"

    # Assert heading entropy (emergent fluid evasion) >= 1.5 bits
    entropy = metrics["mean_heading_entropy"]
    assert entropy >= 1.5, f"Heading entropy {entropy:.2f} < 1.5 bits"

    # Assert reachable area coverage > 95%
    coverage = metrics["coverage_ratio"]
    assert coverage >= 0.95, f"Reachable coverage ratio {coverage:.3f} < 0.95 threshold"
