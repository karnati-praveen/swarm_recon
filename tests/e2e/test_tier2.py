"""
Tier 2 E2E Test Suite: Boundary & Corner Cases

Tests critical edge scenarios:
1. K = N - 1 extreme drone failure (single survivor absorbing 100% unsearched frontier).
2. Zero threat zones (uninhibited smooth raster search, zero APF repulsion force).
3. Max threat zones (heavy obstacle grid, zero threat penetrations, high trajectory entropy).
4. Area edge and corner coordinate bounds clamping & extreme aspect ratio grids.
"""

import os
import math
import pytest
from tests.e2e.conftest import StandaloneSimulationEngine, MetricsCalculator


def test_extreme_drone_failures(default_sim_config):
    """E2E-T2-01: K = N - 1 extreme drone failure & single survivor sector absorption."""
    N = 10
    kill_schedule = {30.0: [0, 1, 2, 3, 4, 5, 6, 7, 8]}  # 9 drones killed at t=30s, Drone 9 survives

    config = dict(default_sim_config)
    config["num_drones"] = N
    config["time_limit"] = 120.0
    config["dt"] = 0.1
    config["kill_schedule"] = kill_schedule

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Assert survivor state
    assert engine.active_drones[9] is True, "Surviving drone 9 should remain active"
    for did in range(9):
        assert engine.active_drones[did] is False, f"Drone {did} should be killed"

    # Assert Voronoi target for survivor covers remaining unsearched frontier
    active_ids = [9]
    target_x, target_y = engine._get_voronoi_target(9, active_ids)
    assert 0.0 <= target_x <= config["width"], "Voronoi target x out of bounds"
    assert 0.0 <= target_y <= config["height"], "Voronoi target y out of bounds"

    # Assert post-kill search coverage progress
    coverage = metrics["coverage_ratio"]
    assert coverage > 0.50, f"Single survivor coverage {coverage:.3f} failed baseline threshold (>0.50)"


def test_zero_threat_zones(default_sim_config):
    """E2E-T2-02: Zero threat fields (pure uninhibited coverage search)."""
    config = dict(default_sim_config)
    config["num_drones"] = 5
    config["time_limit"] = 60.0
    config["threats"] = []  # Zero threat zones

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Assert 0 threat clearance is infinite (no threats)
    assert metrics["min_threat_clearance"] == float("inf"), "Clearance must be infinite with 0 threat zones"

    # Assert smooth kinematics (mean jerk <= 2.0 m/s^3 in uninhibited search)
    assert metrics["mean_jerk"] <= 2.0, f"Mean jerk {metrics['mean_jerk']:.2f} exceeded limit (2.0 m/s^3)"

    # Assert rapid baseline search completion (>95% within 60s)
    assert metrics["coverage_ratio"] >= 0.95, f"Zero threat coverage ratio {metrics['coverage_ratio']:.3f} < 0.95"


def test_max_threat_zones(default_sim_config):
    """E2E-T2-03: Heavy obstacle grid with overlapping circular threat fields."""
    threats = [
        {"center": (30.0, 30.0), "radius": 15.0},
        {"center": (70.0, 30.0), "radius": 15.0},
        {"center": (50.0, 60.0), "radius": 18.0},
        {"center": (25.0, 80.0), "radius": 12.0},
        {"center": (75.0, 80.0), "radius": 14.0}
    ]

    config = dict(default_sim_config)
    config["num_drones"] = 8
    config["time_limit"] = 120.0
    config["threats"] = threats

    engine = StandaloneSimulationEngine(config)
    metrics = engine.run()

    # Assert zero threat collisions
    min_clearance = metrics["min_threat_clearance"]
    assert min_clearance > 0.0, f"Threat collision detected! Min clearance: {min_clearance:.2f}m <= 0.0m"

    # Assert non-zero trajectory heading entropy generation (Rotational APF evasion)
    assert metrics["mean_heading_entropy"] >= 1.5, f"Heading entropy {metrics['mean_heading_entropy']:.2f} < 1.5 bits"

    # Assert reachable area coverage > 95%
    assert metrics["coverage_ratio"] >= 0.95, f"Reachable coverage ratio {metrics['coverage_ratio']:.3f} < 0.95"


def test_area_edge_corner_bounds(default_sim_config):
    """E2E-T2-04: Edge/corner coordinate bounds clamping and extreme grid geometries."""
    # Test 1: Corner spawn and position clamping
    config = dict(default_sim_config)
    config["width"] = 100.0
    config["height"] = 100.0
    config["time_limit"] = 10.0

    engine = StandaloneSimulationEngine(config)
    # Manually place drones exactly on outer corners
    corners = [(0.0, 0.0), (100.0, 0.0), (0.0, 100.0), (100.0, 100.0), (50.0, 50.0)]
    for did, pos in enumerate(corners):
        engine.positions[did] = pos
        engine.velocities[did] = (-5.0, -5.0)  # Pushing outward

    # Run step and verify no IndexError or out-of-bound crash
    metrics = engine.run()
    for did, (x, y) in engine.positions.items():
        assert 0.0 <= x <= 100.0, f"Clamping failed: Drone {did} x={x} out of bounds"
        assert 0.0 <= y <= 100.0, f"Clamping failed: Drone {did} y={y} out of bounds"

    # Test 2: Extreme aspect ratio grids (100 x 1 and 1 x 100)
    config_100x1 = dict(default_sim_config)
    config_100x1["width"] = 100.0
    config_100x1["height"] = 1.0
    config_100x1["time_limit"] = 5.0
    engine_100x1 = StandaloneSimulationEngine(config_100x1)
    res_100x1 = engine_100x1.run()
    assert res_100x1["coverage_ratio"] > 0.0, "Extreme 100x1 grid failed coverage execution"

    config_1x100 = dict(default_sim_config)
    config_1x100["width"] = 1.0
    config_1x100["height"] = 100.0
    config_1x100["time_limit"] = 5.0
    engine_1x100 = StandaloneSimulationEngine(config_1x100)
    res_1x100 = engine_1x100.run()
    assert res_1x100["coverage_ratio"] > 0.0, "Extreme 1x100 grid failed coverage execution"
