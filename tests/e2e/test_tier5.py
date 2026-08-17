"""
Tier 5 E2E Test Suite: Extension Requirement R1 (Target Handoff & Encirclement)

Verifies dynamic multi-mode swarm state machine, orbital encirclement standoff radius bounds,
blended APF threat avoidance during encirclement, and target clearing revert to Voronoi search.
"""

import os
import pytest
from typing import Dict, Any
from tests.e2e.conftest import StandaloneSimulationEngine, MetricsCalculator


def test_target_detection_mode_transition(default_sim_config: Dict[str, Any]):
    """
    E2E-T5-01: Verifies immediate swarm state machine transition from SEARCH to TARGET_TRACKING
    upon receiving a TARGET_FOUND broadcast event.
    """
    config = default_sim_config.copy()
    config["num_drones"] = 5
    config["time_limit"] = 30.0
    config["target_events"] = {
        20.0: {"type": "DETECT", "target_pos": (50.0, 50.0)}
    }

    engine = StandaloneSimulationEngine(config)
    results = engine.run()
    frames = results["log_data"]["frames"]

    # Pre-event assertion (t < 20.0s): All drones in SEARCH mode
    pre_event_frames = [f for f in frames if f["timestamp"] < 20.0]
    assert len(pre_event_frames) > 0, "No pre-event frames found"
    sample_pre_frame = pre_event_frames[10]  # Frame at t=1.0s
    for drone in sample_pre_frame["drones"]:
        if drone["active"]:
            assert drone["mode"] == "SEARCH", f"Drone {drone['id']} mode was {drone['mode']}, expected SEARCH"
            assert drone["target_detected"] is False

    # Post-event assertion (t >= 20.1s): All drones in TARGET_TRACKING mode
    post_event_frames = [f for f in frames if f["timestamp"] >= 20.1]
    assert len(post_event_frames) > 0, "No post-event frames found"
    sample_post_frame = post_event_frames[5]  # Frame shortly after detection
    for drone in sample_post_frame["drones"]:
        if drone["active"]:
            assert drone["mode"] in ["TARGET_TRACKING", "ENCIRCLE"], f"Drone {drone['id']} mode was {drone['mode']}, expected TARGET_TRACKING"
            assert drone["target_detected"] is True
            assert drone["target_position"] == [50.0, 50.0] or drone["target_position"] == (50.0, 50.0)


def test_target_encirclement_standoff_radius(default_sim_config: Dict[str, Any]):
    """
    E2E-T5-02: Verifies active tracking drones maintain standoff distance bounds (10.0m <= d <= 20.0m)
    during steady-state encirclement (evaluating after settling window t >= t_detect + 5s).
    """
    config = default_sim_config.copy()
    config["num_drones"] = 8
    config["time_limit"] = 90.0
    config["target_events"] = {
        10.0: {"type": "DETECT", "target_pos": (50.0, 50.0)},
        80.0: {"type": "CLEAR"}
    }

    engine = StandaloneSimulationEngine(config)
    results = engine.run()
    frames = results["log_data"]["frames"]

    # Settling window t in [15.0s, 80.0s]
    metrics = MetricsCalculator.compute_target_standoff_metrics(
        frames, target_pos=(50.0, 50.0), start_time=15.0, end_time=80.0
    )

    assert metrics["min_standoff"] >= 10.0, f"Standoff min bound violated: {metrics['min_standoff']:.2f}m < 10.0m"
    assert metrics["max_standoff"] <= 20.0, f"Standoff max bound violated: {metrics['max_standoff']:.2f}m > 20.0m"
    assert metrics["in_bounds_ratio"] == 1.0, f"In-bounds ratio was {metrics['in_bounds_ratio']:.2f}, expected 1.0"


def test_target_encirclement_threat_avoidance(default_sim_config: Dict[str, Any]):
    """
    E2E-T5-03: Verifies active drones maintain positive threat clearance (d_clearance > 0.0m)
    during target encirclement near circular threat fields.
    """
    config = default_sim_config.copy()
    config["num_drones"] = 8
    config["time_limit"] = 90.0
    config["target_events"] = {
        10.0: {"type": "DETECT", "target_pos": (50.0, 50.0)},
        80.0: {"type": "CLEAR"}
    }
    config["threats"] = [
        {"center": (40.0, 40.0), "radius": 12.0}
    ]

    engine = StandaloneSimulationEngine(config)
    results = engine.run()

    min_clearance = results["min_threat_clearance"]
    assert min_clearance > 0.0, f"Threat collision detected during encirclement: min_clearance={min_clearance:.2f}m <= 0.0m"

    # Verify threat clearance and minimum standoff during APF deflection
    frames = results["log_data"]["frames"]
    metrics = MetricsCalculator.compute_target_standoff_metrics(
        frames, target_pos=(50.0, 50.0), start_time=15.0, end_time=80.0
    )
    assert metrics["min_standoff"] >= 10.0, f"Standoff min bound violated: {metrics['min_standoff']:.2f}m < 10.0m"


def test_target_clearing_revert_to_voronoi(default_sim_config: Dict[str, Any]):
    """
    E2E-T5-04: Verifies swarm shifts mode back to SEARCH and resumes Voronoi search coverage
    expansion (delta coverage >= 0.05) after receiving TARGET_CLEARED event.
    """
    config = default_sim_config.copy()
    config["num_drones"] = 6
    config["time_limit"] = 90.0
    config["target_events"] = {
        10.0: {"type": "DETECT", "target_pos": (50.0, 50.0)},
        40.0: {"type": "CLEAR"}
    }

    engine = StandaloneSimulationEngine(config)
    results = engine.run()
    frames = results["log_data"]["frames"]

    # Post-clearing mode assertion (t = 40.1s)
    post_clear_frames = [f for f in frames if f["timestamp"] >= 40.1]
    assert len(post_clear_frames) > 0, "No post-clear frames found"
    sample_post_frame = post_clear_frames[1]
    for drone in sample_post_frame["drones"]:
        if drone["active"]:
            assert drone["mode"] == "SEARCH", f"Drone {drone['id']} mode was {drone['mode']}, expected SEARCH after clearing"
            assert drone["target_detected"] is False

    # Coverage progression assertion: Coverage at t=40.0s vs t=90.0s
    frames_at_40 = [f for f in frames if f["timestamp"] <= 40.0]
    coverage_40 = MetricsCalculator.compute_reachable_coverage(
        config["width"], config["height"], config["resolution"], config["sensor_radius"],
        frames_at_40, config["threats"]
    )
    coverage_90 = results["coverage_ratio"]

    delta_coverage = coverage_90 - coverage_40
    assert delta_coverage >= 0.05, f"Post-clearing coverage delta too low: {delta_coverage:.4f} < 0.05"
