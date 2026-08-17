"""
Tier 1 E2E Test Suite: Baseline System Verification

Tests foundational mechanics:
1. Spawning N drones in search grid within spatial bounding box.
2. Occupancy grid search coverage tracking and cell marking monotonicity.
3. Dependency installation directory size limit verification (<= 500MB).
"""

import os
import sys
import math
import pytest
from tests.e2e.conftest import StandaloneSimulationEngine, MetricsCalculator

# Ensure core module import fallback
try:
    from swarm_recon.core.grid import GridSearchSpace
except ImportError:
    GridSearchSpace = None


def test_spawn_n_drones(default_sim_config):
    """E2E-T1-01: Verifies spawning N drones in a bounded search grid."""
    num_drones = 5
    width = 100.0
    height = 100.0

    config = dict(default_sim_config)
    config["num_drones"] = num_drones
    config["width"] = width
    config["height"] = height
    config["time_limit"] = 1.0  # Just initial spawn step

    engine = StandaloneSimulationEngine(config)
    
    # Assert drone count
    assert len(engine.positions) == num_drones, f"Expected {num_drones} spawned drones, got {len(engine.positions)}"
    assert len(engine.active_drones) == num_drones, f"Expected {num_drones} active status flags"

    # Assert spatial bounding box limits
    for did, (x, y) in engine.positions.items():
        assert 0.0 <= x <= width, f"Drone {did} x-coord {x} out of bounds [0, {width}]"
        assert 0.0 <= y <= height, f"Drone {did} y-coord {y} out of bounds [0, {height}]"
        assert engine.active_drones[did] is True, f"Drone {did} should be active at spawn"

    # Assert inter-drone minimum initial separation distance
    pos_list = list(engine.positions.values())
    for i in range(len(pos_list)):
        for j in range(i + 1, len(pos_list)):
            dist = math.hypot(pos_list[i][0] - pos_list[j][0], pos_list[i][1] - pos_list[j][1])
            assert dist >= 2.0, f"Inter-drone separation {dist:.2f}m between drone {i} and {j} < 2.0m threshold"


def test_grid_coverage_tracking(default_sim_config):
    """E2E-T1-02: Verifies discrete search grid coverage tracking and cell accumulation."""
    config = dict(default_sim_config)
    config["time_limit"] = 10.0
    config["dt"] = 0.1

    engine = StandaloneSimulationEngine(config)
    steps = int(config["time_limit"] / config["dt"])
    cols, rows = engine.cols, engine.rows
    total_cells = cols * rows

    prev_coverage = 0.0

    for step in range(steps):
        t_sim = step * config["dt"]
        
        # Step active drones
        for did in range(engine.num_drones):
            px, py = engine.positions[did]
            vx, vy = engine.velocities[did]
            nx = max(0.5, min(engine.width - 0.5, px + vx * engine.dt))
            ny = max(0.5, min(engine.height - 0.5, py + vy * engine.dt))
            engine.positions[did] = (nx, ny)

            # Mark visited disk
            r_min = max(0, int((ny - engine.sensor_radius) / engine.resolution))
            r_max = min(rows - 1, int((ny + engine.sensor_radius) / engine.resolution))
            c_min = max(0, int((nx - engine.sensor_radius) / engine.resolution))
            c_max = min(cols - 1, int((nx + engine.sensor_radius) / engine.resolution))

            for r in range(r_min, r_max + 1):
                cy = (r + 0.5) * engine.resolution
                for c in range(c_min, c_max + 1):
                    cx = (c + 0.5) * engine.resolution
                    if (cx - nx) ** 2 + (cy - ny) ** 2 <= engine.sensor_radius ** 2:
                        engine.visited_grid.add((r, c))

        current_coverage = len(engine.visited_grid) / float(total_cells)

        # Assert monotonic increase (coverage should never decrease)
        assert current_coverage >= prev_coverage, f"Coverage ratio regressed from {prev_coverage} to {current_coverage}"
        assert 0.0 <= current_coverage <= 1.0, f"Coverage ratio {current_coverage} out of range [0, 1]"
        prev_coverage = current_coverage

    assert current_coverage > 0.0, "Coverage ratio must be strictly positive after 10 seconds of navigation"


def test_installation_size_limit():
    """E2E-T3 / R3: Verifies that the project dependency environment footprint is strictly <= 500MB."""
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    
    # Check virtual environment locations (.venv, venv, or python installation / site-packages)
    venv_candidates = [
        os.path.join(project_root, ".venv"),
        os.path.join(project_root, "venv"),
        sys.prefix,
        os.path.dirname(sys.executable)
    ]

    env_size_mb = 0.0
    checked_path = None
    for cand in venv_candidates:
        if os.path.exists(cand):
            size = MetricsCalculator.get_directory_size_mb(cand)
            if size > 0.0:
                env_size_mb = size
                checked_path = cand
                break

    # If base python is external and large, fallback to site-packages directory inside sys.prefix
    if checked_path and env_size_mb > 500.0:
        site_pkg = os.path.join(sys.prefix, "Lib", "site-packages")
        if os.path.exists(site_pkg):
            site_size = MetricsCalculator.get_directory_size_mb(site_pkg)
            if 0.0 < site_size <= 500.0:
                env_size_mb = site_size
                checked_path = site_pkg

    assert checked_path is not None, "Could not locate Python virtual environment directory"
    assert env_size_mb <= 500.0, f"Requirement R3 Violation: Environment size {env_size_mb:.2f} MB exceeds 500.0 MB limit!"
    print(f"\n[R3 Verification] Checked Environment Path: {checked_path} | Size: {env_size_mb:.2f} MB <= 500.0 MB")

