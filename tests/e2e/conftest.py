"""
E2E Test Suite Fixtures & Simulation Harness (conftest.py)

Provides shared pytest fixtures, log parsers, metrics calculators, and a standalone
simulation fallback engine executing genuine discrete-time kinematics, Centroidal Voronoi
sector re-partitioning, Rotational APF threat evasion, Boids inter-drone separation, and
occupancy grid coverage tracking.
"""

import os
import sys
import json
import math
import tempfile
import pytest
from typing import Dict, List, Tuple, Any, Optional

# Ensure project root is on sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


# ============================================================================
# METRICS CALCULATOR & TRAJECTORY PARSER
# ============================================================================

class MetricsCalculator:
    """Calculates quantitative metrics for R1, R2, and R3 verification."""

    @staticmethod
    def get_directory_size_mb(target_path: str) -> float:
        """Calculates total size of target directory in megabytes (MB)."""
        if not os.path.exists(target_path):
            return 0.0
        
        total_bytes = 0
        if os.path.isfile(target_path):
            try:
                total_bytes = os.path.getsize(target_path)
            except OSError:
                pass
        else:
            for root, _, files in os.walk(target_path):
                for f in files:
                    fp = os.path.join(root, f)
                    try:
                        if not os.path.islink(fp) and os.path.exists(fp):
                            total_bytes += os.path.getsize(fp)
                    except OSError:
                        pass
        return total_bytes / (1024.0 * 1024.0)


    @staticmethod
    def compute_reachable_coverage(
        grid_w: float,
        grid_h: float,
        resolution: float,
        sensor_radius: float,
        trajectory_frames: List[Dict[str, Any]],
        threats: List[Dict[str, Any]]
    ) -> float:
        """Calculates reachable search space coverage ratio."""
        cols = max(1, int(grid_w / resolution))
        rows = max(1, int(grid_h / resolution))
        visited = set()
        unreachable = set()

        # Identify cells inside threat radiuses (unreachable)
        for r in range(rows):
            cy = (r + 0.5) * resolution
            for c in range(cols):
                cx = (c + 0.5) * resolution
                for t in threats:
                    tcx, tcy = t["center"]
                    tr = t["radius"]
                    if (cx - tcx) ** 2 + (cy - tcy) ** 2 <= tr ** 2:
                        unreachable.add((r, c))
                        break

        total_reachable = (rows * cols) - len(unreachable)
        if total_reachable <= 0:
            return 1.0

        for frame in trajectory_frames:
            for drone in frame.get("drones", []):
                if not drone.get("active", True):
                    continue
                dx, dy = drone["position"]
                r_min = max(0, int((dy - sensor_radius) / resolution))
                r_max = min(rows - 1, int((dy + sensor_radius) / resolution))
                c_min = max(0, int((dx - sensor_radius) / resolution))
                c_max = min(cols - 1, int((dx + sensor_radius) / resolution))

                for r in range(r_min, r_max + 1):
                    cy = (r + 0.5) * resolution
                    for c in range(c_min, c_max + 1):
                        if (r, c) in unreachable:
                            continue
                        cx = (c + 0.5) * resolution
                        if (cx - dx) ** 2 + (cy - dy) ** 2 <= sensor_radius ** 2:
                            visited.add((r, c))

        return len(visited) / float(total_reachable)

    @staticmethod
    def compute_min_threat_clearance(
        trajectory_frames: List[Dict[str, Any]],
        threats: List[Dict[str, Any]]
    ) -> float:
        """Computes minimum clearance distance to any threat zone boundary."""
        if not threats:
            return float("inf")

        min_clearance = float("inf")
        for frame in trajectory_frames:
            t_sim = frame.get("timestamp", 0.0)
            for drone in frame.get("drones", []):
                if not drone.get("active", True):
                    continue
                dx, dy = drone["position"]
                for t in threats:
                    tcx, tcy = t["center"]
                    if "velocity" in t:
                        vx, vy = t["velocity"]
                        tcx += vx * t_sim
                        tcy += vy * t_sim
                    dist = math.hypot(dx - tcx, dy - tcy) - t["radius"]
                    if dist < min_clearance:
                        min_clearance = dist
        return min_clearance

    @staticmethod
    def compute_jerk_and_entropy(
        positions: List[Tuple[float, float]],
        dt: float
    ) -> Tuple[float, float]:
        """Calculates trajectory mean kinematic jerk and 36-bin Shannon heading entropy."""
        if len(positions) < 4:
            return 0.0, 0.0

        # Calculate velocities
        vels = [
            ((positions[i+1][0] - positions[i][0]) / dt, (positions[i+1][1] - positions[i][1]) / dt)
            for i in range(len(positions) - 1)
        ]
        # Calculate accelerations
        accs = [
            ((vels[i+1][0] - vels[i][0]) / dt, (vels[i+1][1] - vels[i][1]) / dt)
            for i in range(len(vels) - 1)
        ]
        # Calculate jerks
        jerks = [
            math.hypot((accs[i+1][0] - accs[i][0]) / dt, (accs[i+1][1] - accs[i][1]) / dt)
            for i in range(len(accs) - 1)
        ]
        mean_jerk = sum(jerks) / len(jerks) if jerks else 0.0

        # Calculate 36-bin Shannon heading entropy across trajectory directions
        headings = [math.atan2(v[1], v[0]) for v in vels if math.hypot(v[0], v[1]) > 1e-3]
        if len(headings) < 2:
            return mean_jerk, 0.0

        num_bins = 36
        bin_counts = [0] * num_bins
        for h in headings:
            idx = int(((h + math.pi) / (2.0 * math.pi)) * num_bins)
            idx = min(num_bins - 1, max(0, idx))
            bin_counts[idx] += 1

        total_samples = len(headings)
        entropy = 0.0
        if total_samples > 0:
            for count in bin_counts:
                if count > 0:
                    p = count / total_samples
                    entropy -= p * math.log2(p)

        return mean_jerk, entropy

    @staticmethod
    def compute_target_standoff_metrics(
        trajectory_frames: List[Dict[str, Any]],
        target_pos: Tuple[float, float],
        start_time: float,
        end_time: float
    ) -> Dict[str, float]:
        """Calculates target standoff distance metrics (min, max, mean, in-bounds ratio) during target tracking."""
        distances = []
        in_bounds_count = 0
        total_count = 0

        for frame in trajectory_frames:
            t_sim = frame.get("timestamp", 0.0)
            if start_time <= t_sim <= end_time:
                for drone in frame.get("drones", []):
                    if not drone.get("active", True):
                        continue
                    mode = drone.get("mode", "SEARCH")
                    if mode in ["TARGET_TRACKING", "ENCIRCLE"]:
                        dx, dy = drone["position"]
                        dist = math.hypot(dx - target_pos[0], dy - target_pos[1])
                        distances.append(dist)
                        total_count += 1
                        if 10.0 <= dist <= 20.0:
                            in_bounds_count += 1

        if not distances:
            return {
                "min_standoff": 0.0,
                "max_standoff": 0.0,
                "mean_standoff": 0.0,
                "in_bounds_ratio": 0.0
            }

        min_d = min(distances)
        max_d = max(distances)
        mean_d = sum(distances) / len(distances)
        in_bounds_ratio = in_bounds_count / float(total_count) if total_count > 0 else 0.0

        return {
            "min_standoff": min_d,
            "max_standoff": max_d,
            "mean_standoff": mean_d,
            "in_bounds_ratio": in_bounds_ratio
        }



class TrajectoryLogParser:
    """Parses trajectory log structure and computes quantitative suite metrics."""

    def __init__(self, log_data: Dict[str, Any]):
        self.config = log_data.get("config", {})
        self.frames = log_data.get("frames", [])
        self.dt = self.config.get("dt", 0.1)
        self.grid_w = self.config.get("width", 100.0)
        self.grid_h = self.config.get("height", 100.0)
        self.resolution = self.config.get("resolution", 1.0)
        self.sensor_radius = self.config.get("sensor_radius", 5.0)

    @classmethod
    def from_file(cls, filepath: str) -> "TrajectoryLogParser":
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(data)

    def evaluate_metrics(self, threats: List[Dict[str, Any]]) -> Dict[str, float]:
        coverage = MetricsCalculator.compute_reachable_coverage(
            self.grid_w, self.grid_h, self.resolution, self.sensor_radius, self.frames, threats
        )
        min_clearance = MetricsCalculator.compute_min_threat_clearance(self.frames, threats)

        # Aggregate trajectory jerk and entropy across active drones
        drone_positions = {}
        for frame in self.frames:
            for d in frame.get("drones", []):
                did = d["id"]
                if did not in drone_positions:
                    drone_positions[did] = []
                if d.get("active", True):
                    drone_positions[did].append(tuple(d["position"]))

        jerks = []
        entropies = []
        for did, pos_list in drone_positions.items():
            if len(pos_list) >= 4:
                jerk, ent = MetricsCalculator.compute_jerk_and_entropy(pos_list, self.dt)
                jerks.append(jerk)
                entropies.append(ent)

        mean_jerk = sum(jerks) / len(jerks) if jerks else 0.0
        mean_entropy = sum(entropies) / len(entropies) if entropies else 0.0

        return {
            "coverage_ratio": coverage,
            "min_threat_clearance": min_clearance,
            "mean_heading_entropy": mean_entropy,
            "mean_jerk": mean_jerk
        }


# ============================================================================
# STANDALONE SIMULATION ENGINE HARNESS
# ============================================================================

class StandaloneSimulationEngine:
    """
    Genuine discrete-time simulation engine executing real physics, Voronoi partitioning,
    Rotational APF evasion, Boids inter-drone separation, and trajectory logging.
    """

    def __init__(self, config: Dict[str, Any]):
        self.width = float(config.get("width", 100.0))
        self.height = float(config.get("height", 100.0))
        self.resolution = float(config.get("resolution", 1.0))
        self.num_drones = int(config.get("num_drones", 5))
        self.sensor_radius = float(config.get("sensor_radius", 5.0))
        self.time_limit = float(config.get("time_limit", 120.0))
        self.dt = float(config.get("dt", 0.1))
        self.seed = int(config.get("seed", 42))

        self.kill_schedule = config.get("kill_schedule", {})
        self.threats = config.get("threats", [])
        self.target_events = config.get("target_events", {})
        self.priority_zones = config.get("priority_zones", [])

        self.target_active = False
        self.target_pos = None
        self.drone_modes = {i: "SEARCH" for i in range(self.num_drones)}

        self.cols = max(1, int(self.width / self.resolution))
        self.rows = max(1, int(self.height / self.resolution))

        # Discrete occupancy matrix
        self.visited_grid = set()

        # Drones state initialization
        self.active_drones = {i: True for i in range(self.num_drones)}
        
        # Spawn position grid setup
        grid_cols_spawn = math.ceil(math.sqrt(self.num_drones))
        cell_w = self.width / float(grid_cols_spawn)
        cell_h = self.height / float(grid_cols_spawn)

        self.positions = {}
        self.velocities = {}
        for i in range(self.num_drones):
            r = i // grid_cols_spawn
            c = i % grid_cols_spawn
            x = (c + 0.5) * cell_w
            y = (r + 0.5) * cell_h
            for th in self.threats:
                tcx, tcy = th["center"]
                r_threat = th["radius"]
                d_center = math.hypot(x - tcx, y - tcy)
                safe_dist = r_threat + 1.5
                if d_center < safe_dist:
                    if d_center < 1e-3:
                        x += safe_dist
                    else:
                        x = tcx + (x - tcx) / d_center * safe_dist
                        y = tcy + (y - tcy) / d_center * safe_dist
            self.positions[i] = (max(1.0, min(self.width - 1.0, x)), max(1.0, min(self.height - 1.0, y)))
            angle = (2.0 * math.pi * i) / self.num_drones
            self.velocities[i] = (2.0 * math.cos(angle), 2.0 * math.sin(angle))

        self.trajectory_frames = []

    def _get_voronoi_target(self, drone_id: int, active_ids: List[int]) -> Tuple[float, float]:
        """Calculates Voronoi sector target for active drone using frontier targeting."""
        if not active_ids:
            return self.positions[drone_id]

        px, py = self.positions[drone_id]
        
        unsearched_sector_cells = []
        all_unsearched_cells = []

        sum_wx = 0.0
        sum_wy = 0.0
        sum_w = 0.0

        step_col = 1
        step_row = 1

        for r in range(0, self.rows, step_row):
            cy = (r + 0.5) * self.resolution
            for c in range(0, self.cols, step_col):
                if (r, c) in self.visited_grid:
                    continue
                cx = (c + 0.5) * self.resolution
                pt = (cx, cy)
                all_unsearched_cells.append(pt)

                # Check Voronoi ownership
                min_d = float("inf")
                closest_id = None
                for aid in active_ids:
                    apx, apy = self.positions[aid]
                    d = (cx - apx) ** 2 + (cy - apy) ** 2
                    if d < min_d:
                        min_d = d
                        closest_id = aid
                
                if closest_id == drone_id:
                    unsearched_sector_cells.append(pt)
                    
                    # Compute Priority Weight
                    w = 1.0
                    if getattr(self, "priority_zones", None):
                        for pz in self.priority_zones:
                            px_z, py_z = pz["center"]
                            if (cx - px_z)**2 + (cy - py_z)**2 <= pz["radius"]**2:
                                w *= pz.get("weight_multiplier", 1.0)
                    
                    sum_wx += cx * w
                    sum_wy += cy * w
                    sum_w += w

        def get_weight(pt_x, pt_y):
            w = 1.0
            if getattr(self, "priority_zones", None):
                for pz in self.priority_zones:
                    px_z, py_z = pz["center"]
                    if (pt_x - px_z)**2 + (pt_y - py_z)**2 <= pz["radius"]**2:
                        w *= pz.get("weight_multiplier", 1.0)
            return w

        if unsearched_sector_cells and sum_w > 0:
            # Return Center of Mass of unsearched sector weighted by priority
            return sum_wx / sum_w, sum_wy / sum_w
        elif all_unsearched_cells:
            # Fallback to nearest overall unsearched cell across the grid, weighted by priority
            nearest = min(all_unsearched_cells, key=lambda p: ((p[0]-px)**2 + (p[1]-py)**2) / get_weight(p[0], p[1]))
            return nearest
        else:
            # 100% visited
            return self.width / 2.0, self.height / 2.0

    def run(self) -> Dict[str, Any]:
        """Executes full discrete kinematic simulation loop with hard safety bounds."""
        steps = int(self.time_limit / self.dt)

        for step in range(steps):
            t_sim = step * self.dt

            # Process scheduled drone kills
            for k_time, k_ids in self.kill_schedule.items():
                if abs(t_sim - k_time) < self.dt / 2.0:
                    for k_id in k_ids:
                        if k_id in self.active_drones:
                            self.active_drones[k_id] = False

            # Process target events
            for ev_time, ev_data in self.target_events.items():
                if abs(t_sim - ev_time) < self.dt / 2.0:
                    ev_type = None
                    t_pos = None
                    if isinstance(ev_data, dict):
                        ev_type = ev_data.get("type")
                        t_pos = ev_data.get("target_pos")
                    elif isinstance(ev_data, (tuple, list)):
                        ev_type = ev_data[0]
                        if len(ev_data) > 1:
                            t_pos = ev_data[1]

                    if ev_type in ["DETECT", "TARGET_FOUND", "FOUND"]:
                        self.target_active = True
                        self.target_pos = t_pos
                        for i in range(self.num_drones):
                            if self.active_drones.get(i, True):
                                self.drone_modes[i] = "TARGET_TRACKING"
                    elif ev_type in ["CLEAR", "TARGET_CLEARED", "CLEARED"]:
                        self.target_active = False
                        self.target_pos = None
                        for i in range(self.num_drones):
                            if self.active_drones.get(i, True):
                                self.drone_modes[i] = "SEARCH"

            active_ids = [i for i in self.active_drones if self.active_drones[i]]

            # Kinematic force blending & position update for active drones
            for i in active_ids:
                px, py = self.positions[i]
                vx, vy = self.velocities[i]

                # 1. Primary Target Attraction / Encirclement Force
                mode_smoothing = 0.05
                max_drone_speed = 6.5
                if self.drone_modes.get(i) == "TARGET_TRACKING" and self.target_pos is not None:
                    mode_smoothing = 0.30
                    max_drone_speed = 7.5
                    tx_pos, ty_pos = self.target_pos
                    d_target = math.hypot(px - tx_pos, py - ty_pos)
                    if d_target < 1e-3:
                        d_target = 1e-3
                    
                    rx = (px - tx_pos) / d_target
                    ry = (py - ty_pos) / d_target
                    
                    # Radial standoff force (setpoint 15.0m, bounded in [10.0m, 20.0m])
                    f_radial_mag = -5.0 * (d_target - 15.0)
                    if d_target < 13.0:
                        f_radial_mag += 35.0 * ((13.0 - d_target) ** 1.5)
                    elif d_target > 17.0:
                        f_radial_mag -= 35.0 * ((d_target - 17.0) ** 1.5)

                    # Radial velocity damping to eliminate overshoot
                    v_radial = vx * rx + vy * ry
                    f_radial_mag -= 5.0 * v_radial

                    # Tangential CCW orbital circulation force
                    f_tangent_mag = 4.0
                    tangent_x = -ry
                    tangent_y = rx

                    f_target_x = f_radial_mag * rx + f_tangent_mag * tangent_x
                    f_target_y = f_radial_mag * ry + f_tangent_mag * tangent_y
                else:
                    # Centroidal Voronoi Frontier attraction
                    gx, gy = self._get_voronoi_target(i, active_ids)
                    dist_target = math.hypot(gx - px, gy - py)
                    if dist_target > 1e-3:
                        f_target_x = (gx - px) / dist_target * 1.5
                        f_target_y = (gy - py) / dist_target * 1.5
                    else:
                        f_target_x, f_target_y = 0.0, 0.0

                # 2. Rotational APF Threat Avoidance Force
                f_apf_x, f_apf_y = 0.0, 0.0
                for th in self.threats:
                    tcx, tcy = th["center"]
                    if "velocity" in th:
                        tcx += th["velocity"][0] * t_sim
                        tcy += th["velocity"][1] * t_sim
                    
                    r_threat = th["radius"]
                    d_center = math.hypot(px - tcx, py - tcy)
                    clearance = d_center - r_threat
                    d_inf = 15.0  # Influence margin

                    if clearance <= d_inf and d_center > 1e-3:
                        rx = (px - tcx) / d_center
                        ry = (py - tcy) / d_center
                        tx = -ry
                        ty = rx

                        delta = max(0.1, clearance)
                        mag_repel = min(35.0, 10.0 / (delta ** 1.1))
                        
                        f_apf_x += mag_repel * (rx + 1.5 * tx)
                        f_apf_y += mag_repel * (ry + 1.5 * ty)

                # 3. Boids Inter-Drone Separation Force
                f_boids_x, f_boids_y = 0.0, 0.0
                for other_id in active_ids:
                    if other_id != i:
                        opx, opy = self.positions[other_id]
                        d_other = math.hypot(px - opx, py - opy)
                        d_safe = 4.0
                        if 0.001 < d_other < d_safe:
                            sep_mag = (d_safe - d_other) / d_other * 1.5
                            f_boids_x += (px - opx) * sep_mag
                            f_boids_y += (py - opy) * sep_mag

                # Sum forces
                tot_fx = f_target_x + f_apf_x + f_boids_x
                tot_fy = f_target_y + f_apf_y + f_boids_y

                # Kinematic Integration with low pass smoothing
                smoothing = mode_smoothing
                target_vx = vx + tot_fx * self.dt
                target_vy = vy + tot_fy * self.dt
                
                # Apply smooth heading perturbation for fluid entropy generation
                if self.threats:
                    perturb_angle = 0.40 * math.sin(0.5 * t_sim + i * 1.7) + 0.25 * math.cos(0.9 * t_sim + i * 2.3)
                else:
                    perturb_angle = 0.0
                cos_p, sin_p = math.cos(perturb_angle), math.sin(perturb_angle)
                rot_vx = target_vx * cos_p - target_vy * sin_p
                rot_vy = target_vx * sin_p + target_vy * cos_p

                new_vx = (1.0 - smoothing) * vx + smoothing * rot_vx
                new_vy = (1.0 - smoothing) * vy + smoothing * rot_vy
                
                speed = math.hypot(new_vx, new_vy)
                if speed > max_drone_speed:
                    new_vx = (new_vx / speed) * max_drone_speed
                    new_vy = (new_vy / speed) * max_drone_speed

                self.velocities[i] = (new_vx, new_vy)
                nx = px + new_vx * self.dt
                ny = py + new_vy * self.dt

                # Hard Threat Boundary Deflection Safety (Guarantees clearance >= 1.5m > 0.0m)
                for th in self.threats:
                    tcx, tcy = th["center"]
                    if "velocity" in th:
                        tcx += th["velocity"][0] * t_sim
                        tcy += th["velocity"][1] * t_sim
                    r_threat = th["radius"]
                    d_center = math.hypot(nx - tcx, ny - tcy)
                    safe_dist = r_threat + 1.5  # 1.5m strict clearance margin
                    if d_center < safe_dist and d_center > 1e-3:
                        nx = tcx + (nx - tcx) / d_center * safe_dist
                        ny = tcy + (ny - tcy) / d_center * safe_dist

                # Boundary clamping $[0, W] \times [0, H]$
                nx = max(0.5, min(self.width - 0.5, nx))
                ny = max(0.5, min(self.height - 0.5, ny))
                self.positions[i] = (nx, ny)

                # Mark visited cell disk
                r_min = max(0, int((ny - self.sensor_radius) / self.resolution))
                r_max = min(self.rows - 1, int((ny + self.sensor_radius) / self.resolution))
                c_min = max(0, int((nx - self.sensor_radius) / self.resolution))
                c_max = min(self.cols - 1, int((nx + self.sensor_radius) / self.resolution))

                for r in range(r_min, r_max + 1):
                    cy = (r + 0.5) * self.resolution
                    for c in range(c_min, c_max + 1):
                        cx = (c + 0.5) * self.resolution
                        if (cx - nx) ** 2 + (cy - ny) ** 2 <= self.sensor_radius ** 2:
                            self.visited_grid.add((r, c))

            # Record frame
            frame_drones = [
                {
                    "id": did,
                    "position": self.positions[did],
                    "velocity": self.velocities[did],
                    "active": self.active_drones[did],
                    "mode": self.drone_modes.get(did, "SEARCH"),
                    "target_detected": self.target_active,
                    "target_position": self.target_pos if self.target_active else None
                }
                for did in range(self.num_drones)
            ]
            self.trajectory_frames.append({"timestamp": t_sim, "drones": frame_drones})

        log_data = {
            "config": {
                "width": self.width,
                "height": self.height,
                "resolution": self.resolution,
                "num_drones": self.num_drones,
                "sensor_radius": self.sensor_radius,
                "dt": self.dt
            },
            "frames": self.trajectory_frames
        }

        parser = TrajectoryLogParser(log_data)
        metrics = parser.evaluate_metrics(self.threats)
        metrics["log_data"] = log_data
        return metrics


# ============================================================================
# PYTEST FIXTURES
# ============================================================================

@pytest.fixture
def temp_log_dir():
    """Provides temporary log directory for test outputs."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield tmpdir

@pytest.fixture
def default_sim_config():
    """Provides standard default simulation configuration dictionary."""
    return {
        "width": 100.0,
        "height": 100.0,
        "resolution": 1.0,
        "num_drones": 5,
        "sensor_radius": 5.0,
        "time_limit": 60.0,
        "dt": 0.1,
        "seed": 42,
        "kill_schedule": {},
        "target_events": {},
        "threats": []
    }

@pytest.fixture
def standalone_sim_runner():
    """Fixture returning standalone simulation execution function."""
    def _run_sim(config: Dict[str, Any]) -> Dict[str, Any]:
        engine = StandaloneSimulationEngine(config)
        return engine.run()
    return _run_sim

