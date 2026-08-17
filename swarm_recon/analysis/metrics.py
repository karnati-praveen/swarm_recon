"""
SwarmMetrics — Quantitative analysis of TrajectoryLog outputs.

All methods are static and operate on a TrajectoryLog instance.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple, Union

from swarm_recon.config import TrajectoryLog, ThreatZone, DroneState, SwarmMode
from swarm_recon.core.grid import GridSearchSpace


class SwarmMetrics:
    """
    Static metrics computation for swarm simulation evaluation.

    Provides:
    - Coverage ratio (R1)
    - Minimum threat clearance (R2)
    - Heading entropy (R2)
    - Mean kinematic jerk (R2)
    - Target standoff distances (M-EXT1)
    - Standoff maintenance check (M-EXT1)
    - Mode transition event logging (M-EXT1)
    """

    @staticmethod
    def coverage_ratio(log: TrajectoryLog) -> float:
        """
        Replay the trajectory through a fresh GridSearchSpace and return
        the final fraction of cells visited.

        Args:
            log: A completed TrajectoryLog from SimulationEngine.

        Returns:
            Coverage ratio in [0.0, 1.0].
        """
        config = log.config
        grid = GridSearchSpace(
            width=config.width,
            height=config.height,
            resolution=config.resolution,
        )
        for frame in log.frames:
            for state in frame.drone_states.values():
                if state.active:
                    grid.mark_visited(
                        state.position[0],
                        state.position[1],
                        config.sensor_radius,
                    )
        return grid.get_coverage_ratio()

    @staticmethod
    def min_threat_clearance(log: TrajectoryLog) -> float:
        """
        Return the minimum signed clearance distance between any active drone
        and any threat zone boundary across all frames.

        Clearance = distance_to_center - threat_radius.
        Negative values indicate penetration.

        Args:
            log: A completed TrajectoryLog.

        Returns:
            Minimum clearance in meters. Returns float('inf') if no threats.
        """
        if not log.threat_zones:
            return float("inf")

        min_clearance = float("inf")
        for frame in log.frames:
            for state in frame.drone_states.values():
                if not state.active:
                    continue
                px, py = state.position
                for threat in log.threat_zones:
                    tcx, tcy = threat.center
                    d = math.hypot(px - tcx, py - tcy)
                    clearance = d - threat.radius
                    if clearance < min_clearance:
                        min_clearance = clearance

        return min_clearance

    @staticmethod
    def heading_entropy(log: TrajectoryLog) -> float:
        """
        Compute mean Shannon heading-change entropy across all active drone trajectories.

        Uses 36-bin histogram of per-step heading angle deltas.
        Higher entropy = more unpredictable, fluid trajectories.

        Args:
            log: A completed TrajectoryLog.

        Returns:
            Mean heading entropy in bits. Returns 0.0 if insufficient data.
        """
        dt = log.config.dt

        # Build per-drone position histories
        positions: Dict[int, List[Tuple[float, float]]] = {}
        for frame in log.frames:
            for did, state in frame.drone_states.items():
                if state.active:
                    if did not in positions:
                        positions[did] = []
                    positions[did].append(state.position)

        entropies = []
        for did, pos_list in positions.items():
            if len(pos_list) < 4:
                continue
            ent = SwarmMetrics._trajectory_entropy(pos_list, dt)
            entropies.append(ent)

        if not entropies:
            return 0.0
        return sum(entropies) / len(entropies)

    @staticmethod
    def mean_jerk(log: TrajectoryLog) -> float:
        """
        Compute mean kinematic jerk (rate of change of acceleration) across
        all active drone trajectories.

        Args:
            log: A completed TrajectoryLog.

        Returns:
            Mean jerk magnitude in m/s³. Returns 0.0 if insufficient data.
        """
        dt = log.config.dt

        positions: Dict[int, List[Tuple[float, float]]] = {}
        for frame in log.frames:
            for did, state in frame.drone_states.items():
                if state.active:
                    if did not in positions:
                        positions[did] = []
                    positions[did].append(state.position)

        jerks = []
        for did, pos_list in positions.items():
            if len(pos_list) < 4:
                continue
            j = SwarmMetrics._trajectory_jerk(pos_list, dt)
            jerks.append(j)

        if not jerks:
            return 0.0
        return sum(jerks) / len(jerks)

    @staticmethod
    def target_standoff_distances(
        log: TrajectoryLog,
        target_id: Optional[Union[int, str]] = None,
    ) -> Dict[str, Any]:
        """
        Compute time series and summary statistics of distances between tracking drones
        and the active target position.

        Args:
            log: TrajectoryLog from SimulationEngine.
            target_id: Optional target ID filter.

        Returns:
            Dict containing 'time_series' (dict mapping timestamp -> {drone_id: dist})
            and 'summary' (dict with mean, std, min, max, median, sample_count).
        """
        time_series: Dict[float, Dict[int, float]] = {}
        all_distances: List[float] = []

        for frame in log.frames:
            target_pos = None
            if frame.target_state and isinstance(frame.target_state, dict):
                raw_pos = frame.target_state.get("position")
                if raw_pos and len(raw_pos) == 2:
                    target_pos = (float(raw_pos[0]), float(raw_pos[1]))

            if target_pos is None:
                continue

            frame_dists: Dict[int, float] = {}
            for did, state in frame.drone_states.items():
                if not state.active:
                    continue
                mode_str = state.mode.value if hasattr(state.mode, "value") else str(state.mode)
                if mode_str == "TARGET_TRACKING":
                    dist = math.hypot(state.position[0] - target_pos[0], state.position[1] - target_pos[1])
                    frame_dists[did] = dist
                    all_distances.append(dist)

            if frame_dists:
                time_series[frame.timestamp] = frame_dists

        if not all_distances:
            summary = {
                "mean": 0.0,
                "std": 0.0,
                "min": 0.0,
                "max": 0.0,
                "median": 0.0,
                "sample_count": 0,
            }
        else:
            sorted_dists = sorted(all_distances)
            count = len(sorted_dists)
            mean_val = sum(sorted_dists) / count
            var_val = sum((x - mean_val) ** 2 for x in sorted_dists) / count
            std_val = math.sqrt(var_val)
            min_val = sorted_dists[0]
            max_val = sorted_dists[-1]
            if count % 2 == 1:
                median_val = sorted_dists[count // 2]
            else:
                median_val = (sorted_dists[count // 2 - 1] + sorted_dists[count // 2]) / 2.0

            summary = {
                "mean": mean_val,
                "std": std_val,
                "min": min_val,
                "max": max_val,
                "median": median_val,
                "sample_count": count,
            }

        return {
            "time_series": time_series,
            "summary": summary,
        }

    @staticmethod
    def is_standoff_maintained(
        log: TrajectoryLog,
        min_r: float = 10.0,
        max_r: float = 20.0,
        tolerance_ratio: float = 0.95,
        settling_window: float = 15.0,
    ) -> Dict[str, Any]:
        """
        Verify whether tracking drones maintain target standoff radius within [min_r, max_r].

        Evaluates distance samples during steady-state tracking phase, skipping initial settling
        approach window (default 8.0s for drones at max speed 5m/s from 100x100m grid corners).

        Args:
            log: TrajectoryLog from SimulationEngine.
            min_r: Minimum standoff radius threshold in meters (default 10.0).
            max_r: Maximum standoff radius threshold in meters (default 20.0).
            tolerance_ratio: Minimum required fraction of samples inside bounds (default 0.95).
            settling_window: Initial approach transition window in seconds (default 8.0).

        Returns:
            Dict containing 'maintained' (bool), 'in_range_ratio', 'total_samples', etc.
        """
        standoff_data = SwarmMetrics.target_standoff_distances(log)
        ts = standoff_data["time_series"]

        if not ts:
            return {
                "maintained": False,
                "in_range_ratio": 0.0,
                "total_samples": 0,
                "in_range_samples": 0,
                "out_of_range_samples": 0,
                "min_observed": 0.0,
                "max_observed": 0.0,
                "bounds": (min_r, max_r),
            }

        first_timestamp = min(ts.keys())

        eval_samples: List[float] = []
        for t, d_dict in ts.items():
            if t < first_timestamp + settling_window:
                continue
            eval_samples.extend(d_dict.values())

        if not eval_samples:
            for d_dict in ts.values():
                eval_samples.extend(d_dict.values())

        if not eval_samples:
            return {
                "maintained": False,
                "in_range_ratio": 0.0,
                "total_samples": 0,
                "in_range_samples": 0,
                "out_of_range_samples": 0,
                "min_observed": 0.0,
                "max_observed": 0.0,
                "bounds": (min_r, max_r),
            }

        total_count = len(eval_samples)
        in_range_count = sum(1 for d in eval_samples if min_r <= d <= max_r)
        in_range_ratio = in_range_count / total_count
        maintained = in_range_ratio >= tolerance_ratio

        return {
            "maintained": maintained,
            "in_range_ratio": in_range_ratio,
            "total_samples": total_count,
            "in_range_samples": in_range_count,
            "out_of_range_samples": total_count - in_range_count,
            "min_observed": min(eval_samples),
            "max_observed": max(eval_samples),
            "bounds": (min_r, max_r),
        }

    @staticmethod
    def mode_transition_events(log: TrajectoryLog) -> List[Dict[str, Any]]:
        """
        Record all swarm mode transition events across frames.

        Args:
            log: TrajectoryLog from SimulationEngine.

        Returns:
            List of transition event dictionaries.
        """
        events: List[Dict[str, Any]] = []
        prev_modes: Dict[int, str] = {}

        for frame in log.frames:
            for did, state in frame.drone_states.items():
                if not state.active:
                    continue
                curr_mode = state.mode.value if hasattr(state.mode, "value") else str(state.mode)
                if did in prev_modes:
                    old_mode = prev_modes[did]
                    if old_mode != curr_mode:
                        events.append({
                            "timestamp": frame.timestamp,
                            "drone_id": did,
                            "from_mode": old_mode,
                            "to_mode": curr_mode,
                            "target_position": list(state.target_position) if state.target_position else None,
                        })
                prev_modes[did] = curr_mode

        return events

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _trajectory_entropy(
        positions: List[Tuple[float, float]],
        dt: float,
        num_bins: int = 36,
    ) -> float:
        """Compute Shannon heading-change entropy for a single trajectory."""
        vels = [
            (
                (positions[i + 1][0] - positions[i][0]) / dt,
                (positions[i + 1][1] - positions[i][1]) / dt,
            )
            for i in range(len(positions) - 1)
        ]
        headings = [
            math.atan2(v[1], v[0])
            for v in vels
            if math.hypot(v[0], v[1]) > 1e-3
        ]
        if len(headings) < 2:
            return 0.0

        # Heading angle deltas, wrapped to [-pi, pi]
        deltas = []
        for i in range(len(headings) - 1):
            dh = headings[i + 1] - headings[i]
            dh = (dh + math.pi) % (2.0 * math.pi) - math.pi
            deltas.append(dh)

        bins = [0] * num_bins
        for dh in deltas:
            idx = int(((dh + math.pi) / (2.0 * math.pi)) * num_bins)
            idx = min(num_bins - 1, max(0, idx))
            bins[idx] += 1

        total = len(deltas)
        entropy = 0.0
        for count in bins:
            if count > 0:
                p = count / total
                entropy -= p * math.log2(p)
        return entropy

    @staticmethod
    def _trajectory_jerk(
        positions: List[Tuple[float, float]],
        dt: float,
    ) -> float:
        """Compute mean jerk magnitude for a single trajectory."""
        vels = [
            (
                (positions[i + 1][0] - positions[i][0]) / dt,
                (positions[i + 1][1] - positions[i][1]) / dt,
            )
            for i in range(len(positions) - 1)
        ]
        accs = [
            (
                (vels[i + 1][0] - vels[i][0]) / dt,
                (vels[i + 1][1] - vels[i][1]) / dt,
            )
            for i in range(len(vels) - 1)
        ]
        jerks = [
            math.hypot(
                (accs[i + 1][0] - accs[i][0]) / dt,
                (accs[i + 1][1] - accs[i][1]) / dt,
            )
            for i in range(len(accs) - 1)
        ]
        return sum(jerks) / len(jerks) if jerks else 0.0

