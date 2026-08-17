"""
SimulationEngine — Discrete-time kinematic simulation loop.

Orchestrates N SwarmAgent drones over a 2D search area, injecting
drone kill events and target detection/clearance schedules at specified times,
and recording full trajectory logs with multi-mode telemetry tracking.
"""

from __future__ import annotations

import math
import os
import random
from typing import Any, Dict, List, Optional, Union

from swarm_recon.config import (
    SimulationConfig,
    ThreatZone,
    DroneState,
    TrajectoryFrame,
    TrajectoryLog,
    SwarmMode,
    PacketType,
    TargetState,
    TelemetryPacket,
    _clean_val,
)
from swarm_recon.core.grid import GridSearchSpace
from swarm_recon.agents.drone import SwarmAgent


class SimulationEngine:
    """
    Runs a full discrete-time swarm reconnaissance simulation.

    Args:
        config: Simulation parameters (area size, num drones, dt, etc.)
        threats: List of circular threat zones to avoid.
        kill_at: Optional dict mapping simulation time (float) -> list of drone IDs to kill.
        target_schedule: Optional dict/list mapping simulation time to target detection/clearing events.
    """

    def __init__(
        self,
        config: SimulationConfig,
        threats: Optional[List[ThreatZone]] = None,
        kill_at: Optional[Dict[float, List[int]]] = None,
        target_schedule: Optional[Union[Dict[float, Dict[str, Any]], List[Dict[str, Any]]]] = None,
    ) -> None:
        self._config = config
        self._threats: List[ThreatZone] = threats or []
        self._kill_at: Dict[float, List[int]] = kill_at or {}
        self._target_schedule = target_schedule or {}

        # Seed random for reproducibility
        if config.random_seed is not None:
            random.seed(config.random_seed)

        # Initialize shared grid
        self._grid = GridSearchSpace(
            width=config.width,
            height=config.height,
            resolution=config.resolution,
        )

        # Spawn drones in a grid layout across the search area
        self._agents: Dict[int, SwarmAgent] = {}
        self._spawn_agents()

    def _spawn_agents(self) -> None:
        """Distribute N drones evenly across the search area in a grid pattern."""
        n = self._config.num_drones
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        cell_w = self._config.width / float(cols)
        cell_h = self._config.height / float(rows)

        for i in range(n):
            row = i // cols
            col = i % cols
            x = (col + 0.5) * cell_w
            y = (row + 0.5) * cell_h
            # Clamp to valid world bounds
            x = max(0.5, min(self._config.width - 0.5, x))
            y = max(0.5, min(self._config.height - 0.5, y))

            # Initial velocity directed outward from spawn center
            cx, cy = self._config.width / 2.0, self._config.height / 2.0
            angle = math.atan2(y - cy, x - cx)
            speed = self._config.max_drone_speed * 0.4
            vx = speed * math.cos(angle)
            vy = speed * math.sin(angle)

            self._agents[i] = SwarmAgent(
                drone_id=i,
                config=self._config,
                initial_position=(x, y),
                initial_velocity=(vx, vy),
            )

    def _is_rf_connected(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> bool:
        """
        Check physical RF link connectivity between pos1 and pos2.
        Returns True if Euclidean distance <= comm_range AND line segment pos1-pos2 does not
        intersect the circular jamming zone (or have endpoints inside it).
        """
        dist = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
        if dist > self._config.comm_range:
            return False

        if self._config.jamming_center is not None and self._config.jamming_radius > 0.0:
            jcx, jcy = self._config.jamming_center
            jr = self._config.jamming_radius

            # Endpoint checks
            d1 = math.hypot(pos1[0] - jcx, pos1[1] - jcy)
            d2 = math.hypot(pos2[0] - jcx, pos2[1] - jcy)
            if d1 <= jr or d2 <= jr:
                return False

            # Line segment intersection check
            vx = pos2[0] - pos1[0]
            vy = pos2[1] - pos1[1]
            seg_len_sq = vx * vx + vy * vy
            if seg_len_sq > 1e-12:
                t = ((jcx - pos1[0]) * vx + (jcy - pos1[1]) * vy) / seg_len_sq
                t_clamped = max(0.0, min(1.0, t))
                closest_x = pos1[0] + t_clamped * vx
                closest_y = pos1[1] + t_clamped * vy
                d_closest = math.hypot(closest_x - jcx, closest_y - jcy)
                if d_closest <= jr:
                    return False

        return True

    def step(self, t_sim: Optional[float] = None) -> TrajectoryFrame:
        """
        Advance simulation by one discrete time step dt at timestamp t_sim.
        Executes kill schedule, target events, P2P mesh routing & Data Mule forwarding,
        agent updates, and returns the resulting TrajectoryFrame.
        """
        if t_sim is None:
            t_sim = getattr(self, "_current_t", 0.0)
            self._current_t = t_sim + self._config.dt

        dt = self._config.dt

        # --- Apply kill schedule ---
        for kill_time, kill_ids in self._kill_at.items():
            if abs(t_sim - float(kill_time)) < dt * 0.5:
                for kid in kill_ids:
                    if kid in self._agents:
                        self._agents[kid].kill()

        # --- Telemetry Message Bus & Target Schedule Processing ---
        pending_packets: List[TelemetryPacket] = []

        if isinstance(self._target_schedule, dict):
            for sched_time, event_info in self._target_schedule.items():
                if abs(t_sim - float(sched_time)) < dt * 0.5:
                    action = str(event_info.get("event") or event_info.get("action") or "").upper()
                    if "FOUND" in action or "DETECT" in action:
                        pos = event_info.get("position") or event_info.get("target_pos") or (50.0, 50.0)
                        tid = event_info.get("target_id", 1)
                        detector_id = event_info.get("detecting_drone") if "detecting_drone" in event_info else event_info.get("drone_id", 0)
                        detector = self._agents.get(detector_id)
                        if detector is None or not detector.active:
                            active_agents = [a for a in self._agents.values() if a.active]
                            if active_agents:
                                detector = active_agents[0]
                        if detector is not None:
                            pkt = detector.detect_target(arg1=tid, arg2=pos)
                            self._active_target_state = pkt.target_state
                            pending_packets.append(pkt)
                    elif "CLEAR" in action:
                        tid = event_info.get("target_id", 1)
                        clearer_id = event_info.get("drone_id", 0)
                        clearer = self._agents.get(clearer_id)
                        if clearer is None or not clearer.active:
                            active_agents = [a for a in self._agents.values() if a.active]
                            if active_agents:
                                clearer = active_agents[0]
                        if clearer is not None:
                            pkt = clearer.clear_target(target_id=tid)
                            self._active_target_state = None
                            pending_packets.append(pkt)

        elif isinstance(self._target_schedule, list):
            for event_info in self._target_schedule:
                sched_time = float(event_info.get("time", event_info.get("timestamp", -1.0)))
                if abs(t_sim - sched_time) < dt * 0.5:
                    action = str(event_info.get("event") or event_info.get("action") or "").upper()
                    if "FOUND" in action or "DETECT" in action:
                        pos = event_info.get("position") or event_info.get("target_pos") or (50.0, 50.0)
                        tid = event_info.get("target_id", 1)
                        detector_id = event_info.get("detecting_drone") if "detecting_drone" in event_info else event_info.get("drone_id", 0)
                        detector = self._agents.get(detector_id)
                        if detector is None or not detector.active:
                            active_agents = [a for a in self._agents.values() if a.active]
                            if active_agents:
                                detector = active_agents[0]
                        if detector is not None:
                            pkt = detector.detect_target(arg1=tid, arg2=pos)
                            self._active_target_state = pkt.target_state
                            pending_packets.append(pkt)
                    elif "CLEAR" in action:
                        tid = event_info.get("target_id", 1)
                        clearer_id = event_info.get("drone_id", 0)
                        clearer = self._agents.get(clearer_id)
                        if clearer is None or not clearer.active:
                            active_agents = [a for a in self._agents.values() if a.active]
                            if active_agents:
                                clearer = active_agents[0]
                        if clearer is not None:
                            pkt = clearer.clear_target(target_id=tid)
                            self._active_target_state = None
                            pending_packets.append(pkt)

        # Continuous target telemetry heartbeat while target is active
        if getattr(self, "_active_target_state", None) is not None and not pending_packets:
            self._active_target_state.timestamp = t_sim
            detector_id = self._active_target_state.detected_by if self._active_target_state.detected_by >= 0 else 0
            detector = self._agents.get(detector_id)
            if detector:
                detector._last_target_packet_time = t_sim
                detector._sequence_counter += 1
                seq_id = detector._sequence_counter
            else:
                seq_id = int(t_sim * 10)
            pkt = TelemetryPacket(
                sender_id=detector_id,
                packet_type=PacketType.TARGET_FOUND,
                target_state=self._active_target_state,
                timestamp=t_sim,
                source_id=detector_id,
                destination_id=-1,
                sequence_id=seq_id,
                hop_count=0,
                ttl=10,
                relayed_by=[],
            )
            pkt.sign()  # HMAC authenticate
            if detector:
                detector._seen_packet_ids.add(pkt.packet_id)
                detector._mule_cache[pkt.packet_id] = {
                    "packet": pkt,
                    "arrival_time": t_sim,
                }
            pending_packets.append(pkt)

        # Broadcast newly generated packets and mule packets across RF connected links
        tx_candidates: List[Tuple[int, TelemetryPacket]] = []
        for pkt in pending_packets:
            tx_candidates.append((pkt.sender_id, pkt))

        for agent in self._agents.values():
            if agent.active:
                mule_pkts = agent.get_mule_packets(t_sim)
                for mpkt in mule_pkts:
                    tx_candidates.append((agent.drone_id, mpkt))

        for sender_id, pkt in tx_candidates:
            sender_agent = self._agents.get(sender_id)
            if sender_agent is None or not sender_agent.active:
                continue
            sender_pos = (sender_agent._x, sender_agent._y)

            for receiver_id, receiver_agent in self._agents.items():
                if receiver_id == sender_id or not receiver_agent.active:
                    continue
                receiver_pos = (receiver_agent._x, receiver_agent._y)

                if not self._is_rf_connected(sender_pos, receiver_pos):
                    continue

                if self._config.packet_drop_rate > 0.0 and random.random() < self._config.packet_drop_rate:
                    continue

                receiver_agent.receive_telemetry_packet(pkt)

        # --- Collect current peer states ---
        peer_states: Dict[int, DroneState] = {
            aid: agent.get_state()
            for aid, agent in self._agents.items()
        }

        # --- Update all active agents ---
        for agent in self._agents.values():
            if agent.active:
                agent.update(dt, peer_states, self._threats, self._grid)

        # --- Record trajectory frame ---
        current_states = {
            aid: agent.get_state() for aid, agent in self._agents.items()
        }
        active_count = sum(1 for s in current_states.values() if s.active)
        coverage = self._grid.get_coverage_ratio()

        mode_counts: Dict[str, int] = {"SEARCH": 0, "TARGET_UNCONFIRMED": 0, "TARGET_TRACKING": 0}
        active_target_dict: Optional[Dict[str, Any]] = None

        for state in current_states.values():
            if state.active:
                mode_name = state.mode.value if hasattr(state.mode, "value") else str(state.mode)
                mode_counts[mode_name] = mode_counts.get(mode_name, 0) + 1
                if state.mode == SwarmMode.TARGET_TRACKING and state.target_position is not None:
                    active_target_dict = {
                        "target_id": 1,
                        "position": list(state.target_position),
                        "timestamp": t_sim,
                        "status": "active",
                    }

        return TrajectoryFrame(
            timestamp=t_sim,
            drone_states=current_states,
            active_drone_count=active_count,
            coverage_ratio=coverage,
            target_state=active_target_dict,
            mode_counts=mode_counts,
        )

    def run(self) -> TrajectoryLog:
        """
        Execute the full discrete-time simulation loop.

        Returns:
            A complete TrajectoryLog with all frames recorded.
        """
        log = TrajectoryLog(
            config=self._config,
            threat_zones=self._threats,
            frames=[],
            metadata={
                "kill_schedule": {
                    str(t): ids for t, ids in self._kill_at.items()
                },
                "target_schedule": _clean_val(self._target_schedule),
            },
        )

        dt = self._config.dt
        total_steps = self._config.total_steps

        self._active_target_state: Optional[TargetState] = None
        self._current_t = 0.0

        for step in range(total_steps):
            t_sim = step * dt
            frame = self.step(t_sim)
            log.frames.append(frame)

        return log

    def run_and_save(self, output_path: str) -> TrajectoryLog:
        """
        Run the simulation and save the trajectory log to a JSON file.

        Args:
            output_path: File path to write the JSON log.

        Returns:
            The completed TrajectoryLog.
        """
        log = self.run()
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        log.save_json(output_path)
        return log

