"""
SwarmAgent — Decentralized drone agent with P2P heartbeat, dynamic Voronoi
sector assignment, Rotational APF threat evasion, Boids separation, and target encirclement.
"""

from __future__ import annotations

import hashlib
import hmac
import math
from typing import Any, Dict, List, Optional, Tuple, Union

from swarm_recon.config import (
    DroneState,
    ThreatZone,
    SimulationConfig,
    SwarmMode,
    PacketType,
    TargetState,
    TelemetryPacket,
)
from swarm_recon.core.grid import GridSearchSpace
from swarm_recon.evasion.forces import EvaderForces


class SwarmAgent:
    """
    A single decentralized drone agent in the swarm.

    Each agent independently:
    - Broadcasts a P2P heartbeat every tick.
    - Detects when peer drones have gone silent (heartbeat timeout).
    - Maintains a spatial Voronoi sector in SEARCH mode.
    - Re-partitions every REPARTITION_INTERVAL steps AND whenever the active set changes.
    - Navigates toward the nearest unvisited cell within its geographic sector in SEARCH mode.
    - Transitions to TARGET_TRACKING mode upon target detection / telemetry broadcast.
    - Executes target encirclement (radial standoff + orbital drive + peer spacing) in TARGET_TRACKING mode.
    - Automatically reverts to SEARCH mode on target clear or target packet loss timeout (> 5.0s).
    - Evades threats using Rotational Artificial Potential Fields in both modes.
    - Maintains separation from peers via Boids rules in both modes.
    - Adds stochastic heading perturbations for unpredictable trajectories.
    """

    # Force weighting constants
    _W_TARGET = 6.0        # Sector-attraction weight (strong coverage drive)
    _W_APF = 1.0           # APF threat-repulsion weight
    _W_BOIDS = 1.0         # Boids separation weight
    _APF_INFLUENCE = 15.0  # APF influence margin beyond threat radius (m)
    _APF_MAX_FORCE = 35.0  # Per-threat APF force cap
    _BOIDS_DIST = 4.0      # Boids separation distance (m)
    _BOIDS_STR = 1.5       # Boids separation strength
    _MOMENTUM = 0.65       # Velocity momentum (lower = faster force response)
    _THREAT_MARGIN = 1.5   # Hard clearance margin beyond threat radius (m)
    _COLLISION_MARGIN = 1.5  # Hard inter-drone collision avoidance radius (m)
    _REPARTITION_INTERVAL = 20  # Steps between periodic spatial repartitions (every 2s)

    def __init__(
        self,
        drone_id: int,
        config: SimulationConfig,
        initial_position: Tuple[float, float],
        initial_velocity: Optional[Tuple[float, float]] = None,
    ) -> None:
        self._id = drone_id
        self._config = config
        self._x, self._y = float(initial_position[0]), float(initial_position[1])
        if initial_velocity is not None:
            self._vx, self._vy = float(initial_velocity[0]), float(initial_velocity[1])
        else:
            angle = (2.0 * math.pi * drone_id) / max(1, config.num_drones)
            self._vx = 2.0 * math.cos(angle)
            self._vy = 2.0 * math.sin(angle)

        self._active: bool = True
        self._last_heartbeat: float = 0.0
        # _spatial_sector: ALL cells in drone's Voronoi region (geographic, not filtered)
        self._spatial_sector: List[Tuple[int, int]] = []
        self._known_active: set = set()
        self._sector_id: Optional[int] = drone_id
        self._time: float = 0.0
        self._step_count: int = 0

        # Multi-mode target tracking state
        self._mode: SwarmMode = SwarmMode.SEARCH
        self._target_state: Optional[TargetState] = None
        self._target_position: Optional[Tuple[float, float]] = None
        self._last_target_packet_time: float = 0.0

        # Mesh Routing & Data Mule state
        self._seen_packet_ids: set[str] = set()
        self._mule_cache: dict[str, dict] = {}
        self._sequence_counter: int = 0

        # Consensus state for target confirmation
        self._corroboration_count: int = 0
        self._corroboration_sources: set[int] = set()
        self._unconfirmed_target_time: float = 0.0
        self._unconfirmed_target_state: Optional[TargetState] = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def drone_id(self) -> int:
        return self._id

    @property
    def active(self) -> bool:
        return self._active

    @property
    def mode(self) -> SwarmMode:
        return self._mode

    @property
    def target_position(self) -> Optional[Tuple[float, float]]:
        return self._target_position

    def kill(self) -> None:
        """Deactivate this drone (simulates loss/crash)."""
        self._active = False

    def get_state(self) -> DroneState:
        """Return a snapshot DroneState for broadcasting to peers."""
        return DroneState(
            id=self._id,
            position=(self._x, self._y),
            velocity=(self._vx, self._vy),
            heading=math.atan2(self._vy, self._vx),
            active=self._active,
            sector_id=self._sector_id,
            last_heartbeat=self._last_heartbeat,
            mode=self._mode,
            target_position=self._target_position,
        )

    def receive_telemetry_packet(self, packet: TelemetryPacket) -> bool:
        """Receive and process a P2P telemetry packet from another drone."""
        if not self._active:
            return False

        # HMAC authentication check
        if self._config.require_hmac and not packet.verify_hmac():
            return False  # Reject spoofed / unauthenticated packet

        pkt_id = packet.packet_id
        if pkt_id in self._seen_packet_ids:
            return False

        self._seen_packet_ids.add(pkt_id)

        if packet.destination_id in (-1, self._id):
            if packet.packet_type == PacketType.TARGET_FOUND and packet.target_state is not None:
                # Confidence-weighted consensus: go UNCONFIRMED first
                if self._mode == SwarmMode.SEARCH:
                    self._mode = SwarmMode.TARGET_UNCONFIRMED
                    self._unconfirmed_target_state = packet.target_state
                    self._unconfirmed_target_time = packet.timestamp
                    self._corroboration_count = 1  # The detector counts as 1
                    self._corroboration_sources = {packet.source_id}
                    self._target_position = packet.target_state.position
                    self._last_target_packet_time = packet.timestamp
                elif self._mode == SwarmMode.TARGET_UNCONFIRMED:
                    # Additional TARGET_FOUND from same target reinforces
                    if packet.source_id not in self._corroboration_sources:
                        self._corroboration_count += 1
                        self._corroboration_sources.add(packet.source_id)
                    self._last_target_packet_time = packet.timestamp
                elif self._mode == SwarmMode.TARGET_TRACKING:
                    # Already tracking — update target state
                    self._target_state = packet.target_state
                    self._target_position = packet.target_state.position
                    self._last_target_packet_time = packet.timestamp

            elif packet.packet_type == PacketType.TARGET_CORROBORATE:
                # Another drone independently confirms the target
                if self._mode == SwarmMode.TARGET_UNCONFIRMED:
                    if packet.source_id not in self._corroboration_sources:
                        self._corroboration_count += 1
                        self._corroboration_sources.add(packet.source_id)
                    self._last_target_packet_time = packet.timestamp

            elif packet.packet_type == PacketType.TARGET_CLEARED:
                if self._mode in (SwarmMode.TARGET_TRACKING, SwarmMode.TARGET_UNCONFIRMED):
                    self._mode = SwarmMode.SEARCH
                    self._target_state = None
                    self._target_position = None
                    self._last_target_packet_time = 0.0
                    self._known_active = set()
                    self._corroboration_count = 0
                    self._corroboration_sources = set()
                    self._unconfirmed_target_state = None

        if packet.hop_count < packet.ttl:
            relay_packet = TelemetryPacket(
                sender_id=self._id,
                packet_type=packet.packet_type,
                target_state=packet.target_state,
                timestamp=packet.timestamp,
                source_id=packet.source_id,
                destination_id=packet.destination_id,
                sequence_id=packet.sequence_id,
                hop_count=packet.hop_count + 1,
                ttl=packet.ttl,
                relayed_by=list(packet.relayed_by) + [self._id],
                hmac_digest=packet.hmac_digest,  # Preserve original HMAC
            )
            self._mule_cache[pkt_id] = {
                "packet": relay_packet,
                "arrival_time": self._time,
            }

        return True

    def get_mule_packets(self, current_time: float) -> List[TelemetryPacket]:
        """Return active non-expired cached packets and prune expired ones from _mule_cache."""
        if not self._active:
            return []

        active_packets: List[TelemetryPacket] = []
        expired_ids: List[str] = []

        for pkt_id, item in self._mule_cache.items():
            arrival_time = item["arrival_time"]
            pkt = item["packet"]
            if (current_time - arrival_time) <= self._config.mule_cache_ttl and pkt.hop_count < pkt.ttl:
                active_packets.append(pkt)
            else:
                expired_ids.append(pkt_id)

        for pkt_id in expired_ids:
            del self._mule_cache[pkt_id]

        return active_packets

    def detect_target(
        self,
        arg1: Any = None,
        arg2: Any = None,
        target_pos: Optional[Tuple[float, float]] = None,
        target_id: Union[int, str] = 1,
    ) -> TelemetryPacket:
        """
        Trigger target detection by this drone agent.
        Supports both positional and keyword arguments for target_pos and target_id.
        Returns the generated TARGET_FOUND TelemetryPacket.
        """
        pos = target_pos
        tid = target_id

        if pos is None:
            if isinstance(arg1, (tuple, list)) and len(arg1) == 2:
                pos = (float(arg1[0]), float(arg1[1]))
                if arg2 is not None:
                    tid = arg2
            elif isinstance(arg2, (tuple, list)) and len(arg2) == 2:
                pos = (float(arg2[0]), float(arg2[1]))
                if arg1 is not None:
                    tid = arg1

        if pos is None or not isinstance(pos, (tuple, list)) or len(pos) != 2:
            raise ValueError(f"Invalid target detection arguments: arg1={arg1!r}, arg2={arg2!r}, target_pos={target_pos!r}")

        t_pos = (float(pos[0]), float(pos[1]))

        self._mode = SwarmMode.TARGET_TRACKING
        self._target_state = TargetState(
            target_id=tid,
            position=t_pos,
            timestamp=self._time,
            status="active",
            detected_by=self._id,
        )
        self._target_position = t_pos
        self._last_target_packet_time = self._time

        self._sequence_counter += 1
        pkt = TelemetryPacket(
            sender_id=self._id,
            packet_type=PacketType.TARGET_FOUND,
            target_state=self._target_state,
            timestamp=self._time,
            source_id=self._id,
            destination_id=-1,
            sequence_id=self._sequence_counter,
            hop_count=0,
            ttl=10,
            relayed_by=[],
        )
        pkt.sign()  # HMAC authenticate
        self._seen_packet_ids.add(pkt.packet_id)
        if pkt.hop_count < pkt.ttl:
            self._mule_cache[pkt.packet_id] = {
                "packet": pkt,
                "arrival_time": self._time,
            }
        return pkt

    def clear_target(self, target_id: Union[int, str] = 1) -> TelemetryPacket:
        """
        Trigger target clearance by this drone agent.
        Returns the generated TARGET_CLEARED TelemetryPacket.
        """
        self._mode = SwarmMode.SEARCH
        self._target_state = None
        self._target_position = None
        self._last_target_packet_time = 0.0
        self._known_active = set()

        self._sequence_counter += 1
        pkt = TelemetryPacket(
            sender_id=self._id,
            packet_type=PacketType.TARGET_CLEARED,
            target_state=None,
            timestamp=self._time,
            source_id=self._id,
            destination_id=-1,
            sequence_id=self._sequence_counter,
            hop_count=0,
            ttl=10,
            relayed_by=[],
        )
        pkt.sign()  # HMAC authenticate
        self._seen_packet_ids.add(pkt.packet_id)
        if pkt.hop_count < pkt.ttl:
            self._mule_cache[pkt.packet_id] = {
                "packet": pkt,
                "arrival_time": self._time,
            }
        return pkt

    def update(
        self,
        dt: float,
        peers: Dict[int, DroneState],
        threats: List[ThreatZone],
        grid: GridSearchSpace,
    ) -> None:
        """
        Advance agent one simulation step.

        Steps:
        1. Broadcast own heartbeat.
        2. Detect dead peers via heartbeat timeout.
        3. Check target loss timeout and auto-revert to SEARCH mode if silent > target_loss_timeout.
        4. Re-partition spatial Voronoi if active set changed OR on periodic interval.
        5. Compute force vector (target encirclement if TARGET_TRACKING, sector attraction if SEARCH).
        6. Retain Rotational APF threat evasion and Boids peer separation in both modes.
        7. Integrate velocity and position.
        8. Enforce hard threat boundary and world boundary.
        9. Mark visited cells on grid.
        """
        if not self._active:
            return

        self._time += dt
        self._step_count += 1
        self._last_heartbeat = self._time

        # --- Target loss timeout auto-reversion check ---
        if self._mode == SwarmMode.TARGET_TRACKING:
            if (self._time - self._last_target_packet_time) > self._config.target_loss_timeout:
                self._mode = SwarmMode.SEARCH
                self._target_state = None
                self._target_position = None
                self._known_active = set()

        # --- Consensus promotion / rejection ---
        if self._mode == SwarmMode.TARGET_UNCONFIRMED:
            if self._corroboration_count >= self._config.consensus_required:
                # Enough corroborations — promote to full TARGET_TRACKING
                self._mode = SwarmMode.TARGET_TRACKING
                self._target_state = self._unconfirmed_target_state
            elif (self._time - self._unconfirmed_target_time) > self._config.consensus_timeout:
                # Timeout without enough corroboration — reject as false target
                self._mode = SwarmMode.SEARCH
                self._target_state = None
                self._target_position = None
                self._unconfirmed_target_state = None
                self._corroboration_count = 0
                self._corroboration_sources = set()

        # --- P2P liveness detection ---
        current_active: set = {self._id}
        for pid, pstate in peers.items():
            if pid == self._id:
                continue
            if pstate.active and not pstate.is_stale(self._time, self._config.heartbeat_timeout):
                current_active.add(pid)

        # Re-partition when active set changes OR on periodic interval
        needs_repartition = (
            current_active != self._known_active
            or self._step_count % self._REPARTITION_INTERVAL == 0
        )
        if needs_repartition:
            self._known_active = current_active
            self._do_repartition(peers, grid)

        # --- Compute attraction / tracking force ---
        if self._mode == SwarmMode.TARGET_TRACKING and self._target_position is not None:
            peer_list = list(peers.values())
            fx_target, fy_target = EvaderForces.target_encirclement(
                self._x,
                self._y,
                self._target_position,
                peer_list,
                standoff_radius=self._config.standoff_radius_nominal,
            )
        elif self._mode == SwarmMode.TARGET_UNCONFIRMED and self._target_position is not None:
            # Move toward unconfirmed target for visual corroboration
            fx_target, fy_target = self._sector_attraction(self._target_position[0], self._target_position[1])
        else:
            target = self._get_sector_target(grid)
            fx_target, fy_target = self._sector_attraction(target[0], target[1])

        # --- Compute threat evasion and Boids separation forces ---
        fx_apf, fy_apf = self._rotational_apf(threats)
        fx_boids, fy_boids = self._boids_separation(peers)

        total_fx = fx_target + fx_apf + fx_boids
        total_fy = fy_target + fy_apf + fy_boids

        # --- Velocity integration with momentum ---
        new_vx = self._MOMENTUM * self._vx + (1.0 - self._MOMENTUM) * (self._vx + total_fx * dt)
        new_vy = self._MOMENTUM * self._vy + (1.0 - self._MOMENTUM) * (self._vy + total_fy * dt)

        # --- Stochastic heading perturbation ---
        new_vx, new_vy = self._stochastic_perturb(new_vx, new_vy)

        # --- Speed clamping ---
        speed = math.hypot(new_vx, new_vy)
        if speed > self._config.max_drone_speed:
            scale = self._config.max_drone_speed / speed
            new_vx *= scale
            new_vy *= scale
        elif speed < 0.5 and speed > 1e-6:
            # Ensure minimum forward motion to prevent stalling
            scale = 0.5 / speed
            new_vx *= scale
            new_vy *= scale

        self._vx, self._vy = new_vx, new_vy

        # --- Position integration ---
        nx = self._x + self._vx * dt
        ny = self._y + self._vy * dt

        # --- Hard threat boundary enforcement ---
        for threat in threats:
            tcx, tcy = threat.center
            d = math.hypot(nx - tcx, ny - tcy)
            safe = threat.radius + self._THREAT_MARGIN
            if d < safe and d > 1e-6:
                nx = tcx + (nx - tcx) / d * safe
                ny = tcy + (ny - tcy) / d * safe

        # --- Hard inter-drone collision avoidance ---
        collision_r = self._config.collision_radius
        for pid, pstate in peers.items():
            if pid == self._id or not pstate.active:
                continue
            px, py = pstate.position
            d = math.hypot(nx - px, ny - py)
            if d < collision_r and d > 1e-6:
                # Push this drone outward from the peer
                nx = px + (nx - px) / d * collision_r
                ny = py + (ny - py) / d * collision_r

        # --- World boundary clamping ---
        nx = max(0.5, min(self._config.width - 0.5, nx))
        ny = max(0.5, min(self._config.height - 0.5, ny))

        self._x, self._y = nx, ny

        # --- Mark grid visited ---
        grid.mark_visited(self._x, self._y, self._config.sensor_radius)

    def get_force_vector(
        self,
        peers: Dict[int, DroneState],
        threats: List[ThreatZone],
    ) -> Tuple[float, float]:
        """Expose blended force vector (used for testing and analysis)."""
        fx_apf, fy_apf = self._rotational_apf(threats)
        fx_boids, fy_boids = self._boids_separation(peers)
        return fx_apf + fx_boids, fy_apf + fy_boids

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _do_repartition(
        self,
        peers: Dict[int, DroneState],
        grid: GridSearchSpace,
    ) -> None:
        """Compute spatial Voronoi partition (geographic, all cells, not filtered to unvisited)."""
        active_positions: Dict[int, Tuple[float, float]] = {self._id: (self._x, self._y)}
        for pid, pstate in peers.items():
            if pid != self._id and pid in self._known_active:
                active_positions[pid] = pstate.position
        # unvisited_only=False: stable geographic sectors that don't shrink as cells are visited
        partition = grid.repartition(active_positions, unvisited_only=False)
        self._spatial_sector = partition.get(self._id, [])

    def _get_sector_target(self, grid: GridSearchSpace) -> Tuple[float, float]:
        """Return world-space coordinates of the nearest unvisited cell in geographic sector."""
        best_dist_sq = float("inf")
        best_pos: Optional[Tuple[float, float]] = None

        for r, c in self._spatial_sector:
            if grid.is_cell_visited(r, c):
                continue
            wx, wy = grid.cell_to_world(r, c)
            d_sq = (wx - self._x) ** 2 + (wy - self._y) ** 2
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_pos = (wx, wy)

        if best_pos is not None:
            return best_pos

        # Fallback: any unvisited cell globally (when entire sector is fully covered)
        all_unvisited = grid.get_unvisited_cells()
        if all_unvisited:
            best = min(
                all_unvisited,
                key=lambda rc: (grid.cell_to_world(rc[0], rc[1])[0] - self._x) ** 2
                             + (grid.cell_to_world(rc[0], rc[1])[1] - self._y) ** 2,
            )
            return grid.cell_to_world(best[0], best[1])

        # All visited — return center
        return self._config.width / 2.0, self._config.height / 2.0

    def _sector_attraction(self, tx: float, ty: float) -> Tuple[float, float]:
        """Normalized attraction force toward target point."""
        dx, dy = tx - self._x, ty - self._y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return 0.0, 0.0
        return (dx / dist) * self._W_TARGET, (dy / dist) * self._W_TARGET

    def _rotational_apf(self, threats: List[ThreatZone]) -> Tuple[float, float]:
        """
        Rotational Artificial Potential Field repulsion.
        Combines radial repulsion + tangential (orbital) rotation.
        """
        fx, fy = 0.0, 0.0
        for threat in threats:
            tcx, tcy = threat.center
            d_center = math.hypot(self._x - tcx, self._y - tcy)
            clearance = d_center - threat.radius
            if clearance > self._APF_INFLUENCE or d_center < 1e-6:
                continue

            # Radial (repulsion) unit vector
            rx = (self._x - tcx) / d_center
            ry = (self._y - tcy) / d_center
            # Tangential (orbital) unit vector
            tx_ = -ry
            ty_ = rx

            delta = max(0.1, clearance)
            mag = min(self._APF_MAX_FORCE, threat.severity * 10.0 / (delta ** 1.1))

            fx += mag * (rx + 1.5 * tx_)
            fy += mag * (ry + 1.5 * ty_)

        return fx * self._W_APF, fy * self._W_APF

    def _boids_separation(self, peers: Dict[int, DroneState]) -> Tuple[float, float]:
        """Boids separation force: push away from too-close active peers."""
        fx, fy = 0.0, 0.0
        for pid, pstate in peers.items():
            if pid == self._id or not pstate.active:
                continue
            dx = self._x - pstate.position[0]
            dy = self._y - pstate.position[1]
            dist = math.hypot(dx, dy)
            if 1e-3 < dist < self._BOIDS_DIST:
                mag = self._BOIDS_STR * (self._BOIDS_DIST - dist) / dist
                fx += dx * mag
                fy += dy * mag
        return fx * self._W_BOIDS, fy * self._W_BOIDS

    def _stochastic_perturb(self, vx: float, vy: float) -> Tuple[float, float]:
        """
        Apply smooth deterministic heading perturbation for trajectory entropy.
        Magnitude 0.15/0.09 balances R2 entropy (>= 1.5 bits) with R1 coverage
        efficiency — small enough to stay near coverage paths, large enough to
        generate diverse heading distributions.
        """
        angle = (
            0.15 * math.sin(0.4 * self._time + self._id * 1.7)
            + 0.09 * math.cos(0.7 * self._time + self._id * 2.3)
        )
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        return vx * cos_a - vy * sin_a, vx * sin_a + vy * cos_a
