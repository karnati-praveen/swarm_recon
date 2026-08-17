"""
Data schemas, configuration classes, and trajectory logging models for swarm_recon.

This module defines all core dataclasses used across simulation runtime, consensus,
evasion dynamics, and verification logging.
"""

from dataclasses import dataclass, field
from enum import Enum
import hashlib
import hmac
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

# Shared symmetric key for HMAC authentication (simulated)
_SWARM_SHARED_KEY = b"swarm_recon_defense_key_2026"


def _clean_val(val: Any) -> Any:
    """Recursively convert numpy scalars, tuples, or custom types to standard Python types."""
    if hasattr(val, "item") and callable(getattr(val, "item")):
        return val.item()
    if isinstance(val, (tuple, list)):
        return [_clean_val(v) for v in val]
    if isinstance(val, dict):
        return {str(k): _clean_val(v) for k, v in val.items()}
    return val


def is_valid_number(
    val: Any,
    min_val: Optional[float] = None,
    max_val: Optional[float] = None,
    allow_none: bool = False,
    min_inclusive: bool = True,
    max_inclusive: bool = True,
) -> bool:
    """
    Validate whether a value is a valid finite real number (int or float, excluding bool, NaN, and Inf).

    Args:
        val: The value to inspect.
        min_val: Optional lower numerical bound.
        max_val: Optional upper numerical bound.
        allow_none: If True, None is considered valid.
        min_inclusive: If True, val >= min_val is required; if False, val > min_val.
        max_inclusive: If True, val <= max_val is required; if False, val < max_val.

    Returns:
        True if val passes all type, finite number, and range checks; False otherwise.
    """
    if val is None:
        return allow_none
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        return False
    if math.isnan(val) or math.isinf(val):
        return False
    if min_val is not None:
        if min_inclusive:
            if val < min_val:
                return False
        else:
            if val <= min_val:
                return False
    if max_val is not None:
        if max_inclusive:
            if val > max_val:
                return False
        else:
            if val >= max_val:
                return False
    return True


def validate_number(
    val: Any,
    field_name: str,
    min_val: Optional[float] = None,
    max_val: Optional[float] = None,
    allow_none: bool = False,
    min_inclusive: bool = True,
    max_inclusive: bool = True,
) -> None:
    """
    Assert that a field value is a valid finite real number.
    Raises ValueError with a detailed message if validation fails.
    """
    if not is_valid_number(
        val,
        min_val=min_val,
        max_val=max_val,
        allow_none=allow_none,
        min_inclusive=min_inclusive,
        max_inclusive=max_inclusive,
    ):
        raise ValueError(
            f"Invalid numeric value for {field_name}: {val!r}. "
            f"Must be a finite int or float (excluding bool, NaN, Inf)."
        )


class SwarmMode(str, Enum):
    """Swarm operating modes for area search and target tracking."""
    SEARCH = "SEARCH"
    TARGET_UNCONFIRMED = "TARGET_UNCONFIRMED"
    TARGET_TRACKING = "TARGET_TRACKING"


class PacketType(str, Enum):
    """Telemetry packet type identifiers."""
    HEARTBEAT = "HEARTBEAT"
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_CORROBORATE = "TARGET_CORROBORATE"
    TARGET_CLEARED = "TARGET_CLEARED"


@dataclass
class TargetState:
    """
    Represents the spatial and status information of a tracked target.

    Attributes:
        target_id: Unique target identifier (int or str).
        position: 2D coordinates (x, y) of target in meters.
        timestamp: Simulation timestamp when detected or updated.
        status: Target state string ('active', 'cleared', etc.).
        detected_by: Drone ID that detected the target.
    """

    target_id: Union[int, str]
    position: Tuple[float, float]
    timestamp: float = 0.0
    status: str = "active"
    detected_by: int = -1

    def __post_init__(self) -> None:
        if isinstance(self.target_id, bool) or not isinstance(self.target_id, (int, str)):
            raise ValueError(f"target_id must be int or str (not bool), got {self.target_id!r}")
        if not isinstance(self.position, (tuple, list)) or len(self.position) != 2:
            raise ValueError(f"position must be a 2-tuple or 2-list, got {self.position!r}")
        validate_number(self.position[0], "position[0]")
        validate_number(self.position[1], "position[1]")
        validate_number(self.timestamp, "timestamp", min_val=0.0)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize TargetState to dictionary."""
        return {
            "target_id": self.target_id,
            "position": [float(self.position[0]), float(self.position[1])],
            "timestamp": float(self.timestamp),
            "status": str(self.status),
            "detected_by": int(self.detected_by),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TargetState":
        """Instantiate TargetState from dictionary."""
        pos = data["position"]
        return cls(
            target_id=data["target_id"],
            position=(float(pos[0]), float(pos[1])),
            timestamp=float(data.get("timestamp", 0.0)),
            status=str(data.get("status", "active")),
            detected_by=int(data.get("detected_by", -1)),
        )


_GLOBAL_SEQ_COUNTER: int = 0


@dataclass
class TelemetryPacket:
    """
    P2P broadcast packet payload between swarm agents.

    Attributes:
        sender_id: Drone ID sending the packet.
        packet_type: Type of packet (HEARTBEAT, TARGET_FOUND, TARGET_CLEARED).
        target_state: Optional target payload data.
        timestamp: Simulation timestamp when sent.
        source_id: Drone ID originating the packet.
        destination_id: Final destination drone ID or broadcast (-1).
        sequence_id: Sequence number for origin drone packet deduplication.
        hop_count: Current relay hop count.
        ttl: Maximum time to live (hop limit).
        relayed_by: Ordered list of drone IDs that relayed this packet.
    """

    sender_id: int
    packet_type: PacketType
    target_state: Optional[TargetState] = None
    timestamp: float = 0.0
    source_id: int = 0
    destination_id: Union[int, str] = -1
    sequence_id: int = 0
    hop_count: int = 0
    ttl: int = 10
    relayed_by: List[int] = field(default_factory=list)
    hmac_digest: str = ""

    def __post_init__(self) -> None:
        global _GLOBAL_SEQ_COUNTER
        if self.sequence_id == 0:
            _GLOBAL_SEQ_COUNTER += 1
            self.sequence_id = _GLOBAL_SEQ_COUNTER

        if isinstance(self.sender_id, bool) or not isinstance(self.sender_id, int) or self.sender_id < 0:
            raise ValueError(f"sender_id must be non-negative int, got {self.sender_id!r}")
        if isinstance(self.packet_type, str):
            self.packet_type = PacketType(self.packet_type)
        if not isinstance(self.packet_type, PacketType):
            raise ValueError(f"packet_type must be PacketType enum, got {self.packet_type!r}")
        validate_number(self.timestamp, "timestamp", min_val=0.0)
        if isinstance(self.source_id, bool) or not isinstance(self.source_id, int):
            raise ValueError(f"source_id must be int, got {self.source_id!r}")
        if isinstance(self.destination_id, bool) or not isinstance(self.destination_id, (int, str)):
            raise ValueError(f"destination_id must be int or str, got {self.destination_id!r}")
        if isinstance(self.sequence_id, bool) or not isinstance(self.sequence_id, int) or self.sequence_id < 0:
            raise ValueError(f"sequence_id must be non-negative int, got {self.sequence_id!r}")
        if isinstance(self.hop_count, bool) or not isinstance(self.hop_count, int) or self.hop_count < 0:
            raise ValueError(f"hop_count must be non-negative int, got {self.hop_count!r}")
        if isinstance(self.ttl, bool) or not isinstance(self.ttl, int) or self.ttl < 0:
            raise ValueError(f"ttl must be non-negative int, got {self.ttl!r}")
        if not isinstance(self.relayed_by, list):
            raise ValueError(f"relayed_by must be list, got {self.relayed_by!r}")

    @property
    def packet_id(self) -> str:
        """Unique global packet identifier for deduplication."""
        return f"{self.source_id}_{self.sequence_id}"

    def compute_hmac(self, key: bytes = _SWARM_SHARED_KEY) -> str:
        """Compute HMAC-SHA256 over critical packet fields for authentication."""
        msg = f"{self.source_id}:{self.sender_id}:{self.sequence_id}:{self.packet_type.value}:{self.timestamp}"
        return hmac.new(key, msg.encode(), hashlib.sha256).hexdigest()[:16]

    def sign(self, key: bytes = _SWARM_SHARED_KEY) -> None:
        """Sign this packet by computing and setting the HMAC digest."""
        self.hmac_digest = self.compute_hmac(key)

    def verify_hmac(self, key: bytes = _SWARM_SHARED_KEY) -> bool:
        """Verify packet HMAC digest. Returns True if authentic, False if spoofed."""
        if not self.hmac_digest:
            return False
        return hmac.compare_digest(self.hmac_digest, self.compute_hmac(key))

    def to_dict(self) -> Dict[str, Any]:
        """Serialize TelemetryPacket to dictionary."""
        return {
            "sender_id": int(self.sender_id),
            "packet_type": str(self.packet_type.value),
            "target_state": self.target_state.to_dict() if self.target_state else None,
            "timestamp": float(self.timestamp),
            "source_id": int(self.source_id),
            "destination_id": self.destination_id,
            "sequence_id": int(self.sequence_id),
            "hop_count": int(self.hop_count),
            "ttl": int(self.ttl),
            "relayed_by": [int(x) for x in self.relayed_by],
            "hmac_digest": self.hmac_digest,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TelemetryPacket":
        """Instantiate TelemetryPacket from dictionary."""
        tgt_data = data.get("target_state")
        return cls(
            sender_id=int(data["sender_id"]),
            packet_type=PacketType(data["packet_type"]),
            target_state=TargetState.from_dict(tgt_data) if tgt_data else None,
            timestamp=float(data.get("timestamp", 0.0)),
            source_id=int(data.get("source_id", 0)),
            destination_id=data.get("destination_id", -1),
            sequence_id=int(data.get("sequence_id", 0)),
            hop_count=int(data.get("hop_count", 0)),
            ttl=int(data.get("ttl", 10)),
            relayed_by=list(data.get("relayed_by", [])),
        )


@dataclass
class DroneState:
    """
    Represents the spatial, kinematic, and assignment status of a single drone agent.

    Attributes:
        id: Unique drone identifier.
        position: 2D coordinates (x, y) in meters.
        velocity: 2D velocity vector (vx, vy) in m/s.
        heading: Orientation angle in radians.
        active: Liveness flag (True = active, False = lost/killed).
        sector_id: Identifier of the sector assigned to this drone.
        last_heartbeat: Simulation timestamp of last received heartbeat.
        mode: Operating mode (SwarmMode.SEARCH or SwarmMode.TARGET_TRACKING).
        target_position: Optional target position coordinates (x, y) being tracked.
    """

    id: int
    position: Tuple[float, float] = (0.0, 0.0)
    velocity: Tuple[float, float] = (0.0, 0.0)
    heading: float = 0.0
    active: bool = True
    sector_id: Optional[Union[int, str]] = None
    last_heartbeat: float = 0.0
    mode: SwarmMode = SwarmMode.SEARCH
    target_position: Optional[Tuple[float, float]] = None

    def __post_init__(self) -> None:
        if isinstance(self.id, bool) or not isinstance(self.id, int) or self.id < 0:
            raise ValueError(f"id must be non-negative integer (not bool), got {self.id!r}")

        if not isinstance(self.position, (tuple, list)) or len(self.position) != 2:
            raise ValueError(f"position must be a 2-tuple or 2-list, got {self.position!r}")
        validate_number(self.position[0], "position[0]")
        validate_number(self.position[1], "position[1]")

        if not isinstance(self.velocity, (tuple, list)) or len(self.velocity) != 2:
            raise ValueError(f"velocity must be a 2-tuple or 2-list, got {self.velocity!r}")
        validate_number(self.velocity[0], "velocity[0]")
        validate_number(self.velocity[1], "velocity[1]")

        validate_number(self.heading, "heading")

        if not isinstance(self.active, bool):
            raise ValueError(f"active must be boolean, got {self.active!r}")

        if self.sector_id is not None:
            if isinstance(self.sector_id, bool):
                raise ValueError(f"sector_id cannot be bool, got {self.sector_id!r}")
            if not isinstance(self.sector_id, (int, str)):
                raise ValueError(f"sector_id must be int or str, got {self.sector_id!r}")
            if isinstance(self.sector_id, int) and self.sector_id < 0:
                raise ValueError(f"sector_id int must be non-negative, got {self.sector_id!r}")

        validate_number(self.last_heartbeat, "last_heartbeat", min_val=0.0)

        if isinstance(self.mode, str):
            self.mode = SwarmMode(self.mode)
        if not isinstance(self.mode, SwarmMode):
            raise ValueError(f"mode must be SwarmMode enum, got {self.mode!r}")

        if self.target_position is not None:
            if not isinstance(self.target_position, (tuple, list)) or len(self.target_position) != 2:
                raise ValueError(f"target_position must be a 2-tuple or 2-list, got {self.target_position!r}")
            validate_number(self.target_position[0], "target_position[0]")
            validate_number(self.target_position[1], "target_position[1]")

    @property
    def speed(self) -> float:
        """Calculate scalar kinematic speed magnitude in m/s."""
        return math.hypot(self.velocity[0], self.velocity[1])

    def distance_to(self, other: Any) -> float:
        """Compute Euclidean distance to another position or DroneState."""
        if hasattr(other, "position"):
            other_pos = getattr(other, "position")
        elif isinstance(other, (tuple, list)) and len(other) >= 2:
            other_pos = (float(other[0]), float(other[1]))
        else:
            raise ValueError(f"Invalid target for distance calculation: {other}")
        return math.hypot(
            self.position[0] - other_pos[0],
            self.position[1] - other_pos[1],
        )

    def is_stale(self, current_time: float, timeout: float = 3.0) -> bool:
        """Check if heartbeat is older than timeout threshold."""
        return (current_time - self.last_heartbeat) > timeout

    def to_dict(self) -> Dict[str, Any]:
        """Serialize DroneState to a JSON-compatible dictionary."""
        return {
            "id": int(self.id),
            "position": [float(self.position[0]), float(self.position[1])],
            "velocity": [float(self.velocity[0]), float(self.velocity[1])],
            "heading": float(self.heading),
            "active": bool(self.active),
            "sector_id": self.sector_id,
            "last_heartbeat": float(self.last_heartbeat),
            "mode": str(self.mode.value),
            "target_position": [float(self.target_position[0]), float(self.target_position[1])] if self.target_position else None,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DroneState":
        """Instantiate DroneState from a dictionary."""
        pos = data.get("position", (0.0, 0.0))
        vel = data.get("velocity", (0.0, 0.0))
        mode_raw = data.get("mode", SwarmMode.SEARCH)
        mode = SwarmMode(mode_raw) if isinstance(mode_raw, str) else mode_raw
        tgt_pos_raw = data.get("target_position")
        target_position = (float(tgt_pos_raw[0]), float(tgt_pos_raw[1])) if tgt_pos_raw else None
        return cls(
            id=int(data["id"]),
            position=(float(pos[0]), float(pos[1])),
            velocity=(float(vel[0]), float(vel[1])),
            heading=float(data.get("heading", 0.0)),
            active=bool(data.get("active", True)),
            sector_id=data.get("sector_id"),
            last_heartbeat=float(data.get("last_heartbeat", 0.0)),
            mode=mode,
            target_position=target_position,
        )


@dataclass
class ThreatZone:
    """
    Represents a circular spatial threat area that repels drones.

    Attributes:
        id: Unique threat zone identifier.
        center: 2D center coordinates (cx, cy).
        radius: Radius of hazard field in spatial units.
        severity: Force magnitude multiplier for artificial potential field.
    """

    id: int
    center: Tuple[float, float]
    radius: float
    severity: float = 1.0

    def __post_init__(self) -> None:
        if isinstance(self.id, bool) or not isinstance(self.id, int) or self.id < 0:
            raise ValueError(f"id must be non-negative integer (not bool), got {self.id!r}")
        if not isinstance(self.center, (tuple, list)) or len(self.center) != 2:
            raise ValueError(f"center must be a 2-tuple or 2-list, got {self.center!r}")
        validate_number(self.center[0], "center[0]")
        validate_number(self.center[1], "center[1]")
        validate_number(self.radius, "radius", min_val=0.0, min_inclusive=False)
        validate_number(self.severity, "severity", min_val=0.0, min_inclusive=False)

    def contains_point(
        self, x: Union[float, Tuple[float, float], List[float]], y: Optional[float] = None
    ) -> bool:
        """Check if a given 2D point is inside the threat radius."""
        if y is None and isinstance(x, (tuple, list)):
            px, py = float(x[0]), float(x[1])
        elif y is not None and isinstance(x, (int, float)):
            px, py = float(x), float(y)
        else:
            raise ValueError(f"Invalid point arguments: x={x}, y={y}")
        dist = math.hypot(px - self.center[0], py - self.center[1])
        return dist <= self.radius

    def distance_to_boundary(
        self, x: Union[float, Tuple[float, float], List[float]], y: Optional[float] = None
    ) -> float:
        """
        Compute signed distance to threat boundary.
        Negative values indicate position inside threat radius.
        """
        if y is None and isinstance(x, (tuple, list)):
            px, py = float(x[0]), float(x[1])
        elif y is not None and isinstance(x, (int, float)):
            px, py = float(x), float(y)
        else:
            raise ValueError(f"Invalid point arguments: x={x}, y={y}")
        dist = math.hypot(px - self.center[0], py - self.center[1])
        return dist - self.radius

    def to_dict(self) -> Dict[str, Any]:
        """Serialize ThreatZone to a JSON-compatible dictionary."""
        return {
            "id": int(self.id),
            "center": [float(self.center[0]), float(self.center[1])],
            "radius": float(self.radius),
            "severity": float(self.severity),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ThreatZone":
        """Instantiate ThreatZone from a dictionary."""
        center_raw = data["center"]
        return cls(
            id=int(data["id"]),
            center=(float(center_raw[0]), float(center_raw[1])),
            radius=float(data["radius"]),
            severity=float(data.get("severity", 1.0)),
        )


@dataclass
class SimulationConfig:
    """
    Global simulation parameters and area boundaries.

    Attributes:
        width: 2D search area width in spatial units.
        height: 2D search area height in spatial units.
        resolution: Spatial size of each grid cell.
        dt: Kinematic integration time step in seconds.
        total_time: Maximum simulation duration limit in seconds.
        num_drones: Initial swarm count (N).
        sensor_radius: Drone reconnaissance coverage radius.
        heartbeat_timeout: Heartbeat loss threshold in seconds.
        max_drone_speed: Maximum speed limit for drone movement.
        random_seed: Random seed for reproducible simulations.
        standoff_radius_min: Minimum target standoff radius in meters.
        standoff_radius_max: Maximum target standoff radius in meters.
        standoff_radius_nominal: Target standoff radius nominal setting in meters.
        target_loss_timeout: Timeout threshold in seconds to revert to search if target packet lost.
        comm_range: P2P RF communication range in meters.
        packet_drop_rate: Probabilistic link packet drop rate (0.0 to 1.0).
        jamming_center: Optional (x, y) center coordinates of circular RF jamming zone.
        jamming_radius: Radius of RF jamming zone in meters.
        mule_cache_ttl: Maximum Data Mule cache retention time in seconds.
    """

    width: float = 100.0
    height: float = 100.0
    resolution: float = 1.0
    dt: float = 0.1
    total_time: float = 120.0
    num_drones: int = 10
    sensor_radius: float = 5.0
    heartbeat_timeout: float = 3.0
    max_drone_speed: float = 5.0
    random_seed: Optional[int] = None
    standoff_radius_min: float = 10.0
    standoff_radius_max: float = 20.0
    standoff_radius_nominal: float = 15.0
    target_loss_timeout: float = 5.0
    comm_range: float = 30.0
    packet_drop_rate: float = 0.0
    jamming_center: Optional[Tuple[float, float]] = None
    jamming_radius: float = 0.0
    mule_cache_ttl: float = 30.0
    collision_radius: float = 1.5       # Hard inter-drone collision avoidance radius (m)
    consensus_required: int = 2          # Min corroborations before full target tracking
    consensus_timeout: float = 5.0      # Seconds to wait for corroboration before rejecting
    require_hmac: bool = True            # Require HMAC verification on received packets

    def __post_init__(self) -> None:
        """Validate input ranges and types."""
        validate_number(self.width, "width", min_val=0.0, min_inclusive=False)
        validate_number(self.height, "height", min_val=0.0, min_inclusive=False)
        validate_number(self.resolution, "resolution", min_val=0.0, min_inclusive=False)
        validate_number(self.dt, "dt", min_val=0.0, min_inclusive=False)
        validate_number(self.total_time, "total_time", min_val=0.0, min_inclusive=False)

        if isinstance(self.num_drones, bool) or not isinstance(self.num_drones, int) or self.num_drones <= 0:
            raise ValueError(f"num_drones must be positive integer (not bool), got {self.num_drones!r}")

        validate_number(self.sensor_radius, "sensor_radius", min_val=0.0, min_inclusive=False)
        validate_number(self.heartbeat_timeout, "heartbeat_timeout", min_val=0.0, min_inclusive=False)
        validate_number(self.max_drone_speed, "max_drone_speed", min_val=0.0, min_inclusive=False)

        if self.random_seed is not None:
            if isinstance(self.random_seed, bool) or not isinstance(self.random_seed, int):
                raise ValueError(f"random_seed must be an integer or None, got {self.random_seed!r}")

        validate_number(self.standoff_radius_min, "standoff_radius_min", min_val=0.0, min_inclusive=False)
        validate_number(self.standoff_radius_max, "standoff_radius_max", min_val=0.0, min_inclusive=False)
        validate_number(self.standoff_radius_nominal, "standoff_radius_nominal", min_val=0.0, min_inclusive=False)
        validate_number(self.target_loss_timeout, "target_loss_timeout", min_val=0.0, min_inclusive=False)
        validate_number(self.comm_range, "comm_range", min_val=0.0, min_inclusive=False)
        validate_number(self.packet_drop_rate, "packet_drop_rate", min_val=0.0, max_val=1.0)

        if self.jamming_center is not None:
            if not isinstance(self.jamming_center, (tuple, list)) or len(self.jamming_center) != 2:
                raise ValueError(f"jamming_center must be a 2-tuple or 2-list, got {self.jamming_center!r}")
            validate_number(self.jamming_center[0], "jamming_center[0]")
            validate_number(self.jamming_center[1], "jamming_center[1]")
            self.jamming_center = (float(self.jamming_center[0]), float(self.jamming_center[1]))

        validate_number(self.jamming_radius, "jamming_radius", min_val=0.0)
        validate_number(self.mule_cache_ttl, "mule_cache_ttl", min_val=0.0, min_inclusive=False)

    @property
    def grid_dimensions(self) -> Tuple[int, int]:
        """Compute grid matrix columns and rows (cols, rows)."""
        cols = int(math.ceil(self.width / self.resolution))
        rows = int(math.ceil(self.height / self.resolution))
        return cols, rows

    @property
    def grid_shape(self) -> Tuple[int, int]:
        """Compute grid matrix shape (rows, cols)."""
        rows = int(math.ceil(self.height / self.resolution))
        cols = int(math.ceil(self.width / self.resolution))
        return rows, cols

    @property
    def total_cells(self) -> int:
        """Total number of discrete cells in search grid."""
        cols, rows = self.grid_dimensions
        return cols * rows

    @property
    def total_steps(self) -> int:
        """Total time steps expected in simulation run."""
        return int(math.ceil(self.total_time / self.dt))

    def to_dict(self) -> Dict[str, Any]:
        """Serialize configuration to a standard dictionary."""
        return {
            "width": float(self.width),
            "height": float(self.height),
            "resolution": float(self.resolution),
            "dt": float(self.dt),
            "total_time": float(self.total_time),
            "num_drones": int(self.num_drones),
            "sensor_radius": float(self.sensor_radius),
            "heartbeat_timeout": float(self.heartbeat_timeout),
            "max_drone_speed": float(self.max_drone_speed),
            "random_seed": self.random_seed,
            "standoff_radius_min": float(self.standoff_radius_min),
            "standoff_radius_max": float(self.standoff_radius_max),
            "standoff_radius_nominal": float(self.standoff_radius_nominal),
            "target_loss_timeout": float(self.target_loss_timeout),
            "comm_range": float(self.comm_range),
            "packet_drop_rate": float(self.packet_drop_rate),
            "jamming_center": list(self.jamming_center) if self.jamming_center is not None else None,
            "jamming_radius": float(self.jamming_radius),
            "mule_cache_ttl": float(self.mule_cache_ttl),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SimulationConfig":
        """Instantiate SimulationConfig from dictionary."""
        jc_raw = data.get("jamming_center")
        jamming_center = (float(jc_raw[0]), float(jc_raw[1])) if jc_raw is not None else None
        return cls(
            width=float(data.get("width", 100.0)),
            height=float(data.get("height", 100.0)),
            resolution=float(data.get("resolution", 1.0)),
            dt=float(data.get("dt", 0.1)),
            total_time=float(data.get("total_time", 120.0)),
            num_drones=int(data.get("num_drones", 10)),
            sensor_radius=float(data.get("sensor_radius", 5.0)),
            heartbeat_timeout=float(data.get("heartbeat_timeout", 3.0)),
            max_drone_speed=float(data.get("max_drone_speed", 5.0)),
            random_seed=data.get("random_seed"),
            standoff_radius_min=float(data.get("standoff_radius_min", 10.0)),
            standoff_radius_max=float(data.get("standoff_radius_max", 20.0)),
            standoff_radius_nominal=float(data.get("standoff_radius_nominal", 15.0)),
            target_loss_timeout=float(data.get("target_loss_timeout", 5.0)),
            comm_range=float(data.get("comm_range", 30.0)),
            packet_drop_rate=float(data.get("packet_drop_rate", 0.0)),
            jamming_center=jamming_center,
            jamming_radius=float(data.get("jamming_radius", 0.0)),
            mule_cache_ttl=float(data.get("mule_cache_ttl", 30.0)),
        )

    def to_json_file(self, filepath: Union[str, Path]) -> None:
        """Save configuration to JSON file path."""
        path = Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def from_json_file(cls, filepath: Union[str, Path]) -> "SimulationConfig":
        """Load configuration from JSON file path."""
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)


@dataclass
class TrajectoryFrame:
    """
    Snapshot of all active drones and coverage status at a specific time tick.

    Attributes:
        timestamp: Simulation time tick in seconds.
        drone_states: Dictionary mapping drone ID to DroneState.
        active_drone_count: Count of active drones.
        coverage_ratio: Current occupancy coverage ratio (0.0 to 1.0).
        target_state: Optional target tracking state snapshot dictionary.
        mode_counts: Map of mode names to count of drones in that mode.
    """

    timestamp: float
    drone_states: Dict[int, DroneState]
    active_drone_count: int
    coverage_ratio: float = 0.0
    target_state: Optional[Dict[str, Any]] = None
    mode_counts: Dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate TrajectoryFrame fields."""
        validate_number(self.timestamp, "timestamp", min_val=0.0)
        if not isinstance(self.drone_states, dict):
            raise ValueError(f"drone_states must be dict, got {self.drone_states!r}")
        for d_id, state in self.drone_states.items():
            if isinstance(d_id, bool) or not isinstance(d_id, int) or d_id < 0:
                raise ValueError(f"drone_states key must be non-negative integer, got {d_id!r}")
            if not isinstance(state, DroneState):
                raise ValueError(f"drone_states value must be DroneState instance, got {state!r}")

        if isinstance(self.active_drone_count, bool) or not isinstance(self.active_drone_count, int) or self.active_drone_count < 0:
            raise ValueError(f"active_drone_count must be non-negative integer, got {self.active_drone_count!r}")

        validate_number(self.coverage_ratio, "coverage_ratio", min_val=0.0, max_val=1.0)

        if self.target_state is not None:
            if isinstance(self.target_state, TargetState):
                self.target_state = self.target_state.to_dict()
            elif not isinstance(self.target_state, dict):
                raise ValueError(f"target_state must be dict or TargetState, got {self.target_state!r}")
        if not isinstance(self.mode_counts, dict):
            raise ValueError(f"mode_counts must be dict, got {self.mode_counts!r}")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize frame to dictionary."""
        return {
            "timestamp": float(self.timestamp),
            "drone_states": {
                str(d_id): state.to_dict() for d_id, state in self.drone_states.items()
            },
            "active_drone_count": int(self.active_drone_count),
            "coverage_ratio": float(self.coverage_ratio),
            "target_state": _clean_val(self.target_state) if self.target_state else None,
            "mode_counts": {str(k): int(v) for k, v in self.mode_counts.items()},
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TrajectoryFrame":
        """Instantiate frame from dictionary."""
        drone_states_raw = data.get("drone_states", {})
        drone_states = {
            int(d_id): DroneState.from_dict(s_data)
            for d_id, s_data in drone_states_raw.items()
        }
        return cls(
            timestamp=float(data["timestamp"]),
            drone_states=drone_states,
            active_drone_count=int(data["active_drone_count"]),
            coverage_ratio=float(data.get("coverage_ratio", 0.0)),
            target_state=data.get("target_state"),
            mode_counts={str(k): int(v) for k, v in data.get("mode_counts", {}).items()},
        )



@dataclass
class TrajectoryLog:
    """
    Complete trajectory log recording an entire simulation run.

    Attributes:
        config: Simulation settings used.
        threat_zones: List of active threat zones during the run.
        frames: Sequential list of TrajectoryFrame snapshots.
        metadata: Execution metadata dictionary.
    """

    config: SimulationConfig
    threat_zones: List[ThreatZone] = field(default_factory=list)
    frames: List[TrajectoryFrame] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate TrajectoryLog fields."""
        if not isinstance(self.config, SimulationConfig):
            raise ValueError(f"config must be SimulationConfig instance, got {self.config!r}")
        if not isinstance(self.threat_zones, list):
            raise ValueError(f"threat_zones must be list, got {self.threat_zones!r}")
        for tz in self.threat_zones:
            if not isinstance(tz, ThreatZone):
                raise ValueError(f"threat_zones element must be ThreatZone instance, got {tz!r}")
        if not isinstance(self.frames, list):
            raise ValueError(f"frames must be list, got {self.frames!r}")
        for frame in self.frames:
            if not isinstance(frame, TrajectoryFrame):
                raise ValueError(f"frames element must be TrajectoryFrame instance, got {frame!r}")
        if not isinstance(self.metadata, dict):
            raise ValueError(f"metadata must be dict, got {self.metadata!r}")

    def to_dict(self) -> Dict[str, Any]:
        """Serialize full trajectory log to dictionary."""
        return {
            "config": self.config.to_dict(),
            "threat_zones": [tz.to_dict() for tz in self.threat_zones],
            "frames": [frame.to_dict() for frame in self.frames],
            "metadata": _clean_val(self.metadata),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TrajectoryLog":
        """Instantiate TrajectoryLog from dictionary."""
        config = SimulationConfig.from_dict(data["config"])
        threat_zones = [
            ThreatZone.from_dict(tz_data) for tz_data in data.get("threat_zones", [])
        ]
        frames = [
            TrajectoryFrame.from_dict(f_data) for f_data in data.get("frames", [])
        ]
        return cls(
            config=config,
            threat_zones=threat_zones,
            frames=frames,
            metadata=data.get("metadata", {}),
        )

    def save_json(self, filepath: Union[str, Path]) -> None:
        """Write full trajectory log to JSON file."""
        path = Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_json(cls, filepath: Union[str, Path]) -> "TrajectoryLog":
        """Read full trajectory log from JSON file."""
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)
