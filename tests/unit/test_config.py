"""
Unit tests for swarm_recon/config.py data schemas and configuration classes.
"""

import json
from pathlib import Path
import pytest
from swarm_recon.config import (
    DroneState,
    ThreatZone,
    SimulationConfig,
    TrajectoryFrame,
    TrajectoryLog,
    is_valid_number,
    validate_number,
)


class TestDroneState:
    """Unit tests for DroneState dataclass."""

    def test_drone_state_defaults(self):
        drone = DroneState(id=1, position=(10.0, 20.0))
        assert drone.id == 1
        assert drone.position == (10.0, 20.0)
        assert drone.velocity == (0.0, 0.0)
        assert drone.heading == 0.0
        assert drone.active is True
        assert drone.sector_id is None
        assert drone.last_heartbeat == 0.0

    def test_drone_state_custom_values(self):
        drone = DroneState(
            id=5,
            position=(15.5, 30.2),
            velocity=(3.0, 4.0),
            heading=1.57,
            active=False,
            sector_id="sector_A",
            last_heartbeat=12.5,
        )
        assert drone.id == 5
        assert drone.position == (15.5, 30.2)
        assert drone.velocity == (3.0, 4.0)
        assert drone.heading == 1.57
        assert drone.active is False
        assert drone.sector_id == "sector_A"
        assert drone.last_heartbeat == 12.5
        assert drone.speed == pytest.approx(5.0)

    def test_drone_state_distance_to(self):
        drone1 = DroneState(id=1, position=(0.0, 0.0))
        drone2 = DroneState(id=2, position=(3.0, 4.0))
        assert drone1.distance_to(drone2) == pytest.approx(5.0)
        assert drone1.distance_to((3.0, 4.0)) == pytest.approx(5.0)
        assert drone1.distance_to([3.0, 4.0]) == pytest.approx(5.0)

    def test_drone_state_is_stale(self):
        drone = DroneState(id=1, last_heartbeat=10.0)
        assert drone.is_stale(current_time=12.0, timeout=3.0) is False
        assert drone.is_stale(current_time=13.0, timeout=3.0) is False
        assert drone.is_stale(current_time=13.1, timeout=3.0) is True

    def test_drone_state_serialization(self):
        drone = DroneState(
            id=2,
            position=(5.0, 5.0),
            velocity=(0.5, 0.5),
            heading=0.78,
            active=True,
            sector_id=1,
            last_heartbeat=3.0,
        )
        data = drone.to_dict()
        assert isinstance(data, dict)
        assert data["id"] == 2
        assert data["position"] == [5.0, 5.0]

        reconstructed = DroneState.from_dict(data)
        assert reconstructed.id == drone.id
        assert reconstructed.position == drone.position
        assert reconstructed.velocity == drone.velocity
        assert reconstructed.heading == pytest.approx(drone.heading)
        assert reconstructed.active == drone.active
        assert reconstructed.sector_id == drone.sector_id
        assert reconstructed.last_heartbeat == drone.last_heartbeat

    def test_drone_state_json_compatibility(self):
        drone = DroneState(id=3, position=(1.0, 2.0))
        json_str = json.dumps(drone.to_dict())
        loaded_dict = json.loads(json_str)
        reconstructed = DroneState.from_dict(loaded_dict)
        assert reconstructed.id == drone.id
        assert tuple(reconstructed.position) == drone.position

    def test_drone_state_invalid_id(self):
        with pytest.raises(ValueError):
            DroneState(id=-1, position=(0.0, 0.0))


class TestThreatZone:
    """Unit tests for ThreatZone dataclass."""

    def test_threat_zone_defaults(self):
        tz = ThreatZone(id=101, center=(50.0, 50.0), radius=15.0)
        assert tz.id == 101
        assert tz.center == (50.0, 50.0)
        assert tz.radius == 15.0
        assert tz.severity == 1.0

    def test_threat_zone_contains_and_boundary(self):
        tz = ThreatZone(id=1, center=(10.0, 10.0), radius=5.0)
        # Inside
        assert tz.contains_point(10.0, 10.0) is True
        assert tz.contains_point((13.0, 10.0)) is True
        assert tz.distance_to_boundary(10.0, 10.0) == pytest.approx(-5.0)
        assert tz.distance_to_boundary((13.0, 10.0)) == pytest.approx(-2.0)

        # On boundary
        assert tz.contains_point(15.0, 10.0) is True
        assert tz.distance_to_boundary(15.0, 10.0) == pytest.approx(0.0)

        # Outside
        assert tz.contains_point(20.0, 10.0) is False
        assert tz.distance_to_boundary(20.0, 10.0) == pytest.approx(5.0)

    @pytest.mark.parametrize("invalid_radius", [0.0, -5.0])
    def test_threat_zone_invalid_radius(self, invalid_radius):
        with pytest.raises(ValueError):
            ThreatZone(id=1, center=(0.0, 0.0), radius=invalid_radius)

    @pytest.mark.parametrize("invalid_severity", [0.0, -1.0])
    def test_threat_zone_invalid_severity(self, invalid_severity):
        with pytest.raises(ValueError):
            ThreatZone(id=1, center=(0.0, 0.0), radius=5.0, severity=invalid_severity)

    def test_threat_zone_serialization(self):
        tz = ThreatZone(id=102, center=(25.0, 75.0), radius=10.0, severity=2.5)
        data = tz.to_dict()
        reconstructed = ThreatZone.from_dict(data)
        assert reconstructed.id == tz.id
        assert reconstructed.center == tz.center
        assert reconstructed.radius == tz.radius
        assert reconstructed.severity == tz.severity


class TestSimulationConfig:
    """Unit tests for SimulationConfig dataclass."""

    def test_sim_config_defaults(self):
        cfg = SimulationConfig()
        assert cfg.width == 100.0
        assert cfg.height == 100.0
        assert cfg.resolution == 1.0
        assert cfg.dt == 0.1
        assert cfg.total_time == 120.0
        assert cfg.num_drones == 10
        assert cfg.sensor_radius == 5.0

    def test_sim_config_derived_properties(self):
        cfg = SimulationConfig(width=200.0, height=100.0, resolution=0.5, dt=0.05, total_time=10.0)
        assert cfg.grid_dimensions == (400, 200)  # (cols, rows)
        assert cfg.grid_shape == (200, 400)       # (rows, cols)
        assert cfg.total_cells == 80000
        assert cfg.total_steps == 200              # int(10.0 / 0.05)

    @pytest.mark.parametrize(
        "kwarg",
        [
            {"width": -10.0},
            {"height": 0.0},
            {"resolution": 0.0},
            {"dt": -0.1},
            {"num_drones": 0},
            {"sensor_radius": -1.0},
            {"heartbeat_timeout": 0.0},
            {"max_drone_speed": -2.0},
        ],
    )
    def test_sim_config_validation(self, kwarg):
        with pytest.raises(ValueError):
            SimulationConfig(**kwarg)

    def test_sim_config_serialization(self):
        cfg = SimulationConfig(width=50.0, height=50.0, num_drones=4)
        data = cfg.to_dict()
        reconstructed = SimulationConfig.from_dict(data)
        assert reconstructed == cfg

    def test_sim_config_file_io(self, tmp_path: Path):
        cfg = SimulationConfig(width=80.0, height=60.0, resolution=2.0)
        file_path = tmp_path / "sub_dir" / "config.json"
        cfg.to_json_file(file_path)

        assert file_path.exists()
        loaded_cfg = SimulationConfig.from_json_file(file_path)
        assert loaded_cfg == cfg


class TestTrajectoryLog:
    """Unit tests for TrajectoryFrame and TrajectoryLog."""

    def test_trajectory_frame_serialization(self):
        drone1 = DroneState(id=0, position=(1.0, 2.0))
        drone2 = DroneState(id=1, position=(3.0, 4.0))
        frame = TrajectoryFrame(
            timestamp=0.5,
            drone_states={0: drone1, 1: drone2},
            active_drone_count=2,
            coverage_ratio=0.05,
        )
        data = frame.to_dict()
        reconstructed = TrajectoryFrame.from_dict(data)

        assert reconstructed.timestamp == 0.5
        assert reconstructed.active_drone_count == 2
        assert reconstructed.coverage_ratio == pytest.approx(0.05)
        assert reconstructed.drone_states[0].position == (1.0, 2.0)
        assert reconstructed.drone_states[1].position == (3.0, 4.0)

    def test_trajectory_log_serialization_and_json(self, tmp_path: Path):
        cfg = SimulationConfig(width=50.0, height=50.0)
        tz = ThreatZone(id=1, center=(25.0, 25.0), radius=5.0)
        drone = DroneState(id=0, position=(10.0, 10.0))
        frame = TrajectoryFrame(
            timestamp=0.0,
            drone_states={0: drone},
            active_drone_count=1,
            coverage_ratio=0.01,
        )
        traj_log = TrajectoryLog(
            config=cfg,
            threat_zones=[tz],
            frames=[frame],
            metadata={"experiment": "test_run", "seed": 42},
        )

        log_path = tmp_path / "traj.json"
        traj_log.save_json(log_path)

        assert log_path.exists()
        loaded_log = TrajectoryLog.load_json(log_path)

        assert loaded_log.config == cfg
        assert len(loaded_log.threat_zones) == 1
        assert loaded_log.threat_zones[0].radius == 5.0
        assert len(loaded_log.frames) == 1
        assert loaded_log.frames[0].timestamp == 0.0
        assert loaded_log.metadata["experiment"] == "test_run"


class TestIsValidNumber:
    """Unit tests for is_valid_number function."""

    def test_valid_numbers(self):
        assert is_valid_number(0) is True
        assert is_valid_number(42) is True
        assert is_valid_number(-3.14) is True
        assert is_valid_number(1e-5) is True

    @pytest.mark.parametrize("invalid_val", [True, False, None, "123", [], {}, (1, 2)])
    def test_type_rejections(self, invalid_val):
        assert is_valid_number(invalid_val) is False

    @pytest.mark.parametrize("nan_inf", [float("nan"), float("inf"), float("-inf")])
    def test_nan_inf_rejections(self, nan_inf):
        assert is_valid_number(nan_inf) is False

    def test_bounds_checks(self):
        assert is_valid_number(5.0, min_val=0.0, max_val=10.0) is True
        assert is_valid_number(-1.0, min_val=0.0) is False
        assert is_valid_number(11.0, max_val=10.0) is False

        # Inclusive vs exclusive bounds
        assert is_valid_number(0.0, min_val=0.0, min_inclusive=True) is True
        assert is_valid_number(0.0, min_val=0.0, min_inclusive=False) is False
        assert is_valid_number(10.0, max_val=10.0, max_inclusive=True) is True
        assert is_valid_number(10.0, max_val=10.0, max_inclusive=False) is False

    def test_allow_none(self):
        assert is_valid_number(None, allow_none=True) is True
        assert is_valid_number(None, allow_none=False) is False


class TestDroneStateNaNInfRejection:
    """Unit tests for DroneState NaN and Inf rejection and adversarial input validation."""

    @pytest.mark.parametrize(
        "kwargs",
        [
            # Invalid ID (NaN, Inf, negative, boolean)
            {"id": float("nan")},
            {"id": float("inf")},
            {"id": -1},
            {"id": True},
            # Invalid Position (x or y is nan, inf, -inf)
            {"position": (float("nan"), 0.0)},
            {"position": (0.0, float("nan"))},
            {"position": (float("inf"), 0.0)},
            {"position": (0.0, float("inf"))},
            {"position": (float("-inf"), 0.0)},
            {"position": (0.0, float("-inf"))},
            # Invalid Velocity (vx or vy is nan, inf, -inf)
            {"velocity": (float("nan"), 0.0)},
            {"velocity": (0.0, float("nan"))},
            {"velocity": (float("inf"), 0.0)},
            {"velocity": (0.0, float("inf"))},
            {"velocity": (float("-inf"), 0.0)},
            {"velocity": (0.0, float("-inf"))},
            # Invalid Heading (nan, inf, -inf)
            {"heading": float("nan")},
            {"heading": float("inf")},
            {"heading": float("-inf")},
            # Invalid Last Heartbeat (nan, inf, -inf, negative)
            {"last_heartbeat": float("nan")},
            {"last_heartbeat": float("inf")},
            {"last_heartbeat": float("-inf")},
            {"last_heartbeat": -0.5},
            # Invalid Active (non-bool)
            {"active": "True"},
            # Invalid Sector ID (bool, negative int, wrong type)
            {"sector_id": True},
            {"sector_id": -5},
            {"sector_id": [1, 2]},
        ],
    )
    def test_drone_state_nan_inf_rejection(self, kwargs):
        """Verify that passing NaN, Inf, or invalid values to DroneState fields raises ValueError."""
        default_kwargs = {"id": 1, "position": (0.0, 0.0)}
        default_kwargs.update(kwargs)
        with pytest.raises(ValueError):
            DroneState(**default_kwargs)


class TestThreatZoneNaNInfRejection:
    """Unit tests for ThreatZone NaN and Inf rejection and adversarial input validation."""

    @pytest.mark.parametrize(
        "kwargs",
        [
            # Invalid ID
            {"id": float("nan")},
            {"id": float("inf")},
            {"id": -1},
            {"id": True},
            # Invalid Center (cx or cy is nan, inf, -inf)
            {"center": (float("nan"), 0.0)},
            {"center": (0.0, float("nan"))},
            {"center": (float("inf"), 0.0)},
            {"center": (0.0, float("inf"))},
            {"center": (float("-inf"), 0.0)},
            {"center": (0.0, float("-inf"))},
            # Invalid Radius (nan, inf, -inf, 0, negative)
            {"radius": float("nan")},
            {"radius": float("inf")},
            {"radius": float("-inf")},
            {"radius": 0.0},
            {"radius": -5.0},
            # Invalid Severity (nan, inf, -inf, 0, negative)
            {"severity": float("nan")},
            {"severity": float("inf")},
            {"severity": float("-inf")},
            {"severity": 0.0},
            {"severity": -1.0},
        ],
    )
    def test_threat_zone_nan_inf_rejection(self, kwargs):
        """Verify that passing NaN, Inf, or invalid values to ThreatZone fields raises ValueError."""
        default_kwargs = {"id": 101, "center": (10.0, 10.0), "radius": 5.0, "severity": 1.0}
        default_kwargs.update(kwargs)
        with pytest.raises(ValueError):
            ThreatZone(**default_kwargs)


class TestSimulationConfigNaNInfRejection:
    """Unit tests for SimulationConfig NaN and Inf rejection and adversarial input validation."""

    @pytest.mark.parametrize(
        "kwarg",
        [
            # Width nan, inf, -inf, 0, negative
            {"width": float("nan")},
            {"width": float("inf")},
            {"width": float("-inf")},
            {"width": 0.0},
            {"width": -10.0},
            # Height nan, inf, -inf, 0, negative
            {"height": float("nan")},
            {"height": float("inf")},
            {"height": float("-inf")},
            {"height": 0.0},
            # Resolution nan, inf, -inf, 0, negative
            {"resolution": float("nan")},
            {"resolution": float("inf")},
            {"resolution": float("-inf")},
            {"resolution": 0.0},
            # dt nan, inf, -inf, 0, negative
            {"dt": float("nan")},
            {"dt": float("inf")},
            {"dt": float("-inf")},
            {"dt": 0.0},
            # total_time nan, inf, -inf, 0, negative
            {"total_time": float("nan")},
            {"total_time": float("inf")},
            {"total_time": float("-inf")},
            {"total_time": 0.0},
            # num_drones nan, inf, negative, zero, bool
            {"num_drones": float("nan")},
            {"num_drones": float("inf")},
            {"num_drones": -1},
            {"num_drones": 0},
            {"num_drones": True},
            # sensor_radius nan, inf, -inf, 0, negative
            {"sensor_radius": float("nan")},
            {"sensor_radius": float("inf")},
            {"sensor_radius": float("-inf")},
            {"sensor_radius": 0.0},
            # heartbeat_timeout nan, inf, -inf, 0, negative
            {"heartbeat_timeout": float("nan")},
            {"heartbeat_timeout": float("inf")},
            {"heartbeat_timeout": float("-inf")},
            {"heartbeat_timeout": 0.0},
            # max_drone_speed nan, inf, -inf, 0, negative
            {"max_drone_speed": float("nan")},
            {"max_drone_speed": float("inf")},
            {"max_drone_speed": float("-inf")},
            {"max_drone_speed": 0.0},
            # random_seed bool, float
            {"random_seed": True},
            {"random_seed": 3.14},
        ],
    )
    def test_sim_config_nan_inf_rejection(self, kwarg):
        """Verify that passing NaN, Inf, or invalid values to SimulationConfig fields raises ValueError."""
        with pytest.raises(ValueError):
            SimulationConfig(**kwarg)


class TestTrajectoryFrameNaNInfRejection:
    """Unit tests for TrajectoryFrame NaN/Inf and type rejection."""

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"timestamp": float("nan")},
            {"timestamp": float("inf")},
            {"timestamp": -1.0},
            {"coverage_ratio": float("nan")},
            {"coverage_ratio": float("inf")},
            {"coverage_ratio": -0.1},
            {"coverage_ratio": 1.5},
            {"active_drone_count": -1},
            {"active_drone_count": True},
            {"drone_states": "invalid"},
            {"drone_states": {True: DroneState(id=0)}},
        ],
    )
    def test_trajectory_frame_rejection(self, kwargs):
        default_kwargs = {
            "timestamp": 0.0,
            "drone_states": {0: DroneState(id=0)},
            "active_drone_count": 1,
            "coverage_ratio": 0.5,
        }
        default_kwargs.update(kwargs)
        with pytest.raises(ValueError):
            TrajectoryFrame(**default_kwargs)


class TestTrajectoryLogNaNInfRejection:
    """Unit tests for TrajectoryLog field validation."""

    def test_trajectory_log_rejection(self):
        cfg = SimulationConfig()
        tz = ThreatZone(id=1, center=(10.0, 10.0), radius=5.0)
        frame = TrajectoryFrame(
            timestamp=0.0,
            drone_states={0: DroneState(id=0)},
            active_drone_count=1,
            coverage_ratio=0.1,
        )

        with pytest.raises(ValueError):
            TrajectoryLog(config="invalid_config")

        with pytest.raises(ValueError):
            TrajectoryLog(config=cfg, threat_zones=["not_a_threat_zone"])

        with pytest.raises(ValueError):
            TrajectoryLog(config=cfg, frames=["not_a_frame"])

        with pytest.raises(ValueError):
            TrajectoryLog(config=cfg, metadata="not_a_dict")
