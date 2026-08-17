# Project: Decentralized Swarm Reconnaissance System (with Defense SDK Extension)

## Architecture
The system is built as a modular Python package (`swarm_recon`) adhering to strict lightweight dependency constraints (<500MB total installation size).

### Core Components
1. **Grid & Search Space Engine (`swarm_recon/core/`)**: Discrete $W \times H$ occupancy grid matrix tracking cell coverage state, drone positions, and sector boundaries.
2. **Decentralized Consensus & Partitioning (`swarm_recon/agents/`)**: P2P heartbeat liveness protocol and dynamic Centroidal Voronoi sector reassignment algorithm handling mid-mission drone failures ($K$ killed).
3. **Emergent Evasion Dynamics (`swarm_recon/evasion/`)**: Rotational Artificial Potential Fields (APF) + Boids flocking separation + stochastic heading perturbation for fluid threat avoidance and unpredictable trajectories.
4. **Target-Triggered Voronoi Collapse ("Hunter-Killer" Handoff)**: Multi-mode swarm state machine (`SwarmMode.SEARCH` <-> `SwarmMode.TARGET_TRACKING`), P2P `TargetTelemetry` broadcast, radial standoff ($10\text{m}-20\text{m}$) + tangential orbital encirclement blended with Rotational APF threat evasion, and auto-reversion to Voronoi search.
5. **ROS 2 / MAVLink Defense SDK Integration (`SDK_GUIDE.md` & `swarm_recon/sdk_template.py`)**: Architectural guide and code template providing `SwarmReconROS2Node`, `MAVLinkBridge`, and `SwarmSDKAdapter` for ROS 2 nodes and PX4/ArduPilot MAVLink telemetry (`MAV_CMD_NAV_WAYPOINT`, `SET_POSITION_TARGET_LOCAL_NED`) with zero-dependency fallback mocks.
6. **Simulation Engine & Trajectory Logger (`swarm_recon/simulation/`)**: Discrete-time kinematic loop ($\Delta t = 0.1\text{s}$) with dynamic kill injection, target event schedules, circular threat zone fields, and full trajectory logging.
7. **Verification & Analysis Tools (`swarm_recon/analysis/`, `scripts/`, `tests/`)**: Automated scripts (`verify_r1.py`, `verify_r2.py`, `verify_r3.py`, `verify_target_handoff.py`, `scripts/test_runner.py`) evaluating coverage (>95%), threat avoidance & fluidity, target handoff standoff & encirclement, and dependency size (<500MB).

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Occupancy Grid Engine | Discrete 2D search space grid & cell coverage tracking | M1 | Survey |
| 2 | Data Schemas & Config | `DroneState`, `ThreatZone`, `SimulationConfig`, trajectory log schemas | M1 | Survey |
| 3 | P2P Heartbeat Protocol | Decentralized liveness tracking & failure detection | M2 | Survey / R1 |
| 4 | Dynamic Sector Reassignment | Centroidal Voronoi / Grid sector re-partitioning when drones are lost | M2 | Survey / R1 |
| 5 | Rotational APF Threat Avoidance | Dynamic repelling & orbiting force vectors around circular threat fields | M3 | Survey / R2 |
| 6 | Boids Flocking & Stochastic Noise | Inter-drone separation, alignment, and stochastic heading entropy generation | M3 | Survey / R2 |
| 7 | Simulation Runner & Logger | Discrete-time kinematic simulation loop & trajectory log persistence | M4 | Survey |
| 8 | R1 Verification Script (`verify_r1.py`) | Automated test spawning $N$ drones, killing $K$ mid-sim, asserting >95% coverage | M5 | Survey / R1 |
| 9 | R2 Verification Script (`verify_r2.py`) | Automated trajectory analysis evaluating threat avoidance, jerk bounds & entropy | M5 | Survey / R2 |
| 10 | R3 Verification Script (`verify_r3.py`) | Automated dependency directory size check asserting <= 500MB | M5 | Survey / R3 |
| 11 | Unified Test Runner (`test_runner.py`) | E2E test runner executing all verification benchmarks with structured JSON output | M5 | Survey |
| 12 | E2E Test Suite (Tiers 1-4) | Comprehensive test suite for requirements R1, R2, R3 | E2E-Track | Survey |
| 13 | Swarm Mode State Machine & Telemetry | `SwarmMode.SEARCH` / `TARGET_TRACKING`, `TargetTelemetry`, `TelemetryPacket` P2P broadcast | M-EXT1 | Survey / Ext R1 |
| 14 | Target Encirclement & APF Evasion | `EvaderForces.target_encirclement` standoff (10m-20m) + orbital drive + APF threat avoidance | M-EXT1 | Survey / Ext R1 |
| 15 | Simulation Target Telemetry Bus | `SimulationEngine` packet routing bus & target detect/clear event schedules | M-EXT1 | Survey / Ext R1 |
| 16 | ROS 2 / MAVLink SDK Guide & Template | `SDK_GUIDE.md` & `swarm_recon/sdk_template.py` (ROS 2 node & PX4/ArduPilot MAVLink bridge) | M-EXT2 | Survey / Ext R2 |
| 17 | Target Handoff Sim & Size Check Script | `verify_target_handoff.py`, `scripts/test_runner.py` 4-test runner, `verify_r3.py` <500MB check | M-EXT3 | Survey / Ext R1-R3 |
| 18 | Extension E2E Test Suite (Tier 5) | Comprehensive E2E target handoff test suite (`test_tier5.py`, updated `conftest.py`) | E2E-EXT | Survey / Ext |
| 19 | Multi-Hop Mesh Routing Header & Data Model | `TelemetryPacket` with `source_id`, `destination_id`, `sequence_id`, `hop_count`, `ttl`, `relayed_by`/`path_history`, `packet_id` | M-EXT-MESH1 | Survey / Mesh R1 |
| 20 | RF-Denied Jamming & Link Loss Sim Engine | `SimulationConfig` (`comm_range`, `packet_drop_rate=0.50`, `jamming_center`, `jamming_radius`), RF line-of-sight raycasting in `SimulationEngine` | M-EXT-MESH1 | Survey / Mesh R1 |
| 21 | Store-and-Forward Data Mule Swarm Agent | `SwarmAgent` store-and-forward caching (`_mule_cache`, `_seen_packet_ids`), multi-hop neighbor relaying & Data Mule transport | M-EXT-MESH1 | Survey / Mesh R1 |
| 22 | RF-Denied Mesh Verification & Size Check | `verify_mesh_handoff.py` (>=2 hops under RF jamming, 50% link loss, <500MB check) & `test_runner.py` 5-test runner | M-EXT-MESH2 | Survey / Mesh R1-R2 |
| 23 | Tier 6 E2E Integration Test Suite | Comprehensive E2E mesh handoff test suite (`test_tier6.py` and `conftest.py` harness update) | E2E-MESH | Survey / Mesh R1 |

## Milestones
| # | Name | Scope | Dependencies | Status | Sub-orchestrator Conv ID |
|---|------|-------|-------------|--------|--------------------------|
| Baseline M1-M6 | Baseline System | Core framework, Voronoi search, APF evasion, baseline scripts | None | DONE | Completed |
| Extension M-EXT1-4 | Target Handoff & SDK Extension | Hunter-Killer Voronoi collapse, SDK Guide, `verify_target_handoff.py` | Baseline | DONE | Completed |
| E2E-MESH | Mesh Handoff E2E Testing Track | Tier 6 tests (`test_tier6.py`), `conftest.py` mesh support, `TEST_INFRA.md`, `TEST_READY.md` | M-EXT-MESH1 | IN_PROGRESS | TBD |
| M-EXT-MESH1 | Multi-Hop Mesh & Data Mule Core | `TelemetryPacket` mesh headers, `SimulationEngine` RF jamming & link loss, `SwarmAgent` store-and-forward caching | Baseline | IN_PROGRESS | TBD |
| M-EXT-MESH2 | Mesh Verification & Size Check Script | `verify_mesh_handoff.py` (>=2 hops, 50% drop rate, <500MB), `scripts/test_runner.py` 5-test runner | M-EXT-MESH1 | PLANNED | TBD |
| M-EXT-MESH3 | Mesh Extension Final Acceptance | Pass 100% E2E tests (Tiers 1-6) + Tier 6 white-box coverage hardening | M-EXT-MESH2, E2E-MESH | PLANNED | TBD |

## Interface Contracts

### 1. Swarm State & Grid Interface (`swarm_recon/core/`)
```python
class GridSearchSpace:
    def __init__(self, width: float, height: float, resolution: float = 1.0): ...
    def mark_visited(self, x: float, y: float, sensor_radius: float) -> int: ...
    def get_coverage_ratio(self) -> float: ...
    def repartition(self, active_drone_positions: dict[int, tuple[float, float]]) -> dict[int, list[tuple[int, int]]]: ...
```

### 2. Drone Agent & Multi-Hop Mesh Interface (`swarm_recon/agents/` & `swarm_recon/config.py`)
```python
@dataclass
class TelemetryPacket:
    sender_id: int
    packet_type: PacketType
    target_state: Optional[TargetState] = None
    timestamp: float = 0.0
    source_id: int = 0
    destination_id: Union[int, str] = -1  # -1 = Broadcast to all
    sequence_id: int = 0
    hop_count: int = 0
    ttl: int = 10
    relayed_by: list[int] = field(default_factory=list)

    @property
    def packet_id(self) -> str:
        return f"{self.source_id}_{self.sequence_id}"

class SwarmAgent:
    def update(self, dt: float, peers: dict[int, DroneState], threats: list[ThreatZone], grid: GridSearchSpace) -> None: ...
    def receive_telemetry_packet(self, packet: TelemetryPacket) -> bool: ...
    def get_mule_packets(self, current_time: float) -> list[TelemetryPacket]: ...
    def detect_target(self, target_pos: tuple[float, float], target_id: int = 1) -> TelemetryPacket: ...
    def clear_target(self, target_id: int = 1) -> None: ...
```

### 3. Simulation Engine RF & Mesh Interface (`swarm_recon/simulation/engine.py`)
```python
class SimulationEngine:
    def _is_rf_connected(self, pos1: tuple[float, float], pos2: tuple[float, float]) -> bool: ...
    def step(self) -> None: ...
```

### 4. Verification CLI Protocols
- `python scripts/verify_r1.py --drones 10 --killed 3 --time-limit 120 --seed 42` -> Exit 0 if coverage > 0.95 else 1.
- `python scripts/verify_r2.py --trajectory-file logs/traj.json --threats-file config/threats.json` -> Exit 0 if zero threat collisions and entropy >= 1.5 bits else 1.
- `python scripts/verify_r3.py --target-dir .venv --max-size-mb 500` -> Exit 0 if size <= 500MB else 1.
- `python scripts/verify_target_handoff.py --drones 10 --time-limit 120 --detect-time 30 --clear-time 90 --seed 42` -> Exit 0 if mode transition, standoff (10m-20m), zero threat collisions pass else 1.
- `python scripts/verify_mesh_handoff.py --drones 6 --comm-range 30 --jamming-radius 25 --packet-drop-rate 0.50 --seed 42` -> Exit 0 if telemetry arrives across >=2 hops under RF jamming and 50% packet drop rate else 1.
- `python scripts/test_runner.py --output results.json` -> Executes all 5 verification scripts, exit 0 if all pass.

## Code Layout
```
swarm_recon/
├── __init__.py
├── config.py             # Data schemas (DroneState, TargetTelemetry, TelemetryPacket, SwarmMode, SimulationConfig)
├── sdk_template.py       # ROS 2 & MAVLink SDK integration template module
├── core/                 # Occupancy grid & spatial partitioning
│   ├── __init__.py
│   └── grid.py
├── agents/               # P2P Heartbeat & multi-mode drone agent
│   ├── __init__.py
│   └── drone.py
├── evasion/              # Rotational APF, Boids & target encirclement forces
│   ├── __init__.py
│   └── forces.py
├── simulation/           # Kinematic simulation loop, target event bus & trajectory logger
│   ├── __init__.py
│   └── engine.py
└── analysis/             # Metrics calculation (coverage, jerk, entropy, standoff distances)
    ├── __init__.py
    └── metrics.py

scripts/
├── verify_r1.py          # R1 dynamic search & reassignment verification
├── verify_r2.py          # R2 threat evasion & fluidity verification
├── verify_r3.py          # R3 dependency folder size check
├── verify_target_handoff.py  # R1/Ext target handoff & encirclement verification
└── test_runner.py        # Unified E2E test runner executing all 4 verifications

SDK_GUIDE.md              # ROS 2 & MAVLink SDK Architecture Guide

tests/                    # Test suite directory
├── unit/                 # Module unit tests
└── e2e/                  # Tier 1-5 E2E integration tests
```
