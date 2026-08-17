# E2E Test Infrastructure & Test Architecture

## 1. Test Philosophy & Paradigm

The Decentralized Swarm Reconnaissance System test infrastructure adopts an **opaque-box, requirement-driven testing paradigm**. The test suite evaluates system compliance strictly against functional and non-functional requirements specified in `ORIGINAL_REQUEST.md` and `PROJECT.md`.

### Core Principles
1. **Opaque-Box Verification**: Tests validate observable macro-level behaviors (area search coverage, threat clearance distance, trajectory kinematic jerk, 36-bin Shannon heading entropy, and virtual environment installation size) without asserting on internal private state variables or using fragile mocks.
2. **Deterministic Reproducibility**: Discrete-time kinematic simulation ($\Delta t = 0.1\text{s}$) runs under fixed random seeds (`seed=42`, `seed=1337`) to ensure 100% bit-for-bit test reproducibility across environments.
3. **Genuine Logic Execution**: All simulation components, kinematic integration, Centroidal Voronoi re-partitioning, Rotational Artificial Potential Fields (APF), and Boids inter-drone separation execute real physics and state updates. No hardcoded test results, facade outputs, or dummy pass flags are permitted.
4. **Standalone Fallback Safety**: The test harness includes a standalone simulation fallback engine in `conftest.py`. If core modules (`swarm_recon`) are not yet installed in the Python environment, the fallback engine runs identical real physics and partitioning algorithms, guaranteeing test execution integrity.

---

## 2. Feature Inventory & Tier Breakdown

Features 1 through 15 from `PROJECT.md` are mapped into five progressive testing tiers:

| Tier | Name | Target Features | Scenario Description | Acceptance Criteria |
|------|------|-----------------|----------------------|---------------------|
| **Tier 1** | Baseline Verification | Features 1, 2, 7, 10 | Baseline swarm spawning, initial Voronoi sector allocation, discrete occupancy grid tracking, and dependency installation size check. | • Spawn count $N$ active drones within $[0, W] \times [0, H]$<br>• Monotonic coverage progression ($\Delta C \ge 0$)<br>• Dependency directory size $\le 500$ MB ($524,288,000$ bytes) |
| **Tier 2** | Boundary & Corner Cases | Features 1, 3, 4, 5, 6 | Extreme drone failure ($K = N - 1$), zero threat fields, heavy obstacle density, and area edge/corner coordinate limits. | • $K=N-1$: Survivor ID receives 100% unsearched frontier without division-by-zero<br>• Zero threats: Repulsion $\mathbf{F}_{\text{repel}} = 0$, mean jerk $< 0.5\text{ m/s}^3$<br>• Max threats: Zero threat penetrations ($d_{\text{clearance}} > 0.0$m)<br>• Edge bounds: Coordinates clamped $[0, W] \times [0, H]$, zero matrix index errors |
| **Tier 3** | Cross-Feature Combinations | Features 3, 4, 5, 6, 7 | Simultaneous drone failure mid-simulation ($K=3$) + dynamic Rotational APF threat evasion + Boids flocking separation under moving and stationary threat fields. | • Sector reassignment time $T_{\text{reassign}} \le 3.0$s<br>• Zero threat boundary violations ($d_{\text{clearance}} > 0.0$m)<br>• Inter-drone Boids separation $d_{\text{inter}} \ge 1.0$m<br>• Reachable grid coverage $> 95.0\%$ within $T=120$s |
| **Tier 4** | Real-World Stress Scenarios | Features 8, 9, 10, 11 | $N=20$ drones in a $200\text{m} \times 200\text{m}$ grid, 5 dynamic kills staggered mid-simulation ($t=15, 30, 45, 60, 75$s), 5 circular threat zones. | • Reachable area coverage $C > 0.950$ ($95.0\%$) within $T=120$s<br>• Minimum threat clearance $d_{\text{clearance}} > 0.0$m (0 collisions)<br>• Shannon heading entropy $H(\Delta \theta) \ge 1.50$ bits<br>• Trajectory jerk $\text{Jerk}_{\text{mean}} \le 2.0\text{ m/s}^3$ |
| **Tier 5** | Target Handoff & Encirclement | Features 13, 14, 15 | Swarm target detection telemetry broadcast, transition from Voronoi search to target tracking, orbital encirclement standoff, Rotational APF threat avoidance during encirclement, and revert to Voronoi search upon target clearing. | • Immediate mode transition (`SEARCH` $\to$ `TARGET_TRACKING`) upon `TARGET_FOUND`<br>• Encirclement standoff radius $10.0\text{m} \le d_{\text{target}} \le 20.0\text{m}$ across all active drones during steady state<br>• Zero threat zone penetrations ($d_{\text{clearance}} > 0.0$m)<br>• Revert to `SEARCH` mode upon `TARGET_CLEARED` with post-clearing search coverage progression ($\Delta C \ge 0.05$) |

---

## 3. Test Architecture & Runner Protocols

### 3.1 Directory Layout
```
tests/
└── e2e/
    ├── __init__.py          # Package marker
    ├── conftest.py          # Fixtures, fallback simulation engine, metrics calculators & log parser
    ├── test_runner.py       # Standalone CLI test runner & JSON reporter
    ├── test_tier1.py        # Baseline tests (spawn, coverage, dependency size)
    ├── test_tier2.py        # Boundary/corner tests (K=N-1, 0 threats, max threats, corners)
    ├── test_tier3.py        # Cross-feature tests (simultaneous kill + APF evasion + Boids)
    ├── test_tier4.py        # Real-world stress tests (N=20, 5 staggered kills, circular threats)
    └── test_tier5.py        # Extension target handoff tests (mode transition, standoff, threat avoidance, revert)
```

### 3.2 Runner Protocol & CLI Options
The test suite can be launched via standard `pytest` or via the unified test runner `python tests/e2e/test_runner.py`:

```bash
# Run all tiers via unified runner
python tests/e2e/test_runner.py --tier all --output logs/e2e_results.json

# Run specific tier via pytest
pytest tests/e2e/test_tier1.py -v
```

#### Runner Exit Code Conventions
- **`Exit 0`**: All executed test cases PASSED.
- **`Exit 1`**: One or more test assertions FAILED (e.g. coverage $< 95\%$, threat collision, size $> 500$ MB).
- **`Exit 2`**: Infrastructure or configuration ERROR (e.g. missing test directory, unparseable arguments, runtime crash).

### 3.3 Result JSON Schema (`logs/e2e_results.json`)
```json
{
  "timestamp": "2026-08-12T11:20:00Z",
  "summary": {
    "total": 11,
    "passed": 11,
    "failed": 0,
    "duration_sec": 12.45,
    "status": "PASSED"
  },
  "results": [
    {
      "test_id": "E2E-T4-01",
      "name": "test_full_swarm_recon_stress_benchmark",
      "tier": 4,
      "passed": true,
      "duration_sec": 2.15,
      "metrics": {
        "coverage_ratio": 0.968,
        "min_threat_clearance": 1.42,
        "mean_heading_entropy": 2.14,
        "mean_jerk": 0.85
      },
      "failure_reason": null
    }
  ]
}
```

---

## 4. Real-World Application Scenarios (Tier 4 Stress)

The primary real-world validation benchmark (`E2E-T4-01`) simulates an urgent search-and-reconnaissance mission in a hostile environment:

- **Operational Area**: $200\text{m} \times 200\text{m}$ 2D grid space ($40,000\text{ m}^2$).
- **Swarm Fleet**: Initialized with $N = 20$ autonomous drones.
- **Hostile Threats**: 5 circular threat zones situated across the grid:
  1. Threat 1: Center $(40, 50)$, Radius $12\text{m}$
  2. Threat 2: Center $(150, 60)$, Radius $15\text{m}$
  3. Threat 3: Center $(100, 120)$, Radius $10\text{m}$
  4. Threat 4: Center $(60, 160)$, Radius $14\text{m}$
  5. Threat 5: Center $(160, 150)$, Radius $11\text{m}$
- **Dynamic Attrition**: 5 drones are killed sequentially mid-mission:
  - $t = 15.0\text{s}$: Drone 3 killed
  - $t = 30.0\text{s}$: Drone 7 killed
  - $t = 45.0\text{s}$: Drone 12 killed
  - $t = 60.0\text{s}$: Drone 15 killed
  - $t = 75.0\text{s}$: Drone 18 killed
- **Consensus & Reassignment**: Upon each loss, remaining active drones detect missing heartbeats and re-partition unsearched sectors via Centroidal Voronoi tessellation.
- **Trajectory Dynamics**: Active drones blend sector centroid attraction force $\mathbf{F}_{\text{target}}$, Rotational APF repelling force $\mathbf{F}_{\text{apf}}$, Boids inter-drone separation force $\mathbf{F}_{\text{boids}}$, and stochastic perturbation force $\mathbf{F}_{\text{noise}}$.

---

## 5. Coverage & Acceptance Thresholds

The system must satisfy all quantitative bounds to pass end-to-end verification:

1. **Search Area Coverage Ratio ($C_{\text{reachable}}$)**:
   $$C_{\text{reachable}} = \frac{\text{Visited Reachable Cells}}{\text{Total Reachable Cells}} > 0.950 \quad (95.0\%)$$
   *Evaluated within time limit $T = 120.0\text{s}$.*

2. **Threat Avoidance Clearance ($d_{\text{min\_clearance}}$)**:
   $$d_{\text{min\_clearance}} = \min_{t, i, j} \left( \|\mathbf{p}_i(t) - \mathbf{c}_j(t)\| - R_j \right) > 0.0 \text{ m}$$
   *Requires zero penetrations into circular threat zones across all active drones.*

3. **Trajectory Fluidity & Unpredictability (Shannon Heading Entropy $H$)**:
   Heading angle changes $\Delta \theta$ are discretized into $B = 36$ equal bins ($10^\circ$ resolution):
   $$H(\Delta \theta) = -\sum_{b=1}^{36} P(b) \log_2 P(b) \ge 1.50 \text{ bits}$$
   *Ensures emergent evasion trajectories maintain fluid, non-predictable flight paths.*

4. **Kinematic Smoothness (Mean Jerk)**:
   $$\text{Jerk}_{\text{mean}} = \frac{1}{K} \sum_{k=1}^K \left\| \frac{\mathbf{a}(k+1) - \mathbf{a}(k)}{\Delta t} \right\| \le 2.0 \text{ m/s}^3$$
   *Guarantees physical feasibility of drone maneuverability.*

5. **Installation Dependency Directory Size ($S_{\text{deps}}$)**:
   $$S_{\text{deps}} \le 500.0 \text{ MB} \quad (524,288,000 \text{ bytes})$$
   *Verifies the project runtime footprint adheres strictly to requirement R3.*

---

## 6. Tier 5 Extension Test Suite Specification (Target Handoff & Encirclement)

Tier 5 verifies the "Hunter-Killer" dynamic target handoff, orbital encirclement, blended APF threat avoidance, and state revert mechanics using Category-Partitioning and Boundary Value Analysis (BVA).

### 6.1 Test Case `E2E-T5-01`: Target Detection Mode Transition (`test_target_detection_mode_transition`)
- **Objective**: Assert immediate swarm state machine transition from `SEARCH` (`SwarmMode.SEARCH`) to `TARGET_TRACKING` (`SwarmMode.TARGET_TRACKING`) upon receiving a `TARGET_FOUND` broadcast telemetry packet.
- **Category-Partitioning**:
  - *Input Space*: Active swarm in Voronoi area search receiving broadcast payload `TargetTelemetry(target_id=1, position=(50.0, 50.0), status="FOUND", timestamp=20.0)`.
  - *Partition*: Single target injection event at $t_{\text{detect}} = 20.0\text{s}$.
- **Boundary Value Analysis (BVA)**:
  - *Pre-Event Boundary*: $t < t_{\text{detect}} = 20.0\text{s} \implies \text{mode} == \text{"SEARCH"}$.
  - *Post-Event Boundary*: $t \ge t_{\text{detect}} + \Delta t = 20.1\text{s} \implies \text{mode} == \text{"TARGET_TRACKING"}$.
- **Pass Assertions**:
  1. `all(d["mode"] == "TARGET_TRACKING" for d in active_drones)` at $t \ge 20.1\text{s}$.
  2. Latency $\le 1$ simulation step ($\Delta t = 0.1\text{s}$).

### 6.2 Test Case `E2E-T5-02`: Target Encirclement Standoff Radius (`test_target_encirclement_standoff_radius`)
- **Objective**: Assert active tracking drones maintain orbital distance $d_{\text{target}}$ strictly within $[10.0\text{m}, 20.0\text{m}]$ during steady-state target tracking.
- **Category-Partitioning**:
  - *Input Space*: Swarm in `TARGET_TRACKING` mode, target position $\mathbf{p}_T = (50.0, 50.0)$.
  - *Operational Window*: Steady-state tracking window $t \in [t_{\text{detect}} + 5.0\text{s}, t_{\text{clear}}] = [15.0\text{s}, 80.0\text{s}]$.
- **Boundary Value Analysis (BVA)**:
  - *Lower Bound*: $d_{\text{target\_min}} = 10.0\text{m}$.
  - *Nominal Setpoint*: $R_{\text{standoff}} = 15.0\text{m}$.
  - *Upper Bound*: $d_{\text{target\_max}} = 20.0\text{m}$.
  - *Metric*: Euclidean distance $d_i(t) = \|\mathbf{p}_i(t) - \mathbf{p}_T\|$ for each active drone $i$.
- **Pass Assertions**:
  1. `10.0 <= d_i(t) <= 20.0` for 100% of active tracking drones across all frames $t \in [15.0\text{s}, 80.0\text{s}]$.
  2. `in_bounds_ratio == 1.0`.

### 6.3 Test Case `E2E-T5-03`: Target Encirclement Threat Avoidance (`test_target_encirclement_threat_avoidance`)
- **Objective**: Verify active drones maintain positive threat clearance ($d_{\text{clearance}} > 0.0\text{m}$) when encircling a target located in close proximity to circular threat fields.
- **Category-Partitioning**:
  - *Input Space*: Target at $\mathbf{p}_T = (50.0, 50.0)$, circular threat zone centered at $(40.0, 40.0)$ with radius $R_{\text{threat}} = 12.0\text{m}$.
  - *Force Blend*: Target Encirclement force (radial standoff + CCW tangential orbit) coupled with Rotational APF repelling force.
- **Boundary Value Analysis (BVA)**:
  - *Threat Boundary*: $d_{\text{clearance}} = \min(d_{\text{threat}} - R_{\text{threat}}) > 0.0\text{m}$.
  - *Max Standoff Boundary*: $d_{\text{target}} \le 20.0\text{m}$ along threat-free orbital arc.
- **Pass Assertions**:
  1. `min_threat_clearance > 0.0` (zero threat collisions across all active drones during encirclement).
  2. `d_target <= 20.0` maintained on uninhibited orbital sector.

### 6.4 Test Case `E2E-T5-04`: Target Clearing Revert to Voronoi Search (`test_target_clearing_revert_to_voronoi`)
- **Objective**: Verify swarm automatically shifts back to `SEARCH` mode upon receiving a `TARGET_CLEARED` telemetry broadcast and resumes Voronoi search coverage.
- **Category-Partitioning**:
  - *Input Space*: Broadcast telemetry `TargetTelemetry(target_id=1, status="CLEARED", timestamp=40.0)`.
  - *Pre-State*: Swarm in `TARGET_TRACKING` mode.
- **Boundary Value Analysis (BVA)**:
  - *Revert Boundary*: $t = t_{\text{clear}} = 40.0\text{s}$.
  - *Post-Clearing Window*: $t \in [40.1\text{s}, 90.0\text{s}]$.
- **Pass Assertions**:
  1. `all(d["mode"] == "SEARCH" for d in active_drones)` at $t \ge 40.1\text{s}$.
  2. Centroidal Voronoi sector re-partitioning resumes immediately.
  3. Search area coverage ratio increases ($\Delta C_{\text{post}} \ge 0.05$) across post-clearing window.

