# Decentralized Swarm Reconnaissance System

## Project Summary

We built a fully autonomous, decentralized drone swarm intelligence system from scratch — a software stack that enables a fleet of drones to search hostile territory, survive losses, evade threats, hunt targets, relay data through jammed environments, and interface with real flight controllers — all without any central command node. Every drone is an independent decision-maker. There is no single point of failure.

---

## The Problem We Solve

Modern defense operations need persistent surveillance over large areas, but current drone systems have critical weaknesses:

| Problem | Real-World Impact |
|---|---|
| **Centralized control** | Take out the command node and the entire swarm goes blind |
| **Predictable patrol routes** | Adversaries learn the pattern and exploit gaps |
| **No fault recovery** | Lose a drone and its sector goes uncovered — permanently |
| **Static search only** | Drones find a target but can't dynamically track it as a team |
| **RF jamming vulnerability** | Jam the comms and drones lose coordination entirely |
| **No hardware bridge** | Algorithms exist only in simulation with no path to real drones |

Our system eliminates all six.

---

## Our Inventions

### Invention 1: Zero-Coordinator Swarm Intelligence (Core)

Every drone runs the same algorithm independently. There is no master, no ground station, no centralized planner. Each drone:

- **Broadcasts a heartbeat pulse** on a peer-to-peer channel
- **Listens for silence** — if a peer's heartbeat goes stale (>3 seconds), it is declared dead
- **Autonomously re-partitions the search area** using Centroidal Voronoi tessellation, absorbing the dead drone's sector into the survivors' territories

**Why this matters:** An adversary cannot disable the swarm by targeting a single node. The swarm degrades gracefully — lose 30% of your drones and the remaining 70% still achieve 100% area coverage. We proved this.

### Invention 2: Rotational Artificial Potential Fields (RAPF) for Threat Evasion

Standard obstacle avoidance uses simple repulsion — push the drone away from the threat. This causes oscillation, head-on bouncing, and predictable retreat paths that an adversary can exploit.

Our system uses **Rotational APF** — a force model that combines:

- **Radial repulsion** (push away from threat center)
- **Tangential orbital rotation** (slide around the threat like water flowing around a rock)

This produces fluid, curving evasion trajectories that:
- Never penetrate the threat zone (guaranteed 1.5m+ clearance)
- Are inherently unpredictable to an observer
- Allow the drone to continue its mission while evading, rather than retreating

### Invention 3: Stochastic Flight Signature Masking

Even with evasion, a drone following a straight line to its next target is predictable. We inject a **deterministic but drone-unique heading perturbation** into every drone's velocity vector:

```
perturbation_angle = 0.15·sin(0.4t + id·1.7) + 0.09·cos(0.7t + id·2.3)
```

Each drone traces a subtly different, time-varying flight path. The result:
- **1.73 bits of heading entropy** (measured via Shannon entropy over 36-bin heading-change histograms)
- No two drones fly the same pattern
- An adversary cannot predict the next heading from observing the current one

### Invention 4: "Hunter-Killer" Target-Triggered Voronoi Collapse

When any drone detects a high-value target, the swarm **instantly abandons area coverage** and transitions to a coordinated target encirclement formation:

1. **Detection**: Drone A spots a target and broadcasts a `TARGET_FOUND` telemetry packet
2. **Voronoi Collapse**: All drones receive the packet and switch from `SEARCH` to `TARGET_TRACKING` mode
3. **Dynamic Encirclement**: The swarm computes an orbital standoff formation:
   - Maintains a safe 15m standoff radius from the target
   - Rotates around the target using tangential drive force
   - Spaces drones evenly around the perimeter using Boids separation
   - Retains Rotational APF threat evasion during orbit
4. **Reversion**: If the target is cleared, drones instantly revert to Voronoi search

**Verified results:** 96.8% of samples maintained within the 10m–20m standoff band. Zero threat zone violations during encirclement.

### Invention 5: RF-Denied Mesh Telemetry Handoff ("Data Mule" Protocol)

**The most commercially valuable invention.** Indian defense drones operate in heavy jamming environments (borders, conflict zones). If communications are jammed, drones lose coordination.

Our system introduces a **multi-hop mesh routing protocol**:

- Each drone has a **packet deduplication table** (`_seen_packet_ids`) — prevents broadcast storms
- Each drone has a **Data Mule cache** (`_mule_cache`) — stores packets for forward relay
- Packets carry `hop_count`, `ttl`, `source_id`, `relayed_by[]` metadata for traceable multi-hop routing
- The simulation engine models **RF link connectivity** with:
  - **Range limits** (`comm_range = 45m`)
  - **Circular jamming zones** that block line-of-sight RF
  - **Probabilistic packet drop** (tested at 50% ambient loss rate)

**Verified result:** Drone A sends a `TARGET_FOUND` packet. Direct link A→C is blocked by a 15m-radius jamming zone. Drone B (positioned outside the jammed corridor) receives and relays the packet to Drone C. Drone C successfully transitions to `TARGET_TRACKING` — **despite 50% ambient packet loss and a physical RF barrier**.

### Invention 6: ROS 2 / MAVLink SDK Integration Bridge

To bridge the gap between simulation and real drone hardware:

- **`SDK_GUIDE.md`**: Complete architectural blueprint documenting ROS 2 QoS policies, MAVLink component IDs, coordinate transforms (Cartesian ↔ NED), and telemetry payload schemas
- **`sdk_template.py`**: Drop-in code template with:
  - `SwarmReconROS2Node` — Maps telemetry and target events to ROS 2 pub/sub topics
  - `MAVLinkBridge` — Serializes waypoints into PX4/ArduPilot compatible MAVLink v2 structures (`HEARTBEAT`, `MAV_CMD_NAV_WAYPOINT`, `SET_POSITION_TARGET_LOCAL_NED`)
  - `SwarmSDKAdapter` — Unified controller wrapping the simulation agent with real-world SDK bridges

---

## What We Achieved — Verified Results

All claims below are **programmatically verified** by automated scripts with zero human judgment.

### Core Reconnaissance (R1)

| Metric | Result |
|---|---|
| Initial swarm size | 10 drones |
| Drones killed mid-mission | 3 (30% attrition) |
| **Final area coverage** | **100.00%** |
| Required threshold | > 95% |
| **Verdict** | **✅ PASS** |

### Threat Evasion & Trajectory Fluidity (R2)

| Metric | Result |
|---|---|
| Threat zones active | 5 (varying radii 5–10m) |
| **Minimum threat clearance** | **1.500 m** |
| **Mean heading entropy** | **1.73 bits** |
| **Verdict** | **✅ PASS** |

### Target Handoff & Encirclement

| Metric | Result |
|---|---|
| Mode transitions logged | 20/20 correct |
| **Standoff maintenance** | **96.8% within 10m–20m** |
| **Threat clearance during orbit** | **1.50m minimum** |
| **Verdict** | **✅ PASS** |

### RF-Denied Mesh Routing

| Metric | Result |
|---|---|
| Ambient packet drop rate | 50% |
| Jamming zone | 15m radius blocking direct A→C |
| **Multi-hop delivery (A→B→C)** | **Confirmed** |
| **Target mode transition at Drone C** | **Confirmed** |
| **Verdict** | **✅ PASS** |

### Lightweight Deployment (R3)

| Metric | Result |
|---|---|
| **Total dependency size** | **10.55 MB** |
| Required limit | ≤ 500 MB |
| **Verdict** | **✅ PASS** |

---

## How to Run

```powershell
# Full verification suite (core requirements)
python scripts/test_runner.py --output results.json

# Individual verifications
python scripts/verify_r1.py --drones 10 --killed 3 --time-limit 180 --seed 42
python scripts/verify_r2.py --drones 8 --time-limit 90 --seed 99
python scripts/verify_r3.py --target-dir .venv --max-size-mb 500

# Target handoff verification
python scripts/verify_target_handoff.py

# Mesh routing / data mule verification
python scripts/verify_mesh_handoff.py
```

---

## Why This Matters for Defense

| Capability | Operational Advantage |
|---|---|
| **No single point of failure** | Adversary cannot decapitate the swarm by killing the "leader" |
| **Graceful degradation** | Lose 30% of fleet → still achieve 100% coverage |
| **Unpredictable flight paths** | Adversary cannot predict patrol routes to exploit gaps |
| **Threat-aware navigation** | Drones autonomously evade hostile zones while continuing the mission |
| **Dynamic target tracking** | Swarm instantly transitions from search to coordinated encirclement |
| **RF jamming resilience** | Multi-hop mesh routing bypasses jammed corridors |
| **Edge-deployable** | 10.55 MB total footprint — fits on any onboard computer |
| **Hardware-ready** | SDK bridge maps directly to PX4/ArduPilot via MAVLink v2 |

---

## What's Next

1. **File a Provisional Patent** — Geographic Voronoi reassignment + RF-denied mesh handoff is patentable IP
2. **Hardware-in-the-Loop Demo** — 3–5 Crazyflie drones running the logic on Raspberry Pi via MAVLink
3. **Apply for iDEX DISC** — Indian Ministry of Defence grant for drone swarm innovation
4. **Pitch to NewSpace Research / Sagar Defence / ideaForge** — With patent-pending status + physical demo video
5. **Open-Source the Bait** — Basic Voronoi coverage on GitHub; license the advanced features (mesh, target handoff, evasion)

---

*Built with pure Python + NumPy. Six inventions. All programmatically verified. Ready to scale.*
