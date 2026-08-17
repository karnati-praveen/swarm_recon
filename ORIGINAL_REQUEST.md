# Original User Request

## 2026-08-12T05:40:27Z

# Teamwork Project Prompt — Final

> Status: Launched
> Goal: Craft prompt → get user approval → delegate to teamwork_preview

An advanced, next-generation Decentralized Swarm Reconnaissance System. The software will enable a drone swarm to autonomously divide a search area and dynamically reassign sectors if units are lost, utilizing emergent behaviors (e.g., fluid formation flying, predictive threat evasion) that push beyond current standard swarm algorithms.

Working directory: ~/teamwork_projects/swarm_recon
Integrity mode: benchmark

## Requirements

### R1. Decentralized Search & Dynamic Reassignment
Develop a decentralized algorithm that allows a swarm of drones to autonomously divide a given 2D search area. If a drone is lost or disconnected, the remaining drones must dynamically reassign the unsearched sectors among themselves.

### R2. Emergent Evasion Behaviors
Implement unpredictable, fluid flight patterns. Drones must dynamically alter their trajectories to evade simulated threats or avoid predictable patrol routes, while still fulfilling the primary search objective.

### R3. Installation Size Limit
The total size of all installed dependencies and libraries must not exceed 500MB.

## Acceptance Criteria

### R1 Verification (Simulation)
- [ ] A programmatic simulation can spawn $N$ drones in a bounded area.
- [ ] When $K$ drones are randomly removed ("killed") mid-simulation, the remaining drones successfully cover >95% of the search area within a fixed time limit.

### R2 Verification (Trajectory Analysis)
- [ ] An automated script analyzes the drone trajectories and confirms they avoid randomly placed "threat zones" while maintaining formation fluidity.

### R3 Verification (Size Check)
- [ ] A script verifies that the project's dependency folder (e.g., `venv` or `node_modules`) is under 500MB.

## Follow-up — 2026-08-12T15:28:39Z

# Teamwork Project Prompt — Final

> Status: Launched
> Goal: Craft prompt → get user approval → delegate to teamwork_preview

Commercial-grade defense SDK extension for the `swarm_recon` package: implementing "Hunter-Killer" Dynamic Target Handoff ("Target-Triggered Voronoi Collapse") and providing an architectural integration guide for ROS 2 / MAVLink SDK deployment.

Working directory: C:\Users\karna\teamwork_projects\swarm_recon
Integrity mode: benchmark

## Requirements

### R1. Target-Triggered Voronoi Collapse ("Hunter-Killer" Target Handoff)
Implement dynamic multi-mode swarm behavior:
- Default mode: Decentralized Voronoi reconnaissance & search with Rotational APF threat evasion.
- Target mode: When any drone detects a target (simulated signal/location), it broadcasts a "Target Found" telemetry packet.
- Dynamic transition: The swarm automatically collapses area search and shifts into an encircling target tracking formation (maintaining target standoff radius while avoiding threat zones). If the target is cleared, drones automatically revert to Voronoi search.

### R2. ROS 2 / MAVLink SDK Integration Architecture Guide
Provide an architecture guide & code template (`SDK_GUIDE.md` & `swarm_recon/sdk_template.py`) showing how to interface `swarm_recon` with ROS 2 nodes and PX4/ArduPilot MAVLink telemetry (`MAV_CMD_NAV_WAYPOINT`).

### R3. Installation Size Limit
The total size of all installed dependencies and libraries must not exceed 500MB.

## Acceptance Criteria

### R1 Verification (Target Handoff & Encirclement)
- [ ] Programmatic simulation script (`verify_target_handoff.py`) demonstrates swarm transitioning from Voronoi search to target encirclement upon target detection.
- [ ] Encirclement formation maintains target standoff radius (10m - 20m) while avoiding active threat zones.

### R2 Verification (Documentation & Template)
- [ ] `SDK_GUIDE.md` and `swarm_recon/sdk_template.py` exist and detail MAVLink / ROS 2 payload binding logic.

### R3 Verification (Size Check)
- [ ] Script verifies dependency directory size remains under 500MB.

## Follow-up — 2026-08-13T05:36:10Z

# Teamwork Project Prompt — Final

> Status: Launched
> Goal: Craft prompt → get user approval → delegate to teamwork_preview

Implement "Invention 1: RF-Denied Mesh Handoff Protocol" for the `swarm_recon` package. This feature will simulate data-mule telemetry relaying across a decentralized drone mesh network under heavy RF jamming/packet loss, satisfying the final piece of the commercial roadmap.

Working directory: C:\Users\karna\teamwork_projects\swarm_recon
Integrity mode: benchmark
STRICT CONSTRAINT: You MUST use a maximum of 5 subagents AT ONE TIME across your entire orchestration tree. Do not spawn more than 5 total subagents simultaneously.

## Requirements

### R1. RF-Denied Mesh Telemetry Routing
Modify the `SwarmAgent` P2P telemetry logic to support multi-hop mesh routing. 
- Introduce simulated RF jamming/packet loss where direct peer-to-peer ranges are limited.
- Implement a "Data Mule" protocol: If Drone A cannot reach the Base Station (or distant Drone C) directly, it forwards its telemetry packets to an intermediate neighboring Drone B.
- Drone B caches and relays the packets until they reach their destination.

### R2. Installation Size Limit
The total size of all installed dependencies and libraries must not exceed 500MB (keep it pure Python/NumPy).

## Acceptance Criteria

### R1 Verification (Mesh Handoff & Relay)
- [ ] Programmatic simulation script (`verify_mesh_handoff.py`) demonstrates drones successfully passing telemetry packets (e.g., Target Found events) across at least 2 hops (Drone A -> Drone B -> Drone C) when direct communication between A and C is artificially blocked by a simulated jamming radius.
- [ ] Telemetry packets successfully arrive at the destination despite 50% simulated ambient packet drop rates on individual links.

### R2 Verification (Size Check)
- [ ] Script verifies dependency directory size remains under 500MB.
