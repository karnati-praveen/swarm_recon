"""
swarm_recon/sdk_template.py — Commercial Defense SDK Integration Template.

Provides ROS 2 node wrapper and MAVLink telemetry bridge binding SwarmAgent
runtime state and waypoints to standard ROS 2 topics and MAVLink v2 messages.

Designed with duck-typing fallback mocks for environments without rclpy or pymavlink,
maintaining the <500MB dependency limit requirement (R3).
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

# Ensure project root is in sys.path when script is executed directly
_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from swarm_recon.config import DroneState, SimulationConfig, ThreatZone, SwarmMode
from swarm_recon.agents.drone import SwarmAgent


# ==============================================================================
# 1. Telemetry Payload Schemas
# ==============================================================================

@dataclass
class TargetTelemetry:
    """Telemetry payload broadcast when a target is detected or updated."""
    target_id: int
    position: Tuple[float, float]
    confidence: float
    timestamp: float
    detecting_drone_id: int
    standoff_radius: float = 15.0
    cleared: bool = False

    def to_dict(self) -> Dict[str, Any]:
        """Serialize payload to JSON-compatible dictionary."""
        return {
            "target_id": int(self.target_id),
            "position": [float(self.position[0]), float(self.position[1])],
            "confidence": float(self.confidence),
            "timestamp": float(self.timestamp),
            "detecting_drone_id": int(self.detecting_drone_id),
            "standoff_radius": float(self.standoff_radius),
            "cleared": bool(self.cleared),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> TargetTelemetry:
        """Instantiate payload from dictionary."""
        pos = data["position"]
        return cls(
            target_id=int(data["target_id"]),
            position=(float(pos[0]), float(pos[1])),
            confidence=float(data["confidence"]),
            timestamp=float(data["timestamp"]),
            detecting_drone_id=int(data["detecting_drone_id"]),
            standoff_radius=float(data.get("standoff_radius", 15.0)),
            cleared=bool(data.get("cleared", False)),
        )


# ==============================================================================
# 2. ROS 2 Node Wrapper / Fallback Mock
# ==============================================================================

class SwarmReconROS2Node:
    """
    ROS 2 Node wrapper interfacing SwarmAgent state with ROS 2 topics.
    Emulates rclpy interface when rclpy is not installed.
    """

    def __init__(self, drone_id: int, node_name: str = "swarm_recon_node") -> None:
        self.drone_id = drone_id
        self.node_name = f"{node_name}_{drone_id}"
        self.published_messages: List[Dict[str, Any]] = []
        self.received_peers: Dict[int, DroneState] = {}
        self.active_target: Optional[TargetTelemetry] = None

    def publish_telemetry(self, state: DroneState) -> Dict[str, Any]:
        """Publish DroneState to ~/telemetry topic (Best Effort QoS)."""
        msg = {
            "topic": f"/drone_{self.drone_id}/telemetry",
            "stamp": time.time(),
            "drone_id": state.id,
            "position": {"x": state.position[0], "y": state.position[1], "z": 10.0},
            "velocity": {"vx": state.velocity[0], "vy": state.velocity[1], "vz": 0.0},
            "heading": state.heading,
            "active": state.active,
            "sector_id": state.sector_id,
            "last_heartbeat": state.last_heartbeat,
        }
        self.published_messages.append(msg)
        return msg

    def publish_target_found(self, telemetry: TargetTelemetry) -> Dict[str, Any]:
        """Publish TargetTelemetry to ~/target_found topic (Reliable QoS)."""
        msg = {
            "topic": f"/drone_{self.drone_id}/target_found",
            "stamp": time.time(),
            "payload": telemetry.to_dict(),
        }
        self.published_messages.append(msg)
        self.active_target = telemetry
        return msg

    def receive_peer_telemetry(self, msg: Dict[str, Any]) -> None:
        """Process received peer telemetry message."""
        peer_id = msg["drone_id"]
        pos = msg["position"]
        vel = msg["velocity"]
        state = DroneState(
            id=peer_id,
            position=(pos["x"], pos["y"]),
            velocity=(vel["vx"], vel["vy"]),
            heading=msg.get("heading", 0.0),
            active=msg.get("active", True),
            sector_id=msg.get("sector_id"),
            last_heartbeat=msg.get("last_heartbeat", time.time()),
        )
        self.received_peers[peer_id] = state

    def receive_target_broadcast(self, msg: Dict[str, Any]) -> None:
        """Process broadcast target telemetry from peer drone."""
        telemetry = TargetTelemetry.from_dict(msg["payload"])
        if telemetry.cleared:
            self.active_target = None
        else:
            self.active_target = telemetry


# ==============================================================================
# 3. MAVLink Message Bridge (PX4 & ArduPilot)
# ==============================================================================

class MAVLinkBridge:
    """
    MAVLink message builder and telemetry parser for PX4 and ArduPilot integration.
    Generates dictionary and byte structures for MAVLink v2 packets.
    """

    MAV_CMD_NAV_WAYPOINT = 16
    SET_POSITION_TARGET_LOCAL_NED = 84
    MAV_FRAME_LOCAL_NED = 1
    MAV_COMP_ID_ONBOARD_COMPUTER = 191

    def __init__(self, drone_id: int, altitude: float = 10.0) -> None:
        self.drone_id = drone_id
        self.system_id = drone_id + 1
        self.comp_id = self.MAV_COMP_ID_ONBOARD_COMPUTER
        self.altitude = altitude

    def convert_swarm_to_ned(
        self, x_recon: float, y_recon: float, z_recon: float = 10.0
    ) -> Tuple[float, float, float]:
        """Convert Swarm Recon 2D Cartesian (m) to MAVLink Local NED (North, East, Down in meters)."""
        x_ned = y_recon  # North
        y_ned = x_recon  # East
        z_ned = -z_recon # Down
        return x_ned, y_ned, z_ned

    def encode_heartbeat(self) -> Dict[str, Any]:
        """Generate 1 Hz HEARTBEAT packet (#0)."""
        return {
            "msgid": 0,
            "name": "HEARTBEAT",
            "system_id": self.system_id,
            "component_id": self.comp_id,
            "type": 18,         # MAV_TYPE_ONBOARD_CONTROLLER
            "autopilot": 8,    # MAV_AUTOPILOT_INVALID
            "base_mode": 1,    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            "custom_mode": 0,
            "system_status": 4, # MAV_STATE_ACTIVE
        }

    def encode_nav_waypoint(
        self, target_x: float, target_y: float, yaw: float = 0.0, hold_time: float = 0.0
    ) -> Dict[str, Any]:
        """Generate MAV_CMD_NAV_WAYPOINT command (#16) for autopilot waypoint navigation."""
        x_ned, y_ned, z_ned = self.convert_swarm_to_ned(target_x, target_y, self.altitude)
        return {
            "msgid": 76,  # COMMAND_LONG
            "name": "COMMAND_LONG",
            "target_system": self.system_id,
            "target_component": 1,  # MAV_COMP_ID_AUTOPILOT1
            "command": self.MAV_CMD_NAV_WAYPOINT,
            "confirmation": 0,
            "param1": float(hold_time), # Hold time (s)
            "param2": 1.0,               # Accept radius (m)
            "param3": 0.0,               # Pass radius (m)
            "param4": float(yaw),        # Yaw angle
            "param5": float(x_ned),      # Local X (North)
            "param6": float(y_ned),      # Local Y (East)
            "param7": float(z_ned),      # Local Z (Down)
        }

    def encode_set_position_target_local_ned(
        self, x: float, y: float, vx: float = 0.0, vy: float = 0.0, yaw: float = 0.0
    ) -> Dict[str, Any]:
        """Generate SET_POSITION_TARGET_LOCAL_NED message (#84) for 10 Hz position/velocity control."""
        x_ned, y_ned, z_ned = self.convert_swarm_to_ned(x, y, self.altitude)
        vx_ned, vy_ned, vz_ned = vy, vx, 0.0
        return {
            "msgid": 84,
            "name": "SET_POSITION_TARGET_LOCAL_NED",
            "time_boot_ms": int(time.time() * 1000) & 0xFFFFFFFF,
            "target_system": self.system_id,
            "target_component": 1,
            "coordinate_frame": self.MAV_FRAME_LOCAL_NED,
            "type_mask": 0b0000101111000000, # Position + Velocity + Yaw enable (0x0BC0)
            "x": float(x_ned),
            "y": float(y_ned),
            "z": float(z_ned),
            "vx": float(vx_ned),
            "vy": float(vy_ned),
            "vz": float(vz_ned),
            "yaw": float(yaw),
        }

    def parse_local_position_ned(self, msg: Dict[str, Any]) -> Tuple[float, float, float, float]:
        """Parse autopilot LOCAL_POSITION_NED feedback into Swarm 2D Cartesian (x, y, vx, vy)."""
        x_ned = msg.get("x", 0.0)
        y_ned = msg.get("y", 0.0)
        vx_ned = msg.get("vx", 0.0)
        vy_ned = msg.get("vy", 0.0)
        # Convert NED to Swarm Cartesian: x_recon = y_ned, y_recon = x_ned
        return y_ned, x_ned, vy_ned, vx_ned


# ==============================================================================
# 4. Unified Swarm SDK Adapter
# ==============================================================================

class SwarmSDKAdapter:
    """
    Unified Integration Adapter connecting a SwarmAgent instance to ROS 2 & MAVLink.
    Handles mode transitions, waypoint generation, and target encirclement telemetry.
    """

    def __init__(self, agent: SwarmAgent, altitude: float = 10.0) -> None:
        self.agent = agent
        self.ros_node = SwarmReconROS2Node(drone_id=agent.drone_id)
        self.mavlink_bridge = MAVLinkBridge(drone_id=agent.drone_id, altitude=altitude)
        self.operating_mode: str = "VORONOI_SEARCH"
        self.active_target: Optional[TargetTelemetry] = None

    def trigger_target_found(self, target_pos: Tuple[float, float], target_id: int = 1) -> TargetTelemetry:
        """Trigger local target discovery, broadcast telemetry, and update operating mode."""
        telemetry = TargetTelemetry(
            target_id=target_id,
            position=target_pos,
            confidence=0.98,
            timestamp=time.time(),
            detecting_drone_id=self.agent.drone_id,
            standoff_radius=15.0,
        )
        self.ros_node.publish_target_found(telemetry)
        self.active_target = telemetry
        self.operating_mode = "TARGET_ENCIRCLEMENT"
        self.agent.target_detected = True
        self.agent.detected_target_position = target_pos
        self.agent.detect_target(target_pos, target_id)
        return telemetry

    def trigger_target_cleared(self, target_id: int = 1) -> TargetTelemetry:
        """Trigger local target clearance event, broadcast telemetry, and revert operating mode."""
        telemetry = TargetTelemetry(
            target_id=target_id,
            position=(0.0, 0.0),
            confidence=0.0,
            timestamp=time.time(),
            detecting_drone_id=self.agent.drone_id,
            standoff_radius=15.0,
            cleared=True,
        )
        self.ros_node.publish_target_found(telemetry)
        self.active_target = None
        self.operating_mode = "VORONOI_SEARCH"
        self.agent.target_detected = False
        self.agent.detected_target_position = None
        self.agent.clear_target(target_id)
        return telemetry

    def calculate_encirclement_waypoint(self, current_pos: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate tangential orbit waypoint around target position maintaining standoff radius."""
        if self.active_target is None:
            return current_pos
        tx, ty = self.active_target.position
        r = float(self.active_target.standoff_radius)
        # Strictly clamp standoff radius between 10.0m and 20.0m
        r = min(20.0, max(10.0, r))
        dx = current_pos[0] - tx
        dy = current_pos[1] - ty
        current_angle = math.atan2(dy, dx)
        
        num_drones_cfg = getattr(self.agent, "config", getattr(self.agent, "_config", None))
        num_drones = num_drones_cfg.num_drones if num_drones_cfg else 4
        drone_offset = (2.0 * math.pi / max(1, num_drones)) * self.agent.drone_id

        # Tangential orbit offset step angle with multi-drone spacing offset
        orbit_angle = current_angle + 0.3 + drone_offset
        wx = tx + r * math.cos(orbit_angle)
        wy = ty + r * math.sin(orbit_angle)
        return wx, wy

    def step_and_export_commands(
        self, dt: float, peers: Dict[int, DroneState], threats: List[ThreatZone]
    ) -> Dict[str, Any]:
        """Advance agent state step and export ROS 2 telemetry & MAVLink waypoint command."""
        # Synchronize adapter state with self.ros_node.active_target (handles peer target broadcasts & clear events)
        if self.ros_node.active_target is not None and not self.ros_node.active_target.cleared:
            self.active_target = self.ros_node.active_target
            self.operating_mode = "TARGET_ENCIRCLEMENT"
            self.agent.target_detected = True
            self.agent.detected_target_position = self.active_target.position
            if getattr(self.agent, "mode", None) != SwarmMode.TARGET_TRACKING:
                self.agent.detect_target(self.active_target.position, self.active_target.target_id)
        elif self.ros_node.active_target is None or self.ros_node.active_target.cleared:
            self.active_target = None
            self.operating_mode = "VORONOI_SEARCH"
            self.agent.target_detected = False
            self.agent.detected_target_position = None
            if getattr(self.agent, "mode", None) == SwarmMode.TARGET_TRACKING:
                self.agent.clear_target()

        state = self.agent.get_state()
        
        # Publish state to ROS 2
        ros_telemetry = self.ros_node.publish_telemetry(state)

        # Generate navigation command based on mode
        if self.operating_mode == "TARGET_ENCIRCLEMENT" and self.active_target is not None:
            waypoint = self.calculate_encirclement_waypoint(state.position)
        else:
            waypoint = (state.position[0] + state.velocity[0] * dt, state.position[1] + state.velocity[1] * dt)

        # Build MAVLink Commands
        mav_waypoint = self.mavlink_bridge.encode_nav_waypoint(
            target_x=waypoint[0], target_y=waypoint[1], yaw=state.heading
        )
        mav_position_target = self.mavlink_bridge.encode_set_position_target_local_ned(
            x=waypoint[0], y=waypoint[1], vx=state.velocity[0], vy=state.velocity[1], yaw=state.heading
        )

        return {
            "drone_id": self.agent.drone_id,
            "mode": self.operating_mode,
            "ros2_telemetry": ros_telemetry,
            "mavlink_waypoint": mav_waypoint,
            "mavlink_position_target": mav_position_target,
            "waypoint": waypoint,
        }


# ==============================================================================
# 5. Executable Demonstration Harness
# ==============================================================================

if __name__ == "__main__":
    print("=== swarm_recon Commercial Defense SDK Integration Template ===")
    config = SimulationConfig(width=100.0, height=100.0, num_drones=4)
    agent = SwarmAgent(drone_id=0, config=config, initial_position=(10.0, 10.0))
    adapter = SwarmSDKAdapter(agent=agent)

    print(f"[*] Initialized Adapter for Drone {agent.drone_id} in Mode: {adapter.operating_mode}")
    
    # 1. Step Voronoi Search Mode
    output = adapter.step_and_export_commands(dt=0.1, peers={}, threats=[])
    print(f"[*] Voronoi Waypoint: {output['waypoint']}")
    print(f"[*] MAVLink MAV_CMD_NAV_WAYPOINT Command: {output['mavlink_waypoint']['command']} -> Param5 (X_NED): {output['mavlink_waypoint']['param5']:.2f}, Param6 (Y_NED): {output['mavlink_waypoint']['param6']:.2f}")

    # 2. Trigger Target Found Event ("Hunter-Killer" Target Handoff)
    print("\n[*] Target Detected at (50.0, 50.0)! Broadcasting TargetTelemetry...")
    adapter.trigger_target_found(target_pos=(50.0, 50.0), target_id=1)
    
    # 3. Step Encirclement Mode
    output_encircling = adapter.step_and_export_commands(dt=0.1, peers={}, threats=[])
    print(f"[*] Target Encirclement Mode Active!")
    print(f"[*] Encirclement Orbit Waypoint: {output_encircling['waypoint']}")
    print(f"[*] ROS 2 Target Telemetry Published: {adapter.ros_node.published_messages[-1]['topic']}")
    print("=== SDK Integration Template Verification Complete ===")
