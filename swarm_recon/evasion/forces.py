"""
EvaderForces — Stateless force computation primitives for swarm evasion.

All methods are static and pure: they take positions/states and return
force vectors. No side effects, no shared state.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

from swarm_recon.config import ThreatZone, DroneState


class EvaderForces:
    """
    Collection of static force-computation methods used by SwarmAgent.

    Provides:
    - Rotational APF threat repulsion
    - Boids inter-drone separation
    - Sector-attraction force
    - Stochastic heading perturbation
    - Speed clamping
    """

    _APF_INFLUENCE = 15.0   # Influence distance beyond threat radius (m)
    _APF_MAX_FORCE = 35.0   # Max force contribution per threat

    @staticmethod
    def rotational_apf(
        px: float,
        py: float,
        threats: List[ThreatZone],
    ) -> Tuple[float, float]:
        """
        Compute total Rotational APF force from all threat zones.

        For each threat zone within influence range, computes:
        - Radial repulsion (push directly away from threat center)
        - Tangential rotation (orbit around threat)
        Combined with severity scaling. Per-threat force capped at APF_MAX_FORCE.

        Args:
            px: Drone x-position.
            py: Drone y-position.
            threats: List of ThreatZone objects.

        Returns:
            (fx, fy) total APF force vector.
        """
        fx, fy = 0.0, 0.0
        for threat in threats:
            tcx, tcy = threat.center
            d_center = math.hypot(px - tcx, py - tcy)
            clearance = d_center - threat.radius

            if clearance > EvaderForces._APF_INFLUENCE or d_center < 1e-6:
                continue

            # Radial unit vector (away from threat center)
            rx = (px - tcx) / d_center
            ry = (py - tcy) / d_center
            # Tangential unit vector (perpendicular, for orbital motion)
            tx_ = -ry
            ty_ = rx

            delta = max(0.1, clearance)
            mag = min(EvaderForces._APF_MAX_FORCE, threat.severity * 10.0 / (delta ** 1.1))

            fx += mag * (rx + 1.5 * tx_)
            fy += mag * (ry + 1.5 * ty_)

        return fx, fy

    @staticmethod
    def boids_separation(
        px: float,
        py: float,
        peers: List[DroneState],
        separation_dist: float = 4.0,
        strength: float = 1.5,
    ) -> Tuple[float, float]:
        """
        Compute Boids separation force from nearby peer drones.

        Pushes drone away from any active peer within `separation_dist`.

        Args:
            px: Drone x-position.
            py: Drone y-position.
            peers: List of peer DroneState objects (excluding self).
            separation_dist: Distance threshold for separation (m).
            strength: Separation force multiplier.

        Returns:
            (fx, fy) Boids separation force vector.
        """
        fx, fy = 0.0, 0.0
        for peer in peers:
            if not peer.active:
                continue
            dx = px - peer.position[0]
            dy = py - peer.position[1]
            dist = math.hypot(dx, dy)
            if 1e-3 < dist < separation_dist:
                mag = strength * (separation_dist - dist) / dist
                fx += dx * mag
                fy += dy * mag
        return fx, fy

    @staticmethod
    def sector_attraction(
        px: float,
        py: float,
        target_x: float,
        target_y: float,
        strength: float = 4.0,
    ) -> Tuple[float, float]:
        """
        Compute normalized attraction force toward a sector target point.

        Args:
            px: Drone x-position.
            py: Drone y-position.
            target_x: Target x-coordinate.
            target_y: Target y-coordinate.
            strength: Attraction force magnitude.

        Returns:
            (fx, fy) attraction force vector.
        """
        dx = target_x - px
        dy = target_y - py
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return 0.0, 0.0
        return (dx / dist) * strength, (dy / dist) * strength

    @staticmethod
    def stochastic_perturbation(
        vx: float,
        vy: float,
        t: float,
        drone_id: int,
    ) -> Tuple[float, float]:
        """
        Apply smooth deterministic heading perturbation for trajectory entropy.

        Rotates the velocity vector by a small time-varying angle, creating
        unpredictable but fluid flight trajectories.

        Perturbation angle = 0.25*sin(0.4*t + id*1.7) + 0.15*cos(0.7*t + id*2.3)

        Args:
            vx: Current x-velocity.
            vy: Current y-velocity.
            t: Current simulation time.
            drone_id: Drone identifier (ensures per-drone uniqueness).

        Returns:
            (vx, vy) perturbed velocity vector.
        """
        angle = (
            0.25 * math.sin(0.4 * t + drone_id * 1.7)
            + 0.15 * math.cos(0.7 * t + drone_id * 2.3)
        )
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return vx * cos_a - vy * sin_a, vx * sin_a + vy * cos_a

    @staticmethod
    def clamp_speed(
        vx: float,
        vy: float,
        max_speed: float,
    ) -> Tuple[float, float]:
        """
        Clamp velocity vector magnitude to max_speed.

        Args:
            vx: x-velocity component.
            vy: y-velocity component.
            max_speed: Maximum allowed speed magnitude.

        Returns:
            (vx, vy) clamped velocity.
        """
        speed = math.hypot(vx, vy)
        if speed > max_speed and speed > 1e-9:
            scale = max_speed / speed
            return vx * scale, vy * scale
        return vx, vy

    @staticmethod
    def target_encirclement(
        px: float,
        py: float,
        target_pos: Tuple[float, float],
        peers: List[DroneState],
        standoff_radius: float = 15.0,
    ) -> Tuple[float, float]:
        """
        Compute target encirclement force vector.

        Combines:
        a. Radial standoff attraction/repulsion force to maintain distance ~ standoff_radius.
        b. Tangential orbital drive force (2.0 m/s^2) for orbital circulation.
        c. Peer spacing force along perimeter to maintain even angular spacing.

        Args:
            px: Drone x-position.
            py: Drone y-position.
            target_pos: Target (x, y) position.
            peers: List of peer DroneState objects.
            standoff_radius: Nominal standoff radius in meters (default 15.0).

        Returns:
            (fx, fy) target encirclement force vector.
        """
        tx, ty = float(target_pos[0]), float(target_pos[1])
        dx = px - tx
        dy = py - ty
        r = math.hypot(dx, dy)

        if r < 1e-6:
            r = 1e-6
            dx, dy = 1e-6, 0.0

        # Outward radial unit vector (from target to drone)
        rx = dx / r
        ry = dy / r

        # Inward radial unit vector (from drone to target)
        ux = -rx
        uy = -ry

        # Tangential unit vector (counter-clockwise orbit around target)
        tau_x = -ry
        tau_y = rx

        # a. Radial standoff force:
        # If r > standoff_radius, (r - standoff_radius) > 0 => force in direction of u (inward attraction)
        # If r < standoff_radius, (r - standoff_radius) < 0 => force in direction of u (outward repulsion)
        k_radial = 4.0
        dr = r - standoff_radius
        f_radial_mag = k_radial * dr
        # Cap radial force magnitude to avoid extreme acceleration
        f_radial_mag = max(-35.0, min(35.0, f_radial_mag))
        f_radial_x = f_radial_mag * ux
        f_radial_y = f_radial_mag * uy

        # b. Tangential orbital drive force (2.0 m/s^2)
        k_orbit = 2.0
        f_orbit_x = k_orbit * tau_x
        f_orbit_y = k_orbit * tau_y

        # c. Peer spacing force along perimeter
        my_angle = math.atan2(dy, dx)
        f_spacing_x, f_spacing_y = 0.0, 0.0
        k_spacing = 1.2

        for peer in peers:
            if not peer.active:
                continue
            p_dx = peer.position[0] - tx
            p_dy = peer.position[1] - ty
            p_r = math.hypot(p_dx, p_dy)
            if p_r < 1e-3:
                continue
            peer_angle = math.atan2(p_dy, p_dx)
            diff = (my_angle - peer_angle + math.pi) % (2.0 * math.pi) - math.pi
            if 1e-3 < abs(diff) < 1.5:
                push_dir = 1.0 if diff > 0 else -1.0
                mag = k_spacing * (1.5 - abs(diff)) / max(0.1, abs(diff))
                f_spacing_x += push_dir * mag * tau_x
                f_spacing_y += push_dir * mag * tau_y

        total_fx = f_radial_x + f_orbit_x + f_spacing_x
        total_fy = f_radial_y + f_orbit_y + f_spacing_y

        return total_fx, total_fy

