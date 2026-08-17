"""
Swarm Reconnaissance System (`swarm_recon`).

A decentralized swarm reconnaissance framework featuring dynamic sector reassignment,
emergent evasion dynamics, and programmatic verification tools.
"""

from swarm_recon.config import (
    DroneState,
    ThreatZone,
    SimulationConfig,
    TrajectoryFrame,
    TrajectoryLog,
)
from swarm_recon.core.grid import GridSearchSpace

__version__ = "0.1.0"
__author__ = "Swarm Recon Team"

__all__ = [
    "DroneState",
    "ThreatZone",
    "SimulationConfig",
    "TrajectoryFrame",
    "TrajectoryLog",
    "GridSearchSpace",
    "__version__",
    "__author__",
]
