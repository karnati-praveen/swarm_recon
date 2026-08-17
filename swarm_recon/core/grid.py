"""
Occupancy grid engine and search space representation for swarm reconnaissance.
"""

import math
from typing import Dict, List, Optional, Tuple
import numpy as np


class GridSearchSpace:
    """
    Discrete 2D occupancy grid tracking coverage, cell visited state,
    coordinate transformations, and Centroidal Voronoi re-partitioning.
    """

    def __init__(self, width: float, height: float, resolution: float = 1.0):
        if not (isinstance(width, (int, float)) and not isinstance(width, bool) and math.isfinite(width) and width > 0.0):
            raise ValueError(f"Width ({width}) must be a positive finite float.")
        if not (isinstance(height, (int, float)) and not isinstance(height, bool) and math.isfinite(height) and height > 0.0):
            raise ValueError(f"Height ({height}) must be a positive finite float.")
        if not (isinstance(resolution, (int, float)) and not isinstance(resolution, bool) and math.isfinite(resolution) and resolution > 0.0):
            raise ValueError(f"Resolution ({resolution}) must be a positive finite float.")

        self._width: float = float(width)
        self._height: float = float(height)
        self._resolution: float = float(resolution)

        self._num_cols: int = int(self._width / self._resolution)
        self._num_rows: int = int(self._height / self._resolution)

        if self._num_cols <= 0 or self._num_rows <= 0:
            raise ValueError(
                f"Grid resolution ({resolution}) is too large for dimensions "
                f"({width}x{height}), resulting in 0 grid cells."
            )

        self._total_cells: int = self._num_rows * self._num_cols
        self._grid: np.ndarray = np.zeros((self._num_rows, self._num_cols), dtype=bool)
        self._visited_count: int = 0

        # Pre-compute cell center coordinate vectors for fast operations
        cols_vec = (np.arange(self._num_cols) + 0.5) * self._resolution
        rows_vec = (np.arange(self._num_rows) + 0.5) * self._resolution
        self._cc_all, self._rr_all = np.meshgrid(cols_vec, rows_vec)

    @property
    def width(self) -> float:
        """Search area width in spatial units."""
        return self._width

    @property
    def height(self) -> float:
        """Search area height in spatial units."""
        return self._height

    @property
    def resolution(self) -> float:
        """Spatial resolution of each grid cell."""
        return self._resolution

    @property
    def num_rows(self) -> int:
        """Number of grid rows (y-axis cells)."""
        return self._num_rows

    @property
    def num_cols(self) -> int:
        """Number of grid columns (x-axis cells)."""
        return self._num_cols

    @property
    def total_cells(self) -> int:
        """Total number of cells in grid matrix."""
        return self._total_cells

    @property
    def visited_count(self) -> int:
        """Count of visited grid cells."""
        return self._visited_count

    @property
    def grid(self) -> np.ndarray:
        """Return a copy of the 2D boolean grid matrix."""
        return self._grid.copy()

    def get_visited_grid(self) -> np.ndarray:
        """Return a copy of the 2D boolean grid matrix."""
        return self._grid.copy()

    def cell_to_world(self, r: int, c: int) -> Tuple[float, float]:
        """Convert cell index (r, c) to cell center world coordinates (x, y)."""
        if isinstance(r, bool) or isinstance(c, bool) or not isinstance(r, (int, np.integer)) or not isinstance(c, (int, np.integer)):
            raise TypeError(f"Cell indices r ({r}) and c ({c}) must be integers.")
        if not self.is_valid_cell(int(r), int(c)):
            raise IndexError(f"Cell index ({r}, {c}) is out of grid bounds ({self._num_rows}, {self._num_cols}).")
        x = (c + 0.5) * self._resolution
        y = (r + 0.5) * self._resolution
        return x, y

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert continuous world coordinates (x, y) to clamped cell indices (r, c)."""
        if not (isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and isinstance(y, (int, float)) and not isinstance(y, bool) and math.isfinite(y)):
            raise ValueError(f"World coordinates x ({x}) and y ({y}) must be finite numbers.")
        c = int(np.floor(x / self._resolution))
        r = int(np.floor(y / self._resolution))
        c = int(np.clip(c, 0, self._num_cols - 1))
        r = int(np.clip(r, 0, self._num_rows - 1))
        return r, c

    def is_valid_cell(self, r: int, c: int) -> bool:
        """Check if grid cell index (r, c) is within valid bounds."""
        if isinstance(r, bool) or isinstance(c, bool) or not isinstance(r, (int, np.integer)) or not isinstance(c, (int, np.integer)):
            return False
        return 0 <= r < self._num_rows and 0 <= c < self._num_cols

    def is_within_bounds(self, x: float, y: float) -> bool:
        """Check if continuous world position (x, y) is within [0, width] x [0, height]."""
        if not (isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and isinstance(y, (int, float)) and not isinstance(y, bool) and math.isfinite(y)):
            return False
        return 0.0 <= x <= self._width and 0.0 <= y <= self._height

    def is_visited(self, r: int, c: int) -> bool:
        """Return True if cell (r, c) is marked visited."""
        if not self.is_valid_cell(r, c):
            raise IndexError(f"Cell index ({r}, {c}) is out of grid bounds.")
        return bool(self._grid[r, c])

    def is_cell_visited(self, r: int, c: int) -> bool:
        """Alias for is_visited(r, c)."""
        return self.is_visited(r, c)

    def mark_visited(self, x: float, y: float, sensor_radius: float) -> int:
        """
        Mark all grid cells within Euclidean distance `sensor_radius` from (x, y) as visited.
        Returns the number of newly visited cells.
        """
        if not isinstance(sensor_radius, (int, float)) or isinstance(sensor_radius, bool) or math.isnan(sensor_radius) or math.isinf(sensor_radius) or sensor_radius < 0.0:
            raise ValueError(f"sensor_radius must be a finite non-negative float, got {sensor_radius}")
        if sensor_radius == 0.0:
            return 0

        if not (isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and isinstance(y, (int, float)) and not isinstance(y, bool) and math.isfinite(y)):
            return 0

        c_min = max(0, int(np.floor((x - sensor_radius) / self._resolution)))
        c_max = min(self._num_cols - 1, int(np.floor((x + sensor_radius) / self._resolution)))
        r_min = max(0, int(np.floor((y - sensor_radius) / self._resolution)))
        r_max = min(self._num_rows - 1, int(np.floor((y + sensor_radius) / self._resolution)))

        if c_min > c_max or r_min > r_max:
            return 0

        cols_sub = (np.arange(c_min, c_max + 1) + 0.5) * self._resolution
        rows_sub = (np.arange(r_min, r_max + 1) + 0.5) * self._resolution
        cc_sub, rr_sub = np.meshgrid(cols_sub, rows_sub)

        dist_sq = (cc_sub - x) ** 2 + (rr_sub - y) ** 2
        mask = dist_sq <= (sensor_radius ** 2)

        subgrid = self._grid[r_min : r_max + 1, c_min : c_max + 1]
        newly_visited = int(np.count_nonzero(~subgrid & mask))
        subgrid |= mask
        self._visited_count += newly_visited

        return newly_visited

    def get_coverage_ratio(self) -> float:
        """Return the fraction of total grid cells visited (visited_count / total_cells)."""
        return float(self._visited_count) / float(self._total_cells)

    def repartition(
        self,
        active_drone_positions: Dict[int, Tuple[float, float]],
        unvisited_only: bool = False,
    ) -> Dict[int, List[Tuple[int, int]]]:
        """
        Assign grid cells (r, c) to the nearest active drone via Voronoi partitioning.
        Returns mapping of drone ID -> list of (r, c) grid cell coordinates.
        """
        if not active_drone_positions:
            return {}

        valid_drone_ids = []
        valid_coords = []

        for d_id, pos in active_drone_positions.items():
            if pos is None or not isinstance(pos, (tuple, list)) or len(pos) != 2:
                continue
            x, y = pos[0], pos[1]
            if isinstance(x, (int, float)) and not isinstance(x, bool) and isinstance(y, (int, float)) and not isinstance(y, bool) and math.isfinite(x) and math.isfinite(y):
                valid_drone_ids.append(d_id)
                valid_coords.append((float(x), float(y)))

        partition: Dict[int, List[Tuple[int, int]]] = {d_id: [] for d_id in active_drone_positions.keys()}

        if not valid_drone_ids:
            return partition

        drone_coords = np.array(valid_coords, dtype=float)

        # Cell centers array of shape (num_rows, num_cols, 2)
        cell_centers = np.stack([self._cc_all, self._rr_all], axis=-1)  # (num_rows, num_cols, 2)

        # Broadcasting distance calculation
        diff = cell_centers[:, :, np.newaxis, :] - drone_coords[np.newaxis, np.newaxis, :, :]
        dists_sq = np.sum(diff ** 2, axis=-1)  # shape (num_rows, num_cols, K)

        nearest_indices = np.argmin(dists_sq, axis=2)  # shape (num_rows, num_cols)

        for idx, drone_id in enumerate(valid_drone_ids):
            condition = nearest_indices == idx
            if unvisited_only:
                condition &= ~self._grid
            r_indices, c_indices = np.where(condition)
            partition[drone_id] = list(zip(r_indices.tolist(), c_indices.tolist()))

        return partition

    def reset(self) -> None:
        """Reset all cells to unvisited state."""
        self._grid.fill(False)
        self._visited_count = 0

    def get_unvisited_cells(self) -> List[Tuple[int, int]]:
        """Return list of (r, c) cell coordinates that remain unvisited."""
        r_indices, c_indices = np.where(~self._grid)
        return list(zip(r_indices.tolist(), c_indices.tolist()))

