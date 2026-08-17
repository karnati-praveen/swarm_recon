"""
Unit tests for swarm_recon/core/grid.py Occupancy Grid Engine.
"""

import numpy as np
import pytest
from swarm_recon.core.grid import GridSearchSpace


class TestGridCreationAndBounds:
    """Tests for grid initialization and coordinate transformations."""

    def test_grid_dimensions(self):
        grid = GridSearchSpace(width=100.0, height=50.0, resolution=1.0)
        assert grid.width == 100.0
        assert grid.height == 50.0
        assert grid.resolution == 1.0
        assert grid.num_rows == 50
        assert grid.num_cols == 100
        assert grid.total_cells == 5000
        assert grid.visited_count == 0
        assert grid.get_coverage_ratio() == 0.0

    def test_grid_resolution_scaling(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=0.5)
        assert grid.num_rows == 20
        assert grid.num_cols == 20
        assert grid.total_cells == 400

    @pytest.mark.parametrize(
        "w, h, res",
        [
            (0.0, 10.0, 1.0),
            (10.0, -5.0, 1.0),
            (10.0, 10.0, 0.0),
            (10.0, 10.0, -1.0),
            (0.5, 0.5, 1.0),  # 0 cells resulting
        ],
    )
    def test_invalid_grid_creation(self, w, h, res):
        with pytest.raises(ValueError):
            GridSearchSpace(width=w, height=h, resolution=res)

    def test_world_to_cell_conversion(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        assert grid.world_to_cell(0.0, 0.0) == (0, 0)
        assert grid.world_to_cell(2.5, 3.7) == (3, 2)
        assert grid.world_to_cell(9.9, 9.9) == (9, 9)

    def test_world_to_cell_clamping(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        # Continuous coords outside [0, 10] get clamped to valid cell range [0, 9]
        assert grid.world_to_cell(-5.0, 5.0) == (5, 0)
        assert grid.world_to_cell(5.0, -10.0) == (0, 5)
        assert grid.world_to_cell(15.0, 20.0) == (9, 9)

    def test_cell_to_world_conversion(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        assert grid.cell_to_world(0, 0) == (0.5, 0.5)
        assert grid.cell_to_world(3, 2) == (2.5, 3.5)

    def test_cell_to_world_invalid(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        with pytest.raises(IndexError):
            grid.cell_to_world(10, 10)
        with pytest.raises(IndexError):
            grid.cell_to_world(-1, 0)

    def test_bounds_and_validity_helpers(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        assert grid.is_within_bounds(5.0, 5.0) is True
        assert grid.is_within_bounds(-0.1, 5.0) is False
        assert grid.is_valid_cell(0, 0) is True
        assert grid.is_valid_cell(10, 5) is False


class TestMarkVisited:
    """Tests for sensing marking and cell coverage accumulation."""

    def test_single_drone_mark_visited_count(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        # Drone at world (5.5, 5.5), radius = 1.0
        # Cell centers within distance 1.0:
        # (5.5, 5.5) dist=0; (4.5, 5.5), (6.5, 5.5), (5.5, 4.5), (5.5, 6.5) dist=1.0
        # Total = 5 cells.
        newly_visited = grid.mark_visited(x=5.5, y=5.5, sensor_radius=1.0)
        assert newly_visited == 5
        assert grid.visited_count == 5
        assert grid.get_coverage_ratio() == pytest.approx(5 / 100.0)

    def test_mark_visited_idempotency(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        first_count = grid.mark_visited(x=5.5, y=5.5, sensor_radius=1.0)
        assert first_count == 5
        second_count = grid.mark_visited(x=5.5, y=5.5, sensor_radius=1.0)
        assert second_count == 0
        assert grid.visited_count == 5

    def test_mark_visited_bounding_box_optimization(self):
        grid = GridSearchSpace(width=20.0, height=20.0, resolution=1.0)
        grid.mark_visited(10.5, 10.5, sensor_radius=2.0)

        # Cell (r=8, c=8) center (8.5, 8.5) -> dist^2 = (8.5-10.5)^2 + (8.5-10.5)^2 = 4 + 4 = 8 > 4 -> False
        cell_8_8 = grid.world_to_cell(8.5, 8.5)
        assert grid.is_visited(*cell_8_8) is False

        # Cell (r=9, c=10) center (10.5, 9.5) -> dist^2 = 0 + 1 = 1 <= 4 -> True
        cell_9_10 = grid.world_to_cell(10.5, 9.5)
        assert grid.is_visited(*cell_9_10) is True

    def test_multiple_drones_mark_visited_overlap(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        c1 = grid.mark_visited(x=3.5, y=5.5, sensor_radius=1.0)  # 5 cells
        c2 = grid.mark_visited(x=4.5, y=5.5, sensor_radius=1.0)  # Overlaps with c1
        assert c1 == 5
        assert 0 < c2 < 5
        assert grid.visited_count == c1 + c2
        assert grid.get_coverage_ratio() == pytest.approx((c1 + c2) / 100.0)


class TestCoverageRatio:
    """Tests for get_coverage_ratio calculation accuracy."""

    def test_coverage_ratio_initial(self):
        grid = GridSearchSpace(width=20.0, height=20.0, resolution=1.0)
        assert grid.get_coverage_ratio() == 0.0

    def test_coverage_ratio_partial(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        grid.mark_visited(1.5, 1.5, sensor_radius=0.5)  # 1 cell
        assert grid.get_coverage_ratio() == pytest.approx(0.01)

    def test_coverage_ratio_full(self):
        grid = GridSearchSpace(width=5.0, height=5.0, resolution=1.0)
        newly_visited = grid.mark_visited(2.5, 2.5, sensor_radius=10.0)
        assert newly_visited == 25
        assert grid.get_coverage_ratio() == 1.0


class TestRepartition:
    """Tests for Centroidal Voronoi repartitioning logic."""

    def test_repartition_single_drone(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        drones = {1: (5.0, 5.0)}
        partitions = grid.repartition(drones)
        assert list(partitions.keys()) == [1]
        assert len(partitions[1]) == 100

    def test_repartition_two_drones_equal_split(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        drones = {1: (2.5, 5.0), 2: (7.5, 5.0)}
        partitions = grid.repartition(drones)
        assert len(partitions[1]) == 50
        assert len(partitions[2]) == 50
        # Drone 1 gets cols 0..4, Drone 2 gets cols 5..9
        for r, c in partitions[1]:
            assert c < 5
        for r, c in partitions[2]:
            assert c >= 5

    def test_repartition_n_drones_quadrants(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        drones = {
            1: (2.5, 2.5),
            2: (7.5, 2.5),
            3: (2.5, 7.5),
            4: (7.5, 7.5),
        }
        partitions = grid.repartition(drones)
        for d_id in range(1, 5):
            assert len(partitions[d_id]) == 25

    def test_repartition_properties(self):
        grid = GridSearchSpace(width=15.0, height=15.0, resolution=1.0)
        drones = {1: (2.0, 3.0), 2: (12.0, 4.0), 3: (7.0, 11.0)}
        partitions = grid.repartition(drones)

        # 1. Exhaustive: sum of partition lengths equals total cells
        total_assigned = sum(len(cells) for cells in partitions.values())
        assert total_assigned == grid.total_cells

        # 2. Disjoint: no duplicate cells across partitions
        all_cells = set()
        for d_id, cells in partitions.items():
            cell_set = set(cells)
            assert len(cell_set) == len(cells)  # No duplicates within drone
            assert all_cells.isdisjoint(cell_set)  # No overlap with other drones
            all_cells.update(cell_set)

        # 3. Distance minimization (Voronoi condition)
        for d_id, cells in partitions.items():
            d_x, d_y = drones[d_id]
            for r, c in cells:
                c_x, c_y = grid.cell_to_world(r, c)
                dist_assigned = (c_x - d_x) ** 2 + (c_y - d_y) ** 2
                for other_id, (o_x, o_y) in drones.items():
                    dist_other = (c_x - o_x) ** 2 + (c_y - o_y) ** 2
                    assert dist_assigned <= dist_other + 1e-9

    def test_repartition_unvisited_only(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        grid.mark_visited(2.5, 5.0, sensor_radius=1.0)  # Mark some cells visited
        drones = {1: (2.5, 5.0), 2: (7.5, 5.0)}
        partitions_all = grid.repartition(drones, unvisited_only=False)
        partitions_unvisited = grid.repartition(drones, unvisited_only=True)

        total_unvisited_assigned = sum(len(c) for c in partitions_unvisited.values())
        assert total_unvisited_assigned == grid.total_cells - grid.visited_count
        assert len(partitions_unvisited[1]) < len(partitions_all[1])


class TestGridEdgeCases:
    """Tests for edge cases and boundary conditions."""

    def test_drone_partially_out_of_bounds(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        newly_visited = grid.mark_visited(x=-0.5, y=5.0, sensor_radius=2.0)
        assert newly_visited > 0
        assert grid.get_coverage_ratio() < 1.0

    def test_drone_completely_out_of_bounds(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        newly_visited = grid.mark_visited(x=-50.0, y=-50.0, sensor_radius=5.0)
        assert newly_visited == 0
        assert grid.get_coverage_ratio() == 0.0

    def test_sensor_radius_larger_than_grid(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        newly_visited = grid.mark_visited(x=5.0, y=5.0, sensor_radius=100.0)
        assert newly_visited == 100
        assert grid.get_coverage_ratio() == 1.0

    def test_sensor_radius_zero(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        newly_visited = grid.mark_visited(x=5.5, y=5.5, sensor_radius=0.0)
        assert newly_visited == 0

    def test_sensor_radius_negative(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        with pytest.raises(ValueError):
            grid.mark_visited(x=5.5, y=5.5, sensor_radius=-1.0)

    def test_repartition_zero_active_drones(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        partitions = grid.repartition({})
        assert partitions == {}

    def test_grid_reset(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        grid.mark_visited(5.0, 5.0, sensor_radius=2.0)
        assert grid.visited_count > 0
        grid.reset()
        assert grid.visited_count == 0
        assert grid.get_coverage_ratio() == 0.0
        assert len(grid.get_unvisited_cells()) == 100

    def test_get_visited_grid_returns_copy(self):
        grid = GridSearchSpace(width=5.0, height=5.0, resolution=1.0)
        grid_arr = grid.get_visited_grid()
        assert isinstance(grid_arr, np.ndarray)
        assert grid_arr.shape == (5, 5)
        # Modifying copy doesn't affect internal state
        grid_arr.fill(True)
        assert grid.visited_count == 0


class TestGridSearchSpaceNaNInfDefense:
    """Adversarial unit tests for NaN/Inf defense and input validation in GridSearchSpace."""

    @pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
    def test_init_nan_inf_rejected(self, bad_val):
        with pytest.raises(ValueError):
            GridSearchSpace(width=bad_val, height=10.0, resolution=1.0)
        with pytest.raises(ValueError):
            GridSearchSpace(width=10.0, height=bad_val, resolution=1.0)
        with pytest.raises(ValueError):
            GridSearchSpace(width=10.0, height=10.0, resolution=bad_val)

    @pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
    def test_mark_visited_nan_inf_radius_rejected(self, bad_val):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        with pytest.raises(ValueError):
            grid.mark_visited(5.0, 5.0, sensor_radius=bad_val)

    @pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
    def test_mark_visited_nan_inf_coords_handled_gracefully(self, bad_val):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        # NaN / Inf coordinates return 0 visited cells gracefully without crashing
        assert grid.mark_visited(x=bad_val, y=5.0, sensor_radius=1.0) == 0
        assert grid.mark_visited(x=5.0, y=bad_val, sensor_radius=1.0) == 0

    @pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
    def test_world_to_cell_nan_inf_rejected(self, bad_val):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        with pytest.raises(ValueError):
            grid.world_to_cell(bad_val, 5.0)
        with pytest.raises(ValueError):
            grid.world_to_cell(5.0, bad_val)

    @pytest.mark.parametrize("bad_val", [float("nan"), float("inf"), float("-inf")])
    def test_cell_to_world_nan_inf_rejected(self, bad_val):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        with pytest.raises((TypeError, IndexError)):
            grid.cell_to_world(bad_val, 0)

    def test_cell_to_world_bool_rejected(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        with pytest.raises(TypeError):
            grid.cell_to_world(True, 0)

    def test_repartition_with_nan_inf_drones(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        drones = {
            1: (2.5, 5.0),
            2: (float("nan"), 5.0),
            3: (7.5, float("inf")),
            4: (7.5, 5.0),
        }
        partitions = grid.repartition(drones)
        # All keys preserved in return mapping
        assert set(partitions.keys()) == {1, 2, 3, 4}
        # Corrupted drones get empty cell lists
        assert partitions[2] == []
        assert partitions[3] == []
        # Valid drones split total 100 cells equally (50 each)
        assert len(partitions[1]) == 50
        assert len(partitions[4]) == 50

    def test_repartition_all_drones_corrupted(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        drones = {
            1: (float("nan"), 5.0),
            2: (7.5, float("inf")),
        }
        partitions = grid.repartition(drones)
        assert partitions == {1: [], 2: []}

    def test_bounds_and_validity_helpers_nan_inf(self):
        grid = GridSearchSpace(width=10.0, height=10.0, resolution=1.0)
        assert grid.is_within_bounds(float("nan"), 5.0) is False
        assert grid.is_within_bounds(5.0, float("inf")) is False
        assert grid.is_valid_cell(True, 5) is False
        assert grid.is_valid_cell(5, float("nan")) is False
