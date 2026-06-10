import numpy as np

from nav_goal_go2w_map.grid_builder_core import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GridConfig,
    build_grid,
    _label_components,
)


def _floor_points(x0, x1, y0, y1, z=0.0, step=0.04):
    xs = np.arange(x0, x1, step, dtype=np.float32)
    ys = np.arange(y0, y1, step, dtype=np.float32)
    grid_x, grid_y = np.meshgrid(xs, ys)
    return np.column_stack(
        (grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, z, np.float32))
    )


def _wall_points(x, y0, y1, z0=0.0, z1=2.0, step=0.04):
    ys = np.arange(y0, y1, step, dtype=np.float32)
    zs = np.arange(z0, z1, step, dtype=np.float32)
    grid_y, grid_z = np.meshgrid(ys, zs)
    return np.column_stack(
        (np.full(grid_y.size, x, np.float32), grid_y.ravel(), grid_z.ravel())
    )


def _cell(grid, x, y):
    col = int((x - grid.origin_x) / grid.resolution)
    row = int((y - grid.origin_y) / grid.resolution)
    return grid.data[row, col]


def test_floor_is_free_wall_is_occupied():
    cloud = np.vstack(
        (_floor_points(0, 2, 0, 2), _wall_points(2.0, 0, 2))
    )
    grid = build_grid(cloud, GridConfig(resolution=0.1, min_obstacle_cells=1))
    assert _cell(grid, 1.0, 1.0) == FREE
    assert _cell(grid, 2.0, 1.0) == OCCUPIED


def test_cells_without_points_are_unknown():
    grid = build_grid(_floor_points(0, 1, 0, 1), GridConfig(resolution=0.1))
    # Padding region around the cloud has no points.
    assert grid.data[0, 0] == UNKNOWN


def test_sloped_floor_stays_free():
    # 8 degree ramp: rises ~0.14 m per metre; absolute z reaches 0.42 m at
    # x=3, far above obstacle_z_min, but ground-relative it is all floor.
    points = _floor_points(0, 3, 0, 1)
    points[:, 2] = points[:, 0] * np.tan(np.radians(8.0))
    grid = build_grid(points, GridConfig(resolution=0.1, min_obstacle_cells=1))
    assert _cell(grid, 1.5, 0.5) == FREE
    assert _cell(grid, 2.9, 0.5) == FREE


def test_overhang_above_robot_height_is_ignored():
    floor = _floor_points(0, 2, 0, 2)
    ceiling = _floor_points(0.8, 1.2, 0.8, 1.2, z=2.2)
    grid = build_grid(
        np.vstack((floor, ceiling)),
        GridConfig(resolution=0.1, min_obstacle_cells=1),
    )
    assert _cell(grid, 1.0, 1.0) == FREE


def test_low_obstacle_on_floor_is_occupied():
    floor = _floor_points(0, 2, 0, 2)
    box = _floor_points(0.9, 1.1, 0.9, 1.1, z=0.4)
    grid = build_grid(
        np.vstack((floor, box)),
        GridConfig(resolution=0.1, min_obstacle_cells=1),
    )
    assert _cell(grid, 1.0, 1.0) == OCCUPIED


def test_table_top_without_local_floor_is_occupied():
    # The cell under the table top has NO floor returns of its own: ground
    # reference must come from the neighborhood median.
    floor = _floor_points(0, 2, 0, 2)
    under_table = (
        (floor[:, 0] > 0.8)
        & (floor[:, 0] < 1.2)
        & (floor[:, 1] > 0.8)
        & (floor[:, 1] < 1.2)
    )
    floor = floor[~under_table]
    table = _floor_points(0.8, 1.2, 0.8, 1.2, z=0.7)
    grid = build_grid(
        np.vstack((floor, table)),
        GridConfig(resolution=0.1, min_obstacle_cells=1, ground_fill_radius=3),
    )
    assert _cell(grid, 1.0, 1.0) == OCCUPIED


def test_speckle_removal():
    floor = _floor_points(0, 2, 0, 2)
    speckle = np.array([[1.0, 1.0, 0.5]], dtype=np.float32)
    grid = build_grid(
        np.vstack((floor, speckle)),
        GridConfig(resolution=0.1, min_obstacle_cells=3),
    )
    assert _cell(grid, 1.0, 1.0) == FREE


def test_unknown_island_filling():
    floor = _floor_points(0, 2, 0, 2)
    hole = (
        (floor[:, 0] > 0.85)
        & (floor[:, 0] < 1.15)
        & (floor[:, 1] > 0.85)
        & (floor[:, 1] < 1.15)
    )
    cloud = floor[~hole]
    no_fill = build_grid(
        cloud, GridConfig(resolution=0.1, fill_unknown_islands_smaller_than=0)
    )
    assert _cell(no_fill, 1.0, 1.0) == UNKNOWN
    filled = build_grid(
        cloud, GridConfig(resolution=0.1, fill_unknown_islands_smaller_than=10)
    )
    assert _cell(filled, 1.0, 1.0) == FREE


def test_label_components_counts_and_sizes():
    mask = np.zeros((6, 8), dtype=bool)
    mask[0:2, 0:2] = True            # component of 4
    mask[4, 4] = True                # single cell
    mask[5, 5] = True                # diagonal touch -> same component (8-conn)
    labels, sizes = _label_components(mask)
    assert len(sizes) == 2
    assert sorted(sizes.tolist()) == [2, 4]
    assert labels[0, 0] != 0 and labels[4, 4] != 0
    assert labels[4, 4] == labels[5, 5]
    assert labels[3, 3] == 0


def test_label_components_merges_u_shape():
    # U shape forces a union between two provisional labels.
    mask = np.array(
        [
            [1, 0, 1],
            [1, 0, 1],
            [1, 1, 1],
        ],
        dtype=bool,
    )
    labels, sizes = _label_components(mask)
    assert len(sizes) == 1
    assert sizes[0] == 7
