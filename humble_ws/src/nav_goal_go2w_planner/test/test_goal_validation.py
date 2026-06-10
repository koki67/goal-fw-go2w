"""Unit tests for active-goal reachability validation."""
import pytest

from nav_goal_go2w_planner.goal_validation import (
    Pose2D,
    ValidationGrid,
    validate_goal_reachable,
    world_to_cell,
)


def _grid(rows: list[list[int]], resolution: float = 1.0):
    height = len(rows)
    width = len(rows[0]) if height else 0
    data = [value for row in rows for value in row]
    return data, ValidationGrid(width=width, height=height, resolution=resolution, origin_x=0.0, origin_y=0.0)


def test_world_to_cell_handles_bounds():
    _, grid = _grid([[0, 0], [0, 0]])

    assert world_to_cell(grid, 0.5, 1.5) == (0, 1)
    assert world_to_cell(grid, -0.1, 0.5) is None


def test_goal_reachable_through_free_cells_passes():
    data, grid = _grid([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ])

    result = validate_goal_reachable(
        data, grid, Pose2D(0.5, 0.5), Pose2D(3.5, 0.5), connectivity=4,
    )

    assert result.valid
    assert result.reason == "reachable"


def test_goal_behind_lethal_wall_fails():
    data, grid = _grid([
        [0, 99, 0],
        [0, 99, 0],
        [0, 99, 0],
    ])

    result = validate_goal_reachable(
        data,
        grid,
        Pose2D(0.5, 1.5),
        Pose2D(2.5, 1.5),
        reachable_cost_threshold=80,
        connectivity=8,
    )

    assert not result.valid
    assert result.reason == "goal_unreachable"


def test_unknown_goal_cell_fails_before_reachability_search():
    data, grid = _grid([[0, 0, -1]])

    result = validate_goal_reachable(
        data, grid, Pose2D(0.5, 0.5), Pose2D(2.5, 0.5), reachable_cost_threshold=80,
    )

    assert not result.valid
    assert result.reason == "goal_not_traversable"


def test_moderate_cost_cells_are_reachable_under_threshold():
    data, grid = _grid([[0, 40, 40, 0]])

    result = validate_goal_reachable(
        data,
        grid,
        Pose2D(0.5, 0.5),
        Pose2D(3.5, 0.5),
        reachable_cost_threshold=80,
        connectivity=4,
    )

    assert result.valid


def test_robot_seed_search_handles_unknown_robot_cell():
    data, grid = _grid([[-1, 0, 0]])

    result = validate_goal_reachable(
        data,
        grid,
        Pose2D(0.5, 0.5),
        Pose2D(2.5, 0.5),
        reachable_cost_threshold=80,
        seed_search_radius=1,
        connectivity=4,
    )

    assert result.valid
    assert result.robot_seed_cell == (1, 0)


def test_invalid_connectivity_raises():
    data, grid = _grid([[0]])

    with pytest.raises(ValueError, match="connectivity"):
        validate_goal_reachable(data, grid, Pose2D(0.5, 0.5), Pose2D(0.5, 0.5), connectivity=5)


def test_unknown_goal_rejected_by_default():
    data, grid = _grid([[0, 0, -1]])

    result = validate_goal_reachable(
        data, grid, Pose2D(0.5, 0.5), Pose2D(2.5, 0.5), connectivity=4
    )

    assert not result.valid
    assert result.reason == "goal_not_traversable"


def test_unknown_goal_accepted_with_treat_unknown_as_reachable():
    data, grid = _grid([[0, 0, -1]])

    result = validate_goal_reachable(
        data,
        grid,
        Pose2D(0.5, 0.5),
        Pose2D(2.5, 0.5),
        connectivity=4,
        treat_unknown_as_reachable=True,
    )

    assert result.valid


def test_unknown_corridor_connects_goal_when_unknown_reachable():
    # Free | unknown gap | free: connectivity must pass through the unknown.
    data, grid = _grid([[0, -1, -1, 0]])

    strict = validate_goal_reachable(
        data, grid, Pose2D(0.5, 0.5), Pose2D(3.5, 0.5), connectivity=4
    )
    assert not strict.valid
    assert strict.reason == "goal_unreachable"

    relaxed = validate_goal_reachable(
        data,
        grid,
        Pose2D(0.5, 0.5),
        Pose2D(3.5, 0.5),
        connectivity=4,
        treat_unknown_as_reachable=True,
    )
    assert relaxed.valid


def test_occupied_goal_still_rejected_with_unknown_reachable():
    data, grid = _grid([[0, 0, 100]])

    result = validate_goal_reachable(
        data,
        grid,
        Pose2D(0.5, 0.5),
        Pose2D(2.5, 0.5),
        connectivity=4,
        treat_unknown_as_reachable=True,
    )

    assert not result.valid
    assert result.reason == "goal_not_traversable"
