"""Pure helpers for validating active navigation goals against an OccupancyGrid."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional, Sequence


Cell = tuple[int, int]


@dataclass(frozen=True)
class ValidationGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float


@dataclass(frozen=True)
class GoalValidationResult:
    valid: bool
    reason: str
    robot_seed_cell: Optional[Cell] = None
    goal_cell: Optional[Cell] = None


def validate_goal_reachable(
    map_data: Sequence[int],
    grid: ValidationGrid,
    robot_pose: Pose2D,
    goal_pose: Pose2D,
    *,
    reachable_cost_threshold: int = 0,
    seed_search_radius: int = 3,
    connectivity: int = 8,
    treat_unknown_as_reachable: bool = False,
) -> GoalValidationResult:
    """Validate that goal is traversable and connected to the robot seed cell.

    With ``treat_unknown_as_reachable`` unknown cells (-1) pass the
    traversability check: on a pre-built (incomplete) static map, goals at or
    beyond the mapped boundary must stay valid as long as NavFn plans with
    allow_unknown; only genuinely occupied space invalidates a goal.
    """

    _validate_grid(map_data, grid, connectivity)
    goal_cell = world_to_cell(grid, goal_pose.x, goal_pose.y)
    if goal_cell is None:
        return GoalValidationResult(False, "goal_out_of_bounds")
    if not _is_traversable(
        _value_at(map_data, grid, goal_cell),
        reachable_cost_threshold,
        treat_unknown_as_reachable,
    ):
        return GoalValidationResult(False, "goal_not_traversable", goal_cell=goal_cell)

    robot_seed = find_nearby_traversable_seed(
        map_data,
        grid,
        robot_pose,
        max_radius_cells=seed_search_radius,
        reachable_cost_threshold=reachable_cost_threshold,
        treat_unknown_as_reachable=treat_unknown_as_reachable,
    )
    if robot_seed is None:
        return GoalValidationResult(False, "robot_seed_not_found", goal_cell=goal_cell)
    if robot_seed == goal_cell:
        return GoalValidationResult(True, "reachable", robot_seed, goal_cell)

    if _can_reach_cell(
        map_data,
        grid,
        robot_seed,
        goal_cell,
        reachable_cost_threshold=reachable_cost_threshold,
        connectivity=connectivity,
        treat_unknown_as_reachable=treat_unknown_as_reachable,
    ):
        return GoalValidationResult(True, "reachable", robot_seed, goal_cell)
    return GoalValidationResult(False, "goal_unreachable", robot_seed, goal_cell)


def find_nearby_traversable_seed(
    map_data: Sequence[int],
    grid: ValidationGrid,
    robot_pose: Pose2D,
    *,
    max_radius_cells: int,
    reachable_cost_threshold: int,
    treat_unknown_as_reachable: bool = False,
) -> Optional[Cell]:
    start = world_to_cell(grid, robot_pose.x, robot_pose.y)
    if start is None:
        return None
    if _is_traversable(
        _value_at(map_data, grid, start),
        reachable_cost_threshold,
        treat_unknown_as_reachable,
    ):
        return start

    best_cell: Optional[Cell] = None
    best_dist_sq: Optional[int] = None
    sx, sy = start
    for radius in range(1, max(0, int(max_radius_cells)) + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                cell = (sx + dx, sy + dy)
                if not _in_bounds(grid, cell):
                    continue
                if not _is_traversable(
                    _value_at(map_data, grid, cell),
                    reachable_cost_threshold,
                    treat_unknown_as_reachable,
                ):
                    continue
                dist_sq = dx * dx + dy * dy
                if best_dist_sq is None or dist_sq < best_dist_sq:
                    best_cell = cell
                    best_dist_sq = dist_sq
        if best_cell is not None:
            return best_cell
    return None


def world_to_cell(grid: ValidationGrid, x: float, y: float) -> Optional[Cell]:
    dx = x - grid.origin_x
    dy = y - grid.origin_y
    cy = math.cos(grid.origin_yaw)
    sy = math.sin(grid.origin_yaw)
    local_x = cy * dx + sy * dy
    local_y = -sy * dx + cy * dy
    cell = (
        int(math.floor(local_x / grid.resolution)),
        int(math.floor(local_y / grid.resolution)),
    )
    return cell if _in_bounds(grid, cell) else None


def _can_reach_cell(
    map_data: Sequence[int],
    grid: ValidationGrid,
    seed_cell: Cell,
    goal_cell: Cell,
    *,
    reachable_cost_threshold: int,
    connectivity: int,
    treat_unknown_as_reachable: bool = False,
) -> bool:
    offsets = _neighbor_offsets(connectivity)
    visited: set[Cell] = {seed_cell}
    queue: deque[Cell] = deque([seed_cell])
    while queue:
        cell = queue.popleft()
        for dx, dy in offsets:
            neighbour = (cell[0] + dx, cell[1] + dy)
            if neighbour in visited or not _in_bounds(grid, neighbour):
                continue
            if not _is_traversable(
                _value_at(map_data, grid, neighbour),
                reachable_cost_threshold,
                treat_unknown_as_reachable,
            ):
                continue
            if neighbour == goal_cell:
                return True
            visited.add(neighbour)
            queue.append(neighbour)
    return False


def _is_traversable(
    value: int, threshold: int, treat_unknown_as_reachable: bool = False
) -> bool:
    if treat_unknown_as_reachable and value == -1:
        return True
    if threshold > 0:
        return 0 <= value < threshold
    return value == 0


def _idx(grid: ValidationGrid, cell: Cell) -> int:
    return cell[1] * grid.width + cell[0]


def _value_at(map_data: Sequence[int], grid: ValidationGrid, cell: Cell) -> int:
    return int(map_data[_idx(grid, cell)])


def _in_bounds(grid: ValidationGrid, cell: Cell) -> bool:
    return 0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height


def _neighbor_offsets(connectivity: int) -> tuple[Cell, ...]:
    if connectivity == 4:
        return ((1, 0), (-1, 0), (0, 1), (0, -1))
    if connectivity == 8:
        return ((1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1))
    raise ValueError("connectivity must be 4 or 8")


def _validate_grid(map_data: Sequence[int], grid: ValidationGrid, connectivity: int) -> None:
    if grid.width <= 0 or grid.height <= 0:
        raise ValueError("grid dimensions must be positive")
    if grid.resolution <= 0.0:
        raise ValueError("grid resolution must be positive")
    if len(map_data) != grid.width * grid.height:
        raise ValueError("map_data length does not match grid dimensions")
    _neighbor_offsets(connectivity)
