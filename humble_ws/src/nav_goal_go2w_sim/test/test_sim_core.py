import math

import numpy as np
import pytest

from nav_goal_go2w_sim.sim_core import (
    OCCUPIED,
    Pose2D,
    Twist2D,
    World2D,
    check_collision,
    clamp_twist,
    integrate_omni,
    raycast_pointcloud,
    raycast_scan,
    realized_twist,
    stamp_disc,
)


def _empty_world(width=10, height=10, resolution=1.0):
    return World2D(
        grid=np.zeros((height, width), dtype=np.uint8),
        resolution=resolution,
        origin=(0.0, 0.0),
    )


def _wall_world():
    world = _empty_world()
    world.grid[:, 7] = OCCUPIED
    return world


def test_clamp_twist_limits_each_axis():
    twist = clamp_twist(
        Twist2D(2.0, -3.0, 4.0),
        vx_max=0.3,
        vy_max=0.2,
        wz_max=0.5,
    )

    assert twist == Twist2D(0.3, -0.2, 0.5)


def test_collision_checks_robot_footprint():
    world = _empty_world()
    world.grid[4, 4] = OCCUPIED
    footprint = [(0.35, 0.215), (0.35, -0.215), (-0.35, -0.215), (-0.35, 0.215)]

    assert check_collision(world, Pose2D(4.5, 4.5, 0.0), footprint)
    assert not check_collision(world, Pose2D(1.5, 1.5, 0.0), footprint)


def test_collision_rotates_rectangular_footprint():
    world = _empty_world(width=60, height=60, resolution=0.05)
    cell = world.world_to_grid(1.5, 1.2)
    assert cell is not None
    world.grid[cell[1], cell[0]] = OCCUPIED
    footprint = [(0.35, 0.215), (0.35, -0.215), (-0.35, -0.215), (-0.35, 0.215)]

    assert check_collision(world, Pose2D(1.5, 1.5, math.pi / 2.0), footprint)
    assert not check_collision(world, Pose2D(1.5, 1.5, 0.0), footprint)


def test_omni_integration_rotates_body_velocity_into_world():
    pose = integrate_omni(
        Pose2D(0.0, 0.0, math.pi / 2.0),
        Twist2D(1.0, 0.0, 0.0),
        2.0,
    )

    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.yaw == pytest.approx(math.pi / 2.0)


def test_raycast_observation_reports_hit_cells():
    result = raycast_scan(
        _wall_world(),
        Pose2D(2.5, 5.5, 0.0),
        angle_min=0.0,
        angle_max=0.0,
        angle_inc=1.0,
        range_min=0.0,
        range_max=10.0,
    )

    assert (7, 5) in result.hit_cells


def test_raycast_reports_obstacle_range_and_seen_free_cells():
    result = raycast_scan(
        _wall_world(),
        Pose2D(2.5, 5.5, 0.0),
        angle_min=0.0,
        angle_max=0.0,
        angle_inc=1.0,
        range_min=0.0,
        range_max=10.0,
    )

    assert result.ranges[0] == pytest.approx(4.5, abs=0.51)
    assert (3, 5) in result.seen_free_cells
    assert (6, 5) in result.seen_free_cells


def test_stamp_disc_marks_circle_in_grid():
    grid = np.zeros((20, 20), dtype=np.uint8)
    stamp_disc(
        grid,
        resolution=0.5,
        origin=(0.0, 0.0),
        x=5.0,
        y=5.0,
        radius=0.5,
    )
    center_cell = (10, 10)
    assert grid[center_cell[1], center_cell[0]] == OCCUPIED
    assert grid[0, 0] == 0
    assert grid[19, 19] == 0


def test_realized_twist_reports_actual_body_frame_motion():
    twist = realized_twist(
        Pose2D(0.0, 0.0, math.pi / 2.0),
        Pose2D(0.0, 1.0, math.pi),
        1.0,
    )

    assert twist.vx == pytest.approx(1.0)
    assert twist.vy == pytest.approx(0.0)
    assert twist.wz == pytest.approx(math.pi / 2.0)


def test_raycast_pointcloud_shape_and_bounds():
    world = _wall_world()

    points = raycast_pointcloud(world, Pose2D(5.0, 5.0, 0.0), {"range_max": 3.0})

    assert points.dtype == np.float32
    assert points.ndim == 2
    assert points.shape[1] == 3
    assert points.shape[0] > 0
    assert points[:, 2].min() >= 0.0
    assert points[:, 2].max() <= 0.85 + 1.0e-6


def test_raycast_pointcloud_with_elevation_ramp():
    grid = np.zeros((20, 20), dtype=np.uint8)
    elevation = np.tile(np.arange(20, dtype=np.float32) * 0.01, (20, 1))
    world = World2D(grid=grid, resolution=0.1, origin=(0.0, 0.0), elevation=elevation)

    points = raycast_pointcloud(
        world,
        Pose2D(1.0, 1.0, 0.0),
        {
            "range_max": 1.0,
            "num_rings": 1,
            "vfov_deg": 30.0,
            "hfov_deg": 1.0,
            "az_step_deg": 1.0,
            "sensor_height_m": 0.05,
        },
    )
    assert points.shape[0] == 1
    cell = world.world_to_grid(float(points[0, 0]), float(points[0, 1]))
    assert cell is not None

    assert points[0, 2] == pytest.approx(world.elevation_at_cell(*cell), abs=1.0e-6)


def test_raycast_pointcloud_occlusion():
    world = _empty_world(width=12, height=12, resolution=1.0)
    wall_gx = 5
    world.grid[:, wall_gx] = OCCUPIED

    points = raycast_pointcloud(
        world,
        Pose2D(2.5, 5.5, 0.0),
        {
            "range_max": 8.0,
            "wall_z_max": 3.0,
        },
    )
    wall_x_min = world.origin[0] + wall_gx * world.resolution

    assert points.shape[0] > 0
    assert points[:, 0].max() <= wall_x_min + world.resolution + 1.0e-6


def test_apply_odom_drift_identity_when_zero():
    from nav_goal_go2w_sim.sim_core import Twist2D, apply_odom_drift

    twist = Twist2D(0.3, 0.1, 0.2)
    out = apply_odom_drift(twist, 0.0, 0.0)
    assert (out.vx, out.vy, out.wz) == (0.3, 0.1, 0.2)


def test_apply_odom_drift_accumulates_with_distance():
    import math

    from nav_goal_go2w_sim.sim_core import (
        Pose2D,
        Twist2D,
        apply_odom_drift,
        integrate_omni,
    )

    # Drive straight 10 m at 0.5 m/s; odometry should over-report x by 2 %
    # and accumulate 0.02 rad of heading error per meter traveled.
    true_pose = Pose2D(0.0, 0.0, 0.0)
    odom_pose = Pose2D(0.0, 0.0, 0.0)
    dt = 0.02
    twist = Twist2D(0.5, 0.0, 0.0)
    for _ in range(1000):  # 20 s
        true_pose = integrate_omni(true_pose, twist, dt)
        odom_pose = integrate_omni(
            odom_pose, apply_odom_drift(twist, 0.02, 0.02), dt
        )
    assert true_pose.x == pytest.approx(10.0)
    assert odom_pose.yaw == pytest.approx(0.02 * 10.2, rel=0.05)
    assert odom_pose.x != pytest.approx(true_pose.x, abs=0.05)
    assert math.hypot(odom_pose.x - true_pose.x, odom_pose.y - true_pose.y) > 0.1


def test_world_points_to_drifted_odom_roundtrip():
    import numpy as np

    from nav_goal_go2w_sim.sim_core import Pose2D, world_points_to_drifted_odom

    points = np.array([[2.0, 1.0, 0.3], [0.0, 0.0, 0.0]], dtype=np.float32)
    true_pose = Pose2D(1.0, 1.0, 0.5)
    # No drift: odom pose equals true pose -> identity.
    same = world_points_to_drifted_odom(points, true_pose, true_pose)
    np.testing.assert_allclose(same, points, atol=1e-6)

    # With drift, the point's position RELATIVE TO THE ROBOT must be
    # preserved (same range/bearing in the body frame).
    odom_pose = Pose2D(1.3, 0.9, 0.6)
    moved = world_points_to_drifted_odom(points, true_pose, odom_pose)

    def body(points_xy, pose):
        import math as m

        cos_yaw, sin_yaw = m.cos(-pose.yaw), m.sin(-pose.yaw)
        x = points_xy[:, 0] - pose.x
        y = points_xy[:, 1] - pose.y
        return np.column_stack((cos_yaw * x - sin_yaw * y, sin_yaw * x + cos_yaw * y))

    np.testing.assert_allclose(
        body(points, true_pose), body(moved, odom_pose), atol=1e-5
    )
    np.testing.assert_allclose(moved[:, 2], points[:, 2], atol=1e-6)
