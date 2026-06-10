import math

import numpy as np
import pytest

from nav_goal_go2w_localization.localization_core import (
    make_transform,
    transform_delta,
    yaw_rotation,
)
from nav_goal_go2w_localization.registration import (
    MapTarget,
    RegistrationConfig,
)


def _structured_room(rng, n=30000):
    """Room with walls and floor: enough structure to constrain GICP fully."""
    floor = np.column_stack(
        (rng.uniform(0, 10, n), rng.uniform(0, 8, n), rng.normal(0, 0.01, n))
    )
    walls = []
    for x in (0.0, 10.0):
        walls.append(
            np.column_stack(
                (
                    np.full(n // 4, x) + rng.normal(0, 0.01, n // 4),
                    rng.uniform(0, 8, n // 4),
                    rng.uniform(0, 2.5, n // 4),
                )
            )
        )
    for y in (0.0, 8.0):
        walls.append(
            np.column_stack(
                (
                    rng.uniform(0, 10, n // 4),
                    np.full(n // 4, y) + rng.normal(0, 0.01, n // 4),
                    rng.uniform(0, 2.5, n // 4),
                )
            )
        )
    # asymmetric block so yaw is unambiguous
    block = np.column_stack(
        (
            rng.uniform(2, 3, n // 8),
            rng.uniform(2, 4, n // 8),
            rng.uniform(0, 1.0, n // 8),
        )
    )
    return np.vstack([floor, *walls, block])


@pytest.fixture(scope="module")
def room_map():
    rng = np.random.default_rng(11)
    cloud = _structured_room(rng)
    return MapTarget(
        cloud,
        RegistrationConfig(map_voxel_size=0.1, scan_voxel_size=0.1, num_threads=2),
    ), cloud


def test_recovers_known_transform(room_map):
    target, cloud = room_map
    rng = np.random.default_rng(3)
    scan = cloud[rng.choice(len(cloud), 8000, replace=False)]
    # True odom->map offset; scan is expressed in a frame displaced by inv(T).
    T_true = make_transform(yaw_rotation(0.15), np.array([0.4, -0.3, 0.05]))
    scan_local = (scan - T_true[:3, 3]) @ T_true[:3, :3]
    init = make_transform(yaw_rotation(0.05), np.array([0.2, -0.1, 0.0]))
    outcome = target.register(scan_local, init, max_correspondence_distance=2.0)
    assert outcome.converged
    assert outcome.inlier_fraction > 0.8
    translation, rotation = transform_delta(outcome.T_map_odom, T_true)
    assert translation < 0.05
    assert math.degrees(rotation) < 1.0


def test_identity_when_already_aligned(room_map):
    target, cloud = room_map
    rng = np.random.default_rng(4)
    scan = cloud[rng.choice(len(cloud), 5000, replace=False)]
    outcome = target.register(scan, np.eye(4), max_correspondence_distance=1.0)
    assert outcome.converged
    translation, rotation = transform_delta(outcome.T_map_odom, np.eye(4))
    assert translation < 0.02
    assert math.degrees(rotation) < 0.5


def test_vgicp_variant(room_map):
    _, cloud = room_map
    target = MapTarget(
        cloud,
        RegistrationConfig(
            registration_type="VGICP",
            map_voxel_size=0.1,
            scan_voxel_size=0.1,
            num_threads=2,
            vgicp_voxel_resolution=0.5,
        ),
    )
    rng = np.random.default_rng(5)
    scan = cloud[rng.choice(len(cloud), 5000, replace=False)]
    outcome = target.register(scan, np.eye(4), max_correspondence_distance=1.0)
    assert outcome.converged
    translation, _ = transform_delta(outcome.T_map_odom, np.eye(4))
    assert translation < 0.05


def test_source_size_reflects_downsampling(room_map):
    target, cloud = room_map
    outcome = target.register(
        cloud[:500], np.eye(4), max_correspondence_distance=1.0
    )
    assert 0 < outcome.source_size <= 500


def test_rejects_empty_map():
    with pytest.raises(ValueError):
        MapTarget(np.empty((0, 3)))


def test_rejects_bad_registration_type():
    with pytest.raises(ValueError):
        RegistrationConfig(registration_type="NDT")
