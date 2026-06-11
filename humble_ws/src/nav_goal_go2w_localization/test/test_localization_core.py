import math

import numpy as np
import pytest

from nav_goal_go2w_localization.localization_core import (
    LocalizationStateMachine,
    LocalizerConfig,
    LocalizerState,
    RegistrationOutcome,
    blend_transforms,
    flatten_to_yaw,
    invert_transform,
    make_transform,
    quaternion_to_rotation,
    rotation_to_quaternion,
    transform_delta,
    yaw_rotation,
)


def _transform(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return make_transform(yaw_rotation(yaw), np.array([x, y, z]))


def _good(T):
    return RegistrationOutcome(
        T_map_odom=T, converged=True, num_inliers=900, source_size=1000, error=10.0
    )


def _bad(T):
    return RegistrationOutcome(
        T_map_odom=T, converged=True, num_inliers=300, source_size=1000, error=10.0
    )


# -- SE(3) helpers -----------------------------------------------------------

def test_invert_transform():
    T = _transform(1.0, 2.0, 0.5, yaw=0.7)
    np.testing.assert_allclose(T @ invert_transform(T), np.eye(4), atol=1e-12)


def test_quaternion_roundtrip():
    rng = np.random.default_rng(5)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        rotation = quaternion_to_rotation(*q)
        q2 = np.array(rotation_to_quaternion(rotation))
        # q and -q are the same rotation.
        assert min(np.linalg.norm(q2 - q), np.linalg.norm(q2 + q)) < 1e-9


def test_flatten_to_yaw_removes_roll_pitch():
    tilt = quaternion_to_rotation(*np.array([0.1, 0.05, 0.3, 0.94]))
    T = make_transform(tilt, np.array([1.0, 2.0, 0.3]))
    flat = flatten_to_yaw(T)
    # z axis of the flattened rotation is exactly +z
    np.testing.assert_allclose(flat[:3, 2], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(flat[:3, 3], T[:3, 3])


def test_blend_transforms_midpoint():
    a = _transform(0, 0, 0, yaw=0.0)
    b = _transform(2, 0, 0, yaw=1.0)
    mid = blend_transforms(a, b, 0.5)
    translation, rotation = transform_delta(a, mid)
    assert translation == pytest.approx(1.0)
    assert rotation == pytest.approx(0.5)


def test_blend_transforms_wraps_yaw():
    a = _transform(yaw=math.pi - 0.1)
    b = _transform(yaw=-math.pi + 0.1)
    mid = blend_transforms(a, b, 0.5)
    _, rotation = transform_delta(a, mid)
    assert rotation == pytest.approx(0.1, abs=1e-9)


# -- state machine -----------------------------------------------------------

def _initialized_machine(**config_kwargs):
    machine = LocalizationStateMachine(LocalizerConfig(**config_kwargs))
    machine.set_initial_pose(_transform(5.0, 5.0, yaw=0.5), _transform())
    return machine


def test_initial_pose_derives_map_odom():
    machine = LocalizationStateMachine()
    T_map_base = _transform(3.0, 1.0, yaw=0.3)
    T_odom_base = _transform(1.0, 0.0, yaw=0.1)
    machine.set_initial_pose(T_map_base, T_odom_base)
    assert machine.state == LocalizerState.CONVERGING
    np.testing.assert_allclose(
        machine.T_map_odom @ T_odom_base, T_map_base, atol=1e-12
    )


def test_uninitialized_ignores_outcomes():
    machine = LocalizationStateMachine()
    result = machine.update(_good(_transform()))
    assert not result.accepted
    assert machine.state == LocalizerState.UNINITIALIZED


def test_converging_to_tracking_after_good_count():
    machine = _initialized_machine(converge_good_count=3)
    T = machine.T_map_odom.copy()
    for index in range(3):
        result = machine.update(_good(T))
        assert result.accepted
    assert machine.state == LocalizerState.TRACKING


def test_tracking_degrades_then_loses_on_rejects():
    machine = _initialized_machine(
        converge_good_count=1, degraded_after_rejects=2, lost_after_rejects=4
    )
    T = machine.T_map_odom.copy()
    machine.update(_good(T))
    assert machine.state == LocalizerState.TRACKING
    machine.update(_bad(T))
    assert machine.state == LocalizerState.TRACKING
    machine.update(_bad(T))
    assert machine.state == LocalizerState.DEGRADED
    machine.update(_bad(T))
    machine.update(_bad(T))
    assert machine.state == LocalizerState.LOST
    # good registration recovers
    machine.update(_good(T))
    assert machine.state == LocalizerState.TRACKING


def test_registration_errors_degrade_then_lose_tracking():
    machine = _initialized_machine(
        converge_good_count=1, degraded_after_rejects=2, lost_after_rejects=3
    )
    T = machine.T_map_odom.copy()
    machine.update(_good(T))

    result = machine.reject("registration_error(RuntimeError)")
    assert not result.accepted
    assert result.reason == "registration_error(RuntimeError)"
    assert machine.state == LocalizerState.TRACKING

    machine.reject("registration_error(RuntimeError)")
    assert machine.state == LocalizerState.DEGRADED
    machine.reject("registration_error(RuntimeError)")
    assert machine.state == LocalizerState.LOST
    np.testing.assert_allclose(machine.T_map_odom, T)


def test_uninitialized_ignores_registration_errors():
    machine = LocalizationStateMachine()
    result = machine.reject("registration_error(RuntimeError)")
    assert result.reason == "uninitialized"
    assert machine.state == LocalizerState.UNINITIALIZED


def test_rejected_outcome_does_not_move_transform():
    machine = _initialized_machine(converge_good_count=1)
    T = machine.T_map_odom.copy()
    machine.update(_good(T))
    machine.update(_bad(_transform(99.0, 99.0)))
    np.testing.assert_allclose(machine.T_map_odom, T)


def test_quality_gates():
    machine = _initialized_machine()
    T = machine.T_map_odom.copy()
    not_converged = RegistrationOutcome(T, False, 900, 1000, 10.0)
    assert "not_converged" in machine.update(not_converged).reason
    few_points = RegistrationOutcome(T, True, 90, 100, 1.0)
    assert "too_few_points" in machine.update(few_points).reason
    high_error = RegistrationOutcome(T, True, 900, 1000, 500.0)
    assert "high_error" in machine.update(high_error).reason


def test_non_finite_outcomes_are_rejected_without_moving_transform():
    machine = _initialized_machine(converge_good_count=1)
    T = machine.T_map_odom.copy()

    invalid_transform = T.copy()
    invalid_transform[0, 3] = np.nan
    outcomes = [
        RegistrationOutcome(invalid_transform, True, 900, 1000, 10.0),
        RegistrationOutcome(T, True, 900, 1000, float("nan")),
        RegistrationOutcome(T, True, 900, 1000, float("inf")),
    ]

    for outcome in outcomes:
        result = machine.update(outcome)
        assert not result.accepted
        assert result.reason == "non_finite_result"
        np.testing.assert_allclose(machine.T_map_odom, T)


def test_single_jump_rejected_persistent_jump_accepted():
    machine = _initialized_machine(converge_good_count=1, jump_confirm_count=3)
    T = machine.T_map_odom.copy()
    machine.update(_good(T))
    assert machine.state == LocalizerState.TRACKING

    jumped = T.copy()
    jumped[0, 3] += 2.0  # 2 m jump, way over the 0.5 m gate
    assert not machine.update(_good(jumped)).accepted
    assert not machine.update(_good(jumped)).accepted
    result = machine.update(_good(jumped))
    assert result.accepted  # third consistent observation confirms the slip
    np.testing.assert_allclose(machine.T_map_odom, jumped)


def test_inconsistent_jumps_never_confirm():
    machine = _initialized_machine(converge_good_count=1, jump_confirm_count=3)
    T = machine.T_map_odom.copy()
    machine.update(_good(T))
    for offset in (2.0, 5.0, 8.0, 11.0):  # each far from the previous
        jumped = T.copy()
        jumped[0, 3] += offset
        assert not machine.update(_good(jumped)).accepted


def test_converging_has_no_jump_gate():
    machine = _initialized_machine(converge_good_count=2)
    far = machine.T_map_odom.copy()
    far[0, 3] += 3.0
    assert machine.update(_good(far)).accepted


def test_correction_blend_damps_update():
    machine = _initialized_machine(converge_good_count=1, correction_blend=0.5)
    T = machine.T_map_odom.copy()
    machine.update(_good(T))  # -> TRACKING
    target = T.copy()
    target[0, 3] += 0.4  # under jump gate
    machine.update(_good(target))
    assert machine.T_map_odom[0, 3] == pytest.approx(T[0, 3] + 0.2)


def test_constrain_2d_flattens_accepted_transform():
    machine = _initialized_machine(converge_good_count=1, constrain_2d=True)
    tilted = make_transform(
        quaternion_to_rotation(0.05, 0.05, 0.0, 0.997),
        machine.T_map_odom[:3, 3],
    )
    machine.update(_good(tilted))
    np.testing.assert_allclose(machine.T_map_odom[:3, 2], [0, 0, 1], atol=1e-12)


def test_initialpose_recovers_from_lost():
    machine = _initialized_machine(
        converge_good_count=1, degraded_after_rejects=1, lost_after_rejects=1
    )
    T = machine.T_map_odom.copy()
    machine.update(_good(T))
    machine.update(_bad(T))
    assert machine.state == LocalizerState.LOST
    machine.set_initial_pose(_transform(1.0, 1.0), _transform())
    assert machine.state == LocalizerState.CONVERGING
