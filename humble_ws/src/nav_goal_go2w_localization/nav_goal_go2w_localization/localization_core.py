"""Pure logic for scan-to-map localization: SE(3) helpers, gating, state machine.

No ROS or small_gicp imports — everything here is unit-testable with numpy
alone. The node feeds registration outcomes in; this module decides whether
to accept them, how to update the map->odom transform, and which state the
localizer is in:

    UNINITIALIZED --/initialpose--> CONVERGING --N good--> TRACKING
    TRACKING --consecutive rejects--> DEGRADED --more rejects--> LOST
    DEGRADED/LOST --good registration--> TRACKING
    LOST --/initialpose--> CONVERGING
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field

import numpy as np


class LocalizerState(enum.Enum):
    UNINITIALIZED = "UNINITIALIZED"
    CONVERGING = "CONVERGING"
    TRACKING = "TRACKING"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


# --------------------------------------------------------------------------
# SE(3) helpers (4x4 homogeneous matrices)
# --------------------------------------------------------------------------

def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def rotation_angle_rad(rotation: np.ndarray) -> float:
    """Geodesic angle of a rotation matrix."""
    trace = float(np.trace(rotation))
    return math.acos(min(1.0, max(-1.0, (trace - 1.0) / 2.0)))


def yaw_from_rotation(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def yaw_rotation(yaw: float) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]]
    )


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (x, y, z, w), Shepperd's method."""
    m = rotation
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            float((m[2, 1] - m[1, 2]) / s),
            float((m[0, 2] - m[2, 0]) / s),
            float((m[1, 0] - m[0, 1]) / s),
            0.25 * s,
        )
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (
            0.25 * s,
            float((m[0, 1] + m[1, 0]) / s),
            float((m[0, 2] + m[2, 0]) / s),
            float((m[2, 1] - m[1, 2]) / s),
        )
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return (
            float((m[0, 1] + m[1, 0]) / s),
            0.25 * s,
            float((m[1, 2] + m[2, 1]) / s),
            float((m[0, 2] - m[2, 0]) / s),
        )
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return (
        float((m[0, 2] + m[2, 0]) / s),
        float((m[1, 2] + m[2, 1]) / s),
        0.25 * s,
        float((m[1, 0] - m[0, 1]) / s),
    )


def transform_delta(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """(translation [m], rotation [rad]) between two transforms."""
    translation = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    rotation = rotation_angle_rad(a[:3, :3].T @ b[:3, :3])
    return translation, rotation


def flatten_to_yaw(transform: np.ndarray) -> np.ndarray:
    """Project the rotation to yaw-only (zero roll/pitch), keep translation.

    Applied to map->odom so accumulated registration noise can never tilt the
    map frame relative to odom — a tilted map->odom skews every 2D costmap
    lookup. The z translation is kept: it corrects genuine odometry z drift.
    """
    flattened = transform.copy()
    flattened[:3, :3] = yaw_rotation(yaw_from_rotation(transform[:3, :3]))
    return flattened


def blend_transforms(
    current: np.ndarray, target: np.ndarray, alpha: float
) -> np.ndarray:
    """Interpolate from current toward target by alpha in [0, 1].

    Translation is linear; rotation interpolates the relative yaw (the stack
    flattens map->odom to yaw-only, so axis-angle slerp is unnecessary).
    """
    if alpha >= 1.0:
        return target.copy()
    translation = (1.0 - alpha) * current[:3, 3] + alpha * target[:3, 3]
    yaw_current = yaw_from_rotation(current[:3, :3])
    yaw_target = yaw_from_rotation(target[:3, :3])
    yaw_delta = math.atan2(
        math.sin(yaw_target - yaw_current), math.cos(yaw_target - yaw_current)
    )
    rotation = yaw_rotation(yaw_current + alpha * yaw_delta)
    return make_transform(rotation, translation)


# --------------------------------------------------------------------------
# Registration outcome and gating
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class RegistrationOutcome:
    """What one registration attempt produced (node fills this from small_gicp)."""

    T_map_odom: np.ndarray
    converged: bool
    num_inliers: int
    source_size: int
    error: float

    @property
    def inlier_fraction(self) -> float:
        return self.num_inliers / self.source_size if self.source_size else 0.0

    @property
    def mean_error(self) -> float:
        """Mean optimization error per inlier (algorithm-specific units)."""
        return self.error / self.num_inliers if self.num_inliers else float("inf")


@dataclass(frozen=True)
class LocalizerConfig:
    min_points: int = 200
    min_inlier_fraction: float = 0.6
    max_mean_error: float = 0.10
    max_translation_jump_m: float = 0.5
    max_rotation_jump_deg: float = 10.0
    jump_confirm_count: int = 3
    correction_blend: float = 1.0
    constrain_2d: bool = True
    converge_good_count: int = 3
    degraded_after_rejects: int = 4
    lost_after_rejects: int = 12

    def __post_init__(self) -> None:
        if not 0.0 < self.correction_blend <= 1.0:
            raise ValueError("correction_blend must be in (0, 1]")
        if self.lost_after_rejects < self.degraded_after_rejects:
            raise ValueError("lost_after_rejects must be >= degraded_after_rejects")


@dataclass
class UpdateResult:
    accepted: bool
    reason: str
    state: LocalizerState
    state_changed: bool = False


@dataclass
class LocalizationStateMachine:
    config: LocalizerConfig = field(default_factory=LocalizerConfig)
    state: LocalizerState = LocalizerState.UNINITIALIZED
    T_map_odom: np.ndarray | None = None
    _good_count: int = 0
    _reject_count: int = 0
    _pending_jump: np.ndarray | None = None
    _pending_jump_count: int = 0

    @property
    def has_transform(self) -> bool:
        return self.T_map_odom is not None

    def set_initial_pose(
        self, T_map_base: np.ndarray, T_odom_base: np.ndarray
    ) -> None:
        """Operator initial pose: derive map->odom and start converging."""
        T_map_odom = np.asarray(T_map_base) @ invert_transform(
            np.asarray(T_odom_base)
        )
        if self.config.constrain_2d:
            T_map_odom = flatten_to_yaw(T_map_odom)
        self.T_map_odom = T_map_odom
        self._enter(LocalizerState.CONVERGING)

    def update(self, outcome: RegistrationOutcome) -> UpdateResult:
        """Feed one registration outcome through the gates."""
        if self.state == LocalizerState.UNINITIALIZED or self.T_map_odom is None:
            return UpdateResult(False, "uninitialized", self.state)

        reason = self._quality_gate(outcome)
        if reason is None and self.state in (
            LocalizerState.TRACKING,
            LocalizerState.DEGRADED,
        ):
            reason = self._jump_gate(outcome)

        if reason is not None:
            return self._reject(reason)
        return self._accept(outcome)

    def reject(self, reason: str) -> UpdateResult:
        """Record a failed registration attempt that produced no outcome."""
        if self.state == LocalizerState.UNINITIALIZED or self.T_map_odom is None:
            return UpdateResult(False, "uninitialized", self.state)
        return self._reject(reason)

    # -- internals ---------------------------------------------------------

    def _quality_gate(self, outcome: RegistrationOutcome) -> str | None:
        if outcome.source_size < self.config.min_points:
            return f"too_few_points({outcome.source_size})"
        if not outcome.converged:
            return "not_converged"
        if outcome.inlier_fraction < self.config.min_inlier_fraction:
            return f"low_inlier_fraction({outcome.inlier_fraction:.2f})"
        if outcome.mean_error > self.config.max_mean_error:
            return f"high_error({outcome.mean_error:.3f})"
        return None

    def _jump_gate(self, outcome: RegistrationOutcome) -> str | None:
        translation, rotation = transform_delta(
            self.T_map_odom, outcome.T_map_odom
        )
        max_rotation = math.radians(self.config.max_rotation_jump_deg)
        if (
            translation <= self.config.max_translation_jump_m
            and rotation <= max_rotation
        ):
            self._pending_jump = None
            self._pending_jump_count = 0
            return None

        # Large jump: only accept once the same correction shows up in
        # jump_confirm_count consecutive registrations (a genuine odometry
        # slip persists; a registration glitch does not).
        if self._pending_jump is not None:
            jump_translation, jump_rotation = transform_delta(
                self._pending_jump, outcome.T_map_odom
            )
            consistent = (
                jump_translation <= self.config.max_translation_jump_m
                and jump_rotation <= max_rotation
            )
        else:
            consistent = False

        if consistent:
            self._pending_jump_count += 1
        else:
            self._pending_jump_count = 1
        self._pending_jump = np.asarray(outcome.T_map_odom).copy()

        if self._pending_jump_count >= self.config.jump_confirm_count:
            self._pending_jump = None
            self._pending_jump_count = 0
            return None
        return (
            f"jump({translation:.2f}m,{math.degrees(rotation):.1f}deg,"
            f"seen {self._pending_jump_count}/{self.config.jump_confirm_count})"
        )

    def _accept(self, outcome: RegistrationOutcome) -> UpdateResult:
        proposed = np.asarray(outcome.T_map_odom)
        if self.config.constrain_2d:
            proposed = flatten_to_yaw(proposed)
        if self.state == LocalizerState.CONVERGING:
            # Full replacement while converging: the initial click is coarse
            # and blending toward it would only slow convergence down.
            self.T_map_odom = proposed.copy()
        else:
            self.T_map_odom = blend_transforms(
                self.T_map_odom, proposed, self.config.correction_blend
            )
        self._reject_count = 0
        self._pending_jump = None
        self._pending_jump_count = 0

        if self.state == LocalizerState.CONVERGING:
            self._good_count += 1
            if self._good_count >= self.config.converge_good_count:
                return self._accept_result(LocalizerState.TRACKING)
            return self._accept_result(self.state)
        return self._accept_result(LocalizerState.TRACKING)

    def _reject(self, reason: str) -> UpdateResult:
        self._reject_count += 1
        if self.state == LocalizerState.CONVERGING:
            self._good_count = 0
            return UpdateResult(False, reason, self.state)
        next_state = self.state
        if self._reject_count >= self.config.lost_after_rejects:
            next_state = LocalizerState.LOST
        elif self._reject_count >= self.config.degraded_after_rejects:
            next_state = LocalizerState.DEGRADED
        changed = next_state != self.state
        if changed:
            self.state = next_state
        return UpdateResult(False, reason, self.state, state_changed=changed)

    def _accept_result(self, next_state: LocalizerState) -> UpdateResult:
        changed = next_state != self.state
        if changed:
            self.state = next_state
        return UpdateResult(True, "accepted", self.state, state_changed=changed)

    def _enter(self, state: LocalizerState) -> None:
        self.state = state
        self._good_count = 0
        self._reject_count = 0
        self._pending_jump = None
        self._pending_jump_count = 0
