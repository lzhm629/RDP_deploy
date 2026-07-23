from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


class SafetyViolation(RuntimeError):
    pass


def _quat_wxyz_to_rotation(quaternion_wxyz: np.ndarray) -> Rotation:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm < 1e-8:
        raise SafetyViolation("Target quaternion is invalid")
    quaternion = quaternion / norm
    return Rotation.from_quat(quaternion[[1, 2, 3, 0]])


def _rotation_to_quat_wxyz(rotation: Rotation) -> np.ndarray:
    quaternion_xyzw = rotation.as_quat()
    return quaternion_xyzw[[3, 0, 1, 2]]


def _align_quaternion(reference: np.ndarray, target: np.ndarray) -> np.ndarray:
    reference = np.asarray(reference, dtype=np.float64).reshape(4)
    target = np.asarray(target, dtype=np.float64).reshape(4)
    return -target if float(np.dot(reference, target)) < 0.0 else target


def pose_distance(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    left = np.asarray(left, dtype=np.float64).reshape(7)
    right = np.asarray(right, dtype=np.float64).reshape(7)
    linear = float(np.linalg.norm(left[:3] - right[:3]))
    left_rotation = _quat_wxyz_to_rotation(left[3:])
    right_rotation = _quat_wxyz_to_rotation(right[3:])
    angular = float((left_rotation.inv() * right_rotation).magnitude())
    return linear, angular


@dataclass(frozen=True)
class SafetyLimits:
    workspace_min: np.ndarray
    workspace_max: np.ndarray
    max_start_offset: np.ndarray
    max_force_norm: float
    max_torque_norm: float
    max_target_jump_m: float
    max_target_jump_rad: float
    max_tracking_error_m: float
    max_tracking_error_rad: float
    max_linear_velocity: float
    max_angular_velocity: float

    def __post_init__(self):
        for name in ("workspace_min", "workspace_max", "max_start_offset"):
            value = np.asarray(getattr(self, name), dtype=np.float64)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must contain three finite values")
        if np.any(self.workspace_min >= self.workspace_max):
            raise ValueError("workspace_min must be smaller than workspace_max")
        if np.any(self.max_start_offset <= 0):
            raise ValueError("max_start_offset values must be positive")
        scalar_names = (
            "max_force_norm",
            "max_torque_norm",
            "max_target_jump_m",
            "max_target_jump_rad",
            "max_tracking_error_m",
            "max_tracking_error_rad",
            "max_linear_velocity",
            "max_angular_velocity",
        )
        for name in scalar_names:
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")

    @classmethod
    def from_config(cls, cfg) -> "SafetyLimits":
        return cls(
            workspace_min=np.asarray(cfg.workspace_min, dtype=np.float64),
            workspace_max=np.asarray(cfg.workspace_max, dtype=np.float64),
            max_start_offset=np.asarray(cfg.max_start_offset, dtype=np.float64),
            max_force_norm=float(cfg.max_force_norm),
            max_torque_norm=float(cfg.max_torque_norm),
            max_target_jump_m=float(cfg.max_target_jump_m),
            max_target_jump_rad=float(cfg.max_target_jump_rad),
            max_tracking_error_m=float(cfg.max_tracking_error_m),
            max_tracking_error_rad=float(cfg.max_tracking_error_rad),
            max_linear_velocity=float(cfg.max_linear_velocity),
            max_angular_velocity=float(cfg.max_angular_velocity),
        )


@dataclass(frozen=True)
class SafetyResult:
    target_pose: np.ndarray
    linear_rate_limited: bool
    angular_rate_limited: bool
    candidate_jump_m: float
    candidate_jump_rad: float


class SafetyFilter:
    def __init__(
        self,
        limits: SafetyLimits,
        start_pose: np.ndarray,
        initial_target: np.ndarray,
    ):
        self.limits = limits
        self.start_pose = np.asarray(start_pose, dtype=np.float64).reshape(7).copy()
        self.previous_target = (
            np.asarray(initial_target, dtype=np.float64).reshape(7).copy()
        )
        self._check_position_bounds(self.start_pose[:3])

    def _check_position_bounds(self, position: np.ndarray) -> None:
        position = np.asarray(position, dtype=np.float64).reshape(3)
        if np.any(position < self.limits.workspace_min) or np.any(
            position > self.limits.workspace_max
        ):
            raise SafetyViolation(
                f"Target outside workspace: {position.round(5).tolist()}"
            )

    def check_robot_state(
        self,
        current_pose: np.ndarray,
        wrench: np.ndarray,
    ) -> None:
        current = np.asarray(current_pose, dtype=np.float64).reshape(7)
        wrench = np.asarray(wrench, dtype=np.float64).reshape(6)
        if not np.all(np.isfinite(current)) or not np.all(np.isfinite(wrench)):
            raise SafetyViolation("Robot state contains NaN or infinity")
        force_norm = float(np.linalg.norm(wrench[:3]))
        torque_norm = float(np.linalg.norm(wrench[3:]))
        if force_norm > self.limits.max_force_norm:
            raise SafetyViolation(
                f"Force limit exceeded: {force_norm:.3f} N > "
                f"{self.limits.max_force_norm:.3f} N"
            )
        if torque_norm > self.limits.max_torque_norm:
            raise SafetyViolation(
                f"Torque limit exceeded: {torque_norm:.3f} Nm > "
                f"{self.limits.max_torque_norm:.3f} Nm"
            )

    def filter(
        self,
        candidate_pose: np.ndarray,
        current_pose: np.ndarray,
        dt: float,
    ) -> SafetyResult:
        candidate = np.asarray(candidate_pose, dtype=np.float64).reshape(7).copy()
        current = np.asarray(current_pose, dtype=np.float64).reshape(7)
        if not np.all(np.isfinite(candidate)):
            raise SafetyViolation("Model target contains NaN or infinity")
        candidate[3:] = _rotation_to_quat_wxyz(
            _quat_wxyz_to_rotation(candidate[3:])
        )
        candidate[3:] = _align_quaternion(
            self.previous_target[3:], candidate[3:]
        )

        self._check_position_bounds(candidate[:3])
        start_offset = np.abs(candidate[:3] - self.start_pose[:3])
        if np.any(start_offset > self.limits.max_start_offset):
            raise SafetyViolation(
                f"Target outside start envelope: {start_offset.round(5).tolist()}"
            )

        jump_m, jump_rad = pose_distance(current, candidate)
        if jump_m > self.limits.max_target_jump_m:
            raise SafetyViolation(
                f"Model target jump is too large: {jump_m:.4f} m"
            )
        if jump_rad > self.limits.max_target_jump_rad:
            raise SafetyViolation(
                f"Model rotation jump is too large: {jump_rad:.4f} rad"
            )

        tracking_m, tracking_rad = pose_distance(current, self.previous_target)
        if tracking_m > self.limits.max_tracking_error_m:
            raise SafetyViolation(
                f"TCP tracking error is too large: {tracking_m:.4f} m"
            )
        if tracking_rad > self.limits.max_tracking_error_rad:
            raise SafetyViolation(
                f"TCP rotation tracking error is too large: {tracking_rad:.4f} rad"
            )

        dt = max(float(dt), 1e-4)
        max_linear_step = self.limits.max_linear_velocity * dt
        delta = candidate[:3] - self.previous_target[:3]
        delta_norm = float(np.linalg.norm(delta))
        linear_limited = delta_norm > max_linear_step
        if linear_limited:
            candidate[:3] = (
                self.previous_target[:3]
                + delta * (max_linear_step / max(delta_norm, 1e-8))
            )

        previous_rotation = _quat_wxyz_to_rotation(self.previous_target[3:])
        candidate_rotation = _quat_wxyz_to_rotation(candidate[3:])
        relative_rotation = previous_rotation.inv() * candidate_rotation
        angle = float(relative_rotation.magnitude())
        max_angular_step = self.limits.max_angular_velocity * dt
        angular_limited = angle > max_angular_step
        if angular_limited:
            rotvec = relative_rotation.as_rotvec()
            limited_relative = Rotation.from_rotvec(
                rotvec * (max_angular_step / max(angle, 1e-8))
            )
            candidate[3:] = _rotation_to_quat_wxyz(
                previous_rotation * limited_relative
            )
            candidate[3:] = _align_quaternion(
                self.previous_target[3:], candidate[3:]
            )

        self.previous_target = candidate.copy()
        return SafetyResult(
            target_pose=candidate,
            linear_rate_limited=linear_limited,
            angular_rate_limited=angular_limited,
            candidate_jump_m=jump_m,
            candidate_jump_rad=jump_rad,
        )
