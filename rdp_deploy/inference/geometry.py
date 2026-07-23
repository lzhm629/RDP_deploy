from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def normalize_vector(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    magnitude = np.linalg.norm(values, axis=-1, keepdims=True)
    return values / np.maximum(magnitude, 1e-8)


def rotation_6d_to_matrix(rotation_6d: np.ndarray) -> np.ndarray:
    values = np.asarray(rotation_6d, dtype=np.float64)
    if values.shape[-1] != 6:
        raise ValueError(f"Rotation 6D must end in 6 values, got {values.shape}")
    x_axis = normalize_vector(values[..., :3])
    z_axis = normalize_vector(np.cross(x_axis, values[..., 3:6]))
    y_axis = np.cross(z_axis, x_axis)
    return np.stack((x_axis, y_axis, z_axis), axis=-1)


def pose_9d_to_matrix(pose_9d: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose_9d, dtype=np.float64)
    if pose.shape[-1] != 9:
        raise ValueError(f"Pose 9D must end in 9 values, got {pose.shape}")
    result = np.zeros(pose.shape[:-1] + (4, 4), dtype=np.float64)
    result[..., 3, 3] = 1.0
    result[..., :3, :3] = rotation_6d_to_matrix(pose[..., 3:9])
    result[..., :3, 3] = pose[..., :3]
    return result


def matrix_to_pose_9d(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape[-2:] != (4, 4):
        raise ValueError(f"Pose matrix must end in 4x4, got {matrix.shape}")
    rotation_6d = np.swapaxes(matrix[..., :3, :2], -1, -2).reshape(
        matrix.shape[:-2] + (6,)
    )
    return np.concatenate((matrix[..., :3, 3], rotation_6d), axis=-1)


def absolute_to_relative_pose(
    absolute_pose: np.ndarray,
    base_absolute_pose: np.ndarray,
) -> np.ndarray:
    absolute_matrix = pose_9d_to_matrix(absolute_pose)
    base_matrix = pose_9d_to_matrix(base_absolute_pose)
    relative_matrix = np.linalg.inv(base_matrix) @ absolute_matrix
    return matrix_to_pose_9d(relative_matrix).astype(np.float32)


def relative_actions_to_absolute(
    relative_actions: np.ndarray,
    base_absolute_pose: np.ndarray,
) -> np.ndarray:
    actions = np.asarray(relative_actions, dtype=np.float32).copy()
    if actions.ndim != 2 or actions.shape[1] != 10:
        raise ValueError(f"Expected Tx10 decoded actions, got {actions.shape}")
    base_matrix = pose_9d_to_matrix(np.asarray(base_absolute_pose).reshape(9))
    relative_matrix = pose_9d_to_matrix(actions[:, :9])
    actions[:, :9] = matrix_to_pose_9d(base_matrix @ relative_matrix)
    return actions


def pose_9d_to_flexiv_pose(pose_9d: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose_9d, dtype=np.float64).reshape(9)
    quaternion_xyzw = Rotation.from_matrix(
        rotation_6d_to_matrix(pose[3:9])
    ).as_quat()
    quaternion_wxyz = quaternion_xyzw[[3, 0, 1, 2]]
    return np.concatenate((pose[:3], quaternion_wxyz)).astype(np.float32)
