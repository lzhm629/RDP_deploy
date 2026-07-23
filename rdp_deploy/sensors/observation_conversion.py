from __future__ import annotations

import cv2
import numpy as np
import transforms3d as t3d


def tcp_pose_to_9d(tcp_pose: np.ndarray | list[float]) -> np.ndarray:
    pose = np.asarray(tcp_pose, dtype=np.float64).reshape(-1)
    if pose.size < 7:
        raise ValueError(f"TCP pose must contain 7 values, got {pose.size}")
    rot_mat = t3d.quaternions.quat2mat(pose[3:7])
    rot_6d = rot_mat[:, :2].T.reshape(-1)
    return np.concatenate([pose[:3], rot_6d]).astype(np.float32)


def resize_bgr_image(
    image: np.ndarray,
    resize_shape: tuple[int, int] | None,
) -> np.ndarray:
    result = np.asarray(image)
    if result.ndim == 2:
        result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
    if result.ndim != 3 or result.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 image, got shape {result.shape}")
    if resize_shape is not None:
        result = cv2.resize(result, resize_shape)
    return np.asarray(result, dtype=np.uint8).copy()


def robot_states_to_observation(states: dict, bimanual: bool = False) -> dict:
    obs = {
        "left_robot_tcp_pose": tcp_pose_to_9d(states["leftRobotTCP"]),
        "left_robot_tcp_vel": np.asarray(states["leftRobotTCPVel"], dtype=np.float32)[:6].copy(),
        "left_robot_tcp_wrench": np.asarray(states["leftRobotTCPWrench"], dtype=np.float32)[:6].copy(),
        "left_robot_gripper_width": np.asarray(states["leftGripperState"][:1], dtype=np.float32),
        "left_robot_gripper_force": np.asarray(states["leftGripperState"][1:2], dtype=np.float32),
    }
    if bimanual:
        obs.update({
            "right_robot_tcp_pose": tcp_pose_to_9d(states["rightRobotTCP"]),
            "right_robot_tcp_vel": np.asarray(states["rightRobotTCPVel"], dtype=np.float32)[:6].copy(),
            "right_robot_tcp_wrench": np.asarray(states["rightRobotTCPWrench"], dtype=np.float32)[:6].copy(),
            "right_robot_gripper_width": np.asarray(states["rightGripperState"][:1], dtype=np.float32),
            "right_robot_gripper_force": np.asarray(states["rightGripperState"][1:2], dtype=np.float32),
        })
    return obs


def realsense_to_observation(
    camera_name: str,
    color_image: np.ndarray,
    resize_shape: tuple[int, int] | None,
) -> dict:
    image_keys = {
        "D405": "agentview_image",
        "right_wrist_camera": "right_wrist_img",
    }
    key = image_keys.get(camera_name, f"{camera_name}_image")
    return {key: resize_bgr_image(color_image, resize_shape)}


def xense_observation_keys(sensor_name: str) -> tuple[str, str, str, str]:
    mappings = {
        "left_gripper_camera_1": (
            "left_gripper1_img",
            "left_gripper1_initial_marker",
            "left_gripper1_marker_offset",
            "left_gripper1_force_resultant",
        ),
        "left_gripper_camera_2": (
            "left_gripper2_img",
            "left_gripper2_initial_marker",
            "left_gripper2_marker_offset",
            "left_gripper2_force_resultant",
        ),
        "right_gripper_camera_1": (
            "right_gripper1_img",
            "right_gripper1_initial_marker",
            "right_gripper1_marker_offset",
            "right_gripper1_force_resultant",
        ),
        "right_gripper_camera_2": (
            "right_gripper2_img",
            "right_gripper2_initial_marker",
            "right_gripper2_marker_offset",
            "right_gripper2_force_resultant",
        ),
    }
    if sensor_name not in mappings:
        raise ValueError(f"Unsupported Xense sensor_name: {sensor_name}")
    return mappings[sensor_name]


def xense_to_observation(
    sensor_name: str,
    resize_shape: tuple[int, int] | None,
    marker_dimension: int,
    image: np.ndarray | None = None,
    marker: np.ndarray | None = None,
    marker_reference: np.ndarray | None = None,
    force_resultant: np.ndarray | None = None,
) -> dict:
    image_key, initial_key, offset_key, force_key = xense_observation_keys(sensor_name)
    obs = {}
    if image is not None:
        obs[image_key] = resize_bgr_image(image, resize_shape)
    if marker is not None and marker_reference is not None:
        current = np.asarray(marker, dtype=np.float32).reshape(-1, marker_dimension)
        initial = np.asarray(marker_reference, dtype=np.float32).reshape(-1, marker_dimension)
        if current.shape != initial.shape:
            raise ValueError(
                f"Xense marker shape changed from {initial.shape} to {current.shape}"
            )
        obs[initial_key] = initial.copy()
        obs[offset_key] = (current - initial).astype(np.float32)
    if force_resultant is not None:
        force = np.asarray(force_resultant, dtype=np.float32).reshape(-1)
        if force.size < 6:
            raise ValueError(f"Xense force must contain 6 values, got {force.size}")
        obs[force_key] = force[:6].copy()
    return obs
