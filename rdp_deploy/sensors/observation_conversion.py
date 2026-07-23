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
    color_image: np.ndarray,
    resize_shape: tuple[int, int] | None,
) -> dict:
    bgr = resize_bgr_image(color_image, resize_shape)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    chw = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0
    return {"left_wrist_img": np.ascontiguousarray(chw)}


def stack_model_observation(frame_history: list[dict]) -> dict:
    if not frame_history:
        raise ValueError("Observation history is empty")
    model_keys = (
        "left_wrist_img",
        "left_robot_tcp_pose",
        "left_robot_gripper_width",
        "left_robot_tcp_wrench",
    )
    missing = [
        key
        for key in model_keys
        if any(key not in frame for frame in frame_history)
    ]
    if missing:
        raise ValueError(f"Observation history is missing keys: {sorted(set(missing))}")
    return {
        key: np.expand_dims(
            np.stack([frame[key] for frame in frame_history], axis=0),
            axis=0,
        ).astype(np.float32, copy=False)
        for key in model_keys
    }
