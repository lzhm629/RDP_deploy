from __future__ import annotations

from typing import Any

import cv2
import numpy as np
import transforms3d as t3d


def ros_time_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_to_9d(pose: Any) -> np.ndarray:
    quat = np.array([
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    ], dtype=np.float64)
    rot_mat = t3d.quaternions.quat2mat(quat)
    rot_6d = rot_mat[:, :2].T.reshape(-1)
    trans = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ], dtype=np.float64)
    return np.concatenate([trans, rot_6d]).astype(np.float32)


def twist_to_array(msg: Any) -> np.ndarray:
    return np.array([
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z,
        msg.twist.angular.x,
        msg.twist.angular.y,
        msg.twist.angular.z,
    ], dtype=np.float32)


def wrench_to_array(msg: Any) -> np.ndarray:
    return np.array([
        msg.wrench.force.x,
        msg.wrench.force.y,
        msg.wrench.force.z,
        msg.wrench.torque.x,
        msg.wrench.torque.y,
        msg.wrench.torque.z,
    ], dtype=np.float32)


def gripper_to_arrays(msg: Any) -> tuple[np.ndarray, np.ndarray]:
    width = float(msg.position[0]) if len(msg.position) > 0 else 0.0
    force = float(msg.effort[0]) if len(msg.effort) > 0 else 0.0
    return (
        np.array([width], dtype=np.float32),
        np.array([force], dtype=np.float32),
    )


def decode_image(msg: Any, resize_shape: tuple[int, int] | None = None) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)

    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        channels = 3 if "rgb" in msg.encoding.lower() or "bgr" in msg.encoding.lower() else 1
        expected = int(msg.height) * int(msg.width) * channels
        if data.size < expected:
            return np.zeros((0, 0, 3), dtype=np.uint8)
        image = data[:expected].reshape(int(msg.height), int(msg.width), channels)
        if channels == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    if msg.encoding.lower().startswith("rgb"):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if resize_shape is not None:
        image = cv2.resize(image, resize_shape)
    return image


def decode_marker_offset(msg: Any, marker_dimension: int = 2) -> tuple[np.ndarray, np.ndarray]:
    data = np.frombuffer(msg.data, dtype=np.float32)
    if marker_dimension == 2:
        points = data.reshape(-1, 4)
        return points[:, :2].copy(), points[:, 2:4].copy()
    if marker_dimension == 3:
        points = data.reshape(-1, 6)
        return points[:, :3].copy(), points[:, 3:6].copy()
    raise ValueError(f"Unsupported marker_dimension: {marker_dimension}")


def convert_topic_dict_to_observation(
    topic_dict: dict[str, Any],
    image_resize_shape: tuple[int, int] = (320, 240),
    marker_dimension: int = 2,
) -> dict:
    latest_timestamp = max(ros_time_to_float(msg.header.stamp) for msg in topic_dict.values())
    obs = {
        "timestamp": np.array([latest_timestamp], dtype=np.float64),
    }

    if "/left_tcp_pose" in topic_dict:
        obs["left_robot_tcp_pose"] = pose_to_9d(topic_dict["/left_tcp_pose"].pose)
    if "/left_tcp_vel" in topic_dict:
        obs["left_robot_tcp_vel"] = twist_to_array(topic_dict["/left_tcp_vel"])
    if "/left_tcp_wrench" in topic_dict:
        obs["left_robot_tcp_wrench"] = wrench_to_array(topic_dict["/left_tcp_wrench"])
    if "/left_gripper_state" in topic_dict:
        width, force = gripper_to_arrays(topic_dict["/left_gripper_state"])
        obs["left_robot_gripper_width"] = width
        obs["left_robot_gripper_force"] = force

    if "/right_tcp_pose" in topic_dict:
        obs["right_robot_tcp_pose"] = pose_to_9d(topic_dict["/right_tcp_pose"].pose)
    if "/right_tcp_vel" in topic_dict:
        obs["right_robot_tcp_vel"] = twist_to_array(topic_dict["/right_tcp_vel"])
    if "/right_tcp_wrench" in topic_dict:
        obs["right_robot_tcp_wrench"] = wrench_to_array(topic_dict["/right_tcp_wrench"])
    if "/right_gripper_state" in topic_dict:
        width, force = gripper_to_arrays(topic_dict["/right_gripper_state"])
        obs["right_robot_gripper_width"] = width
        obs["right_robot_gripper_force"] = force

    image_key_by_topic_part = {
        "D405": "agentview_image",
        "right_wrist_camera": "right_wrist_img",
        "left_gripper_camera_1": "left_gripper1_img",
        "left_gripper_camera_2": "left_gripper2_img",
        "right_gripper_camera_1": "right_gripper1_img",
        "right_gripper_camera_2": "right_gripper2_img",
    }
    marker_key_by_topic_part = {
        "left_gripper_camera_1": ("left_gripper1_initial_marker", "left_gripper1_marker_offset"),
        "left_gripper_camera_2": ("left_gripper2_initial_marker", "left_gripper2_marker_offset"),
        "right_gripper_camera_1": ("right_gripper1_initial_marker", "right_gripper1_marker_offset"),
        "right_gripper_camera_2": ("right_gripper2_initial_marker", "right_gripper2_marker_offset"),
    }

    for topic, msg in topic_dict.items():
        if "color/image_raw" in topic:
            for topic_part, obs_key in image_key_by_topic_part.items():
                if topic_part in topic:
                    obs[obs_key] = decode_image(msg, resize_shape=image_resize_shape)
                    break
        elif "marker_offset/information" in topic:
            for topic_part, keys in marker_key_by_topic_part.items():
                if topic_part in topic:
                    marker, offset = decode_marker_offset(msg, marker_dimension=marker_dimension)
                    obs[keys[0]] = marker
                    obs[keys[1]] = offset
                    break
        elif "force_resultant" in topic:
            for topic_part in marker_key_by_topic_part.keys():
                if topic_part in topic:
                    compact_name = topic_part.replace("left_gripper_camera_", "left_gripper")
                    compact_name = compact_name.replace("right_gripper_camera_", "right_gripper")
                    obs[f"{compact_name}_force_resultant"] = wrench_to_array(msg)
                    break

    return obs
