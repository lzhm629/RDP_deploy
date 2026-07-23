from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from rdp_deploy.inference.geometry import (
    absolute_to_relative_pose,
    pose_9d_to_flexiv_pose,
    relative_actions_to_absolute,
)


MODEL_KEYS = (
    "left_wrist_img",
    "left_robot_tcp_pose",
    "left_robot_gripper_width",
    "left_robot_tcp_wrench",
)


@dataclass(frozen=True)
class OfflineInferenceResult:
    latent_actions: np.ndarray
    relative_actions: np.ndarray
    absolute_actions: np.ndarray
    flexiv_target_poses: np.ndarray
    base_absolute_pose: np.ndarray


def validate_observation(observation: dict[str, Any]) -> None:
    expected_shapes = {
        "left_wrist_img": (1, 2, 3, 240, 320),
        "left_robot_tcp_pose": (1, 2, 9),
        "left_robot_gripper_width": (1, 2, 1),
        "left_robot_tcp_wrench": (1, 2, 6),
    }
    missing = [key for key in MODEL_KEYS if key not in observation]
    if missing:
        raise ValueError(f"Observation is missing keys: {missing}")
    for key, shape in expected_shapes.items():
        value = np.asarray(observation[key])
        if value.shape != shape:
            raise ValueError(f"{key} must have shape {shape}, got {value.shape}")
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{key} contains NaN or infinity")
    image = np.asarray(observation["left_wrist_img"])
    if image.min() < 0.0 or image.max() > 1.0:
        raise ValueError("left_wrist_img must be RGB float data in [0, 1]")


def prepare_relative_observation(
    observation: dict[str, Any],
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    validate_observation(observation)
    result = {
        key: np.asarray(observation[key], dtype=np.float32).copy()
        for key in MODEL_KEYS
    }
    base_pose = result["left_robot_tcp_pose"][:, -1, :].copy()
    for batch_index in range(result["left_robot_tcp_pose"].shape[0]):
        result["left_robot_tcp_pose"][batch_index] = absolute_to_relative_pose(
            result["left_robot_tcp_pose"][batch_index],
            base_pose[batch_index],
        )
    return result, base_pose


def run_offline_inference(
    policy,
    observation: dict[str, Any],
    dataset_obs_temporal_downsample_ratio: int = 2,
    decoder_horizon: int = 32,
    seed: int = 42,
) -> OfflineInferenceResult:
    import torch

    if decoder_horizon < 4:
        raise ValueError("decoder_horizon must be at least 4")
    relative_obs, base_pose = prepare_relative_observation(observation)
    device = policy.device
    torch.manual_seed(int(seed))
    if str(device).startswith("cuda"):
        torch.cuda.manual_seed_all(int(seed))

    obs_tensors = {
        key: torch.from_numpy(value).to(device=device)
        for key, value in relative_obs.items()
    }
    with torch.no_grad():
        latent_result = policy.predict_action(
            obs_tensors,
            dataset_obs_temporal_downsample_ratio=(
                int(dataset_obs_temporal_downsample_ratio)
            ),
            return_latent_action=True,
        )
        latent_actions = latent_result["action"]
        if latent_actions.ndim != 3 or latent_actions.shape[0] != 1:
            raise ValueError(
                f"Expected BxTxD latent actions with B=1, got {latent_actions.shape}"
            )

        wrench = obs_tensors["left_robot_tcp_wrench"]
        if wrench.shape[1] < decoder_horizon:
            padding = wrench[:, -1:, :].repeat(
                1, decoder_horizon - wrench.shape[1], 1
            )
            wrench = torch.cat((wrench, padding), dim=1)
        else:
            wrench = wrench[:, -decoder_horizon:, :]

        decoded_result = policy.predict_from_latent_action(
            latent_action=latent_actions[:, 0, :],
            extended_obs_dict={"left_robot_tcp_wrench": wrench},
            extended_obs_last_step=decoder_horizon,
            dataset_obs_temporal_downsample_ratio=(
                int(dataset_obs_temporal_downsample_ratio)
            ),
        )
        relative_actions = decoded_result["action"]

    latent_numpy = latent_actions.detach().cpu().numpy().astype(np.float32)
    relative_numpy = relative_actions.detach().cpu().numpy().astype(np.float32)
    if relative_numpy.shape[0] != 1 or relative_numpy.shape[-1] != 10:
        raise ValueError(f"Expected decoded actions shaped 1xTx10, got {relative_numpy.shape}")
    absolute_actions = relative_actions_to_absolute(
        relative_numpy[0],
        base_pose[0],
    )
    flexiv_poses = np.stack(
        [pose_9d_to_flexiv_pose(action[:9]) for action in absolute_actions],
        axis=0,
    )
    return OfflineInferenceResult(
        latent_actions=latent_numpy,
        relative_actions=relative_numpy,
        absolute_actions=absolute_actions,
        flexiv_target_poses=flexiv_poses,
        base_absolute_pose=base_pose,
    )
