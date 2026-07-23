from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rdp_deploy.inference.geometry import (
    pose_9d_to_flexiv_pose,
    relative_actions_to_absolute,
)
from rdp_deploy.inference.offline_control import prepare_relative_observation


@dataclass(frozen=True)
class LatentPlan:
    latent_actions: np.ndarray
    base_absolute_pose: np.ndarray
    extended_obs_steps: np.ndarray
    normalized_lowdim_max_abs: dict[str, float]


def predict_latent_plan(
    policy,
    observation: dict,
    dataset_obs_temporal_downsample_ratio: int,
    latency_step: int,
    max_normalized_lowdim_abs: float | None = None,
) -> LatentPlan:
    import torch

    relative_obs, base_pose = prepare_relative_observation(observation)
    obs_tensors = {
        key: torch.from_numpy(value).to(device=policy.device)
        for key, value in relative_obs.items()
    }
    normalized_max = {}
    if hasattr(policy, "normalizer"):
        normalized_obs = policy.normalizer.normalize(obs_tensors)
        for key in (
            "left_robot_tcp_pose",
            "left_robot_gripper_width",
            "left_robot_tcp_wrench",
        ):
            maximum = float(normalized_obs[key].abs().max().detach().cpu())
            normalized_max[key] = maximum
            if (
                max_normalized_lowdim_abs is not None
                and maximum > float(max_normalized_lowdim_abs)
            ):
                raise ValueError(
                    f"Normalized observation {key} is out of range: "
                    f"{maximum:.3f} > {float(max_normalized_lowdim_abs):.3f}"
                )
    with torch.no_grad():
        result = policy.predict_action(
            obs_tensors,
            dataset_obs_temporal_downsample_ratio=int(
                dataset_obs_temporal_downsample_ratio
            ),
            return_latent_action=True,
        )
    latent = result["action"].detach().cpu().numpy().astype(np.float32)[0]
    first_extended_step = (
        relative_obs["left_robot_tcp_pose"].shape[1]
        * int(dataset_obs_temporal_downsample_ratio)
    )
    extended_steps = np.arange(
        first_extended_step,
        first_extended_step + len(latent),
        dtype=np.int64,
    )
    latency_step = int(latency_step)
    if latency_step < 0 or latency_step >= len(latent):
        raise ValueError(f"Invalid latency_step={latency_step} for {len(latent)} actions")
    return LatentPlan(
        latent_actions=latent[latency_step:],
        base_absolute_pose=base_pose[0],
        extended_obs_steps=extended_steps[latency_step:],
        normalized_lowdim_max_abs=normalized_max,
    )


def decode_plan_entry(
    policy,
    latent: np.ndarray,
    base_absolute_pose: np.ndarray,
    wrench_history: np.ndarray,
    extended_obs_step: int,
    dataset_obs_temporal_downsample_ratio: int,
) -> tuple[np.ndarray, np.ndarray]:
    import torch

    wrench = np.asarray(wrench_history, dtype=np.float32)
    if wrench.ndim != 2 or wrench.shape[1] != 6:
        raise ValueError(f"Expected Tx6 wrench history, got {wrench.shape}")
    if len(wrench) != int(extended_obs_step):
        raise ValueError(
            f"Expected {extended_obs_step} wrench frames, got {len(wrench)}"
        )
    latent_tensor = torch.from_numpy(
        np.asarray(latent, dtype=np.float32)
    ).unsqueeze(0).to(policy.device)
    wrench_tensor = torch.from_numpy(wrench).unsqueeze(0).to(policy.device)
    with torch.no_grad():
        result = policy.predict_from_latent_action(
            latent_action=latent_tensor,
            extended_obs_dict={"left_robot_tcp_wrench": wrench_tensor},
            extended_obs_last_step=int(extended_obs_step),
            dataset_obs_temporal_downsample_ratio=int(
                dataset_obs_temporal_downsample_ratio
            ),
        )
    relative_action = (
        result["action"][0, -1].detach().cpu().numpy().astype(np.float32)
    )
    absolute_action = relative_actions_to_absolute(
        relative_action[None, :],
        np.asarray(base_absolute_pose, dtype=np.float32),
    )[0]
    flexiv_pose = pose_9d_to_flexiv_pose(absolute_action[:9])
    return absolute_action, flexiv_pose
