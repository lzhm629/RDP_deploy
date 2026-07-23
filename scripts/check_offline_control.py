#!/usr/bin/env python3
import numpy as np

import _bootstrap  # noqa: F401

from rdp_deploy.inference.geometry import (
    absolute_to_relative_pose,
    matrix_to_pose_9d,
    pose_9d_to_flexiv_pose,
    pose_9d_to_matrix,
    relative_actions_to_absolute,
)
from rdp_deploy.inference.offline_control import (
    prepare_relative_observation,
    run_offline_inference,
)


IDENTITY_POSE_9D = np.array(
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    dtype=np.float32,
)


class _FakePolicy:
    def __init__(self):
        import torch

        self._device_anchor = torch.zeros(1)

    @property
    def device(self):
        return self._device_anchor.device

    def predict_action(self, obs_dict, **kwargs):
        import torch

        assert kwargs["return_latent_action"] is True
        return {"action": torch.zeros((1, 29, 64), dtype=torch.float32)}

    def predict_from_latent_action(self, **kwargs):
        import torch

        actions = np.tile(
            np.concatenate((IDENTITY_POSE_9D, [0.02])).astype(np.float32),
            (29, 1),
        )
        actions[:, 0] = np.linspace(0.001, 0.029, 29)
        return {"action": torch.from_numpy(actions).unsqueeze(0)}


def _observation():
    first_pose = IDENTITY_POSE_9D.copy()
    first_pose[:3] = [0.49, -0.01, 0.20]
    last_pose = IDENTITY_POSE_9D.copy()
    last_pose[:3] = [0.50, 0.00, 0.20]
    return {
        "left_wrist_img": np.zeros((1, 2, 3, 240, 320), dtype=np.float32),
        "left_robot_tcp_pose": np.stack((first_pose, last_pose))[None],
        "left_robot_gripper_width": np.full((1, 2, 1), 0.02, dtype=np.float32),
        "left_robot_tcp_wrench": np.zeros((1, 2, 6), dtype=np.float32),
    }


def main() -> int:
    pose = IDENTITY_POSE_9D.copy()
    pose[:3] = [0.5, -0.1, 0.2]
    np.testing.assert_allclose(matrix_to_pose_9d(pose_9d_to_matrix(pose)), pose)
    np.testing.assert_allclose(
        absolute_to_relative_pose(pose[None], pose)[0],
        IDENTITY_POSE_9D,
        atol=1e-6,
    )

    relative = np.tile(
        np.concatenate((IDENTITY_POSE_9D, [0.02])).astype(np.float32),
        (2, 1),
    )
    relative[:, 0] = [0.01, 0.02]
    absolute = relative_actions_to_absolute(relative, pose)
    np.testing.assert_allclose(absolute[:, 0], [0.51, 0.52], atol=1e-6)
    flexiv_pose = pose_9d_to_flexiv_pose(absolute[0, :9])
    np.testing.assert_allclose(flexiv_pose[3:], [1, 0, 0, 0], atol=1e-6)

    prepared, base_pose = prepare_relative_observation(_observation())
    np.testing.assert_allclose(
        prepared["left_robot_tcp_pose"][0, -1],
        IDENTITY_POSE_9D,
        atol=1e-6,
    )
    np.testing.assert_allclose(base_pose[0, :3], [0.5, 0.0, 0.2])

    result = run_offline_inference(_FakePolicy(), _observation())
    assert result.latent_actions.shape == (1, 29, 64)
    assert result.relative_actions.shape == (1, 29, 10)
    assert result.absolute_actions.shape == (29, 10)
    assert result.flexiv_target_poses.shape == (29, 7)
    np.testing.assert_allclose(result.absolute_actions[0, :3], [0.501, 0.0, 0.2])
    print("Offline control dry-run checks passed.")
    print("Hardware initialized: false")
    print("Commands sent: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
