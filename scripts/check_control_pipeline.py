#!/usr/bin/env python3
import numpy as np

import _bootstrap  # noqa: F401

from rdp_deploy.control.buffers import LatentPlanBuffer, TargetMailbox
from rdp_deploy.control.online_policy import decode_plan_entry, predict_latent_plan
from rdp_deploy.control.safety import (
    SafetyFilter,
    SafetyLimits,
    SafetyViolation,
    pose_distance,
)


IDENTITY_POSE_9D = np.array(
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    dtype=np.float32,
)
IDENTITY_FLEXIV = np.array(
    [0.60, -0.20, 0.25, 1.0, 0.0, 0.0, 0.0],
    dtype=np.float64,
)


class _FakePolicy:
    def __init__(self):
        import torch

        self._anchor = torch.zeros(1)

    @property
    def device(self):
        return self._anchor.device

    def predict_action(self, obs_dict, **kwargs):
        import torch

        assert kwargs["return_latent_action"] is True
        return {"action": torch.zeros((1, 29, 64), dtype=torch.float32)}

    def predict_from_latent_action(self, **kwargs):
        import torch

        steps = int(kwargs["extended_obs_last_step"])
        actions = np.tile(
            np.concatenate((IDENTITY_POSE_9D, [0.0])).astype(np.float32),
            (max(1, steps - 3), 1),
        )
        actions[-1, 0] = 0.004
        return {"action": torch.from_numpy(actions).unsqueeze(0)}


def _observation():
    pose = IDENTITY_POSE_9D.copy()
    pose[:3] = IDENTITY_FLEXIV[:3]
    return {
        "left_wrist_img": np.zeros((1, 2, 3, 240, 320), dtype=np.float32),
        "left_robot_tcp_pose": np.stack((pose, pose))[None],
        "left_robot_gripper_width": np.zeros((1, 2, 1), dtype=np.float32),
        "left_robot_tcp_wrench": np.zeros((1, 2, 6), dtype=np.float32),
    }


def main() -> int:
    plan = predict_latent_plan(
        _FakePolicy(),
        _observation(),
        dataset_obs_temporal_downsample_ratio=2,
        latency_step=4,
    )
    assert plan.latent_actions.shape == (25, 64)
    np.testing.assert_array_equal(plan.extended_obs_steps, np.arange(8, 33))

    buffer = LatentPlanBuffer()
    generation = buffer.replace(
        start_tick=10,
        latent_actions=plan.latent_actions,
        base_absolute_pose=plan.base_absolute_pose,
        extended_obs_steps=plan.extended_obs_steps,
    )
    entry = buffer.get(10)
    assert entry is not None and entry.plan_generation == generation
    assert buffer.get(9) is None

    absolute_action, candidate = decode_plan_entry(
        policy=_FakePolicy(),
        latent=entry.latent,
        base_absolute_pose=entry.base_absolute_pose,
        wrench_history=np.zeros((entry.extended_obs_step, 6), dtype=np.float32),
        extended_obs_step=entry.extended_obs_step,
        dataset_obs_temporal_downsample_ratio=2,
    )
    assert absolute_action.shape == (10,)
    np.testing.assert_allclose(candidate[:3], [0.604, -0.2, 0.25], atol=1e-6)

    limits = SafetyLimits(
        workspace_min=np.array([0.45, -0.5, 0.04]),
        workspace_max=np.array([0.85, 0.2, 0.6]),
        max_start_offset=np.array([0.08, 0.08, 0.05]),
        max_force_norm=30.0,
        max_torque_norm=5.0,
        max_target_jump_m=0.02,
        max_target_jump_rad=0.35,
        max_tracking_error_m=0.03,
        max_tracking_error_rad=0.5,
        max_linear_velocity=0.02,
        max_angular_velocity=0.05,
    )
    safety = SafetyFilter(limits, IDENTITY_FLEXIV, IDENTITY_FLEXIV)
    result = safety.filter(candidate, IDENTITY_FLEXIV, dt=1.0 / 24.0)
    assert result.linear_rate_limited
    distance, _ = pose_distance(IDENTITY_FLEXIV, result.target_pose)
    np.testing.assert_allclose(distance, 0.02 / 24.0, atol=1e-7)

    antipodal = IDENTITY_FLEXIV.copy()
    antipodal[3:] *= -1
    sign_safety = SafetyFilter(limits, IDENTITY_FLEXIV, IDENTITY_FLEXIV)
    aligned = sign_safety.filter(antipodal, IDENTITY_FLEXIV, dt=1.0 / 24.0)
    if np.dot(aligned.target_pose[3:], IDENTITY_FLEXIV[3:]) <= 0:
        raise AssertionError("Antipodal quaternion was not aligned")

    safety.check_robot_state(IDENTITY_FLEXIV, np.zeros(6))
    try:
        safety.check_robot_state(
            IDENTITY_FLEXIV, np.array([31.0, 0, 0, 0, 0, 0])
        )
    except SafetyViolation:
        pass
    else:
        raise AssertionError("Force limit did not reject an unsafe wrench")

    mailbox = TargetMailbox()
    mailbox.update(result.target_pose, 10, "test")
    target = mailbox.latest()
    assert target is not None and target.control_tick == 10

    print("Control pipeline: OK")
    print(f"Latent plan shape: {plan.latent_actions.shape}")
    print(f"Extended observation steps: {plan.extended_obs_steps[[0, -1]].tolist()}")
    print(f"Rate-limited step: {distance:.7f} m")
    print("Hardware initialized: false")
    print("Commands sent: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
