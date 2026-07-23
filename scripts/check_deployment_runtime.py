#!/usr/bin/env python3
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from omegaconf import OmegaConf

import _bootstrap  # noqa: F401

import rdp_deploy.control.runtime as runtime_module
from rdp_deploy.control.runtime import DeploymentRuntime
from rdp_deploy.sensors.observation_buffer import ObservationBuffer


IDENTITY_POSE_9D = np.array(
    [0.6, -0.2, 0.25, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    dtype=np.float32,
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

        return {"action": torch.zeros((1, 29, 64), dtype=torch.float32)}

    def predict_from_latent_action(self, **kwargs):
        import torch

        count = max(1, int(kwargs["extended_obs_last_step"]) - 3)
        action = np.tile(
            np.concatenate((IDENTITY_POSE_9D * 0, [0.0])).astype(np.float32),
            (count, 1),
        )
        action[:, 3:9] = np.array([1, 0, 0, 0, 1, 0], dtype=np.float32)
        action[-1, 0] = 0.002
        return {"action": torch.from_numpy(action).unsqueeze(0)}


def _model_observation():
    return {
        "left_wrist_img": np.zeros((1, 2, 3, 240, 320), dtype=np.float32),
        "left_robot_tcp_pose": np.stack(
            (IDENTITY_POSE_9D, IDENTITY_POSE_9D)
        )[None],
        "left_robot_gripper_width": np.zeros((1, 2, 1), dtype=np.float32),
        "left_robot_tcp_wrench": np.zeros((1, 2, 6), dtype=np.float32),
    }


def _raw_observation():
    return {
        "timestamp": np.array([time.time()], dtype=np.float64),
        "left_robot_tcp_pose": IDENTITY_POSE_9D.copy(),
        "left_robot_tcp_wrench": np.zeros(6, dtype=np.float32),
        "left_robot_gripper_width": np.zeros(1, dtype=np.float32),
    }


class _FakeCollector:
    def __init__(self, cfg, robot_client=None):
        self.buffer = ObservationBuffer()
        self.raw_buffer = ObservationBuffer()
        self.buffer.push(_model_observation())
        self.raw_buffer.push(_raw_observation())

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return None

    def assert_devices_fresh(self, max_age_sec):
        return None

    def recent_wrenches(self, count):
        return np.zeros((count, 6), dtype=np.float32)


class _FakeClient:
    def __init__(self):
        self.commands = []
        self.idle_count = 0
        self.closed = False

    def status(self):
        return {"connected": True, "operational": True, "fault": False}

    def send_tcp_target(self, target):
        self.commands.append(np.asarray(target).copy())

    def idle(self):
        self.idle_count += 1

    def close(self):
        self.closed = True


def _config(output_dir):
    return OmegaConf.create({
        "project": {"output_dir": output_dir},
        "model": {
            "dataset_obs_temporal_downsample_ratio": 2,
            "decoder_horizon": 32,
            "max_normalized_lowdim_abs": 20.0,
        },
        "observation": {
            "temporal_downsample_ratio": 2,
        },
        "deployment": {
            "startup_timeout_sec": 1.0,
            "control_fps": 24,
            "inference_fps": 6,
            "robot_send_fps": 90,
            "tcp_action_update_interval": 16,
            "latency_step": 4,
            "observation_timeout_sec": 10.0,
            "target_timeout_sec": 0.25,
            "print_interval_sec": 10.0,
            "motion_confirmation": "MOVE_TEST_ROBOT",
        },
        "safety": {
            "workspace_min": [0.45, -0.5, 0.04],
            "workspace_max": [0.85, 0.2, 0.6],
            "max_start_offset": [0.08, 0.08, 0.05],
            "max_force_norm": 30.0,
            "max_torque_norm": 5.0,
            "max_target_jump_m": 0.02,
            "max_target_jump_rad": 0.35,
            "max_tracking_error_m": 0.03,
            "max_tracking_error_rad": 0.5,
            "max_linear_velocity": 0.02,
            "max_angular_velocity": 0.05,
        },
    })


def main() -> int:
    original_factory = runtime_module.forcemimic_robot_client_from_config
    original_collector = runtime_module.DirectSensorCollector
    clients = []

    def fake_factory(cfg):
        client = _FakeClient()
        clients.append(client)
        return client

    runtime_module.forcemimic_robot_client_from_config = fake_factory
    runtime_module.DirectSensorCollector = _FakeCollector
    try:
        with tempfile.TemporaryDirectory(prefix="rdp_runtime_") as output_dir:
            cfg = _config(output_dir)
            loaded = SimpleNamespace(
                policy=_FakePolicy(),
                checkpoint_path=Path("fake_ldp.ckpt"),
                at_checkpoint_path=Path("fake_at.ckpt"),
            )

            shadow_report = DeploymentRuntime(cfg, loaded).run(
                mode="shadow",
                duration_sec=1.5,
            )
            shadow_client = clients[-1]
            if shadow_report["errors"]:
                raise AssertionError(f"Shadow runtime errors: {shadow_report['errors']}")
            assert shadow_report["stats"]["commands_sent"] == 0
            assert shadow_report["stats"]["plans"] > 0
            assert shadow_report["stats"]["decoded_actions"] > 0
            assert shadow_client.commands == []
            assert shadow_client.idle_count == 0
            assert shadow_client.closed

            clients_before_rejection = len(clients)
            try:
                DeploymentRuntime(cfg, loaded).run(
                    mode="hold",
                    duration_sec=0.1,
                    motion_confirmation=None,
                )
            except ValueError:
                pass
            else:
                raise AssertionError("Motion mode accepted a missing confirmation")
            assert len(clients) == clients_before_rejection

            hold_report = DeploymentRuntime(cfg, loaded).run(
                mode="hold",
                duration_sec=0.25,
                motion_confirmation="MOVE_TEST_ROBOT",
            )
            hold_client = clients[-1]
            assert hold_report["errors"] == []
            assert hold_report["stats"]["commands_sent"] > 0
            assert len(hold_client.commands) == hold_report["stats"]["commands_sent"]
            assert hold_client.idle_count >= 1
            assert hold_client.closed

            print("Deployment runtime: OK")
            print(f"Shadow commands: {shadow_report['stats']['commands_sent']}")
            print(f"Shadow plans: {shadow_report['stats']['plans']}")
            print(
                f"Shadow decoded actions: "
                f"{shadow_report['stats']['decoded_actions']}"
            )
            print(f"Hold commands: {hold_report['stats']['commands_sent']}")
            print(f"Hold IDLE calls: {hold_client.idle_count}")
            print("Real hardware initialized: false")
    finally:
        runtime_module.forcemimic_robot_client_from_config = original_factory
        runtime_module.DirectSensorCollector = original_collector
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
