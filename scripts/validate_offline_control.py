#!/usr/bin/env python3
import argparse
import json
import pickle
from pathlib import Path

import numpy as np

import _bootstrap  # noqa: F401

from rdp_deploy.config import cfg_get, load_config
from rdp_deploy.inference.model_loader import load_policy
from rdp_deploy.inference.offline_control import run_offline_inference
from rdp_deploy.paths import resolve_repo_path


def _resolve_required_path(cli_value, config_value, name):
    value = cli_value if cli_value is not None else config_value
    if not value:
        raise ValueError(f"{name} is required")
    return resolve_repo_path(value)


def _load_snapshot(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"Snapshot not found: {path}")
    with open(path, "rb") as file:
        snapshot = pickle.load(file)
    if not isinstance(snapshot, dict):
        raise ValueError(f"Snapshot must contain a dict, got {type(snapshot).__name__}")
    return snapshot


def _rows(values: np.ndarray, count: int) -> list:
    return np.asarray(values[:count]).round(6).tolist()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run RDP inference and decode robot targets without hardware commands."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint")
    parser.add_argument("--at-checkpoint")
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--device")
    parser.add_argument("--use-ema", choices=["auto", "true", "false"])
    parser.add_argument("--print-steps", type=int, default=None)
    args = parser.parse_args()

    try:
        cfg = load_config(args.config)
        checkpoint = _resolve_required_path(
            args.checkpoint,
            cfg_get(cfg, "model.checkpoint_path"),
            "policy checkpoint (--checkpoint or model.checkpoint_path)",
        )
        at_checkpoint = _resolve_required_path(
            args.at_checkpoint,
            cfg_get(cfg, "model.at_checkpoint_path"),
            "AT checkpoint (--at-checkpoint or model.at_checkpoint_path)",
        )
        snapshot_path = resolve_repo_path(args.snapshot)
        device = args.device or str(cfg_get(cfg, "model.device", "cuda:0"))
        use_ema = args.use_ema or str(cfg_get(cfg, "model.use_ema", "auto"))
        print_steps = int(
            args.print_steps
            if args.print_steps is not None
            else cfg_get(cfg, "model.print_steps", 5)
        )

        loaded = load_policy(
            checkpoint_path=checkpoint,
            at_checkpoint_path=at_checkpoint,
            device=device,
            use_ema=use_ema,
            num_inference_steps=int(cfg_get(cfg, "model.num_inference_steps", 8)),
        )
        result = run_offline_inference(
            policy=loaded.policy,
            observation=_load_snapshot(snapshot_path),
            dataset_obs_temporal_downsample_ratio=int(
                cfg_get(cfg, "model.dataset_obs_temporal_downsample_ratio", 2)
            ),
            decoder_horizon=int(cfg_get(cfg, "model.decoder_horizon", 32)),
            seed=int(cfg_get(cfg, "model.seed", 42)),
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Offline control validation failed: {type(exc).__name__}: {exc}")
        return 1

    output = {
        "safety": {
            "dry_run": True,
            "hardware_initialized": False,
            "commands_sent": 0,
            "gripper_action": "ignored; keep sponge clamped",
        },
        "model": {
            "checkpoint": str(loaded.checkpoint_path),
            "at_checkpoint": str(loaded.at_checkpoint_path),
            "weights": loaded.weights_key,
            "device": loaded.device,
        },
        "shapes": {
            "latent_actions": list(result.latent_actions.shape),
            "relative_actions": list(result.relative_actions.shape),
            "absolute_actions": list(result.absolute_actions.shape),
            "flexiv_target_poses": list(result.flexiv_target_poses.shape),
        },
        "preview": {
            "base_absolute_pose_9d": _rows(result.base_absolute_pose, 1),
            "relative_action_10d": _rows(result.relative_actions[0], print_steps),
            "absolute_action_10d": _rows(result.absolute_actions, print_steps),
            "flexiv_target_pose_xyz_wxyz": _rows(
                result.flexiv_target_poses, print_steps
            ),
        },
    }
    print(json.dumps(output, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
