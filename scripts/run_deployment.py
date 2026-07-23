#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path
import sys


def _ensure_pytorch_cudnn_precedence() -> None:
    if os.environ.get("RDP_CUDNN_PATH_READY") == "1":
        return
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if not conda_prefix:
        return
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    cudnn_lib = (
        Path(conda_prefix)
        / "lib"
        / version
        / "site-packages"
        / "nvidia"
        / "cudnn"
        / "lib"
    )
    if not cudnn_lib.is_dir():
        return
    env = dict(os.environ)
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = (
        str(cudnn_lib) if not existing else f"{cudnn_lib}:{existing}"
    )
    env["RDP_CUDNN_PATH_READY"] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)


_ensure_pytorch_cudnn_precedence()

import _bootstrap  # noqa: E402,F401

from rdp_deploy.config import cfg_get, load_config, resolve_config_paths  # noqa: E402
from rdp_deploy.control.runtime import DeploymentRuntime, VALID_MODES  # noqa: E402
from rdp_deploy.inference.model_loader import load_policy  # noqa: E402
from rdp_deploy.paths import resolve_repo_path  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the complete RDP robot deployment."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--mode", choices=sorted(VALID_MODES))
    parser.add_argument("--duration", type=float)
    parser.add_argument("--confirm-motion")
    args = parser.parse_args()

    cfg = resolve_config_paths(load_config(args.config))
    mode = args.mode or str(cfg_get(cfg, "deployment.default_mode", "shadow"))
    duration = float(
        args.duration
        if args.duration is not None
        else cfg_get(cfg, "deployment.duration_sec", 10.0)
    )
    if duration <= 0:
        print("Deployment failed: duration must be positive")
        return 1

    try:
        print("Loading model before initializing robot hardware...")
        loaded = load_policy(
            checkpoint_path=resolve_repo_path(cfg.model.checkpoint_path),
            at_checkpoint_path=resolve_repo_path(cfg.model.at_checkpoint_path),
            device=str(cfg.model.device),
            use_ema=str(cfg.model.use_ema),
            num_inference_steps=int(cfg.model.num_inference_steps),
        )
        print(
            f"Model loaded: weights={loaded.weights_key}, device={loaded.device}"
        )
        print(f"Starting deployment mode: {mode}")
        runtime = DeploymentRuntime(cfg, loaded)
        report = runtime.run(
            mode=mode,
            duration_sec=duration,
            motion_confirmation=args.confirm_motion,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Deployment failed: {type(exc).__name__}: {exc}")
        return 1

    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
