from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import dill
import hydra
import torch
from omegaconf import OmegaConf


@dataclass(frozen=True)
class LoadedPolicy:
    policy: object
    checkpoint_path: Path
    at_checkpoint_path: Path
    weights_key: str
    device: str


def _load_trusted_checkpoint(path: Path, map_location: str):
    try:
        return torch.load(
            path,
            map_location=map_location,
            pickle_module=dill,
            weights_only=False,
        )
    except TypeError:
        return torch.load(path, map_location=map_location, pickle_module=dill)


def load_policy(
    checkpoint_path: str | Path,
    at_checkpoint_path: str | Path,
    device: str = "cuda:0",
    use_ema: str = "auto",
    num_inference_steps: int = 8,
) -> LoadedPolicy:
    checkpoint = Path(checkpoint_path).expanduser().resolve()
    at_checkpoint = Path(at_checkpoint_path).expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"Policy checkpoint not found: {checkpoint}")
    if not at_checkpoint.is_file():
        raise FileNotFoundError(f"AT checkpoint not found: {at_checkpoint}")
    if use_ema not in {"auto", "true", "false"}:
        raise ValueError("use_ema must be auto, true, or false")
    if str(device).startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA was requested but is unavailable: {device}")

    # Checkpoints contain trusted Dill/OmegaConf objects and must not come from
    # an untrusted source.
    OmegaConf.register_new_resolver("eval", eval, replace=True)
    payload = _load_trusted_checkpoint(checkpoint, map_location="cpu")
    if "cfg" not in payload or "state_dicts" not in payload:
        raise ValueError("Unsupported checkpoint: cfg/state_dicts are missing")

    cfg = OmegaConf.create(OmegaConf.to_container(payload["cfg"], resolve=True))
    OmegaConf.update(cfg, "policy.at.load_dir", str(at_checkpoint), force_add=True)
    OmegaConf.update(cfg, "policy.at.device", str(device), force_add=True)
    policy = hydra.utils.instantiate(cfg.policy)

    state_dicts = payload["state_dicts"]
    training_uses_ema = bool(cfg.training.get("use_ema", False))
    wants_ema = training_uses_ema if use_ema == "auto" else use_ema == "true"
    weights_key = "ema_model" if wants_ema else "model"
    if weights_key not in state_dicts:
        available = ", ".join(sorted(state_dicts))
        raise KeyError(
            f"Checkpoint has no {weights_key!r} weights; available: {available}"
        )

    policy.load_state_dict(state_dicts[weights_key], strict=True)
    policy.at.set_normalizer(policy.normalizer)
    policy.num_inference_steps = int(num_inference_steps)
    policy.eval().to(torch.device(device))
    return LoadedPolicy(
        policy=policy,
        checkpoint_path=checkpoint,
        at_checkpoint_path=at_checkpoint,
        weights_key=weights_key,
        device=str(device),
    )
