from __future__ import annotations

import pickle
import time
from pathlib import Path
from typing import Any

import numpy as np
from omegaconf import OmegaConf

from rdp_deploy.paths import ensure_dir


def timestamp_slug() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def summarize_array(value: Any) -> dict:
    arr = np.asarray(value)
    summary = {
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
    }
    if arr.size > 0 and np.issubdtype(arr.dtype, np.number):
        summary.update({
            "min": float(np.nanmin(arr)),
            "max": float(np.nanmax(arr)),
            "mean": float(np.nanmean(arr)),
        })
    return summary


def summarize_observation(obs: dict) -> dict:
    return {key: summarize_array(value) for key, value in obs.items()}


def save_snapshot(obs: dict, output_dir: str | Path, prefix: str = "snapshot") -> Path:
    out_dir = ensure_dir(output_dir)
    path = out_dir / f"{prefix}_{timestamp_slug()}.pkl"
    with open(path, "wb") as f:
        pickle.dump(obs, f)
    return path


def save_stream(observations: list[dict], output_dir: str | Path, meta: dict | None = None) -> Path:
    out_dir = ensure_dir(output_dir) / f"stream_{timestamp_slug()}"
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(out_dir / "observations.pkl", "wb") as f:
        pickle.dump(observations, f)

    if observations:
        keys = sorted(set().union(*(obs.keys() for obs in observations)))
        for key in keys:
            values = [obs[key] for obs in observations if key in obs]
            try:
                np.save(out_dir / f"{key}.npy", np.stack(values, axis=0))
            except ValueError:
                obj_arr = np.empty((len(values),), dtype=object)
                obj_arr[:] = values
                np.save(out_dir / f"{key}.npy", obj_arr, allow_pickle=True)

    metadata = meta or {}
    metadata["num_observations"] = len(observations)
    if observations:
        metadata["keys"] = sorted(observations[-1].keys())
        metadata["last_observation_summary"] = summarize_observation(observations[-1])
    OmegaConf.save(OmegaConf.create(metadata), out_dir / "meta.yaml")
    return out_dir
