from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class LatencyMonitor:
    values: list[float] = field(default_factory=list)

    def add_observation(self, obs: dict) -> None:
        timestamp = obs.get("timestamp")
        if timestamp is None:
            return
        arr = np.asarray(timestamp).reshape(-1)
        if arr.size == 0:
            return
        self.values.append(time.time() - float(arr[-1]))

    def report(self) -> dict:
        if not self.values:
            return {"count": 0}
        arr = np.asarray(self.values, dtype=np.float64)
        return {
            "count": int(arr.size),
            "min_sec": float(np.min(arr)),
            "max_sec": float(np.max(arr)),
            "mean_sec": float(np.mean(arr)),
            "p95_sec": float(np.percentile(arr, 95)),
        }
