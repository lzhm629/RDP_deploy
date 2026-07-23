from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlanEntry:
    latent: np.ndarray
    base_absolute_pose: np.ndarray
    extended_obs_step: int
    plan_generation: int


class LatentPlanBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._entries: dict[int, PlanEntry] = {}
        self._generation = 0

    def replace(
        self,
        start_tick: int,
        latent_actions: np.ndarray,
        base_absolute_pose: np.ndarray,
        extended_obs_steps: np.ndarray,
    ) -> int:
        latent = np.asarray(latent_actions, dtype=np.float32)
        steps = np.asarray(extended_obs_steps, dtype=np.int64)
        if latent.ndim != 2 or len(latent) != len(steps):
            raise ValueError(
                f"Incompatible latent plan shapes: {latent.shape}, {steps.shape}"
            )
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._entries = {
                start_tick + index: PlanEntry(
                    latent=latent[index].copy(),
                    base_absolute_pose=np.asarray(
                        base_absolute_pose, dtype=np.float32
                    ).reshape(9).copy(),
                    extended_obs_step=int(steps[index]),
                    plan_generation=generation,
                )
                for index in range(len(latent))
            }
            return generation

    def get(self, tick: int) -> PlanEntry | None:
        with self._lock:
            return self._entries.get(tick)


@dataclass(frozen=True)
class TargetState:
    pose: np.ndarray
    updated_monotonic: float
    control_tick: int
    source: str


class TargetMailbox:
    def __init__(self):
        self._lock = threading.Lock()
        self._target: TargetState | None = None

    def update(self, pose: np.ndarray, control_tick: int, source: str) -> None:
        target = np.asarray(pose, dtype=np.float64).reshape(7)
        with self._lock:
            self._target = TargetState(
                pose=target.copy(),
                updated_monotonic=time.monotonic(),
                control_tick=int(control_tick),
                source=str(source),
            )

    def latest(self) -> TargetState | None:
        with self._lock:
            return self._target
