from __future__ import annotations

from typing import Iterable

from rdp_deploy.sensors.observation_serializer import summarize_observation


def missing_observation_keys(obs: dict, required_keys: Iterable[str]) -> list[str]:
    return [key for key in required_keys if key not in obs]


def observation_report(obs: dict, required_keys: Iterable[str] = ()) -> dict:
    return {
        "missing_keys": missing_observation_keys(obs, required_keys),
        "summary": summarize_observation(obs),
    }
