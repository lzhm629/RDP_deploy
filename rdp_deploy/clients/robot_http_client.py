from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import requests


@dataclass
class RobotHttpClient:
    server_ip: str
    server_port: int
    timeout_sec: float = 3.0

    @property
    def base_url(self) -> str:
        return f"http://{self.server_ip}:{self.server_port}"

    def _get(self, endpoint: str) -> Any:
        response = requests.get(f"{self.base_url}{endpoint}", timeout=self.timeout_sec)
        response.raise_for_status()
        return response.json()

    def get_current_robot_states(self) -> dict:
        return dict(self._get("/get_current_robot_states"))

    def get_current_tcp(self, robot_side: str = "left") -> list[float]:
        if robot_side not in {"left", "right"}:
            raise ValueError("robot_side must be 'left' or 'right'")
        return list(self._get(f"/get_current_tcp/{robot_side}"))

    def ping(self) -> tuple[bool, str]:
        try:
            self.get_current_robot_states()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return True, "robot server is reachable"


def robot_client_from_config(cfg) -> RobotHttpClient:
    robot_cfg = cfg.robot
    return RobotHttpClient(
        server_ip=str(robot_cfg.server_ip),
        server_port=int(robot_cfg.server_port),
        timeout_sec=float(robot_cfg.get("request_timeout_sec", 3.0)),
    )
