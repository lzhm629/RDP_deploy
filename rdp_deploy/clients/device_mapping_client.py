from __future__ import annotations

from dataclasses import dataclass
import requests


@dataclass
class DeviceMappingClient:
    server_ip: str
    server_port: int
    timeout_sec: float = 3.0

    @property
    def base_url(self) -> str:
        return f"http://{self.server_ip}:{self.server_port}"

    def get_mapping_json(self) -> dict:
        response = requests.get(f"{self.base_url}/get_mapping", timeout=self.timeout_sec)
        response.raise_for_status()
        return dict(response.json())

    def ping(self) -> tuple[bool, str]:
        try:
            self.get_mapping_json()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return True, "device mapping server is reachable"


def device_mapping_client_from_config(cfg) -> DeviceMappingClient:
    mapping_cfg = cfg.device_mapping
    return DeviceMappingClient(
        server_ip=str(mapping_cfg.server_ip),
        server_port=int(mapping_cfg.server_port),
        timeout_sec=float(mapping_cfg.get("request_timeout_sec", 3.0)),
    )
