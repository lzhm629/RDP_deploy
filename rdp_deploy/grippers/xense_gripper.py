from __future__ import annotations

import warnings


class XenseGripper:
    """Read-only deployment wrapper around the Xense gripper SDK."""

    def __init__(self, gripper_id: str, name: str = "Xense", block: bool = False):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="Failed to install retiles glfw shim before ezgl import.*",
                )
                from xensegripper import XenseGripper as SDKXenseGripper
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "xensegripper is required for direct Xense gripper state acquisition."
            ) from exc

        self.name = str(name)
        self.block = bool(block)
        self.gripper = SDKXenseGripper.create(mac_addr=str(gripper_id))

    def read(self) -> float:
        return self.read_status()["position"]

    def read_status(self) -> dict[str, float]:
        status = self.gripper.get_gripper_status()
        if not isinstance(status, dict):
            raise RuntimeError(
                f"Xense gripper returned invalid status: {type(status).__name__}"
            )
        if "position" not in status:
            raise RuntimeError(
                f"Xense gripper status has no position field: {sorted(status)}"
            )
        return {
            "position": float(status["position"]),
            "velocity": float(status.get("velocity", 0.0)),
            "force": float(status.get("force", 0.0)),
            "temperature": float(status.get("temperature", 0.0)),
        }

    def close(self) -> None:
        gripper = getattr(self, "gripper", None)
        if gripper is None:
            return
        for method_name in ("close", "disconnect"):
            method = getattr(gripper, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
                break
        self.gripper = None
