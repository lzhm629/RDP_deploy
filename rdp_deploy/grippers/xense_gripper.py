from __future__ import annotations


class XenseGripper:
    """Thin deployment wrapper around the Xense gripper class."""

    def __init__(self, gripper_id: str, name: str = "Xense", block: bool = False):
        try:
            from r3kit.devices.gripper.xense.xense import Xense
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "r3kit is required for the Xense gripper. Install the r3kit Python "
                "package, or add the r3kit package root to PYTHONPATH on the deployment computer."
            ) from exc

        self.gripper = Xense(id=gripper_id, name=name)
        self.gripper.block(block)

    def read(self) -> float:
        return float(self.gripper.read())

    def close(self) -> None:
        close = getattr(self.gripper, "close", None)
        if callable(close):
            close()
