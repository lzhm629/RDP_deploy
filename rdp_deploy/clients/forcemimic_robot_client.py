from __future__ import annotations

from dataclasses import dataclass
import threading

import numpy as np

from rdp_deploy.grippers.xense_gripper import XenseGripper
from rdp_deploy.robots.rizon import RIZON_ID, Rizon


@dataclass
class ForcemimicRobotClient:
    robot_id: str = RIZON_ID
    tool_name: str = "hapticexoteleop"
    gripper_id: str = "1659f0e0dde0"
    gripper_name: str = "Xense"
    gripper_block: bool = False

    def __post_init__(self):
        self._robot_lock = threading.RLock()
        self._gripper_lock = threading.RLock()
        self.robot = None
        self.gripper = None
        try:
            self.robot = Rizon(tool_name=self.tool_name, robot_id=self.robot_id)
            self.gripper = XenseGripper(
                gripper_id=self.gripper_id,
                name=self.gripper_name,
                block=self.gripper_block,
            )
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        with self._gripper_lock:
            if getattr(self, "gripper", None) is not None:
                try:
                    self.gripper.close()
                except Exception:
                    pass
                self.gripper = None

        with self._robot_lock:
            if getattr(self, "robot", None) is not None:
                self.robot.close()
                self.robot = None

    def get_current_robot_states(self) -> dict:
        if self.robot is None:
            raise RuntimeError("Rizon robot is not connected")
        if self.gripper is None:
            raise RuntimeError("Xense gripper is not connected")

        with self._gripper_lock:
            gripper_status = self.gripper.read_status()
            gripper_width = gripper_status["position"]
            gripper_force = gripper_status["force"]

        # Read Rizon last so the sample timestamp represents the control state,
        # even if the fixed gripper status call was briefly delayed.
        with self._robot_lock:
            states = self.robot.robot.states()
            tcp_pose = np.asarray(states.tcp_pose, dtype=np.float64)
            wrench = np.asarray(states.ext_wrench_in_tcp, dtype=np.float64)
            tcp_vel = np.asarray(
                getattr(states, "tcp_vel", np.zeros(6)), dtype=np.float64
            )
            if tcp_vel.size < 6:
                tcp_vel = np.zeros(6, dtype=np.float64)

        return {
            "leftRobotTCP": tcp_pose[:7].tolist(),
            "rightRobotTCP": [0.0] * 7,
            "leftRobotTCPVel": tcp_vel[:6].tolist(),
            "rightRobotTCPVel": [0.0] * 6,
            "leftRobotTCPWrench": wrench[:6].tolist(),
            "rightRobotTCPWrench": [0.0] * 6,
            "leftGripperState": [gripper_width, gripper_force],
            "rightGripperState": [0.0, 0.0],
        }

    def get_current_tcp(self) -> list[float]:
        return self.get_current_robot_states()["leftRobotTCP"]

    def send_tcp_target(self, target_pose: np.ndarray) -> None:
        with self._robot_lock:
            if self.robot is None:
                raise RuntimeError("Rizon robot is not connected")
            self.robot.force_comp(target_pose)

    def idle(self) -> None:
        with self._robot_lock:
            if self.robot is not None:
                self.robot.idle()

    def status(self) -> dict:
        with self._robot_lock:
            if self.robot is None:
                return {"connected": False, "operational": False, "fault": True}
            return self.robot.status()

    def ping(self) -> tuple[bool, str]:
        try:
            states = self.get_current_robot_states()
            tcp = states["leftRobotTCP"]
        except Exception as exc:  # noqa: BLE001
            return False, f"{type(exc).__name__}: {exc}"
        return True, f"Rizon is reachable, tcp={tcp}"


def forcemimic_robot_client_from_config(cfg) -> ForcemimicRobotClient:
    robot_cfg = cfg.robot
    return ForcemimicRobotClient(
        robot_id=str(robot_cfg.get("robot_id", RIZON_ID)),
        tool_name=str(robot_cfg.get("tool_name", "hapticexoteleop")),
        gripper_id=str(robot_cfg.get("gripper_id", "1659f0e0dde0")),
        gripper_name=str(robot_cfg.get("gripper_name", "Xense")),
        gripper_block=bool(robot_cfg.get("gripper_block", False)),
    )
