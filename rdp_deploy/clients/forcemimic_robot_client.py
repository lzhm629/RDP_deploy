from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rdp_deploy.grippers.xense_gripper import XenseGripper
from rdp_deploy.robots.rizon import RIZON_ID, Rizon


@dataclass
class ForcemimicRobotClient:
    robot_id: str = RIZON_ID
    tool_name: str = "hapticexoteleop"
    gripper_id: str = "d254505bfaaa"
    gripper_name: str = "Xense"
    gripper_block: bool = False

    def __post_init__(self):
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
        if getattr(self, "gripper", None) is not None:
            try:
                self.gripper.close()
            except Exception:
                pass
            self.gripper = None

        if getattr(self, "robot", None) is not None:
            self.robot.close()
            self.robot = None

    def get_current_robot_states(self) -> dict:
        if self.robot is None:
            raise RuntimeError("Rizon robot is not connected")
        if self.gripper is None:
            raise RuntimeError("Xense gripper is not connected")

        states = self.robot.robot.states()
        tcp_pose = np.asarray(states.tcp_pose, dtype=np.float64)
        wrench = np.asarray(states.ext_wrench_in_tcp, dtype=np.float64)
        tcp_vel = np.asarray(getattr(states, "tcp_vel", np.zeros(6)), dtype=np.float64)
        if tcp_vel.size < 6:
            tcp_vel = np.zeros(6, dtype=np.float64)

        try:
            gripper_width = float(self.gripper.read())
        except Exception:
            gripper_width = 0.0

        return {
            "leftRobotTCP": tcp_pose[:7].tolist(),
            "rightRobotTCP": [0.0] * 7,
            "leftRobotTCPVel": tcp_vel[:6].tolist(),
            "rightRobotTCPVel": [0.0] * 6,
            "leftRobotTCPWrench": wrench[:6].tolist(),
            "rightRobotTCPWrench": [0.0] * 6,
            "leftGripperState": [gripper_width, 0.0],
            "rightGripperState": [0.0, 0.0],
        }

    def get_current_tcp(self) -> list[float]:
        return self.get_current_robot_states()["leftRobotTCP"]

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
        gripper_id=str(robot_cfg.get("gripper_id", "d254505bfaaa")),
        gripper_name=str(robot_cfg.get("gripper_name", "Xense")),
        gripper_block=bool(robot_cfg.get("gripper_block", False)),
    )
