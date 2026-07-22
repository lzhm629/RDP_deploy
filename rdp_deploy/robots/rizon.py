from __future__ import annotations

import logging
import time

import numpy as np


logger = logging.getLogger(__name__)

RIZON_ID = "Rizon4s-063586"

RIZON_MAX_CONTACT_FORCE = 30
RIZON_MAX_CONTACT_TORQUE = 5

RIZON_TCP_MAX_VEL = (0.1, 0.2)
RIZON_TCP_MAX_ACC = (0.1, 0.2)
RIZON_TCP_POSE_EPSILON = (0.001, 0.01)

RIZON_JOINT_MAX_VEL = 0.1
RIZON_JOINT_MAX_ACC = 0.1
RIZON_JOINT_EPSILON = 0.01
RIZON_HOME_JOINTS = np.deg2rad([0, -40, 0, 90, 0, 40, 0])


class Rizon:
    """Minimal deployment wrapper for the Flexiv Rizon robot."""

    def __init__(self, tool_name: str = "Flange", robot_id: str = RIZON_ID):
        try:
            import flexivrdk
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "flexivrdk is required for the Rizon robot publisher. "
                "Install the Flexiv RDK Python package on the deployment computer."
            ) from exc

        self.flexivrdk = flexivrdk
        self.tool_name = tool_name
        self.robot_id = robot_id
        self.DOF = 7

        logger.info("Initializing Rizon robot %s with tool: %s", robot_id, tool_name)
        self.robot = flexivrdk.Robot(robot_id)
        info = self.robot.info()
        self._joint_limits = (np.array(info.q_min), np.array(info.q_max))

        if self.robot.fault():
            logger.warning("Fault occurred on the connected robot, trying to clear ...")
            if not self.robot.ClearFault():
                raise RuntimeError("Fault cannot be cleared on the connected Rizon robot")
            logger.info("Fault on the connected robot is cleared")

        self.robot.Enable()
        while not self.robot.operational():
            time.sleep(1)
        logger.info("Robot is now operational")

        self.robot.SwitchMode(flexivrdk.Mode.IDLE)
        tool = flexivrdk.Tool(self.robot)
        if tool.exist(tool_name):
            tool.Switch(tool_name)
        else:
            raise RuntimeError(f"{tool_name} tool not found")

        self.robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.ExecutePrimitive("ZeroFTSensor", dict())
        while not self.robot.primitive_states()["terminated"]:
            time.sleep(0.1)
        logger.debug("FT sensor zeroed")

    def close(self) -> None:
        try:
            if getattr(self, "robot", None) is None:
                return
            try:
                self.robot.SwitchMode(self.flexivrdk.Mode.IDLE)
            except Exception:
                pass
            if self.robot.connected():
                try:
                    self.robot.Stop()
                except Exception:
                    pass
                try:
                    if self.robot.fault():
                        self.robot.ClearFault()
                except Exception:
                    pass
        except Exception as exc:  # noqa: BLE001
            logger.warning("Error during robot cleanup: %s", exc)
        finally:
            self.robot = None
            logger.debug("Robot connection closed")

    def force_comp(self, target_pose: np.ndarray) -> None:
        self.robot.SendCartesianMotionForce(
            target_pose,
            max_linear_vel=0.02,
            max_angular_vel=0.05,
            max_angular_acc=0.05,
            max_linear_acc=0.05,
        )
