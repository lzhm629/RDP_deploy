#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.clients.forcemimic_robot_client import forcemimic_robot_client_from_config
from rdp_deploy.config import load_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    client = None
    try:
        client = forcemimic_robot_client_from_config(cfg)
        ok, message = client.ping()
        states = client.get_current_robot_states() if ok else {}
    except Exception as exc:  # noqa: BLE001
        ok = False
        message = f"{type(exc).__name__}: {exc}"
        states = {}
    finally:
        if client is not None:
            client.close()

    print(f"Robot connection: {message}")
    if states:
        print(json.dumps({
            "backend": cfg.robot.get("backend", "forcemimic_rizon"),
            "robot_id": cfg.robot.get("robot_id", "Rizon4s-063586"),
            "tool_name": cfg.robot.get("tool_name", "xense_force"),
            "left_tcp": states.get("leftRobotTCP"),
            "left_tcp_vel": states.get("leftRobotTCPVel"),
            "left_tcp_wrench": states.get("leftRobotTCPWrench"),
            "left_gripper_state": states.get("leftGripperState"),
        }, indent=2, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
