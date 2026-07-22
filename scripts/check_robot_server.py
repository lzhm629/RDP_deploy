#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.clients.robot_http_client import robot_client_from_config
from rdp_deploy.config import load_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    client = robot_client_from_config(cfg)
    ok, message = client.ping()
    print(f"Robot server: {message}")
    if not ok:
        return 1

    states = client.get_current_robot_states()
    arm = str(cfg.robot.get("arm", "left"))
    tcp = client.get_current_tcp(arm)
    print(json.dumps({
        "base_url": client.base_url,
        "arm": arm,
        "current_tcp": tcp,
        "left_gripper_state": states.get("leftGripperState"),
        "left_tcp_wrench": states.get("leftRobotTCPWrench"),
        "bimanual": bool(cfg.robot.get("bimanual", False)),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
