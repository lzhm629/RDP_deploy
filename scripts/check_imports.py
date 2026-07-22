#!/usr/bin/env python3
import importlib

import _bootstrap  # noqa: F401


MODULES = [
    "numpy",
    "cv2",
    "requests",
    "omegaconf",
    "loguru",
    "rdp_deploy",
    "rclpy",
    "message_filters",
    "sensor_msgs",
    "geometry_msgs",
    "std_msgs",
]


def main() -> int:
    failed = []
    for name in MODULES:
        try:
            importlib.import_module(name)
            print(f"[OK] {name}")
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] {name}: {type(exc).__name__}: {exc}")
            failed.append(name)

    if failed:
        print("")
        print("Missing imports usually mean one of these is needed:")
        print("  source /opt/ros/jazzy/setup.bash")
        print("  source rdp_deploy_venv/bin/activate")
        print("  export PYTHONPATH=$PWD/RDP_deploy:$PYTHONPATH")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
