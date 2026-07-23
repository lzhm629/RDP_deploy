#!/usr/bin/env python3
import argparse
import importlib
import os
import sys
from importlib import metadata

import _bootstrap  # noqa: F401


CHECKS = {
    "core": [
        ("numpy", "numpy", "1.26.4"),
        ("cv2", "opencv-python", "4.10.0.84"),
        ("yaml", "PyYAML", "6.0.2"),
        ("omegaconf", "omegaconf", None),
        ("loguru", "loguru", "0.7.3"),
        ("transforms3d", "transforms3d", None),
        ("scipy", "scipy", "1.13.1"),
        ("rdp_deploy", None, None),
    ],
    "hardware": [
        ("flexivrdk", "flexivrdk", "1.9.0"),
        ("r3kit", "r3kit", "0.0.2"),
        ("xensesdk", "xensesdk", "2.0.0"),
        ("pyrealsense2", "pyrealsense2", "2.53.1.4623"),
    ],
    "ros": [
        ("rclpy", None, None),
        ("message_filters", None, None),
        ("sensor_msgs", None, None),
        ("geometry_msgs", None, None),
        ("std_msgs", None, None),
    ],
}


def _package_version(distribution: str | None) -> str | None:
    if distribution is None:
        return None
    try:
        return metadata.version(distribution)
    except metadata.PackageNotFoundError:
        return None


def _check_group(group: str) -> list[str]:
    failed = []
    print(f"\n[{group}]")
    for module_name, distribution, expected_version in CHECKS[group]:
        installed_version = _package_version(distribution)
        version_text = f" {installed_version}" if installed_version else ""
        try:
            module = importlib.import_module(module_name)
            module_path = getattr(module, "__file__", None) or "<namespace>"
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] {module_name}{version_text}: {type(exc).__name__}: {exc}")
            failed.append(module_name)
            continue

        if expected_version and installed_version != expected_version:
            print(
                f"[FAIL] {module_name} {installed_version or 'unknown'}: "
                f"expected {expected_version}; path={module_path}"
            )
            failed.append(module_name)
            continue
        print(f"[OK] {module_name}{version_text}: {module_path}")
    return failed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scope",
        choices=["all", *CHECKS],
        default="all",
        help="dependency group to check",
    )
    args = parser.parse_args()

    print(f"Python: {sys.version.split()[0]}")
    print(f"Executable: {sys.executable}")
    print(f"Conda: {os.environ.get('CONDA_PREFIX', '<not active>')}")
    print(f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', '<not sourced>')}")

    groups = list(CHECKS) if args.scope == "all" else [args.scope]
    failed = []
    for group in groups:
        failed.extend(_check_group(group))

    if "ros" in groups and sys.version_info[:2] != (3, 12):
        print(
            "\n[FAIL] ROS2 Jazzy apt packages require the system Python 3.12 ABI; "
            f"the active interpreter is Python {sys.version_info.major}.{sys.version_info.minor}."
        )
        failed.append("python_ros_abi")

    if failed:
        print("\nFailed checks: " + ", ".join(failed))
        if "ros" in groups:
            print("Load ROS2 with: source /opt/ros/jazzy/setup.bash")
        if "core" in groups:
            print("Install project-only dependencies with: python -m pip install -r requirements.txt")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
