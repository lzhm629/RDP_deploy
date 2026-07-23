#!/usr/bin/env python3
import argparse
import importlib
import json

import _bootstrap  # noqa: F401

from rdp_deploy.config import load_config


def _check_import(module_name: str) -> tuple[bool, str]:
    try:
        importlib.import_module(module_name)
    except Exception as exc:  # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}"
    return True, "ok"


def _check_realsense_serials(cfg) -> dict:
    result = {
        "enabled": bool(cfg.devices.realsense.get("enabled", False)),
        "import_ok": None,
        "connected_serials": [],
        "configured_serials": [],
        "missing_serials": [],
    }
    if not result["enabled"]:
        return result

    ok, message = _check_import("pyrealsense2")
    result["import_ok"] = ok
    if not ok:
        result["error"] = message
        return result
    r3kit_ok, r3kit_message = _check_import(
        "r3kit.devices.camera.realsense.general"
    )
    result["r3kit_camera_import_ok"] = r3kit_ok
    if not r3kit_ok:
        result["error"] = r3kit_message
        result["import_ok"] = False
        return result

    import pyrealsense2 as rs

    connected = [
        device.get_info(rs.camera_info.serial_number)
        for device in rs.context().query_devices()
    ]
    configured = [
        str(camera.camera_serial_number)
        for camera in cfg.devices.realsense.get("cameras", [])
    ]
    result["connected_serials"] = connected
    result["configured_serials"] = configured
    result["missing_serials"] = [serial for serial in configured if serial not in connected]
    return result


def _check_xense(cfg) -> dict:
    result = {
        "enabled": bool(cfg.devices.xense.get("enabled", False)),
        "import_ok": None,
        "configured_serials": [],
    }
    if not result["enabled"]:
        return result

    ok, message = _check_import("xensesdk")
    result["import_ok"] = ok
    result["configured_serials"] = [
        str(sensor.serial_number)
        for sensor in cfg.devices.xense.get("sensors", [])
    ]
    if not ok:
        result["error"] = message
    return result


def _check_robot(cfg) -> dict:
    result = {
        "enabled": bool(cfg.devices.robot.get("enabled", False)),
        "backend": str(cfg.robot.get("backend", "forcemimic_rizon")),
        "robot_id": str(cfg.robot.get("robot_id", "Rizon4s-063586")),
        "tool_name": str(cfg.robot.get("tool_name", "hapticexoteleop")),
        "gripper_id": str(cfg.robot.get("gripper_id", "d254505bfaaa")),
    }
    if not result["enabled"]:
        return result

    flexiv_ok, flexiv_message = _check_import("flexivrdk")
    r3kit_ok, r3kit_message = _check_import(
        "r3kit.devices.gripper.xense.xense"
    )
    result["imports"] = {
        "flexivrdk": {"ok": flexiv_ok, "message": flexiv_message},
        "r3kit": {"ok": r3kit_ok, "message": r3kit_message},
    }
    result["import_ok"] = flexiv_ok and r3kit_ok
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)

    report = {
        "robot": _check_robot(cfg),
        "realsense": _check_realsense_serials(cfg),
        "xense": _check_xense(cfg),
        "required_observation_keys": list(cfg.observation.get("required_keys", [])),
    }

    print(json.dumps(report, indent=2, ensure_ascii=False))

    failed = False
    if report["robot"].get("enabled") and not report["robot"].get("import_ok"):
        failed = True
    if report["realsense"].get("enabled"):
        failed = failed or not report["realsense"].get("import_ok")
        failed = failed or bool(report["realsense"].get("missing_serials"))
    if report["xense"].get("enabled"):
        failed = failed or not report["xense"].get("import_ok")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
