#!/usr/bin/env python3
import argparse
import importlib
import json

import _bootstrap  # noqa: F401

from rdp_deploy.clients.forcemimic_robot_client import forcemimic_robot_client_from_config
from rdp_deploy.config import load_config


def _check_import(module_name: str) -> tuple[bool, str]:
    try:
        importlib.import_module(module_name)
    except Exception as exc:  # noqa: BLE001
        return False, f"{type(exc).__name__}: {exc}"
    return True, "ok"


def _check_realsense_serials(cfg) -> dict:
    result = {
        "enabled": bool(cfg.publishers.realsense.get("enabled", False)),
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

    import pyrealsense2 as rs

    connected = [
        device.get_info(rs.camera_info.serial_number)
        for device in rs.context().query_devices()
    ]
    configured = [
        str(camera.camera_serial_number)
        for camera in cfg.publishers.realsense.get("cameras", [])
    ]
    result["connected_serials"] = connected
    result["configured_serials"] = configured
    result["missing_serials"] = [serial for serial in configured if serial not in connected]
    return result


def _check_xense(cfg) -> dict:
    result = {
        "enabled": bool(cfg.publishers.xense.get("enabled", False)),
        "import_ok": None,
        "configured_serials": [],
    }
    if not result["enabled"]:
        return result

    ok, message = _check_import("xensesdk")
    result["import_ok"] = ok
    result["configured_serials"] = [
        str(sensor.serial_number)
        for sensor in cfg.publishers.xense.get("sensors", [])
    ]
    if not ok:
        result["error"] = message
    return result


def _check_robot(cfg) -> dict:
    result = {
        "enabled": bool(cfg.publishers.robot_state.get("enabled", False)),
        "backend": str(cfg.robot.get("backend", "forcemimic_rizon")),
        "robot_id": str(cfg.robot.get("robot_id", "Rizon4s-063231")),
        "tool_name": str(cfg.robot.get("tool_name", "xense_force")),
        "gripper_id": str(cfg.robot.get("gripper_id", "d254505bfaaa")),
    }
    if not result["enabled"]:
        return result

    client = None
    try:
        client = forcemimic_robot_client_from_config(cfg)
        ok, message = client.ping()
    except Exception as exc:  # noqa: BLE001
        ok = False
        message = f"{type(exc).__name__}: {exc}"
    finally:
        if client is not None:
            client.close()
    result["connection_ok"] = ok
    result["message"] = message
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    subscribe_topics = [str(item.name) for item in cfg.sensors.get("subscribe_topics", [])]
    enabled_publish_prefixes = []
    if bool(cfg.publishers.robot_state.get("enabled", False)):
        enabled_publish_prefixes.extend([
            "/left_tcp_pose",
            "/left_gripper_state",
            "/left_tcp_vel",
            "/left_tcp_wrench",
        ])
        if bool(cfg.robot.get("bimanual", False)):
            enabled_publish_prefixes.extend([
                "/right_tcp_pose",
                "/right_gripper_state",
                "/right_tcp_vel",
                "/right_tcp_wrench",
            ])
    if bool(cfg.publishers.realsense.get("enabled", False)):
        enabled_publish_prefixes.extend([
            f"/{camera.camera_name}/color/image_raw"
            for camera in cfg.publishers.realsense.get("cameras", [])
        ])
    if bool(cfg.publishers.xense.get("enabled", False)):
        for sensor in cfg.publishers.xense.get("sensors", []):
            if bool(sensor.get("publish_rectify", True)):
                enabled_publish_prefixes.append(f"/{sensor.sensor_name}/color/image_raw")
            if bool(sensor.get("publish_marker2d", True)):
                enabled_publish_prefixes.append(f"/{sensor.sensor_name}/marker_offset/information")
            if bool(sensor.get("publish_force_resultant", True)):
                enabled_publish_prefixes.append(f"/{sensor.sensor_name}/force_resultant")

    unsubscribed_publishers = sorted(set(enabled_publish_prefixes) - set(subscribe_topics))
    subscriptions_without_configured_publisher = sorted(set(subscribe_topics) - set(enabled_publish_prefixes))

    report = {
        "robot_state": _check_robot(cfg),
        "realsense": _check_realsense_serials(cfg),
        "xense": _check_xense(cfg),
        "subscribe_topics": [dict(item) for item in cfg.sensors.get("subscribe_topics", [])],
        "topic_consistency": {
            "unsubscribed_publishers": unsubscribed_publishers,
            "subscriptions_without_configured_publisher": subscriptions_without_configured_publisher,
        },
    }

    print(json.dumps(report, indent=2, ensure_ascii=False))

    failed = False
    if report["robot_state"].get("enabled") and not report["robot_state"].get("connection_ok"):
        failed = True
    if report["realsense"].get("enabled"):
        failed = failed or not report["realsense"].get("import_ok")
        failed = failed or bool(report["realsense"].get("missing_serials"))
    if report["xense"].get("enabled"):
        failed = failed or not report["xense"].get("import_ok")
    failed = failed or bool(subscriptions_without_configured_publisher)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
