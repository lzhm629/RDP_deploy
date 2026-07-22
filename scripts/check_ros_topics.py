#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.clients.device_mapping_client import device_mapping_client_from_config
from rdp_deploy.config import load_config
from rdp_deploy.diagnostics.ros_topic_checker import compare_topics, get_ros_topics


def _topics_from_device_mapping(cfg) -> list[str]:
    from reactive_diffusion_policy.real_world.device_mapping.device_mapping_utils import get_topic_and_type

    mapping = device_mapping_client_from_config(cfg).get_mapping_model()
    return [topic for topic, _msg_type in get_topic_and_type(mapping)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--no-device-mapping", action="store_true")
    args = parser.parse_args()

    cfg = load_config(args.config)
    available = get_ros_topics()

    expected = list(cfg.sensors.get("expected_ros_topics", []))
    if not args.no_device_mapping:
        expected = sorted(set(expected + _topics_from_device_mapping(cfg)))

    report = compare_topics(available, expected)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if report["missing"]:
        print("")
        print("Missing topics:")
        for topic in report["missing"]:
            print(f"  {topic}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
