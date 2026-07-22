#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.config import load_config
from rdp_deploy.diagnostics.ros_topic_checker import compare_topics, get_ros_topics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    available = get_ros_topics()

    expected = list(cfg.sensors.get("expected_ros_topics", []))

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
