#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.config import load_config, resolve_config_paths
from rdp_deploy.diagnostics.shape_checker import missing_observation_keys, observation_report
from rdp_deploy.sensors.observation_serializer import save_stream
from rdp_deploy.sensors.ros_sensor_subscriber import collect_stream


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--no-save", action="store_true")
    args = parser.parse_args()

    cfg = resolve_config_paths(load_config(args.config))
    duration = float(args.duration or cfg.runtime.duration_sec)
    observations, runtime_report = collect_stream(cfg, duration)
    required = list(cfg.sensors.get("required_observation_keys", []))

    missing = []
    if observations:
        missing = missing_observation_keys(observations[-1], required)
        obs_report = observation_report(observations[-1], required)
    else:
        missing = required
        obs_report = {"missing_keys": required, "summary": {}}

    report = {
        "duration_sec": duration,
        "num_observations": len(observations),
        "last_observation": obs_report,
        "runtime": runtime_report,
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))

    if not observations or missing:
        return 1

    if not args.no_save:
        path = save_stream(observations, cfg.project.output_dir, meta=report)
        print(f"Saved stream: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
