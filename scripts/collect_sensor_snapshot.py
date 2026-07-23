#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.config import load_config, resolve_config_paths
from rdp_deploy.diagnostics.shape_checker import observation_report
from rdp_deploy.sensors.direct_sensor_collector import collect_snapshot
from rdp_deploy.sensors.observation_serializer import save_snapshot


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--no-save", action="store_true")
    args = parser.parse_args()

    cfg = resolve_config_paths(load_config(args.config))
    try:
        obs, runtime_report = collect_snapshot(cfg, timeout_sec=args.timeout)
    except Exception as exc:  # noqa: BLE001
        print(f"Direct sensor collection failed: {type(exc).__name__}: {exc}")
        return 1
    if obs is None:
        print("No synchronized observation received before timeout.")
        print(json.dumps(runtime_report, indent=2, ensure_ascii=False))
        return 1

    required = list(cfg.observation.get("required_keys", []))
    report = observation_report(obs, required)
    print(json.dumps({
        "observation": report,
        "runtime": runtime_report,
    }, indent=2, ensure_ascii=False))

    if report["missing_keys"]:
        return 1

    if not args.no_save and bool(cfg.collection.get("save_snapshot_pickle", True)):
        path = save_snapshot(obs, cfg.project.output_dir)
        print(f"Saved snapshot: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
