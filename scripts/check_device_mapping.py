#!/usr/bin/env python3
import argparse
import json

import _bootstrap  # noqa: F401

from rdp_deploy.clients.device_mapping_client import device_mapping_client_from_config
from rdp_deploy.config import load_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    if not bool(cfg.device_mapping.get("enabled", False)):
        print("Device mapping check skipped: device_mapping.enabled is false.")
        return 0

    client = device_mapping_client_from_config(cfg)
    ok, message = client.ping()
    print(f"Device mapping server: {message}")
    if not ok:
        return 1

    mapping = client.get_mapping_json()
    print(json.dumps(mapping, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
