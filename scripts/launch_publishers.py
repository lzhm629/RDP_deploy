#!/usr/bin/env python3
import argparse

import _bootstrap  # noqa: F401

from rdp_deploy.config import load_config
from rdp_deploy.publishers.launcher import spin_publishers_until_interrupt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    return spin_publishers_until_interrupt(cfg)


if __name__ == "__main__":
    raise SystemExit(main())
