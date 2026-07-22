#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time
from pathlib import Path

import _bootstrap  # noqa: F401


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--no-save", action="store_true")
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    launch_cmd = [
        sys.executable,
        str(script_dir / "launch_publishers.py"),
        "--config",
        args.config,
    ]
    collect_cmd = [
        sys.executable,
        str(script_dir / "collect_sensor_stream.py"),
        "--config",
        args.config,
        "--duration",
        str(args.duration),
    ]
    if args.no_save:
        collect_cmd.append("--no-save")

    publisher_proc = subprocess.Popen(launch_cmd)
    try:
        time.sleep(args.warmup)
        if publisher_proc.poll() is not None:
            print(f"Publisher process exited early with code {publisher_proc.returncode}")
            return int(publisher_proc.returncode or 1)
        return subprocess.call(collect_cmd)
    finally:
        if publisher_proc.poll() is None:
            publisher_proc.terminate()
            try:
                publisher_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                publisher_proc.kill()
                publisher_proc.wait(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
