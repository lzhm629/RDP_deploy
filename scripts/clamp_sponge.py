#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
import warnings
from typing import Any

import _bootstrap  # noqa: F401

from rdp_deploy.config import load_config


DEFAULT_POSITION_MM = 14.6
DEFAULT_SPEED_MM_S = 10.0
DEFAULT_FORCE_N = 10.0


def _finite_in_range(name: str, value: float, lower: float, upper: float) -> float:
    value = float(value)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} must be finite and in [{lower}, {upper}], got {value}")
    return value


def _read_status(gripper: Any) -> dict[str, float]:
    status = gripper.get_gripper_status()
    if not isinstance(status, dict):
        raise RuntimeError(
            f"Xense gripper returned invalid status: {type(status).__name__}"
        )

    required = ("position", "velocity", "force", "temperature")
    missing = [key for key in required if key not in status]
    if missing:
        raise RuntimeError(f"Xense gripper status is missing fields: {missing}")

    result = {key: float(status[key]) for key in required}
    if not all(math.isfinite(value) for value in result.values()):
        raise RuntimeError(f"Xense gripper returned non-finite status: {result}")
    return result


def _close_gripper(gripper: Any) -> None:
    for method_name in ("close", "disconnect"):
        method = getattr(gripper, method_name, None)
        if callable(method):
            try:
                method()
            except Exception:
                pass
            return


def _create_gripper(gripper_id: str) -> Any:
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="Failed to install retiles glfw shim before ezgl import.*",
        )
        from xensegripper import XenseGripper

    return XenseGripper.create(mac_addr=gripper_id)


def _confirmation_token(gripper_id: str) -> str:
    return f"CLAMP_SPONGE_{gripper_id.upper()}"


def _monitor_motion(
    gripper: Any,
    *,
    target_mm: float,
    min_hold_force_n: float,
    max_temperature_c: float,
    timeout_sec: float,
    poll_interval_sec: float,
    velocity_tolerance_mm_s: float,
    position_tolerance_mm: float,
    stable_samples: int,
) -> dict[str, Any]:
    started_at = time.monotonic()
    stable_count = 0
    peak_force_n = 0.0
    samples = 0
    last_status: dict[str, float] | None = None

    while True:
        status = _read_status(gripper)
        samples += 1
        last_status = status
        elapsed = time.monotonic() - started_at
        force_n = abs(status["force"])
        velocity_mm_s = abs(status["velocity"])
        peak_force_n = max(peak_force_n, force_n)

        print(
            "\r"
            f"position={status['position']:7.3f} mm  "
            f"velocity={status['velocity']:+7.3f} mm/s  "
            f"force={status['force']:+7.3f} N  "
            f"temperature={status['temperature']:6.2f} C",
            end="",
            flush=True,
        )

        if status["temperature"] > max_temperature_c:
            print()
            raise RuntimeError(
                f"gripper temperature {status['temperature']:.2f} C exceeds "
                f"limit {max_temperature_c:.2f} C"
            )

        if elapsed >= 0.5 and velocity_mm_s <= velocity_tolerance_mm_s:
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= stable_samples:
            break
        if elapsed >= timeout_sec:
            break
        time.sleep(poll_interval_sec)

    print()
    assert last_status is not None
    elapsed = time.monotonic() - started_at
    position_error_mm = abs(last_status["position"] - target_mm)
    target_reached = position_error_mm <= position_tolerance_mm
    contact_detected = peak_force_n >= min_hold_force_n
    settled = stable_count >= stable_samples

    if not settled:
        outcome = "timeout_while_moving"
    elif contact_detected:
        outcome = "settled_with_contact"
    elif target_reached:
        outcome = "target_reached_without_confirmed_contact"
    else:
        outcome = "settled_without_confirmed_contact"

    return {
        "outcome": outcome,
        "success": bool(settled and contact_detected),
        "target_reached": target_reached,
        "contact_detected": contact_detected,
        "settled": settled,
        "elapsed_sec": elapsed,
        "samples": samples,
        "peak_abs_force_n": peak_force_n,
        "position_error_mm": position_error_mm,
        "final_status": last_status,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read or command the Xense gripper without connecting to the Flexiv robot. "
            "Running without --execute is status-only."
        )
    )
    parser.add_argument(
        "--config",
        default="configs/deploy_wipedish_sensor_only.yaml",
        help="deployment config containing robot.gripper_id",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually send the clamp command; omitted means status-only",
    )
    parser.add_argument(
        "--confirm",
        help="required motion confirmation token printed by a status-only run",
    )
    parser.add_argument("--position-mm", type=float, default=DEFAULT_POSITION_MM)
    parser.add_argument("--speed-mm-s", type=float, default=DEFAULT_SPEED_MM_S)
    parser.add_argument("--force-n", type=float, default=DEFAULT_FORCE_N)
    parser.add_argument("--min-hold-force-n", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--poll-interval-sec", type=float, default=0.1)
    parser.add_argument("--max-temperature-c", type=float, default=70.0)
    parser.add_argument("--velocity-tolerance-mm-s", type=float, default=0.5)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.5)
    parser.add_argument("--stable-samples", type=int, default=5)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    cfg = load_config(args.config)
    gripper_id = str(cfg.robot.gripper_id)
    token = _confirmation_token(gripper_id)

    try:
        position_mm = _finite_in_range("position_mm", args.position_mm, 0.0, 85.0)
        speed_mm_s = _finite_in_range("speed_mm_s", args.speed_mm_s, 0.0, 350.0)
        force_n = _finite_in_range("force_n", args.force_n, 0.0, 60.0)
        min_hold_force_n = _finite_in_range(
            "min_hold_force_n", args.min_hold_force_n, 0.0, 60.0
        )
        timeout_sec = _finite_in_range("timeout_sec", args.timeout_sec, 0.1, 60.0)
        poll_interval_sec = _finite_in_range(
            "poll_interval_sec", args.poll_interval_sec, 0.01, 1.0
        )
        max_temperature_c = _finite_in_range(
            "max_temperature_c", args.max_temperature_c, 20.0, 100.0
        )
        velocity_tolerance = _finite_in_range(
            "velocity_tolerance_mm_s",
            args.velocity_tolerance_mm_s,
            0.0,
            20.0,
        )
        position_tolerance = _finite_in_range(
            "position_tolerance_mm", args.position_tolerance_mm, 0.0, 10.0
        )
        if args.stable_samples < 1 or args.stable_samples > 100:
            raise ValueError("stable_samples must be in [1, 100]")
        if min_hold_force_n > force_n:
            raise ValueError("min_hold_force_n cannot exceed force_n")
        if args.execute and args.confirm != token:
            raise ValueError(
                "motion confirmation mismatch; run without --execute to print "
                "the required token"
            )
    except ValueError as exc:
        print(f"Gripper command rejected: {exc}")
        return 2

    gripper = None
    try:
        gripper = _create_gripper(gripper_id)
        initial_status = _read_status(gripper)
        print(
            json.dumps(
                {
                    "gripper_id": gripper_id,
                    "mode": "execute" if args.execute else "status-only",
                    "initial_status": initial_status,
                    "command": {
                        "position_mm": position_mm,
                        "speed_mm_s": speed_mm_s,
                        "force_n": force_n,
                    },
                    "confirmation_token": token,
                    "flexiv_robot_initialized": False,
                },
                indent=2,
                ensure_ascii=False,
            )
        )

        if not args.execute:
            print("Status-only: no gripper motion command was sent.")
            return 0

        print("Sending one Xense position command. Press Ctrl+C to stop monitoring.")
        gripper.set_position(position_mm, speed_mm_s, force_n)
        result = _monitor_motion(
            gripper,
            target_mm=position_mm,
            min_hold_force_n=min_hold_force_n,
            max_temperature_c=max_temperature_c,
            timeout_sec=timeout_sec,
            poll_interval_sec=poll_interval_sec,
            velocity_tolerance_mm_s=velocity_tolerance,
            position_tolerance_mm=position_tolerance,
            stable_samples=args.stable_samples,
        )
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if result["success"]:
            print("Sponge clamp verified: gripper settled with measured contact force.")
            return 0

        print(
            "Clamp was not verified. The commanded target remains active; inspect the "
            "sponge and status before issuing another command."
        )
        return 1
    except KeyboardInterrupt:
        print(
            "\nMonitoring interrupted. No release command was sent; inspect the gripper."
        )
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"Gripper control failed: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if gripper is not None:
            _close_gripper(gripper)


if __name__ == "__main__":
    raise SystemExit(main())
