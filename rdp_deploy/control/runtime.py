from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path

import numpy as np

from rdp_deploy.clients.forcemimic_robot_client import (
    forcemimic_robot_client_from_config,
)
from rdp_deploy.control.buffers import LatentPlanBuffer, TargetMailbox
from rdp_deploy.control.online_policy import decode_plan_entry, predict_latent_plan
from rdp_deploy.control.safety import SafetyFilter, SafetyLimits, SafetyViolation
from rdp_deploy.inference.geometry import pose_9d_to_flexiv_pose
from rdp_deploy.paths import ensure_dir
from rdp_deploy.sensors.direct_sensor_collector import DirectSensorCollector


MOTION_MODES = {"hold", "execute"}
VALID_MODES = {"shadow", *MOTION_MODES}


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class RuntimeLog:
    def __init__(self, output_dir: str | Path):
        directory = ensure_dir(output_dir)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"deployment_{stamp}.jsonl"
        self._file = open(self.path, "a", encoding="utf-8")
        self._lock = threading.Lock()

    def write(self, event: str, **data) -> None:
        record = {
            "timestamp": time.time(),
            "monotonic": time.monotonic(),
            "event": event,
            **data,
        }
        line = json.dumps(_json_value(record), ensure_ascii=False)
        with self._lock:
            self._file.write(line + "\n")
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            self._file.close()


class DeploymentRuntime:
    def __init__(self, cfg, loaded_policy):
        self.cfg = cfg
        self.loaded_policy = loaded_policy
        self.policy = loaded_policy.policy
        self.control_cfg = cfg.deployment
        self.model_cfg = cfg.model
        self.stop_event = threading.Event()
        self.plan_buffer = LatentPlanBuffer()
        self.target_mailbox = TargetMailbox()
        self.errors: queue.Queue[tuple[str, str]] = queue.Queue()
        self._tick_lock = threading.Lock()
        self._control_tick = 0
        self._stats_lock = threading.Lock()
        self._stats = {
            "inferences": 0,
            "plans": 0,
            "decoded_actions": 0,
            "commands_sent": 0,
            "control_overruns": 0,
            "sender_overruns": 0,
        }
        self.runtime_log: RuntimeLog | None = None

    def _validate_config(self) -> None:
        positive_fields = (
            "startup_timeout_sec",
            "control_fps",
            "inference_fps",
            "robot_send_fps",
            "tcp_action_update_interval",
            "observation_timeout_sec",
            "target_timeout_sec",
            "print_interval_sec",
        )
        for name in positive_fields:
            if float(getattr(self.control_cfg, name)) <= 0:
                raise ValueError(f"deployment.{name} must be positive")
        if int(self.control_cfg.latency_step) < 0:
            raise ValueError("deployment.latency_step must not be negative")
        control_fps = float(self.control_cfg.control_fps)
        inference_fps = float(self.control_cfg.inference_fps)
        ratio = control_fps / inference_fps
        if not ratio.is_integer():
            raise ValueError(
                "deployment.control_fps must be divisible by inference_fps"
            )
        observation_ratio = int(
            self.cfg.observation.temporal_downsample_ratio
        )
        model_ratio = int(
            self.model_cfg.dataset_obs_temporal_downsample_ratio
        )
        if observation_ratio != model_ratio:
            raise ValueError(
                "Observation and training temporal downsample ratios differ: "
                f"{observation_ratio} != {model_ratio}"
            )
        SafetyLimits.from_config(self.cfg.safety)

    def _tick(self) -> int:
        with self._tick_lock:
            return self._control_tick

    def _advance_tick(self) -> None:
        with self._tick_lock:
            self._control_tick += 1

    def _increment(self, key: str) -> None:
        with self._stats_lock:
            self._stats[key] += 1

    def _fail(self, source: str, exc: Exception | str) -> None:
        message = str(exc)
        try:
            self.errors.put_nowait((source, message))
        except queue.Full:
            pass
        if self.runtime_log is not None:
            self.runtime_log.write("failure", source=source, message=message)
        self.stop_event.set()

    def _latest_raw(self, collector: DirectSensorCollector) -> dict:
        timeout = float(self.control_cfg.observation_timeout_sec)
        collector.assert_devices_fresh(timeout)
        raw = collector.raw_buffer.last()
        if raw is None:
            raise RuntimeError("No synchronized raw observation is available")
        timestamp = float(np.asarray(raw["timestamp"]).reshape(-1)[-1])
        age = time.time() - timestamp
        if age > timeout:
            raise RuntimeError(
                f"Observation is stale: {age:.3f}s > {timeout:.3f}s"
            )
        return raw

    def _planner_worker(self, collector: DirectSensorCollector) -> None:
        update_interval = int(self.control_cfg.tcp_action_update_interval)
        inference_interval = int(
            float(self.control_cfg.control_fps)
            / float(self.control_cfg.inference_fps)
        )
        next_inference_tick = 0
        next_update_tick = 0
        while not self.stop_event.is_set():
            tick = self._tick()
            if tick < next_inference_tick:
                self.stop_event.wait(0.002)
                continue
            try:
                observation = collector.buffer.last()
                if observation is None:
                    self.stop_event.wait(0.01)
                    continue
                started = time.perf_counter()
                plan = predict_latent_plan(
                    policy=self.policy,
                    observation=observation,
                    dataset_obs_temporal_downsample_ratio=int(
                        self.model_cfg.dataset_obs_temporal_downsample_ratio
                    ),
                    latency_step=int(self.control_cfg.latency_step),
                    max_normalized_lowdim_abs=float(
                        self.model_cfg.max_normalized_lowdim_abs
                    ),
                )
                if int(plan.extended_obs_steps[-1]) > int(
                    self.model_cfg.decoder_horizon
                ):
                    raise ValueError(
                        "Latent plan requires more wrench history than "
                        "model.decoder_horizon"
                    )
                elapsed = time.perf_counter() - started
                self._increment("inferences")
                generation = None
                published = tick >= next_update_tick
                if published:
                    generation = self.plan_buffer.replace(
                        start_tick=tick,
                        latent_actions=plan.latent_actions,
                        base_absolute_pose=plan.base_absolute_pose,
                        extended_obs_steps=plan.extended_obs_steps,
                    )
                    self._increment("plans")
                    next_update_tick = tick + update_interval
                self.runtime_log.write(
                    "latent_plan",
                    generation=generation,
                    published=published,
                    start_tick=tick,
                    actions=len(plan.latent_actions),
                    inference_sec=elapsed,
                    base_pose=plan.base_absolute_pose,
                    normalized_lowdim_max_abs=(
                        plan.normalized_lowdim_max_abs
                    ),
                )
                next_inference_tick = tick + inference_interval
            except Exception as exc:  # noqa: BLE001
                self._fail("planner", f"{type(exc).__name__}: {exc}")

    def _control_worker(
        self,
        mode: str,
        collector: DirectSensorCollector,
        client,
        safety: SafetyFilter,
        hold_pose: np.ndarray,
    ) -> None:
        control_fps = float(self.control_cfg.control_fps)
        period = 1.0 / control_fps
        while not self.stop_event.is_set():
            started = time.perf_counter()
            tick = self._tick()
            try:
                raw = self._latest_raw(collector)
                status = client.status()
                if not status["connected"] or not status["operational"] or status["fault"]:
                    raise SafetyViolation(f"Unsafe robot status: {status}")
                current_pose = pose_9d_to_flexiv_pose(
                    raw["left_robot_tcp_pose"]
                ).astype(np.float64)
                wrench = np.asarray(
                    raw["left_robot_tcp_wrench"], dtype=np.float64
                )
                safety.check_robot_state(current_pose, wrench)

                if mode == "hold":
                    self.target_mailbox.update(hold_pose, tick, "hold")
                    self.runtime_log.write(
                        "hold_target",
                        tick=tick,
                        current_pose=current_pose,
                        target_pose=hold_pose,
                        wrench=wrench,
                    )
                else:
                    entry = self.plan_buffer.get(tick)
                    if entry is not None:
                        wrench_history = collector.recent_wrenches(
                            entry.extended_obs_step
                        )
                        absolute_action, candidate_pose = decode_plan_entry(
                            policy=self.policy,
                            latent=entry.latent,
                            base_absolute_pose=entry.base_absolute_pose,
                            wrench_history=wrench_history,
                            extended_obs_step=entry.extended_obs_step,
                            dataset_obs_temporal_downsample_ratio=int(
                                self.model_cfg.dataset_obs_temporal_downsample_ratio
                            ),
                        )
                        if mode == "shadow":
                            safety.previous_target = current_pose.copy()
                        result = safety.filter(
                            candidate_pose=candidate_pose,
                            current_pose=current_pose,
                            dt=period,
                        )
                        self._increment("decoded_actions")
                        if mode == "execute":
                            self.target_mailbox.update(
                                result.target_pose, tick, "model"
                            )
                        self.runtime_log.write(
                            "decoded_action",
                            mode=mode,
                            tick=tick,
                            plan_generation=entry.plan_generation,
                            extended_obs_step=entry.extended_obs_step,
                            absolute_action=absolute_action,
                            candidate_pose=candidate_pose,
                            safe_target_pose=result.target_pose,
                            current_pose=current_pose,
                            wrench=wrench,
                            linear_rate_limited=result.linear_rate_limited,
                            angular_rate_limited=result.angular_rate_limited,
                            candidate_jump_m=result.candidate_jump_m,
                            candidate_jump_rad=result.candidate_jump_rad,
                        )
                    elif mode == "execute":
                        self.target_mailbox.update(hold_pose, tick, "waiting_for_plan")
                self._advance_tick()
            except Exception as exc:  # noqa: BLE001
                self._fail("control", f"{type(exc).__name__}: {exc}")

            elapsed = time.perf_counter() - started
            remaining = period - elapsed
            if remaining > 0:
                self.stop_event.wait(remaining)
            else:
                self._increment("control_overruns")
                self.runtime_log.write(
                    "control_overrun",
                    tick=tick,
                    elapsed_sec=elapsed,
                    budget_sec=period,
                )

    def _sender_worker(self, client) -> None:
        send_fps = float(self.control_cfg.robot_send_fps)
        period = 1.0 / send_fps
        status_interval = max(1, int(send_fps / 10.0))
        sender_tick = 0
        while not self.stop_event.is_set():
            started = time.perf_counter()
            try:
                target = self.target_mailbox.latest()
                if target is None:
                    raise RuntimeError("No target is available for the sender")
                age = time.monotonic() - target.updated_monotonic
                timeout = float(self.control_cfg.target_timeout_sec)
                if age > timeout:
                    raise SafetyViolation(
                        f"Control target is stale: {age:.3f}s > {timeout:.3f}s"
                    )
                if sender_tick % status_interval == 0:
                    status = client.status()
                    if (
                        not status["connected"]
                        or not status["operational"]
                        or status["fault"]
                    ):
                        raise SafetyViolation(f"Unsafe robot status: {status}")
                client.send_tcp_target(target.pose)
                self._increment("commands_sent")
                sender_tick += 1
            except Exception as exc:  # noqa: BLE001
                try:
                    client.idle()
                finally:
                    self._fail("sender", f"{type(exc).__name__}: {exc}")
                return

            elapsed = time.perf_counter() - started
            remaining = period - elapsed
            if remaining > 0:
                self.stop_event.wait(remaining)
            else:
                self._increment("sender_overruns")

    def run(
        self,
        mode: str,
        duration_sec: float,
        motion_confirmation: str | None = None,
    ) -> dict:
        self._validate_config()
        if mode not in VALID_MODES:
            raise ValueError(f"Unsupported mode {mode!r}; choose from {sorted(VALID_MODES)}")
        expected_confirmation = str(self.control_cfg.motion_confirmation)
        if mode in MOTION_MODES and motion_confirmation != expected_confirmation:
            raise ValueError(
                f"{mode} sends real robot commands; pass "
                f"--confirm-motion {expected_confirmation}"
            )

        self.runtime_log = RuntimeLog(self.cfg.project.output_dir)
        self.runtime_log.write(
            "runtime_start",
            mode=mode,
            duration_sec=duration_sec,
            checkpoint=str(self.loaded_policy.checkpoint_path),
            at_checkpoint=str(self.loaded_policy.at_checkpoint_path),
        )
        client = None
        collector = None
        threads: list[threading.Thread] = []
        started_wall = time.time()
        try:
            client = forcemimic_robot_client_from_config(self.cfg)
            collector = DirectSensorCollector(self.cfg, robot_client=client)
            with collector:
                startup_timeout = float(self.control_cfg.startup_timeout_sec)
                if not collector.buffer.wait_for_item(startup_timeout):
                    raise RuntimeError(
                        f"No model observation received in {startup_timeout:.1f}s"
                    )
                raw = self._latest_raw(collector)
                start_pose = pose_9d_to_flexiv_pose(
                    raw["left_robot_tcp_pose"]
                ).astype(np.float64)
                start_wrench = np.asarray(
                    raw["left_robot_tcp_wrench"], dtype=np.float64
                )
                safety = SafetyFilter(
                    limits=SafetyLimits.from_config(self.cfg.safety),
                    start_pose=start_pose,
                    initial_target=start_pose,
                )
                safety.check_robot_state(start_pose, start_wrench)
                status = client.status()
                if not status["connected"] or not status["operational"] or status["fault"]:
                    raise SafetyViolation(f"Unsafe startup robot status: {status}")

                self.target_mailbox.update(start_pose, 0, "startup_hold")
                self.runtime_log.write(
                    "hardware_ready",
                    status=status,
                    start_pose=start_pose,
                    start_wrench=start_wrench,
                    start_gripper_width=np.asarray(
                        raw["left_robot_gripper_width"]
                    ),
                )

                if mode in {"shadow", "execute"}:
                    threads.append(threading.Thread(
                        target=self._planner_worker,
                        args=(collector,),
                        name="rdp_planner",
                        daemon=True,
                    ))
                threads.append(threading.Thread(
                    target=self._control_worker,
                    args=(mode, collector, client, safety, start_pose),
                    name="rdp_control",
                    daemon=True,
                ))
                if mode in MOTION_MODES:
                    threads.append(threading.Thread(
                        target=self._sender_worker,
                        args=(client,),
                        name="rdp_sender",
                        daemon=True,
                    ))
                for thread in threads:
                    thread.start()

                deadline = time.monotonic() + float(duration_sec)
                print_interval = float(self.control_cfg.print_interval_sec)
                next_print = time.monotonic()
                while not self.stop_event.is_set() and time.monotonic() < deadline:
                    if time.monotonic() >= next_print:
                        with self._stats_lock:
                            stats = dict(self._stats)
                        print(
                            f"[{mode}] tick={self._tick()} plans={stats['plans']} "
                            f"inferences={stats['inferences']} "
                            f"decoded={stats['decoded_actions']} "
                            f"commands={stats['commands_sent']}"
                        )
                        next_print += print_interval
                    self.stop_event.wait(0.02)
                self.stop_event.set()
                for thread in threads:
                    thread.join(timeout=3.0)
        except KeyboardInterrupt:
            self._fail("main", "KeyboardInterrupt")
        except Exception as exc:
            self._fail("main", f"{type(exc).__name__}: {exc}")
        finally:
            self.stop_event.set()
            if client is not None:
                try:
                    if mode in MOTION_MODES:
                        client.idle()
                finally:
                    client.close()

        error_list = []
        while not self.errors.empty():
            error_list.append(self.errors.get_nowait())
        with self._stats_lock:
            stats = dict(self._stats)
        report = {
            "mode": mode,
            "duration_sec": time.time() - started_wall,
            "stats": stats,
            "errors": error_list,
            "log_path": str(self.runtime_log.path),
        }
        self.runtime_log.write("runtime_stop", report=report)
        self.runtime_log.close()
        return report
