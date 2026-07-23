from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from rdp_deploy.clients.forcemimic_robot_client import forcemimic_robot_client_from_config
from rdp_deploy.config import resolve_config_paths
from rdp_deploy.diagnostics.latency_monitor import LatencyMonitor
from rdp_deploy.sensors.observation_buffer import ObservationBuffer
from rdp_deploy.sensors.observation_conversion import (
    realsense_to_observation,
    robot_states_to_observation,
    xense_to_observation,
)


@dataclass(frozen=True)
class DeviceSample:
    timestamp: float
    monotonic_time: float
    generation: int
    observation: dict


class PollingReader:
    def __init__(
        self,
        name: str,
        fps: float,
        read_fn: Callable[[], dict],
        close_fn: Callable[[], None],
    ):
        self.name = name
        self.period = 0.0 if fps <= 0 else 1.0 / float(fps)
        self._read_fn = read_fn
        self._close_fn = close_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: DeviceSample | None = None
        self._generation = 0
        self._count = 0
        self._first_timestamp: float | None = None
        self._last_timestamp: float | None = None
        self._error_count = 0
        self._errors: list[str] = []

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run,
            name=f"rdp_{self.name}",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            started = time.perf_counter()
            try:
                observation = self._read_fn()
                wall_time = time.time()
                monotonic_time = time.monotonic()
                with self._lock:
                    self._generation += 1
                    self._count += 1
                    if self._first_timestamp is None:
                        self._first_timestamp = wall_time
                    self._last_timestamp = wall_time
                    self._latest = DeviceSample(
                        timestamp=wall_time,
                        monotonic_time=monotonic_time,
                        generation=self._generation,
                        observation=observation,
                    )
            except Exception as exc:  # noqa: BLE001
                message = f"{type(exc).__name__}: {exc}"
                with self._lock:
                    self._error_count += 1
                    self._errors.append(message)
                    self._errors = self._errors[-20:]
                time.sleep(min(0.1, self.period or 0.1))

            elapsed = time.perf_counter() - started
            remaining = self.period - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)

    def latest(self) -> DeviceSample | None:
        with self._lock:
            return self._latest

    def report(self) -> dict:
        with self._lock:
            count = self._count
            first = self._first_timestamp
            last = self._last_timestamp
            latest = self._latest
            error_count = self._error_count
            errors = list(self._errors)
        duration = 0.0 if first is None or last is None else max(0.0, last - first)
        fps = 0.0 if duration <= 0.0 or count < 2 else (count - 1) / duration
        return {
            "count": count,
            "fps": fps,
            "latest_age_sec": None if latest is None else time.monotonic() - latest.monotonic_time,
            "error_count": error_count,
            "recent_errors": errors[-5:],
        }

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close_fn()


class XenseDevice:
    def __init__(self, sensor_cfg, resize_shape: tuple[int, int], marker_dimension: int):
        from xensesdk import Sensor

        self.sensor = None
        self.Sensor = Sensor
        self.sensor_name = str(sensor_cfg.sensor_name)
        self.resize_shape = resize_shape
        self.marker_dimension = marker_dimension
        self.read_rectify = bool(sensor_cfg.get("read_rectify", True))
        self.read_marker2d = bool(sensor_cfg.get("read_marker2d", True))
        self.read_force_resultant = bool(sensor_cfg.get("read_force_resultant", True))
        self.read_timestamp = bool(sensor_cfg.get("read_timestamp", True))

        kwargs = {"use_gpu": bool(sensor_cfg.get("use_gpu", True))}
        if sensor_cfg.get("config_path"):
            kwargs["config_path"] = str(sensor_cfg.config_path)
        if sensor_cfg.get("mac_address"):
            kwargs["mac_addr"] = str(sensor_cfg.mac_address)

        serial_number = str(sensor_cfg.serial_number)
        cam_id = sensor_cfg.get("cam_id")
        try:
            try:
                if cam_id is not None:
                    self.sensor = Sensor.create(serial_number, cam_id=int(cam_id), **kwargs)
                else:
                    self.sensor = Sensor.create(serial_number, **kwargs)
            except TypeError:
                if sensor_cfg.get("config_path"):
                    self.sensor = Sensor.create(
                        serial_number,
                        config_path=str(sensor_cfg.config_path),
                    )
                else:
                    self.sensor = Sensor.create(serial_number)

            if bool(sensor_cfg.get("calibrate_on_start", True)):
                self.sensor.calibrateSensor()

            self.output_types = []
            if self.read_rectify:
                self.output_types.append(Sensor.OutputType.Rectify)
            if self.read_marker2d:
                self.output_types.append(Sensor.OutputType.Marker2D)
            if self.read_force_resultant:
                self.output_types.append(Sensor.OutputType.ForceResultant)
            if self.read_timestamp:
                self.output_types.append(Sensor.OutputType.TimeStamp)
            if not self.output_types:
                raise ValueError(f"Xense {self.sensor_name} has no enabled outputs")

            self.marker_reference = None
            if self.read_marker2d:
                self.marker_reference = np.asarray(
                    self.sensor.selectSensorInfo(Sensor.OutputType.Marker2D),
                    dtype=np.float32,
                ).copy()
        except Exception:
            self.close()
            raise

    def read(self) -> dict:
        values = self.sensor.selectSensorInfo(*self.output_types)
        if len(self.output_types) == 1:
            values = (values,)
        by_type = {
            output_type: values[index]
            for index, output_type in enumerate(self.output_types)
        }
        return xense_to_observation(
            sensor_name=self.sensor_name,
            resize_shape=self.resize_shape,
            marker_dimension=self.marker_dimension,
            image=by_type.get(self.Sensor.OutputType.Rectify),
            marker=by_type.get(self.Sensor.OutputType.Marker2D),
            marker_reference=self.marker_reference,
            force_resultant=by_type.get(self.Sensor.OutputType.ForceResultant),
        )

    def close(self) -> None:
        sensor = getattr(self, "sensor", None)
        if sensor is not None:
            try:
                sensor.release()
            finally:
                self.sensor = None


class RealSenseDevice:
    def __init__(self, camera_cfg, resize_shape: tuple[int, int]):
        import pyrealsense2 as rs

        width, height = [int(value) for value in camera_cfg.get("rgb_resolution", [640, 480])]
        stream_fps = int(camera_cfg.get("stream_fps", camera_cfg.get("fps", 30)))
        serial_number = str(camera_cfg.camera_serial_number)
        self.camera_name = str(camera_cfg.camera_name)
        self.resize_shape = resize_shape
        self.rs = rs
        self.pipeline = rs.pipeline()

        connected_serials = [
            device.get_info(rs.camera_info.serial_number)
            for device in rs.context().query_devices()
        ]
        if serial_number not in connected_serials:
            raise RuntimeError(
                f"RealSense {self.camera_name} serial {serial_number} not found; "
                f"connected serials: {connected_serials}"
            )

        config = rs.config()
        config.enable_device(serial_number)
        config.enable_stream(
            rs.stream.color,
            width,
            height,
            rs.format.bgr8,
            stream_fps,
        )
        try:
            self.pipeline.start(config)
            warmup_frames = int(camera_cfg.get("warmup_frames", stream_fps))
            for _ in range(max(0, warmup_frames)):
                self.pipeline.wait_for_frames()
        except Exception:
            self.close()
            raise

    def read(self) -> dict:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"RealSense {self.camera_name} returned no color image")
        color_image = np.asanyarray(color_frame.get_data()).copy()
        return realsense_to_observation(
            self.camera_name,
            color_image,
            self.resize_shape,
        )

    def close(self) -> None:
        pipeline = getattr(self, "pipeline", None)
        if pipeline is None:
            return
        try:
            pipeline.stop()
        except Exception:
            pass
        self.pipeline = None


def merge_device_samples(
    samples: dict[str, DeviceSample],
    sync_slop_sec: float,
) -> tuple[dict | None, float | None]:
    if not samples:
        return None, None
    timestamps = [sample.timestamp for sample in samples.values()]
    skew = max(timestamps) - min(timestamps)
    if skew > sync_slop_sec:
        return None, skew
    observation = {
        "timestamp": np.array([max(timestamps)], dtype=np.float64),
    }
    for sample in samples.values():
        overlap = observation.keys() & sample.observation.keys()
        if overlap:
            raise ValueError(f"Duplicate observation keys: {sorted(overlap)}")
        observation.update(sample.observation)
    return observation, skew


class DirectSensorCollector:
    def __init__(self, cfg):
        self.cfg = resolve_config_paths(cfg)
        self.buffer = ObservationBuffer(maxlen=int(self.cfg.runtime.obs_buffer_size))
        self.latency_monitor = LatencyMonitor()
        self.readers: list[PollingReader] = []
        self._stop_event = threading.Event()
        self._aggregate_thread: threading.Thread | None = None
        self._sync_skips = 0
        self._last_sync_skew_sec: float | None = None
        self._aggregate_errors: list[str] = []
        self._build_readers()

    def _build_readers(self) -> None:
        resize_shape = tuple(
            int(value)
            for value in self.cfg.data_processing.get("image_resize_shape", [320, 240])
        )
        marker_dimension = int(self.cfg.data_processing.get("marker_dimension", 2))
        try:
            robot_cfg = self.cfg.devices.robot
            if bool(robot_cfg.get("enabled", False)):
                client = forcemimic_robot_client_from_config(self.cfg)
                self.readers.append(PollingReader(
                    name="robot",
                    fps=float(robot_cfg.get("fps", self.cfg.robot.get("read_fps", 90))),
                    read_fn=lambda: robot_states_to_observation(
                        client.get_current_robot_states(),
                        bimanual=bool(self.cfg.robot.get("bimanual", False)),
                    ),
                    close_fn=client.close,
                ))

            realsense_cfg = self.cfg.devices.realsense
            if bool(realsense_cfg.get("enabled", False)):
                for camera_cfg in realsense_cfg.get("cameras", []):
                    device = RealSenseDevice(camera_cfg, resize_shape)
                    self.readers.append(PollingReader(
                        name=f"realsense_{camera_cfg.camera_name}",
                        fps=float(camera_cfg.get("fps", 30)),
                        read_fn=device.read,
                        close_fn=device.close,
                    ))

            xense_cfg = self.cfg.devices.xense
            if bool(xense_cfg.get("enabled", False)):
                for sensor_cfg in xense_cfg.get("sensors", []):
                    device = XenseDevice(sensor_cfg, resize_shape, marker_dimension)
                    self.readers.append(PollingReader(
                        name=f"xense_{sensor_cfg.sensor_name}",
                        fps=float(sensor_cfg.get("fps", 24)),
                        read_fn=device.read,
                        close_fn=device.close,
                    ))
        except Exception:
            self.close()
            raise

        if not self.readers:
            raise ValueError("No direct sensor devices are enabled")

    def __enter__(self):
        for reader in self.readers:
            reader.start()
        self._aggregate_thread = threading.Thread(
            target=self._aggregate,
            name="rdp_observation_aggregator",
            daemon=True,
        )
        self._aggregate_thread.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _aggregate(self) -> None:
        fps = float(self.cfg.runtime.expected_fps)
        period = 0.0 if fps <= 0 else 1.0 / fps
        sync_slop = float(self.cfg.runtime.sync_slop)
        while not self._stop_event.is_set():
            started = time.perf_counter()
            samples = {
                reader.name: sample
                for reader in self.readers
                if (sample := reader.latest()) is not None
            }
            if len(samples) == len(self.readers):
                try:
                    observation, skew = merge_device_samples(samples, sync_slop)
                    self._last_sync_skew_sec = skew
                    if observation is None:
                        self._sync_skips += 1
                    else:
                        self.buffer.push(observation)
                        self.latency_monitor.add_observation(observation)
                except Exception as exc:  # noqa: BLE001
                    self._aggregate_errors.append(
                        f"{type(exc).__name__}: {exc}"
                    )
                    self._aggregate_errors = self._aggregate_errors[-20:]

            elapsed = time.perf_counter() - started
            remaining = period - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)
            elif period == 0:
                self._stop_event.wait(0.001)

    def report(self) -> dict:
        return {
            "buffer": self.buffer.stats(),
            "latency": self.latency_monitor.report(),
            "sync": {
                "slop_sec": float(self.cfg.runtime.sync_slop),
                "last_skew_sec": self._last_sync_skew_sec,
                "skipped_count": self._sync_skips,
                "recent_errors": self._aggregate_errors[-5:],
            },
            "devices": {
                reader.name: reader.report()
                for reader in self.readers
            },
        }

    def close(self) -> None:
        self._stop_event.set()
        if self._aggregate_thread is not None:
            self._aggregate_thread.join(timeout=2.0)
            self._aggregate_thread = None
        while self.readers:
            reader = self.readers.pop()
            try:
                reader.close()
            except Exception:
                pass


def collect_snapshot(cfg, timeout_sec: float | None = None) -> tuple[dict | None, dict]:
    timeout = float(timeout_sec or cfg.runtime.snapshot_timeout_sec)
    with DirectSensorCollector(cfg) as collector:
        if not collector.buffer.wait_for_item(timeout):
            return None, collector.report()
        return collector.buffer.last(), collector.report()


def collect_stream(cfg, duration_sec: float) -> tuple[list[dict], dict]:
    with DirectSensorCollector(cfg) as collector:
        deadline = time.monotonic() + float(duration_sec)
        while time.monotonic() < deadline:
            time.sleep(0.05)
        return collector.buffer.all(), collector.report()
