from __future__ import annotations

import threading
import time

from rdp_deploy.config import resolve_config_paths
from rdp_deploy.diagnostics.latency_monitor import LatencyMonitor
from rdp_deploy.sensors.message_conversion import convert_topic_dict_to_observation
from rdp_deploy.sensors.observation_buffer import ObservationBuffer
from rdp_deploy.sensors.topic_mapping import get_topic_and_type_from_config


def create_sensor_node(cfg):
    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from rclpy.node import Node

    resolved_cfg = resolve_config_paths(cfg)
    subs_name_type = get_topic_and_type_from_config(list(resolved_cfg.sensors.subscribe_topics))

    class SensorOnlyRosNode(Node):
        def __init__(self):
            super().__init__("rdp_deploy_sensor_only")
            self.topic_names = [topic for topic, _msg_type in subs_name_type]
            self.buffer = ObservationBuffer(maxlen=int(resolved_cfg.runtime.obs_buffer_size))
            self.latency_monitor = LatencyMonitor()
            self.callback_errors: list[str] = []
            self.frame_count = 0
            self.started_at = time.time()
            self.image_resize_shape = tuple(resolved_cfg.data_processing.get("image_resize_shape", [320, 240]))
            self.marker_dimension = int(resolved_cfg.data_processing.get("marker_dimension", 2))

            self.subscribers = [
                Subscriber(self, msg_type, topic)
                for topic, msg_type in subs_name_type
            ]
            self.synchronizer = ApproximateTimeSynchronizer(
                self.subscribers,
                queue_size=int(resolved_cfg.runtime.sync_queue_size),
                slop=float(resolved_cfg.runtime.sync_slop),
                allow_headerless=False,
            )
            self.synchronizer.registerCallback(self._callback)

        def _callback(self, *msgs):
            try:
                topic_dict = {
                    self.topic_names[i]: msg
                    for i, msg in enumerate(msgs)
                }
                obs = convert_topic_dict_to_observation(
                    topic_dict,
                    image_resize_shape=self.image_resize_shape,
                    marker_dimension=self.marker_dimension,
                )
                self.buffer.push(obs)
                self.latency_monitor.add_observation(obs)
                self.frame_count += 1
            except Exception as exc:  # noqa: BLE001
                message = f"{type(exc).__name__}: {exc}"
                self.callback_errors.append(message)
                self.get_logger().error(message)

        def report(self) -> dict:
            return {
                "topics": self.topic_names,
                "buffer": self.buffer.stats(),
                "latency": self.latency_monitor.report(),
                "callback_error_count": len(self.callback_errors),
                "recent_callback_errors": self.callback_errors[-5:],
            }

    if not rclpy.ok():
        rclpy.init(args=None)
    return SensorOnlyRosNode()


class RunningCollector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.node = None
        self.executor = None
        self.thread = None

    def __enter__(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = create_sensor_node(self.cfg)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        return self.node

    def __exit__(self, exc_type, exc, tb):
        import rclpy

        if self.executor is not None:
            self.executor.shutdown()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def collect_snapshot(cfg, timeout_sec: float | None = None) -> tuple[dict | None, dict]:
    timeout = float(timeout_sec or cfg.runtime.snapshot_timeout_sec)
    with RunningCollector(cfg) as node:
        if not node.buffer.wait_for_item(timeout):
            return None, node.report()
        return node.buffer.last(), node.report()


def collect_stream(cfg, duration_sec: float) -> tuple[list[dict], dict]:
    with RunningCollector(cfg) as node:
        deadline = time.time() + float(duration_sec)
        while time.time() < deadline:
            time.sleep(0.1)
        return node.buffer.all(), node.report()
