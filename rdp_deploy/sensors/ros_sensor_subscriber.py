from __future__ import annotations

import threading
import time
from typing import Any

from omegaconf import OmegaConf

from rdp_deploy.clients.device_mapping_client import device_mapping_client_from_config
from rdp_deploy.config import resolve_config_paths
from rdp_deploy.diagnostics.latency_monitor import LatencyMonitor
from rdp_deploy.sensors.observation_buffer import ObservationBuffer


def _topic_roles(subs_name_type: list[tuple[str, Any]]) -> dict[str, list[str | None]]:
    roles: dict[str, list[str | None]] = {
        "depth_camera_point_cloud": [None, None, None],
        "depth_camera_rgb": [None, None, None],
        "tactile_camera_rgb": [None, None, None, None],
        "tactile_camera_marker": [None, None, None, None],
    }

    for topic, _msg_type in subs_name_type:
        if "depth/points" in topic:
            if "external_camera" in topic:
                roles["depth_camera_point_cloud"][0] = topic
            elif "left_wrist_camera" in topic:
                roles["depth_camera_point_cloud"][1] = topic
            elif "right_wrist_camera" in topic:
                roles["depth_camera_point_cloud"][2] = topic
        elif "color/image_raw" in topic:
            if "gripper_camera" in topic:
                if "left_gripper_camera_1" in topic:
                    roles["tactile_camera_rgb"][0] = topic
                elif "left_gripper_camera_2" in topic:
                    roles["tactile_camera_rgb"][1] = topic
                elif "right_gripper_camera_1" in topic:
                    roles["tactile_camera_rgb"][2] = topic
                elif "right_gripper_camera_2" in topic:
                    roles["tactile_camera_rgb"][3] = topic
            elif "external_camera" in topic:
                roles["depth_camera_rgb"][0] = topic
            elif "left_wrist_camera" in topic:
                roles["depth_camera_rgb"][1] = topic
            elif "right_wrist_camera" in topic:
                roles["depth_camera_rgb"][2] = topic
        elif "marker_offset/information" in topic:
            if "left_gripper_camera_1" in topic:
                roles["tactile_camera_marker"][0] = topic
            elif "left_gripper_camera_2" in topic:
                roles["tactile_camera_marker"][1] = topic
            elif "right_gripper_camera_1" in topic:
                roles["tactile_camera_marker"][2] = topic
            elif "right_gripper_camera_2" in topic:
                roles["tactile_camera_marker"][3] = topic
    return roles


def create_sensor_node(cfg):
    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from rclpy.node import Node

    from reactive_diffusion_policy.real_world.device_mapping.device_mapping_utils import get_topic_and_type
    from reactive_diffusion_policy.real_world.post_process_utils import DataPostProcessingManager
    from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms
    from reactive_diffusion_policy.real_world.ros_data_converter import ROS2DataConverter

    resolved_cfg = resolve_config_paths(cfg)
    mapping = device_mapping_client_from_config(resolved_cfg).get_mapping_model()
    subs_name_type = get_topic_and_type(mapping)
    roles = _topic_roles(subs_name_type)

    class SensorOnlyRosNode(Node):
        def __init__(self):
            super().__init__("rdp_deploy_sensor_only")
            self.topic_names = [topic for topic, _msg_type in subs_name_type]
            self.buffer = ObservationBuffer(maxlen=int(resolved_cfg.runtime.obs_buffer_size))
            self.latency_monitor = LatencyMonitor()
            self.callback_errors: list[str] = []
            self.frame_count = 0
            self.started_at = time.time()

            if bool(resolved_cfg.transforms.get("enabled", False)):
                transform_option = OmegaConf.create({
                    "calibration_path": resolved_cfg.transforms.calibration_path
                })
            else:
                transform_option = None
            transforms = RealWorldTransforms(option=transform_option)

            processing_kwargs = OmegaConf.to_container(resolved_cfg.data_processing, resolve=True)
            processing_kwargs.pop("debug", None)
            processing_debug = bool(resolved_cfg.data_processing.get("debug", False))
            self.post_processor = DataPostProcessingManager(
                transforms=transforms,
                debug=processing_debug,
                **processing_kwargs,
            )
            self.converter = ROS2DataConverter(
                transforms=transforms,
                depth_camera_point_cloud_topic_names=roles["depth_camera_point_cloud"],
                depth_camera_rgb_topic_names=roles["depth_camera_rgb"],
                tactile_camera_rgb_topic_names=roles["tactile_camera_rgb"],
                tactile_camera_marker_topic_names=roles["tactile_camera_marker"],
                tactile_camera_marker_dimension=int(resolved_cfg.data_processing.get("marker_dimension", 2)),
                debug=processing_debug,
            )

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
                sensor_msg = self.converter.convert_all_data(topic_dict)
                obs = self.post_processor.convert_sensor_msg_to_obs_dict(sensor_msg)
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
