from __future__ import annotations

import signal
import threading
from dataclasses import dataclass

from rdp_deploy.config import resolve_config_paths


@dataclass
class PublisherStack:
    nodes: list
    executor: object
    thread: threading.Thread | None = None

    def start(self, background: bool = False):
        for item in self.nodes:
            self.executor.add_node(item.node)
        if background:
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
        else:
            self.executor.spin()

    def stop(self):
        self.executor.shutdown()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        for item in self.nodes:
            item.destroy_node()


def build_publishers(cfg) -> PublisherStack:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    from rdp_deploy.publishers.realsense_publisher import RealsenseCameraPublisher
    from rdp_deploy.publishers.robot_state_publisher import RobotStatePublisher
    from rdp_deploy.publishers.xense_publisher import XensePublisher

    if not rclpy.ok():
        rclpy.init(args=None)

    cfg = resolve_config_paths(cfg)
    nodes = []

    if bool(cfg.publishers.robot_state.get("enabled", False)):
        nodes.append(RobotStatePublisher(
            cfg=cfg,
            fps=float(cfg.publishers.robot_state.get("fps", cfg.robot.get("publish_fps", 120))),
            bimanual=bool(cfg.robot.get("bimanual", False)),
        ))

    if bool(cfg.publishers.realsense.get("enabled", False)):
        for camera_cfg in cfg.publishers.realsense.get("cameras", []):
            nodes.append(RealsenseCameraPublisher(**dict(camera_cfg)))

    if bool(cfg.publishers.xense.get("enabled", False)):
        for sensor_cfg in cfg.publishers.xense.get("sensors", []):
            nodes.append(XensePublisher(**dict(sensor_cfg)))

    return PublisherStack(nodes=nodes, executor=MultiThreadedExecutor())


def spin_publishers_until_interrupt(cfg) -> int:
    import rclpy

    stack = build_publishers(cfg)
    stop_requested = False

    def _handle_signal(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True
        stack.executor.shutdown()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        stack.start(background=False)
    finally:
        stack.stop()
        if rclpy.ok():
            rclpy.shutdown()
    return 130 if stop_requested else 0
