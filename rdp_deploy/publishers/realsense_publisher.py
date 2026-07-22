from __future__ import annotations

import time

import cv2
import numpy as np


class RealsenseCameraPublisher:
    def __init__(
        self,
        camera_serial_number: str,
        camera_name: str,
        camera_type: str = "D400",
        rgb_resolution: list[int] | tuple[int, int] = (640, 480),
        fps: int = 24,
        stream_fps: int = 30,
        exposure: int | None = 120,
        white_balance: int | None = 5900,
        jpeg_quality: int = 95,
    ):
        import pyrealsense2 as rs
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        class _Node(Node):
            pass

        self.node = _Node(f"rdp_deploy_{camera_name}_publisher")
        self.rs = rs
        self.camera_serial_number = str(camera_serial_number)
        self.camera_name = str(camera_name)
        self.camera_type = str(camera_type)
        self.rgb_resolution = tuple(int(x) for x in rgb_resolution)
        self.fps = int(fps)
        self.stream_fps = int(stream_fps)
        self.jpeg_quality = int(jpeg_quality)
        self.frame_count = 0
        self.prev_time = time.time()

        self.publisher = self.node.create_publisher(Image, f"/{self.camera_name}/color/image_raw", 10)
        self.pipeline = rs.pipeline()
        self._start_camera(exposure=exposure, white_balance=white_balance)
        self.timer = self.node.create_timer(1.0 / self.fps, self._timer_callback)

    def _start_camera(self, exposure: int | None, white_balance: int | None):
        rs = self.rs
        config = rs.config()
        context = rs.context()
        serials = [
            device.get_info(rs.camera_info.serial_number)
            for device in context.query_devices()
        ]
        if self.camera_serial_number not in serials:
            raise RuntimeError(
                f"RealSense {self.camera_name} serial {self.camera_serial_number} not found. "
                f"Connected serials: {serials}"
            )

        config.enable_device(self.camera_serial_number)
        config.enable_stream(
            rs.stream.color,
            self.rgb_resolution[0],
            self.rgb_resolution[1],
            rs.format.bgr8,
            self.stream_fps,
        )
        profile = self.pipeline.start(config)
        device = profile.get_device()
        product_line = str(device.get_info(rs.camera_info.product_line))
        if product_line != self.camera_type:
            self.node.get_logger().warn(
                f"{self.camera_name} product line is {product_line}, config expected {self.camera_type}"
            )

        color_sensor = device.first_color_sensor()
        try:
            color_sensor.set_option(rs.option.global_time_enabled, 1)
        except Exception:
            pass
        if exposure is None:
            color_sensor.set_option(rs.option.enable_auto_exposure, 1.0)
        else:
            color_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
            color_sensor.set_option(rs.option.exposure, float(exposure))
            color_sensor.set_option(rs.option.gain, 0.0)
        if white_balance is None:
            color_sensor.set_option(rs.option.enable_auto_white_balance, 1.0)
        else:
            color_sensor.set_option(rs.option.enable_auto_white_balance, 0.0)
            color_sensor.set_option(rs.option.white_balance, float(white_balance))

        for _ in range(max(1, self.stream_fps)):
            self.pipeline.wait_for_frames()
        self.node.get_logger().info(f"RealSense {self.camera_name} started")

    def destroy_node(self):
        try:
            self.pipeline.stop()
        finally:
            self.node.destroy_node()

    def _timer_callback(self):
        from sensor_msgs.msg import Image

        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        image = np.asanyarray(color_frame.get_data())
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.camera_name}_color_frame"
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.step = int(msg.width) * 3
        msg.data = encoded.tobytes() if ok else image.tobytes()
        self.publisher.publish(msg)

        self.frame_count += 1
        now = time.time()
        if now - self.prev_time >= 5.0:
            fps = self.frame_count / (now - self.prev_time)
            self.node.get_logger().info(f"{self.camera_name} FPS: {fps:.1f}")
            self.frame_count = 0
            self.prev_time = now
