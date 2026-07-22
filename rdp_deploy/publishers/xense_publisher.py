from __future__ import annotations

import struct
import time

import cv2
import numpy as np


class XensePublisher:
    def __init__(
        self,
        sensor_name: str,
        serial_number: str,
        cam_id: int | None = None,
        config_path: str | None = None,
        use_gpu: bool = True,
        fps: int = 24,
        publish_rectify: bool = True,
        publish_marker2d: bool = True,
        publish_force_resultant: bool = True,
        publish_timestamp: bool = True,
        jpeg_quality: int = 95,
    ):
        from geometry_msgs.msg import WrenchStamped
        from rclpy.node import Node
        from sensor_msgs.msg import Image, PointCloud2
        from xensesdk import Sensor

        class _Node(Node):
            pass

        self.node = _Node(f"rdp_deploy_{sensor_name}_publisher")
        self.Sensor = Sensor
        self.sensor_name = str(sensor_name)
        self.fps = int(fps)
        self.jpeg_quality = int(jpeg_quality)
        self.publish_rectify = bool(publish_rectify)
        self.publish_marker2d = bool(publish_marker2d)
        self.publish_force_resultant = bool(publish_force_resultant)
        self.publish_timestamp = bool(publish_timestamp)
        self.initial_marker: np.ndarray | None = None
        self.frame_count = 0
        self.prev_time = time.time()

        kwargs = {"use_gpu": bool(use_gpu)}
        if config_path:
            kwargs["config_path"] = config_path
        if cam_id is not None:
            self.sensor = Sensor.create(serial_number, cam_id=int(cam_id), **kwargs)
        else:
            self.sensor = Sensor.create(serial_number, **kwargs)

        self.output_types = []
        if self.publish_rectify:
            self.output_types.append(Sensor.OutputType.Rectify)
        if self.publish_marker2d:
            self.output_types.append(Sensor.OutputType.Marker2D)
        if self.publish_force_resultant:
            self.output_types.append(Sensor.OutputType.ForceResultant)
        if self.publish_timestamp:
            self.output_types.append(Sensor.OutputType.TimeStamp)

        self.image_pub = self.node.create_publisher(Image, f"/{self.sensor_name}/color/image_raw", 10)
        self.marker_pub = self.node.create_publisher(
            PointCloud2,
            f"/{self.sensor_name}/marker_offset/information",
            10,
        )
        self.force_pub = self.node.create_publisher(WrenchStamped, f"/{self.sensor_name}/force_resultant", 10)
        self.timer = self.node.create_timer(1.0 / self.fps, self._timer_callback)
        self.node.get_logger().info(f"Xense {self.sensor_name} started")

    def destroy_node(self):
        try:
            self.sensor.release()
        finally:
            self.node.destroy_node()

    def _timer_callback(self):
        values = self.sensor.selectSensorInfo(*self.output_types)
        by_type = {
            output_type: values[i]
            for i, output_type in enumerate(self.output_types)
        }
        stamp = self.node.get_clock().now().to_msg()

        if self.publish_rectify and self.Sensor.OutputType.Rectify in by_type:
            self._publish_image(by_type[self.Sensor.OutputType.Rectify], stamp)
        if self.publish_marker2d and self.Sensor.OutputType.Marker2D in by_type:
            self._publish_marker(by_type[self.Sensor.OutputType.Marker2D], stamp)
        if self.publish_force_resultant and self.Sensor.OutputType.ForceResultant in by_type:
            self._publish_force(by_type[self.Sensor.OutputType.ForceResultant], stamp)

        self.frame_count += 1
        now = time.time()
        if now - self.prev_time >= 5.0:
            fps = self.frame_count / (now - self.prev_time)
            self.node.get_logger().info(f"{self.sensor_name} FPS: {fps:.1f}")
            self.frame_count = 0
            self.prev_time = now

    def _publish_image(self, image: np.ndarray, stamp):
        from sensor_msgs.msg import Image

        if image is None:
            return
        image = np.asarray(image)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.sensor_name}_color_frame"
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.step = int(msg.width) * 3
        msg.data = encoded.tobytes() if ok else image.tobytes()
        self.image_pub.publish(msg)

    def _publish_marker(self, marker2d: np.ndarray, stamp):
        from sensor_msgs.msg import PointCloud2, PointField

        if marker2d is None:
            return
        marker = np.asarray(marker2d, dtype=np.float32).reshape(-1, 2)
        if self.initial_marker is None or self.initial_marker.shape != marker.shape:
            self.initial_marker = marker.copy()
        offset = marker - self.initial_marker
        marker_info = np.hstack([self.initial_marker, offset]).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.sensor_name}_marker_offset"
        msg.height = 1
        msg.width = marker_info.shape[0]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.fields = [
            PointField(name="marker_location_x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="marker_location_y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="marker_offset_x", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="marker_offset_y", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = b"".join(struct.pack("ffff", *row) for row in marker_info)
        self.marker_pub.publish(msg)

    def _publish_force(self, force: np.ndarray, stamp):
        from geometry_msgs.msg import WrenchStamped

        arr = np.asarray(force, dtype=np.float32).reshape(-1)
        if arr.size < 6:
            return
        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.sensor_name}_force"
        msg.wrench.force.x = float(arr[0])
        msg.wrench.force.y = float(arr[1])
        msg.wrench.force.z = float(arr[2])
        msg.wrench.torque.x = float(arr[3])
        msg.wrench.torque.y = float(arr[4])
        msg.wrench.torque.z = float(arr[5])
        self.force_pub.publish(msg)
