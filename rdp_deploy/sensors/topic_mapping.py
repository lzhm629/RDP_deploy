from __future__ import annotations

from typing import Any


def get_msg_type(type_name: str) -> Any:
    from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
    from sensor_msgs.msg import Image, JointState, PointCloud2

    msg_types = {
        "geometry_msgs/msg/PoseStamped": PoseStamped,
        "geometry_msgs/msg/TwistStamped": TwistStamped,
        "geometry_msgs/msg/WrenchStamped": WrenchStamped,
        "sensor_msgs/msg/Image": Image,
        "sensor_msgs/msg/JointState": JointState,
        "sensor_msgs/msg/PointCloud2": PointCloud2,
    }
    if type_name not in msg_types:
        raise ValueError(f"Unsupported ROS message type in config: {type_name}")
    return msg_types[type_name]


def get_topic_and_type_from_config(topic_cfgs: list[dict]) -> list[tuple[str, Any]]:
    return [
        (str(item["name"]), get_msg_type(str(item["type"])))
        for item in topic_cfgs
    ]


def get_topic_and_type(mapping: dict) -> list[tuple[str, Any]]:
    from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
    from sensor_msgs.msg import Image, JointState, PointCloud2

    topics: list[tuple[str, Any]] = []

    for camera_name in mapping.get("realsense", {}).keys():
        topics.append((f"/{camera_name}/color/image_raw", Image))

    for camera_name in mapping.get("usb", {}).keys():
        topics.append((f"/{camera_name}/color/image_raw", Image))
        topics.append((f"/{camera_name}/marker_offset/information", PointCloud2))

    topics.extend([
        ("/left_tcp_pose", PoseStamped),
        ("/left_gripper_state", JointState),
        ("/left_tcp_vel", TwistStamped),
        ("/left_tcp_wrench", WrenchStamped),
    ])

    if bool(mapping.get("bimanual_teleop", True)):
        topics.extend([
            ("/right_tcp_pose", PoseStamped),
            ("/right_gripper_state", JointState),
            ("/right_tcp_vel", TwistStamped),
            ("/right_tcp_wrench", WrenchStamped),
        ])

    return topics
