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
