from __future__ import annotations


def get_ros_topics(timeout_sec: float = 2.0) -> dict[str, list[str]]:
    import rclpy
    from rclpy.node import Node

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    node = Node("rdp_deploy_topic_checker")
    try:
        rclpy.spin_once(node, timeout_sec=timeout_sec)
        return {name: types for name, types in node.get_topic_names_and_types()}
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def compare_topics(available: dict[str, list[str]], expected: list[str]) -> dict:
    missing = [topic for topic in expected if topic not in available]
    present = [topic for topic in expected if topic in available]
    return {
        "present": present,
        "missing": missing,
        "available_count": len(available),
    }
