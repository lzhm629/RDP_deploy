from __future__ import annotations

import time

import requests


class RobotStatePublisher:
    def __init__(
        self,
        robot_server_ip: str,
        robot_server_port: int,
        fps: float = 120,
        bimanual: bool = False,
        request_timeout_sec: float = 0.05,
    ):
        from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        class _Node(Node):
            pass

        self.node = _Node("rdp_deploy_robot_state_publisher")
        self.robot_server_ip = robot_server_ip
        self.robot_server_port = int(robot_server_port)
        self.request_timeout_sec = float(request_timeout_sec)
        self.bimanual = bool(bimanual)
        self.session = requests.Session()
        self.frame_count = 0
        self.prev_time = time.time()

        self.left_pose_pub = self.node.create_publisher(PoseStamped, "/left_tcp_pose", 10)
        self.left_gripper_pub = self.node.create_publisher(JointState, "/left_gripper_state", 10)
        self.left_vel_pub = self.node.create_publisher(TwistStamped, "/left_tcp_vel", 10)
        self.left_wrench_pub = self.node.create_publisher(WrenchStamped, "/left_tcp_wrench", 10)
        if self.bimanual:
            self.right_pose_pub = self.node.create_publisher(PoseStamped, "/right_tcp_pose", 10)
            self.right_gripper_pub = self.node.create_publisher(JointState, "/right_gripper_state", 10)
            self.right_vel_pub = self.node.create_publisher(TwistStamped, "/right_tcp_vel", 10)
            self.right_wrench_pub = self.node.create_publisher(WrenchStamped, "/right_tcp_wrench", 10)

        self.timer = self.node.create_timer(1.0 / float(fps), self._timer_callback)

    @property
    def base_url(self) -> str:
        return f"http://{self.robot_server_ip}:{self.robot_server_port}"

    def destroy_node(self):
        self.node.destroy_node()

    def _get_states(self) -> dict:
        response = self.session.get(
            f"{self.base_url}/get_current_robot_states",
            timeout=self.request_timeout_sec,
        )
        response.raise_for_status()
        return dict(response.json())

    def _publish_side(self, side: str, states: dict, stamp):
        from geometry_msgs.msg import Point, PoseStamped, TwistStamped, WrenchStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header

        prefix = "left" if side == "left" else "right"
        tcp = states[f"{prefix}RobotTCP"]
        tcp_vel = states[f"{prefix}RobotTCPVel"]
        wrench = states[f"{prefix}RobotTCPWrench"]
        gripper = states[f"{prefix}GripperState"]

        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=stamp, frame_id=f"{prefix}_tcp")
        pose_msg.pose.position = Point(x=float(tcp[0]), y=float(tcp[1]), z=float(tcp[2]))
        pose_msg.pose.orientation.w = float(tcp[3])
        pose_msg.pose.orientation.x = float(tcp[4])
        pose_msg.pose.orientation.y = float(tcp[5])
        pose_msg.pose.orientation.z = float(tcp[6])

        gripper_msg = JointState()
        gripper_msg.header = Header(stamp=stamp)
        gripper_msg.name = [f"{prefix}_gripper"]
        gripper_msg.position = [float(gripper[0])]
        gripper_msg.effort = [float(gripper[1])]

        vel_msg = TwistStamped()
        vel_msg.header = Header(stamp=stamp)
        vel_msg.twist.linear.x = float(tcp_vel[0])
        vel_msg.twist.linear.y = float(tcp_vel[1])
        vel_msg.twist.linear.z = float(tcp_vel[2])
        vel_msg.twist.angular.x = float(tcp_vel[3])
        vel_msg.twist.angular.y = float(tcp_vel[4])
        vel_msg.twist.angular.z = float(tcp_vel[5])

        wrench_msg = WrenchStamped()
        wrench_msg.header = Header(stamp=stamp)
        wrench_msg.wrench.force.x = float(wrench[0])
        wrench_msg.wrench.force.y = float(wrench[1])
        wrench_msg.wrench.force.z = float(wrench[2])
        wrench_msg.wrench.torque.x = float(wrench[3])
        wrench_msg.wrench.torque.y = float(wrench[4])
        wrench_msg.wrench.torque.z = float(wrench[5])

        getattr(self, f"{prefix}_pose_pub").publish(pose_msg)
        getattr(self, f"{prefix}_gripper_pub").publish(gripper_msg)
        getattr(self, f"{prefix}_vel_pub").publish(vel_msg)
        getattr(self, f"{prefix}_wrench_pub").publish(wrench_msg)

    def _timer_callback(self):
        try:
            states = self._get_states()
            stamp = self.node.get_clock().now().to_msg()
            self._publish_side("left", states, stamp)
            if self.bimanual:
                self._publish_side("right", states, stamp)
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"robot state publish failed: {type(exc).__name__}: {exc}")
            return

        self.frame_count += 1
        now = time.time()
        if now - self.prev_time >= 5.0:
            fps = self.frame_count / (now - self.prev_time)
            self.node.get_logger().info(f"robot state FPS: {fps:.1f}")
            self.frame_count = 0
            self.prev_time = now
