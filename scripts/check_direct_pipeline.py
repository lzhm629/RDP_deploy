#!/usr/bin/env python3
import sys
import time
import types

import cv2
import numpy as np

import _bootstrap  # noqa: F401

from rdp_deploy.grippers.xense_gripper import XenseGripper
from rdp_deploy.sensors.direct_sensor_collector import DeviceSample, merge_device_samples
from rdp_deploy.sensors.direct_sensor_collector import PollingReader
from rdp_deploy.sensors.observation_conversion import (
    realsense_to_observation,
    robot_states_to_observation,
    xense_to_observation,
)


def _assert_shape(observation: dict, key: str, shape: tuple[int, ...]) -> None:
    actual = np.asarray(observation[key]).shape
    if actual != shape:
        raise AssertionError(f"{key}: expected {shape}, got {actual}")


def main() -> int:
    class _FakeGripper:
        def __init__(self):
            self.closed = False

        def get_gripper_status(self):
            return {
                "position": 42.5,
                "velocity": 1.5,
                "force": 3.0,
                "temperature": 30.0,
            }

        def close(self):
            self.closed = True

    fake_device = _FakeGripper()

    class _FakeXenseGripperSDK:
        @classmethod
        def create(cls, mac_addr=None, **kwargs):
            if mac_addr != "fake-mac":
                raise AssertionError(f"Unexpected gripper MAC: {mac_addr}")
            return fake_device

    previous_module = sys.modules.get("xensegripper")
    sys.modules["xensegripper"] = types.SimpleNamespace(
        XenseGripper=_FakeXenseGripperSDK
    )
    try:
        gripper = XenseGripper("fake-mac")
        gripper_status = gripper.read_status()
        gripper.close()
    finally:
        if previous_module is None:
            sys.modules.pop("xensegripper", None)
        else:
            sys.modules["xensegripper"] = previous_module
    if gripper_status["position"] != 42.5 or gripper_status["force"] != 3.0:
        raise AssertionError(f"Unexpected gripper status: {gripper_status}")
    if not fake_device.closed:
        raise AssertionError("Xense gripper cleanup was not called")

    closed = []
    fake_reader = PollingReader(
        name="synthetic",
        fps=100,
        read_fn=lambda: {"synthetic_value": np.array([1.0], dtype=np.float32)},
        close_fn=lambda: closed.append(True),
    )
    fake_reader.start()
    deadline = time.monotonic() + 1.0
    while fake_reader.latest() is None and time.monotonic() < deadline:
        time.sleep(0.01)
    reader_sample = fake_reader.latest()
    reader_report = fake_reader.report()
    fake_reader.close()
    if reader_sample is None or reader_report["count"] < 1 or not closed:
        raise AssertionError("PollingReader did not produce and close a sample")

    robot_states = {
        "leftRobotTCP": [0.5, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0],
        "leftRobotTCPVel": [0.0] * 6,
        "leftRobotTCPWrench": [0.0] * 6,
        "leftGripperState": [0.04, 0.0],
    }
    marker_reference = np.zeros((26, 14, 2), dtype=np.float32)
    marker = marker_reference + 0.25
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (320, 240), 40, (0, 255, 0), -1)

    observations = {
        "robot": robot_states_to_observation(robot_states),
        "realsense_D405": realsense_to_observation("D405", image, (320, 240)),
        "xense_left_gripper_camera_1": xense_to_observation(
            sensor_name="left_gripper_camera_1",
            resize_shape=(320, 240),
            marker_dimension=2,
            image=image,
            marker=marker,
            marker_reference=marker_reference,
            force_resultant=np.arange(6, dtype=np.float32),
        ),
    }
    now = time.time()
    samples = {
        name: DeviceSample(
            timestamp=now + index * 0.001,
            monotonic_time=time.monotonic(),
            generation=1,
            observation=observation,
        )
        for index, (name, observation) in enumerate(observations.items())
    }
    merged, skew = merge_device_samples(samples, sync_slop_sec=0.01)
    if merged is None:
        raise AssertionError(f"Synthetic samples did not synchronize; skew={skew}")

    _assert_shape(merged, "timestamp", (1,))
    _assert_shape(merged, "left_robot_tcp_pose", (9,))
    _assert_shape(merged, "left_robot_tcp_wrench", (6,))
    _assert_shape(merged, "left_robot_gripper_width", (1,))
    _assert_shape(merged, "agentview_image", (240, 320, 3))
    _assert_shape(merged, "left_gripper1_img", (240, 320, 3))
    _assert_shape(merged, "left_gripper1_initial_marker", (364, 2))
    _assert_shape(merged, "left_gripper1_marker_offset", (364, 2))
    _assert_shape(merged, "left_gripper1_force_resultant", (6,))

    samples["robot"] = DeviceSample(
        timestamp=now - 1.0,
        monotonic_time=time.monotonic(),
        generation=2,
        observation=observations["robot"],
    )
    rejected, rejected_skew = merge_device_samples(samples, sync_slop_sec=0.01)
    if rejected is not None or rejected_skew is None:
        raise AssertionError("Out-of-sync samples were not rejected")

    print("Direct pipeline: OK")
    print(f"Observation keys: {sorted(merged)}")
    print(f"Synchronized skew: {skew:.6f} sec")
    print(f"Rejected skew: {rejected_skew:.6f} sec")
    print(f"Polling reader samples: {reader_report['count']}")
    print(f"Gripper position/force: {gripper_status['position']}/{gripper_status['force']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
