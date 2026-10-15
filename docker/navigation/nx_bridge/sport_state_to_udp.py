#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import socket
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState

from robot_clock_sync import ClockCalibrator, stamp_seconds


class SportStateToUdp(Node):
    """Forward Go2 state to a local process without bridging DDS networks."""

    def __init__(self) -> None:
        super().__init__("sport_state_to_udp")
        self.declare_parameter("input_topic", "/sportmodestate")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39100)
        self.declare_parameter("clock_sync_file", "")

        input_topic = self.get_parameter("input_topic").value
        self.udp_target = (
            self.get_parameter("udp_host").value,
            self.get_parameter("udp_port").value,
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.clock_sync_file = str(self.get_parameter("clock_sync_file").value)
        self.calibrator = ClockCalibrator(self.clock_sync_file) if self.clock_sync_file else None
        self._last_drop_log = 0.0
        self.subscription = self.create_subscription(
            SportModeState, input_topic, self._on_state, 10
        )

    def _on_state(self, msg: SportModeState) -> None:
        wall_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        sec = int(msg.stamp.sec)
        nanosec = int(msg.stamp.nanosec)
        try:
            stamp_seconds(sec, nanosec)
            if self.calibrator is not None:
                self.calibrator.observe(sec, nanosec, wall_ns=wall_ns, mono_ns=mono_ns)
        except ValueError as error:
            now = time.monotonic()
            if now - self._last_drop_log >= 1.0:
                self._last_drop_log = now
                self.get_logger().warning(f"dropped invalid SportModeState stamp: {error}")
            return
        payload = {
            "position": [float(value) for value in msg.position],
            "quaternion": [float(value) for value in msg.imu_state.quaternion],
            "source_stamp": {"sec": sec, "nanosec": nanosec},
            "nx_wall_ns": wall_ns,
            "nx_monotonic_ns": mono_ns,
            "clock_session": self.calibrator.session if self.calibrator else "",
        }
        self.socket.sendto(json.dumps(payload).encode(), self.udp_target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SportStateToUdp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
