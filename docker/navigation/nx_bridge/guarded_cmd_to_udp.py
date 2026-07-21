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

"""Forward already-guarded DimOS velocity commands to a local UDP boundary."""

import json
import socket
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class GuardedCmdToUdp(Node):
    """Runs on the NX Wi-Fi DDS participant and never accesses the robot bus."""

    def __init__(self) -> None:
        super().__init__("guarded_cmd_to_udp")
        self.declare_parameter("input_topic", "/dimos/cmd_vel_guarded")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39101)
        self.declare_parameter("stats_interval_sec", 2.0)

        input_topic = self.get_parameter("input_topic").value
        self._udp_destination = (
            self.get_parameter("udp_host").value,
            self.get_parameter("udp_port").value,
        )
        self._stats_interval_sec = self.get_parameter("stats_interval_sec").value
        self._commands_in = 0
        self._udp_sent = 0
        self._udp_errors = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._subscription = self.create_subscription(Twist, input_topic, self._on_command, 10)
        self._stats_timer = self.create_timer(self._stats_interval_sec, self._log_stats)
        self.get_logger().info(
            f"Guarded cmd_vel UDP forwarder started: input={input_topic} "
            f"destination={self._udp_destination[0]}:{self._udp_destination[1]}"
        )

    def _on_command(self, command: Twist) -> None:
        self._commands_in += 1
        payload = {
            "x": float(command.linear.x),
            "y": float(command.linear.y),
            "yaw": float(command.angular.z),
            "timestamp_ns": time.monotonic_ns(),
        }
        try:
            self._socket.sendto(
                json.dumps(payload, separators=(",", ":")).encode(), self._udp_destination
            )
            self._udp_sent += 1
        except OSError as exc:
            self._udp_errors += 1
            self.get_logger().warning(f"Failed to forward guarded cmd_vel over UDP: {exc}")

    def _log_stats(self) -> None:
        self.get_logger().info(
            "guarded cmd_vel udp stats: "
            f"commands_in={self._commands_in} udp_sent={self._udp_sent} "
            f"udp_errors={self._udp_errors}"
        )

    def destroy_node(self) -> bool:
        self._socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuardedCmdToUdp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
