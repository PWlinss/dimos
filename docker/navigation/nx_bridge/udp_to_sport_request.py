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

"""Convert local guarded UDP commands into the two Go2 Sport API movement requests."""

import json
import socket
import time
from typing import Any

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from unitree_api.msg import Request


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003


class UdpToSportRequest(Node):
    """Runs on the NX robot-bus DDS participant with an explicit enable gate."""

    def __init__(self) -> None:
        super().__init__("udp_to_sport_request")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39101)
        self.declare_parameter("enabled", False)
        self.declare_parameter("max_linear_x", 0.8)
        self.declare_parameter("max_linear_y", 0.5)
        self.declare_parameter("max_angular_z", 1.2)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("stats_interval_sec", 2.0)

        self._enabled = self.get_parameter("enabled").value
        self._max_linear_x = self.get_parameter("max_linear_x").value
        self._max_linear_y = self.get_parameter("max_linear_y").value
        self._max_angular_z = self.get_parameter("max_angular_z").value
        self._command_timeout_sec = self.get_parameter("command_timeout_sec").value
        self._stats_interval_sec = self.get_parameter("stats_interval_sec").value
        self._last_command_time = None
        self._udp_packets = 0
        self._malformed_packets = 0
        self._dropped_disabled = 0
        self._move_requests = 0
        self._stop_requests = 0
        self._timeout_stops = 0

        udp_host = self.get_parameter("udp_host").value
        udp_port = self.get_parameter("udp_port").value
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((udp_host, udp_port))
        self._socket.setblocking(False)
        self._publisher = self.create_publisher(Request, "/api/sport/request", 10)
        self._poll_timer = self.create_timer(0.02, self._poll_udp)
        self._timeout_timer = self.create_timer(0.05, self._check_timeout)
        self._stats_timer = self.create_timer(self._stats_interval_sec, self._log_stats)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.get_logger().info(
            f"Go2 Sport gateway started: enabled={self._enabled} udp={udp_host}:{udp_port}"
        )

    def _on_parameters(self, parameters: Any) -> SetParametersResult:
        protected = {"udp_host", "udp_port"}
        if any(parameter.name in protected for parameter in parameters):
            return SetParametersResult(
                successful=False,
                reason="udp_host and udp_port require a gateway restart",
            )

        was_enabled = self._enabled
        for parameter in parameters:
            if parameter.name == "enabled":
                self._enabled = parameter.value
            elif parameter.name == "max_linear_x":
                self._max_linear_x = parameter.value
            elif parameter.name == "max_linear_y":
                self._max_linear_y = parameter.value
            elif parameter.name == "max_angular_z":
                self._max_angular_z = parameter.value
            elif parameter.name == "command_timeout_sec":
                self._command_timeout_sec = parameter.value
            elif parameter.name == "stats_interval_sec":
                self._stats_interval_sec = parameter.value

        if was_enabled and not self._enabled:
            self._last_command_time = None
            self._publish_stop()

        return SetParametersResult(successful=True)

    def _poll_udp(self) -> None:
        while True:
            try:
                payload, _ = self._socket.recvfrom(4096)
            except BlockingIOError:
                return
            self._handle_payload(payload)

    def _handle_payload(self, payload: bytes) -> None:
        self._udp_packets += 1
        try:
            command = json.loads(payload.decode())
            x = self._clamp(float(command["x"]), self._max_linear_x)
            y = self._clamp(float(command["y"]), self._max_linear_y)
            yaw = self._clamp(float(command["yaw"]), self._max_angular_z)
        except (KeyError, TypeError, UnicodeDecodeError, ValueError, json.JSONDecodeError):
            self._malformed_packets += 1
            self.get_logger().warning("Discarded malformed local velocity packet")
            return

        if not self._enabled:
            self._dropped_disabled += 1
            return

        self._last_command_time = self.get_clock().now()
        if x == 0.0 and y == 0.0 and yaw == 0.0:
            self._publish_stop()
        else:
            self._publish_move(x, y, yaw)

    def _check_timeout(self) -> None:
        if not self._enabled or self._last_command_time is None:
            return

        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds / 1e9
        if elapsed > self._command_timeout_sec:
            self._last_command_time = None
            self._timeout_stops += 1
            self._publish_stop()

    def _publish_move(self, x: float, y: float, yaw: float) -> None:
        request = Request()
        request.header.identity.id = int(time.monotonic_ns() & 0x7FFFFFFF)
        request.header.identity.api_id = MOVE_API_ID
        request.parameter = json.dumps({"x": x, "y": y, "z": yaw}, separators=(",", ":"))
        self._publisher.publish(request)
        self._move_requests += 1

    def _publish_stop(self) -> None:
        request = Request()
        request.header.identity.id = int(time.monotonic_ns() & 0x7FFFFFFF)
        request.header.identity.api_id = STOP_MOVE_API_ID
        self._publisher.publish(request)
        self._stop_requests += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            "sport gateway stats: "
            f"enabled={self._enabled} udp_packets={self._udp_packets} "
            f"malformed={self._malformed_packets} "
            f"dropped_disabled={self._dropped_disabled} "
            f"move_requests={self._move_requests} "
            f"stop_requests={self._stop_requests} "
            f"timeout_stops={self._timeout_stops}"
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-abs(limit), min(value, abs(limit)))

    def destroy_node(self) -> bool:
        if self._enabled:
            self._publish_stop()
        self._socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpToSportRequest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
