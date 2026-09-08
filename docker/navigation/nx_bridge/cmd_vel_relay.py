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

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__("nx_cmd_vel_relay")
        self.declare_parameter("input_topic", "/dimos/cmd_vel")
        self.declare_parameter("output_topic", "/dimos/cmd_vel_guarded")
        self.declare_parameter("enabled", False)
        self.declare_parameter("max_linear_x", 0.8)
        self.declare_parameter("max_linear_y", 0.5)
        self.declare_parameter("max_angular_z", 1.2)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("stats_interval_sec", 2.0)

        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        initial_enabled = self.get_parameter("enabled").value
        if not isinstance(initial_enabled, bool):
            raise TypeError("enabled must be a boolean")
        self._enabled = initial_enabled
        self._max_linear_x = self.get_parameter("max_linear_x").value
        self._max_linear_y = self.get_parameter("max_linear_y").value
        self._max_angular_z = self.get_parameter("max_angular_z").value
        self._command_timeout_sec = self.get_parameter("command_timeout_sec").value
        self._stats_interval_sec = self.get_parameter("stats_interval_sec").value
        self._last_command_time = None
        self._commands_in = 0
        self._commands_forwarded = 0
        self._commands_dropped_disabled = 0
        self._timeout_stops = 0

        self.publisher = self.create_publisher(Twist, self._output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self._input_topic, self._on_command, 10
        )
        self.timer = self.create_timer(0.05, self._check_timeout)
        self._stats_timer = self.create_timer(self._stats_interval_sec, self._log_stats)
        self.add_on_set_parameters_callback(self._on_parameters)

        self.get_logger().info(
            f"Guarded cmd_vel relay started: enabled={self._enabled} "
            f"input={self._input_topic} output={self._output_topic}"
        )

    def _on_parameters(self, parameters):
        protected = {"input_topic", "output_topic"}
        if any(parameter.name in protected for parameter in parameters):
            return SetParametersResult(
                successful=False,
                reason="input_topic and output_topic require a relay restart",
            )
        if any(
            parameter.name == "enabled" and not isinstance(parameter.value, bool)
            for parameter in parameters
        ):
            return SetParametersResult(
                successful=False,
                reason="enabled must be a boolean",
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
            self._publish_zero()

        return SetParametersResult(successful=True)

    def _on_command(self, command: Twist) -> None:
        self._commands_in += 1
        if not self._enabled:
            self._commands_dropped_disabled += 1
            return

        guarded = Twist()
        guarded.linear.x = self._clamp(command.linear.x, self._max_linear_x)
        guarded.linear.y = self._clamp(command.linear.y, self._max_linear_y)
        guarded.angular.z = self._clamp(command.angular.z, self._max_angular_z)
        self.publisher.publish(guarded)
        self._commands_forwarded += 1
        self._last_command_time = self.get_clock().now()

    def _check_timeout(self) -> None:
        if not self._enabled or self._last_command_time is None:
            return

        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds / 1e9
        if elapsed > self._command_timeout_sec:
            self._last_command_time = None
            self._timeout_stops += 1
            self._publish_zero()

    def _publish_zero(self) -> None:
        self.publisher.publish(Twist())

    def _log_stats(self) -> None:
        self.get_logger().info(
            "cmd_vel stats: "
            f"enabled={self._enabled} commands_in={self._commands_in} "
            f"forwarded={self._commands_forwarded} "
            f"dropped_disabled={self._commands_dropped_disabled} "
            f"timeout_stops={self._timeout_stops}"
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-abs(limit), min(float(value), abs(limit)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
