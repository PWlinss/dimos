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

"""Emit one safe dashboard goal and count the resulting DimOS cmd_vel messages."""

from __future__ import annotations

import argparse
import time

import rclpy
import socketio
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CmdVelProbe(Node):
    def __init__(self) -> None:
        super().__init__("dimos_goal_cmd_vel_probe")
        self.frames = 0
        self.nonzero_frames = 0
        self.max_linear_x = 0.0
        self.max_angular_z = 0.0
        self.create_subscription(
            Twist,
            "/dimos/cmd_vel",
            self._on_cmd_vel,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_cmd_vel(self, message: Twist) -> None:
        self.frames += 1
        linear_x = abs(message.linear.x)
        angular_z = abs(message.angular.z)
        self.max_linear_x = max(self.max_linear_x, linear_x)
        self.max_angular_z = max(self.max_angular_z, angular_z)
        if linear_x > 1e-4 or angular_z > 1e-4:
            self.nonzero_frames += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    probe = CmdVelProbe()
    client = socketio.Client(reconnection=False)
    try:
        client.connect(
            "http://127.0.0.1:7779",
            transports=["polling"],
            wait=True,
            wait_timeout=10,
        )
        client.emit("click", [args.x, args.y])
        deadline = time.monotonic() + args.duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
    finally:
        if client.connected:
            client.disconnect()
        probe.destroy_node()
        rclpy.shutdown()

    print(
        "cmd_vel_frames={} nonzero_frames={} max_linear_x={:.3f} max_angular_z={:.3f}".format(
            probe.frames,
            probe.nonzero_frames,
            probe.max_linear_x,
            probe.max_angular_z,
        )
    )
    return 0 if probe.nonzero_frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
