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
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from robot_clock_sync import (
    SourceStampGuard,
    map_stamp,
    read_snapshot,
    validate_positive_finite,
)


class UdpToPose(Node):
    """Publish local Go2 state packets into the NX-to-pusai DDS domain."""

    def __init__(self) -> None:
        super().__init__("udp_to_pose")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39100)
        self.declare_parameter("output_topic", "/dimos/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("timestamp_mode", "legacy")
        self.declare_parameter("clock_sync_file", "")
        self.declare_parameter("max_source_age_sec", 0.5)

        udp_host = self.get_parameter("udp_host").value
        udp_port = self.get_parameter("udp_port").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.timestamp_mode = str(self.get_parameter("timestamp_mode").value)
        self.clock_sync_file = str(self.get_parameter("clock_sync_file").value)
        self.max_source_age_sec = validate_positive_finite(
            self.get_parameter("max_source_age_sec").value, "max_source_age_sec"
        )
        self.source_stamp_guard = SourceStampGuard()
        self._last_drop_log = 0.0
        if self.timestamp_mode not in ("legacy", "robot_clock"):
            raise ValueError("timestamp_mode must be 'legacy' or 'robot_clock'")
        if self.timestamp_mode == "robot_clock" and not self.clock_sync_file:
            raise ValueError("robot_clock timestamp_mode requires clock_sync_file")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((udp_host, udp_port))
        self.socket.setblocking(False)
        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.timer = self.create_timer(0.001, self._drain_packets)

    def _drain_packets(self) -> None:
        while True:
            try:
                packet, _ = self.socket.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                payload = json.loads(packet)
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                self._warn_drop(f"dropped invalid UDP pose packet: {error}")
                continue
            try:
                position = payload["position"]
                quaternion = payload["quaternion"]
                if len(position) < 3 or len(quaternion) < 4:
                    raise ValueError("position or quaternion has too few values")
            except (KeyError, TypeError, ValueError) as error:
                self._warn_drop(f"dropped incomplete UDP pose packet: {error}")
                continue

            pose = PoseStamped()
            if self.timestamp_mode == "robot_clock":
                try:
                    source_stamp = payload["source_stamp"]
                    snapshot = read_snapshot(self.clock_sync_file)
                    if payload.get("clock_session") != snapshot["session"]:
                        raise ValueError("packet belongs to a different clock session")
                    sec, nanosec = map_stamp(
                        source_stamp["sec"],
                        source_stamp["nanosec"],
                        snapshot,
                        max_age_sec=self.max_source_age_sec,
                    )
                    self.source_stamp_guard.accept(
                        source_stamp["sec"],
                        source_stamp["nanosec"],
                        snapshot["session"],
                        sec,
                        nanosec,
                    )
                except (KeyError, TypeError, ValueError, OSError) as error:
                    self._warn_drop(f"dropped unmappable UDP pose packet: {error}")
                    continue
                pose.header.stamp = Time(sec=sec, nanosec=nanosec)
            else:
                pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            pose.pose.orientation.w = quaternion[0]
            pose.pose.orientation.x = quaternion[1]
            pose.pose.orientation.y = quaternion[2]
            pose.pose.orientation.z = quaternion[3]
            self.publisher.publish(pose)

    def _warn_drop(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_drop_log >= 1.0:
            self._last_drop_log = now
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpToPose()
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
