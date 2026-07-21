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

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class UdpToPose(Node):
    """Publish local Go2 state packets into the NX-to-pusai DDS domain."""

    def __init__(self) -> None:
        super().__init__("udp_to_pose")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39100)
        self.declare_parameter("output_topic", "/dimos/odom")
        self.declare_parameter("frame_id", "odom")

        udp_host = self.get_parameter("udp_host").value
        udp_port = self.get_parameter("udp_port").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

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
            payload = json.loads(packet)
            position = payload["position"]
            quaternion = payload["quaternion"]

            pose = PoseStamped()
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
