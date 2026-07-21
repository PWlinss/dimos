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

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class AmclPoseToUdp(Node):
    """Forward AMCL pose across local DDS domains without a ROS graph bridge."""

    def __init__(self) -> None:
        super().__init__("amcl_pose_to_udp")
        self.declare_parameter("input_topic", "/amcl_pose")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39101)

        input_topic = self.get_parameter("input_topic").value
        self.udp_target = (
            self.get_parameter("udp_host").value,
            self.get_parameter("udp_port").value,
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, input_topic, self._on_pose, 10
        )
        self.get_logger().info(
            f"AMCL pose UDP bridge started: {input_topic} -> {self.udp_target}"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        payload = {
            "position": [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ],
            "quaternion": [
                float(pose.orientation.w),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
            ],
        }
        self.socket.sendto(json.dumps(payload).encode(), self.udp_target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclPoseToUdp()
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
