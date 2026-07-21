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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class AmclPoseToPose(Node):
    """Adapt AMCL localization into DimOS's map-frame odometry contract."""

    def __init__(self) -> None:
        super().__init__("amcl_pose_to_pose")
        self.declare_parameter("input_topic", "/amcl_pose")
        self.declare_parameter("output_topic", "/dimos/odom")
        self.declare_parameter("frame_id", "map")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, input_topic, self._on_pose, 10
        )
        self.get_logger().info(
            f"AMCL pose adapter started: {input_topic} -> {output_topic}, frame_id={self.frame_id}"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if self.frame_id:
            pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.publisher.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclPoseToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
