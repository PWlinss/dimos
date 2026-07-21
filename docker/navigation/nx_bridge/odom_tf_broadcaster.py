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

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from typing import Optional


class OdomTfBroadcaster(Node):
    """Publish odom->base TF with a small future offset for AMCL lookups."""

    def __init__(self) -> None:
        super().__init__("dimos_odom_tf_broadcaster")
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("future_offset_sec", 0.1)
        self.declare_parameter("max_odom_age_sec", 1.0)

        input_topic = self.get_parameter("input_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.future_offset = Duration(
            seconds=float(self.get_parameter("future_offset_sec").value)
        )
        self.max_odom_age_sec = float(self.get_parameter("max_odom_age_sec").value)
        self.latest_odom: Optional[Odometry] = None
        self.latest_receive_time = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self._on_odom, 10
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_tf)
        self.get_logger().info(
            f"odom TF broadcaster started: {input_topic}, "
            f"{self.parent_frame}->{self.child_frame}, future_offset={self.future_offset.nanoseconds / 1e9:.3f}s"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_receive_time = self.get_clock().now()

    def _publish_tf(self) -> None:
        if self.latest_odom is None or self.latest_receive_time is None:
            return
        now = self.get_clock().now()
        age = (now - self.latest_receive_time).nanoseconds / 1e9
        if age > self.max_odom_age_sec:
            return

        stamp = now + self.future_offset
        transform = TransformStamped()
        transform.header.stamp = Time(
            sec=stamp.seconds_nanoseconds()[0],
            nanosec=stamp.seconds_nanoseconds()[1],
        )
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.latest_odom.pose.pose.position.x
        transform.transform.translation.y = self.latest_odom.pose.pose.position.y
        transform.transform.translation.z = self.latest_odom.pose.pose.position.z
        transform.transform.rotation = self.latest_odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
