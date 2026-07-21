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

from importlib import import_module
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _reliability_policy(value: str) -> ReliabilityPolicy:
    return (
        ReliabilityPolicy.BEST_EFFORT
        if value == "best_effort"
        else ReliabilityPolicy.RELIABLE
    )


def _durability_policy(value: str) -> DurabilityPolicy:
    return (
        DurabilityPolicy.TRANSIENT_LOCAL
        if value == "transient_local"
        else DurabilityPolicy.VOLATILE
    )


class TopicRelay(Node):
    def __init__(self) -> None:
        super().__init__("nx_topic_relay")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("msg_module", "sensor_msgs.msg")
        self.declare_parameter("msg_type", "PointCloud2")
        self.declare_parameter("max_rate_hz", 0.0)
        self.declare_parameter("input_reliability", "reliable")
        self.declare_parameter("output_reliability", "reliable")
        self.declare_parameter("input_durability", "volatile")
        self.declare_parameter("output_durability", "volatile")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        msg_module = self.get_parameter("msg_module").value
        msg_type = self.get_parameter("msg_type").value
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        input_reliability = self.get_parameter("input_reliability").value
        output_reliability = self.get_parameter("output_reliability").value
        input_durability = self.get_parameter("input_durability").value
        output_durability = self.get_parameter("output_durability").value
        self.min_publish_interval = 1.0 / self.max_rate_hz if self.max_rate_hz > 0.0 else 0.0
        self.last_publish_time = 0.0
        self.received_count = 0
        self.published_count = 0
        self.last_report_time = time.monotonic()

        message_cls = getattr(import_module(msg_module), msg_type)
        output_qos = QoSProfile(
            depth=1,
            reliability=_reliability_policy(output_reliability),
            durability=_durability_policy(output_durability),
        )
        input_qos = QoSProfile(
            depth=10,
            reliability=_reliability_policy(input_reliability),
            durability=_durability_policy(input_durability),
        )

        self.publisher = self.create_publisher(message_cls, output_topic, output_qos)
        self.subscription = self.create_subscription(
            message_cls, input_topic, self._relay, input_qos
        )
        self.get_logger().info(
            f"relay started: {input_topic} -> {output_topic} "
            f"type={msg_module}.{msg_type} max_rate_hz={self.max_rate_hz} "
            f"input_qos={input_reliability}/{input_durability} "
            f"output_qos={output_reliability}/{output_durability}"
        )

    def _relay(self, message) -> None:
        now = time.monotonic()
        self.received_count += 1
        if now - self.last_publish_time < self.min_publish_interval:
            self._report(now)
            return
        self.last_publish_time = now
        self.publisher.publish(message)
        self.published_count += 1
        self._report(now)

    def _report(self, now: float) -> None:
        if now - self.last_report_time < 5.0:
            return
        self.get_logger().info(
            f"relay stats: received={self.received_count} published={self.published_count}"
        )
        self.last_report_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
