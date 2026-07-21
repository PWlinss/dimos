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

"""Decode low-rate JPEG CompressedImage frames into local raw ROS Image frames."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CompressedImageToImage(Node):
    def __init__(self) -> None:
        super().__init__("dimos_compressed_image_to_image")
        self.declare_parameter("input_topic", "/dimos/color_image/compressed")
        self.declare_parameter("output_topic", "/dimos/color_image")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.publisher = self.create_publisher(Image, output_topic, qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._decode_and_publish,
            qos,
        )
        self.frame_count = 0
        self.get_logger().info(f"compressed image decoder started: {input_topic} -> {output_topic}")

    def _decode_and_publish(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("failed to decode compressed camera frame")
            return

        msg = Image()
        msg.header = message.header
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(msg.width * 3)
        msg.data = image.tobytes()
        self.publisher.publish(msg)
        self.frame_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompressedImageToImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
