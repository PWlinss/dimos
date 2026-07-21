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

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any


def build_payload(x: float, y: float, yaw: float, frame_id: str) -> dict[str, Any]:
    return {
        "frame_id": frame_id,
        "x": x,
        "y": y,
        "yaw": yaw,
        "orientation": {
            "z": math.sin(yaw / 2.0),
            "w": math.cos(yaw / 2.0),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish one AMCL /initialpose in the robot-bus DDS domain."
    )
    parser.add_argument("--x", type=float, required=True, help="Initial map-frame x in meters.")
    parser.add_argument("--y", type=float, required=True, help="Initial map-frame y in meters.")
    parser.add_argument("--yaw", type=float, required=True, help="Initial yaw in radians.")
    parser.add_argument("--frame-id", default="map", help="Pose frame id. Defaults to map.")
    parser.add_argument("--count", type=int, default=1, help="Number of messages to publish.")
    parser.add_argument("--interval-sec", type=float, default=0.1, help="Delay between publishes.")
    parser.add_argument("--dry-run", action="store_true", help="Print payload and exit.")
    return parser.parse_args()


def publish_initialpose(args: argparse.Namespace) -> None:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init()
    node = rclpy.create_node("dimos_initialpose_once")
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = args.frame_id
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.0685

    try:
        for _ in range(args.count):
            msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=args.interval_sec)
            time.sleep(args.interval_sec)
        print(f"published /initialpose: x={args.x}, y={args.y}, yaw={args.yaw}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    args = parse_args()
    payload = build_payload(args.x, args.y, args.yaw, args.frame_id)
    if args.dry_run:
        print(json.dumps(payload, sort_keys=True))
        return
    publish_initialpose(args)


if __name__ == "__main__":
    main()
