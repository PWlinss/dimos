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

"""Count the real NX point-cloud and odometry streams visible on pusai."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=6.0)
    parser.add_argument("--mode", choices=("raw", "map"), default="raw")
    parser.add_argument("--require-image", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Node("dimos_pusai_nx_data_probe")
    counts = {"pointcloud": 0, "odom": 0, "map": 0, "image": 0}
    pointcloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    odom_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    if args.mode == "raw":
        node.create_subscription(
            PointCloud2,
            "/dimos/registered_scan",
            lambda _: counts.__setitem__("pointcloud", counts["pointcloud"] + 1),
            pointcloud_qos,
        )
    else:
        node.create_subscription(
            OccupancyGrid,
            "/dimos/map",
            lambda _: counts.__setitem__("map", counts["map"] + 1),
            map_qos,
        )
    node.create_subscription(
        PoseStamped,
        "/dimos/odom",
        lambda _: counts.__setitem__("odom", counts["odom"] + 1),
        odom_qos,
    )
    if args.require_image:
        node.create_subscription(
            Image,
            "/dimos/color_image",
            lambda _: counts.__setitem__("image", counts["image"] + 1),
            image_qos,
        )

    deadline = time.monotonic() + args.duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    image_summary = f" image_frames={counts['image']}" if args.require_image else ""
    if args.mode == "raw":
        print(
            f"pointcloud_frames={counts['pointcloud']} odom_frames={counts['odom']}"
            f"{image_summary}"
        )
        return (
            0
            if counts["pointcloud"]
            and counts["odom"]
            and (not args.require_image or counts["image"])
            else 1
        )

    print(f"map_frames={counts['map']} odom_frames={counts['odom']}{image_summary}")
    return (
        0
        if counts["map"] and counts["odom"] and (not args.require_image or counts["image"])
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
