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
from typing import Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # w, x, y, z
Transform = Tuple[Vector3, Quaternion]


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conjugate(q: Quaternion) -> Quaternion:
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_rotate(q: Quaternion, value: Vector3) -> Vector3:
    rotated = _quat_multiply(
        _quat_multiply(q, (0.0, value[0], value[1], value[2])),
        _quat_conjugate(q),
    )
    return (rotated[1], rotated[2], rotated[3])


def _compose(parent_child: Transform, child_leaf: Transform) -> Transform:
    parent_position, parent_orientation = parent_child
    child_position, child_orientation = child_leaf
    rotated_child = _quat_rotate(parent_orientation, child_position)
    position = (
        parent_position[0] + rotated_child[0],
        parent_position[1] + rotated_child[1],
        parent_position[2] + rotated_child[2],
    )
    orientation = _quat_multiply(parent_orientation, child_orientation)
    return position, orientation


class MapTfOdomToUdp(Node):
    """Compose AMCL map->odom TF with odom->base odometry and forward map pose."""

    def __init__(self) -> None:
        super().__init__("map_tf_odom_to_udp")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39101)
        self.declare_parameter("publish_rate", 30.0)

        odom_topic = self.get_parameter("odom_topic").value
        tf_topic = self.get_parameter("tf_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.udp_target = (
            self.get_parameter("udp_host").value,
            self.get_parameter("udp_port").value,
        )
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.map_to_odom: Optional[Transform] = None
        self.odom_to_base: Optional[Transform] = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tf_subscription = self.create_subscription(
            TFMessage, tf_topic, self._on_tf, 100
        )
        self.odom_subscription = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_pose)
        self.get_logger().info(
            f"map pose UDP bridge started: {self.map_frame}->{self.odom_frame} + "
            f"{self.odom_frame}->{self.base_frame} -> {self.udp_target}"
        )

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.map_frame
                and transform.child_frame_id == self.odom_frame
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.map_to_odom = (
                    (translation.x, translation.y, translation.z),
                    (rotation.w, rotation.x, rotation.y, rotation.z),
                )

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.odom_to_base = (
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ),
        )

    def _publish_pose(self) -> None:
        if self.map_to_odom is None or self.odom_to_base is None:
            return
        position, quaternion = _compose(self.map_to_odom, self.odom_to_base)
        payload = {
            "position": [float(value) for value in position],
            "quaternion": [float(value) for value in quaternion],
        }
        self.socket.sendto(json.dumps(payload).encode(), self.udp_target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapTfOdomToUdp()
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
