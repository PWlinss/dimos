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

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState


class SportStateToUdp(Node):
    """Forward Go2 state to a local process without bridging DDS networks."""

    def __init__(self) -> None:
        super().__init__("sport_state_to_udp")
        self.declare_parameter("input_topic", "/sportmodestate")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39100)

        input_topic = self.get_parameter("input_topic").value
        self.udp_target = (
            self.get_parameter("udp_host").value,
            self.get_parameter("udp_port").value,
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscription = self.create_subscription(
            SportModeState, input_topic, self._on_state, 10
        )

    def _on_state(self, msg: SportModeState) -> None:
        payload = {
            "position": [float(value) for value in msg.position],
            "quaternion": [float(value) for value in msg.imu_state.quaternion],
        }
        self.socket.sendto(json.dumps(payload).encode(), self.udp_target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SportStateToUdp()
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
