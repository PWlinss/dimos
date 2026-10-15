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

"""Publish timestamped odometry poses on the DimOS TF service."""

from dataclasses import dataclass

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs import PoseStamped, Transform


@dataclass
class OdomTFRelayConfig(ModuleConfig):
    """Frame names used by :class:`OdomTFRelay`."""

    world_frame_id: str = "odom"
    base_frame_id: str = "base_link"


class OdomTFRelay(Module):
    """Relay accepted odometry poses into the timestamped TF buffer."""

    default_config = OdomTFRelayConfig
    config: OdomTFRelayConfig

    odom: In[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.odom.subscribe(self._publish_tf))

    def _publish_tf(self, odom: PoseStamped) -> None:
        if odom.frame_id != self.config.world_frame_id:
            return
        self.tf.publish(Transform.from_pose(self.config.base_frame_id, odom))


odom_tf_relay = OdomTFRelay.blueprint


__all__ = ["OdomTFRelay", "OdomTFRelayConfig", "odom_tf_relay"]
