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

"""Transform lidar scans into the current odometry/world frame."""

from __future__ import annotations

from dataclasses import dataclass

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import PointCloud2


@dataclass
class Config(ModuleConfig):
    """Static lidar-to-base extrinsic and resulting world-frame label."""

    world_frame_id: str = "world"
    lidar_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lidar_rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


class OdomPointCloudTransformer(Module):
    """Apply the latest base pose and calibrated lidar extrinsic to each scan."""

    default_config = Config
    config: Config

    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    raw_lidar: Out[PointCloud2]
    world_lidar: Out[PointCloud2]
    robot_pose: Out[PoseStamped]

    def __init__(self, **kwargs: object) -> None:
        self._latest_odom: PoseStamped | None = None
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))
        self._disposables.add(Disposable(self.lidar.subscribe(self._on_lidar)))

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom
        self.robot_pose.publish(odom)

    def _on_lidar(self, scan: PointCloud2) -> None:
        self.raw_lidar.publish(scan)
        world_scan = self._transform(scan)
        if world_scan is not None:
            self.world_lidar.publish(world_scan)

    def _transform(self, scan: PointCloud2) -> PointCloud2 | None:
        """Return a scan in world coordinates, or wait until odometry is available."""
        if scan.frame_id == self.config.world_frame_id:
            return scan
        if self._latest_odom is None:
            return None

        base_in_world = Transform(
            translation=Vector3(self._latest_odom.position),
            rotation=Quaternion(self._latest_odom.orientation),
            frame_id=self.config.world_frame_id,
            child_frame_id="base_link",
            ts=self._latest_odom.ts,
        )
        lidar_in_base = Transform(
            translation=Vector3(self.config.lidar_translation),
            rotation=Quaternion(self.config.lidar_rotation),
            frame_id="base_link",
            child_frame_id=scan.frame_id,
            ts=scan.ts,
        )
        return scan.transform(base_in_world + lidar_in_base)


odom_pointcloud_transformer = OdomPointCloudTransformer.blueprint


__all__ = ["OdomPointCloudTransformer", "odom_pointcloud_transformer"]
