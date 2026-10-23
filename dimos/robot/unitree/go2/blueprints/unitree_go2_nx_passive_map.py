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

"""Passive Go2 NX mapping from time-synchronized, odom-frame ROS topics."""

import signal

from dimos.core.blueprints import autoconnect
from dimos.core.transport import ROSTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2

unitree_go2_nx_passive_map = (
    autoconnect(
        voxel_mapper(
            frame_id="odom",
            voxel_size=0.1,
            block_count=100_000,
            publish_interval=2.0,
            carve_columns=False,
            device="CPU:0",
        ),
        cost_mapper(algo="height_cost", config=HeightCostConfig()),
    )
    .transports(
        {
            ("lidar", PointCloud2): ROSTransport(
                "/dimos/registered_scan",
                PointCloud2,
                queue_depth=5,
                reliability="best_effort",
            ),
            ("odom", PoseStamped): ROSTransport("/dimos/odom", PoseStamped),
            ("global_map", PointCloud2): ROSTransport(
                "/dimos/nx_passive/global_map", PointCloud2, queue_depth=1
            ),
            ("global_costmap", OccupancyGrid): ROSTransport(
                "/dimos/nx_passive/costmap", OccupancyGrid, queue_depth=1
            ),
        }
    )
    .global_config(n_workers=2, robot_model="unitree_go2", viewer="none")
)

__all__ = ["unitree_go2_nx_passive_map"]


def main() -> None:
    """Run the mapping coordinator and stop its workers on SIGINT or SIGTERM."""

    def _stop_on_signal(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _stop_on_signal)
    signal.signal(signal.SIGTERM, _stop_on_signal)
    unitree_go2_nx_passive_map.build().loop()


if __name__ == "__main__":
    main()
