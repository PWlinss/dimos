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

import numpy as np

from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import RaycastOccupancyConfig
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import PointCloud2


def test_cost_mapper_publishes_a_costmap_for_each_global_map() -> None:
    mapper = CostMapper()
    try:
        published = []
        mapper.global_costmap.subscribe(published.append)
        cloud = PointCloud2.from_numpy(
            np.array(
                [
                    [0.0, 0.0, 0.0],
                    [0.1, 0.0, 0.0],
                    [0.0, 0.1, 0.0],
                    [0.1, 0.1, 0.05],
                ],
                dtype=np.float32,
            ),
            frame_id="world",
        )

        mapper._on_global_map(cloud)

        assert len(published) == 1
        assert published[0].frame_id == "world"
        assert published[0].grid.size > 0
    finally:
        mapper.stop()


def test_cost_mapper_marks_the_current_robot_footprint_as_free() -> None:
    mapper = CostMapper(robot_free_radius=0.2)
    try:
        mapper._on_odom(PoseStamped(position=(1.0, 2.0, 0.0), frame_id="world"))
        cloud = PointCloud2.from_numpy(
            np.array(
                [
                    [0.0, 0.0, 0.0],
                    [2.0, 0.0, 0.0],
                    [0.0, 2.0, 0.0],
                    [2.0, 2.0, 0.0],
                ],
                dtype=np.float32,
            ),
            frame_id="world",
        )

        costmap = mapper._calculate_costmap(cloud)

        robot_cell = costmap.world_to_grid((1.0, 2.0, 0.0))
        assert costmap.grid[int(robot_cell.y), int(robot_cell.x)] == 0
    finally:
        mapper.stop()


def test_cost_mapper_uses_latest_odom_as_raycast_origin() -> None:
    mapper = CostMapper(
        algo="raycast",
        config=RaycastOccupancyConfig(
            resolution=0.1,
            min_height=0.1,
            max_height=2.0,
            mark_free_radius=0.0,
            ray_origin_x=-10.0,
            ray_origin_y=-10.0,
        ),
    )
    try:
        mapper._on_odom(PoseStamped(position=(1.0, 0.0, 0.0), frame_id="world"))
        cloud = PointCloud2.from_numpy(
            np.array([[3.0, 0.0, 0.5]], dtype=np.float32),
            frame_id="world",
        )

        costmap = mapper._calculate_costmap(cloud)

        midpoint = costmap.world_to_grid((2.0, 0.0, 0.0))
        assert costmap.grid[int(midpoint.y), int(midpoint.x)] == 0
    finally:
        mapper.stop()


def test_cost_mapper_raycast_carves_current_lidar_not_accumulated_global_map() -> None:
    mapper = CostMapper(
        algo="raycast",
        config=RaycastOccupancyConfig(
            resolution=0.1,
            min_height=0.1,
            max_height=2.0,
            mark_free_radius=0.0,
            max_ray_length=8.0,
        ),
    )
    try:
        mapper._on_odom(PoseStamped(position=(0.0, 0.0, 0.0), frame_id="world"))
        current_scan = PointCloud2.from_numpy(
            np.array([[2.0, 0.0, 0.5]], dtype=np.float32),
            frame_id="world",
        )
        global_map = PointCloud2.from_numpy(
            np.array(
                [
                    [2.0, 0.0, 0.5],
                    [0.0, 4.0, 0.5],
                ],
                dtype=np.float32,
            ),
            frame_id="world",
        )
        mapper._on_lidar(current_scan)

        costmap = mapper._calculate_costmap(global_map)

        current_midpoint = costmap.world_to_grid((1.0, 0.0, 0.0))
        stale_midpoint = costmap.world_to_grid((0.0, 2.0, 0.0))
        stale_endpoint = costmap.world_to_grid((0.0, 4.0, 0.0))

        assert costmap.grid[int(current_midpoint.y), int(current_midpoint.x)] == 0
        assert costmap.grid[int(stale_midpoint.y), int(stale_midpoint.x)] == -1
        assert costmap.grid[int(stale_endpoint.y), int(stale_endpoint.x)] == 100
    finally:
        mapper.stop()
