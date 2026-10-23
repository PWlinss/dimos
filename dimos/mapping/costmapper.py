# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import asdict, dataclass, field

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = field(default_factory=HeightCostConfig)
    robot_free_radius: float = 0.25


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[PointCloud2]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    global_costmap: Out[OccupancyGrid]

    def __init__(self, cfg: GlobalConfig = global_config, **kwargs: object) -> None:
        self._latest_odom: PoseStamped | None = None
        self._latest_lidar: PointCloud2 | None = None
        super().__init__(**kwargs)
        self._global_config = cfg

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.global_map.subscribe(self._on_global_map)))
        self._disposables.add(Disposable(self.lidar.subscribe(self._on_lidar)))
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        config = asdict(self.config.config)
        if self.config.algo == "raycast":
            costmap = self._calculate_raycast_costmap(msg, config)
        else:
            costmap = fn(msg, **config)
        self._clear_robot_footprint(costmap)
        return costmap

    def _calculate_raycast_costmap(
        self, global_map: PointCloud2, config: dict[str, object]
    ) -> OccupancyGrid:
        if self._latest_odom is not None:
            config["ray_origin_x"] = self._latest_odom.position.x
            config["ray_origin_y"] = self._latest_odom.position.y
            config["ray_origin_z"] = self._latest_odom.position.z

        current_lidar = self._latest_lidar
        if current_lidar is None:
            return OCCUPANCY_ALGOS["raycast"](global_map, **config)

        max_height = float(config.get("max_height", 2.0))
        raycast_cloud = self._with_bounds_points(
            carving_cloud=current_lidar,
            bounds_cloud=global_map,
            bounds_z=max_height + 1.0,
        )
        costmap = OCCUPANCY_ALGOS["raycast"](raycast_cloud, **config)
        self._overlay_global_obstacles(costmap, global_map, config)
        return costmap

    def _on_odom(self, msg: PoseStamped) -> None:
        self._latest_odom = msg

    def _on_lidar(self, msg: PointCloud2) -> None:
        self._latest_lidar = msg

    def _clear_robot_footprint(self, costmap: OccupancyGrid) -> None:
        """Mark the MID360 near-field blind spot below the current base as free."""
        if self._latest_odom is None or self.config.robot_free_radius <= 0:
            return

        center = costmap.world_to_grid(self._latest_odom.position)
        center_x, center_y = int(center.x), int(center.y)
        radius_cells = int(np.ceil(self.config.robot_free_radius / costmap.resolution))

        for grid_y in range(center_y - radius_cells, center_y + radius_cells + 1):
            for grid_x in range(center_x - radius_cells, center_x + radius_cells + 1):
                if not (0 <= grid_x < costmap.width and 0 <= grid_y < costmap.height):
                    continue
                if (grid_x - center_x) ** 2 + (grid_y - center_y) ** 2 > radius_cells**2:
                    continue
                if costmap.grid[grid_y, grid_x] < 100:
                    costmap.grid[grid_y, grid_x] = 0

    def _with_bounds_points(
        self,
        *,
        carving_cloud: PointCloud2,
        bounds_cloud: PointCloud2,
        bounds_z: float,
    ) -> PointCloud2:
        carving_points, _ = carving_cloud.as_numpy()
        bounds_points, _ = bounds_cloud.as_numpy()

        if len(bounds_points) == 0:
            return carving_cloud

        bounds_only = bounds_points.copy()
        bounds_only[:, 2] = bounds_z
        points = np.vstack([carving_points, bounds_only])
        return PointCloud2.from_numpy(
            points,
            frame_id=carving_cloud.frame_id or bounds_cloud.frame_id,
            timestamp=carving_cloud.ts,
        )

    def _overlay_global_obstacles(
        self,
        costmap: OccupancyGrid,
        global_map: PointCloud2,
        config: dict[str, object],
    ) -> None:
        points, _ = global_map.as_numpy()
        if len(points) == 0:
            return

        min_height = float(config.get("min_height", 0.1))
        max_height = float(config.get("max_height", 2.0))
        obstacle_mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
        obstacle_points = points[obstacle_mask]
        if len(obstacle_points) == 0:
            return

        origin_x = costmap.origin.position.x
        origin_y = costmap.origin.position.y
        grid_x = ((obstacle_points[:, 0] - origin_x) / costmap.resolution).astype(np.int32)
        grid_y = ((obstacle_points[:, 1] - origin_y) / costmap.resolution).astype(np.int32)
        in_bounds = (
            (grid_x >= 0)
            & (grid_x < costmap.width)
            & (grid_y >= 0)
            & (grid_y < costmap.height)
        )
        costmap.grid[grid_y[in_bounds], grid_x[in_bounds]] = CostValues.OCCUPIED

    def _on_global_map(self, msg: PointCloud2) -> None:
        try:
            grid = self._calculate_costmap(msg)
            self.global_costmap.publish(grid)
        except Exception:
            logger.exception("Failed to calculate global costmap", points=len(msg))


cost_mapper = CostMapper.blueprint
