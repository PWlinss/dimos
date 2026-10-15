#!/usr/bin/env python3
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

"""Go2 NX bridge blueprint for old-stack ROS topic integration."""

from __future__ import annotations

from collections import deque
import os
import time
from typing import Any

import numpy as np
from scipy import ndimage  # type: ignore[import-untyped]

from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import ROSTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import (
    GeneralOccupancyConfig,
    HeightCostConfig,
    OccupancyConfig,
    RaycastOccupancyConfig,
    SimpleOccupancyConfig,
)
from dimos.mapping.odom_pointcloud_transform import (
    OdomPointCloudTransformer,
    odom_pointcloud_transformer,
)
from dimos.mapping.voxels import VoxelGridMapper, voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.service.system_configurator import ClockSyncConfigurator
from dimos.visualization.rerun.bridge import _resolve_viewer_mode, rerun_bridge
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis


def _topic(name: str, default: str) -> str:
    return os.getenv(name, default)


_lidar_topic = _topic("DIMOS_NX_LIDAR_TOPIC", "/registered_scan")
_odom_topic = _topic("DIMOS_NX_ODOM_TOPIC", "/odom")
_world_frame_id = os.getenv("DIMOS_NX_WORLD_FRAME_ID", "world")
_color_image_topic = _topic("DIMOS_NX_COLOR_IMAGE_TOPIC", "/color_image")
_cmd_vel_topic = _topic("DIMOS_NX_CMD_VEL_TOPIC", "/cmd_vel")
_map_topic = _topic("DIMOS_NX_MAP_TOPIC", "/dimos/map")
_map_occupied_threshold = int(os.getenv("DIMOS_NX_MAP_OCCUPIED_THRESHOLD", "50"))
_map_min_occupied_component_cells = int(
    os.getenv("DIMOS_NX_MAP_MIN_OCCUPIED_COMPONENT_CELLS", "6")
)
_map_unknown_hole_max_cells = int(os.getenv("DIMOS_NX_MAP_UNKNOWN_HOLE_MAX_CELLS", "80"))
_map_free_closing_radius_cells = int(
    os.getenv("DIMOS_NX_MAP_FREE_CLOSING_RADIUS_CELLS", "1")
)
_map_navigation_inflation_radius = float(
    os.getenv("DIMOS_NX_MAP_NAVIGATION_INFLATION_RADIUS", "0.2")
)
_reachable_costmap_min_interval = float(
    os.getenv("DIMOS_NX_REACHABLE_COSTMAP_MIN_INTERVAL", "1.0")
)


def _float(name: str, default: float) -> float:
    return float(os.getenv(name, str(default)))


_lidar_translation = tuple(
    _float(name, default)
    for name, default in (
        ("DIMOS_NX_LIDAR_X", 0.0),
        ("DIMOS_NX_LIDAR_Y", 0.0),
        ("DIMOS_NX_LIDAR_Z", 0.0),
    )
)
_lidar_rotation = tuple(
    _float(name, default)
    for name, default in (
        ("DIMOS_NX_LIDAR_QX", 0.0),
        ("DIMOS_NX_LIDAR_QY", 0.0),
        ("DIMOS_NX_LIDAR_QZ", 0.0),
        ("DIMOS_NX_LIDAR_QW", 1.0),
    )
)
_rerun_map_voxel_size = _float("DIMOS_NX_RERUN_MAP_VOXEL_SIZE", 0.4)
_rerun_heavy_interval = _float("DIMOS_NX_RERUN_HEAVY_INTERVAL", 5.0)
_rerun_image_interval = _float("DIMOS_NX_RERUN_IMAGE_INTERVAL", 0.2)
_rerun_pointcloud_interval = _float(
    "DIMOS_NX_RERUN_POINTCLOUD_INTERVAL", _rerun_heavy_interval
)
_planner_linear_speed = _float("DIMOS_NX_PLANNER_LINEAR_SPEED", 0.4)
_planner_angular_speed = _float("DIMOS_NX_PLANNER_ANGULAR_SPEED", 0.55)
_costmap_algo = os.getenv("DIMOS_NX_COSTMAP_ALGO", "general")


def _int(name: str, default: int) -> int:
    return int(os.getenv(name, str(default)))


def _costmap_config(algo: str) -> OccupancyConfig:
    resolution = _float("DIMOS_NX_COSTMAP_RESOLUTION", 0.05)

    if algo == "height_cost":
        return HeightCostConfig(
            resolution=resolution,
            can_pass_under=_float("DIMOS_NX_COSTMAP_CAN_PASS_UNDER", 0.6),
            can_climb=_float("DIMOS_NX_COSTMAP_CAN_CLIMB", 0.15),
            ignore_noise=_float("DIMOS_NX_COSTMAP_IGNORE_NOISE", 0.05),
            smoothing=_float("DIMOS_NX_COSTMAP_SMOOTHING", 1.0),
        )

    if algo == "simple":
        return SimpleOccupancyConfig(
            resolution=resolution,
            min_height=_float("DIMOS_NX_COSTMAP_MIN_HEIGHT", 0.1),
            max_height=_float("DIMOS_NX_COSTMAP_MAX_HEIGHT", 2.0),
            closing_iterations=_int("DIMOS_NX_COSTMAP_CLOSING_ITERATIONS", 1),
            closing_connectivity=_int("DIMOS_NX_COSTMAP_CLOSING_CONNECTIVITY", 2),
            can_pass_under=_float("DIMOS_NX_COSTMAP_CAN_PASS_UNDER", 0.6),
            can_climb=_float("DIMOS_NX_COSTMAP_CAN_CLIMB", 0.15),
            ignore_noise=_float("DIMOS_NX_COSTMAP_IGNORE_NOISE", 0.05),
            smoothing=_float("DIMOS_NX_COSTMAP_SMOOTHING", 1.0),
        )

    if algo == "general":
        return GeneralOccupancyConfig(
            resolution=resolution,
            min_height=_float("DIMOS_NX_COSTMAP_MIN_HEIGHT", 0.1),
            max_height=_float("DIMOS_NX_COSTMAP_MAX_HEIGHT", 2.0),
            mark_free_radius=_float("DIMOS_NX_COSTMAP_MARK_FREE_RADIUS", 0.4),
        )

    if algo == "raycast":
        return RaycastOccupancyConfig(
            resolution=resolution,
            min_height=_float("DIMOS_NX_COSTMAP_MIN_HEIGHT", 0.1),
            max_height=_float("DIMOS_NX_COSTMAP_MAX_HEIGHT", 2.0),
            mark_free_radius=_float("DIMOS_NX_COSTMAP_MARK_FREE_RADIUS", 0.25),
            max_ray_length=_float("DIMOS_NX_COSTMAP_MAX_RAY_LENGTH", 8.0),
        )

    raise ValueError(f"Unsupported DIMOS_NX_COSTMAP_ALGO={algo!r}")


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    visual_grid = grid.voxel_downsample(_rerun_map_voxel_size)
    try:
        return visual_grid.to_rerun(voxel_size=_rerun_map_voxel_size, mode="boxes")
    except TypeError:
        return visual_grid.to_rerun(bottom_cutoff=0)


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _convert_reachable_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Greens",
        z_offset=0.035,
        opacity=0.55,
        background="#000000",
    )


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
        ),
        rr.Arrows3D(
            origins=[[0.0, 0.0, 0.0]],
            vectors=[[0.6, 0.0, 0.0]],
            colors=[(255, 80, 80)],
        ),
    ]


def _component_touches_border(component: np.ndarray) -> bool:
    return bool(
        component[0, :].any()
        or component[-1, :].any()
        or component[:, 0].any()
        or component[:, -1].any()
    )


def _remove_small_occupied_components(
    grid: np.ndarray,
    *,
    min_component_cells: int,
) -> np.ndarray:
    if min_component_cells <= 1:
        return grid

    cleaned = grid.copy()
    occupied_labels, count = ndimage.label(
        cleaned >= CostValues.OCCUPIED,
        structure=np.ones((3, 3), dtype=np.uint8),
    )
    for label in range(1, count + 1):
        component = occupied_labels == label
        if int(component.sum()) < min_component_cells:
            cleaned[component] = CostValues.FREE
    return cleaned


def _fill_small_enclosed_unknown_holes(
    grid: np.ndarray,
    *,
    max_hole_cells: int,
) -> np.ndarray:
    if max_hole_cells <= 0:
        return grid

    cleaned = grid.copy()
    unknown_labels, count = ndimage.label(
        cleaned == CostValues.UNKNOWN,
        structure=np.ones((3, 3), dtype=np.uint8),
    )
    for label in range(1, count + 1):
        component = unknown_labels == label
        if _component_touches_border(component):
            continue
        if int(component.sum()) <= max_hole_cells:
            cleaned[component] = CostValues.FREE
    return cleaned


def _close_free_space_cracks(
    grid: np.ndarray,
    *,
    radius_cells: int,
) -> np.ndarray:
    if radius_cells <= 0:
        return grid

    cleaned = grid.copy()
    free = cleaned == CostValues.FREE
    unknown_y, unknown_x = np.nonzero(cleaned == CostValues.UNKNOWN)

    for y, x in zip(unknown_y, unknown_x, strict=True):
        left = max(0, x - radius_cells)
        right = min(cleaned.shape[1], x + radius_cells + 1)
        top = max(0, y - radius_cells)
        bottom = min(cleaned.shape[0], y + radius_cells + 1)

        has_left_free = free[y, left:x].any()
        has_right_free = free[y, x + 1 : right].any()
        has_top_free = free[top:y, x].any()
        has_bottom_free = free[y + 1 : bottom, x].any()

        if (has_left_free and has_right_free) or (has_top_free and has_bottom_free):
            cleaned[y, x] = CostValues.FREE

    return cleaned


def reachable_costmap_from_pose(
    costmap: OccupancyGrid,
    robot_pose: PoseStamped,
) -> OccupancyGrid:
    """Return a debug map containing only free cells reachable from robot_pose."""

    reachable_grid = np.full(costmap.grid.shape, CostValues.UNKNOWN, dtype=np.int8)
    start = costmap.world_to_grid(robot_pose.position)
    start_x = int(start.x)
    start_y = int(start.y)

    if not (0 <= start_x < costmap.width and 0 <= start_y < costmap.height):
        return OccupancyGrid(
            grid=reachable_grid,
            resolution=costmap.resolution,
            origin=costmap.origin,
            frame_id=costmap.frame_id,
            ts=costmap.ts,
        )

    if costmap.grid[start_y, start_x] != CostValues.FREE:
        return OccupancyGrid(
            grid=reachable_grid,
            resolution=costmap.resolution,
            origin=costmap.origin,
            frame_id=costmap.frame_id,
            ts=costmap.ts,
        )

    queue: deque[tuple[int, int]] = deque([(start_x, start_y)])
    visited = np.zeros(costmap.grid.shape, dtype=bool)
    visited[start_y, start_x] = True

    directions = (
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    )

    while queue:
        x, y = queue.popleft()
        reachable_grid[y, x] = CostValues.FREE

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if not (0 <= nx < costmap.width and 0 <= ny < costmap.height):
                continue
            if visited[ny, nx] or costmap.grid[ny, nx] != CostValues.FREE:
                continue
            visited[ny, nx] = True
            queue.append((nx, ny))

    return OccupancyGrid(
        grid=reachable_grid,
        resolution=costmap.resolution,
        origin=costmap.origin,
        frame_id=costmap.frame_id,
        ts=costmap.ts,
    )


def map_to_navigation_costmap(
    raw_map: OccupancyGrid,
    *,
    occupied_threshold: int = _map_occupied_threshold,
    min_occupied_component_cells: int = _map_min_occupied_component_cells,
    unknown_hole_max_cells: int = _map_unknown_hole_max_cells,
    free_closing_radius_cells: int = _map_free_closing_radius_cells,
    inflation_radius: float = _map_navigation_inflation_radius,
) -> OccupancyGrid:
    """Convert a static ROS map into a planner-oriented navigation costmap."""

    grid = raw_map.grid.copy()
    unknown = grid == CostValues.UNKNOWN
    occupied = grid >= occupied_threshold

    normalized = np.full(grid.shape, CostValues.FREE, dtype=np.int8)
    normalized[unknown] = CostValues.UNKNOWN
    normalized[occupied] = CostValues.OCCUPIED

    cleaned = _remove_small_occupied_components(
        normalized,
        min_component_cells=min_occupied_component_cells,
    )
    cleaned = _fill_small_enclosed_unknown_holes(
        cleaned,
        max_hole_cells=unknown_hole_max_cells,
    )
    cleaned = _close_free_space_cracks(
        cleaned,
        radius_cells=free_closing_radius_cells,
    )
    costmap = OccupancyGrid(
        grid=cleaned,
        resolution=raw_map.resolution,
        origin=raw_map.origin,
        frame_id=raw_map.frame_id,
        ts=raw_map.ts,
    )

    if inflation_radius <= 0:
        return costmap
    return simple_inflate(costmap, inflation_radius)


class MapToNavigationCostmap(Module):
    """Clean a mature ROS occupancy map before publishing DimOS' costmap stream."""

    map: In[OccupancyGrid]
    odom: In[PoseStamped]
    global_costmap: Out[OccupancyGrid]
    reachable_costmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()
        self._latest_costmap: OccupancyGrid | None = None
        self._latest_odom: PoseStamped | None = None
        self._last_reachable_publish_time = 0.0
        self._disposables.add(
            self.map.subscribe(
                lambda map_msg: self._publish_navigation_costmaps(
                    map_to_navigation_costmap(map_msg)
                )
            )
        )
        self._disposables.add(self.odom.subscribe(self._on_odom))

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom
        if self._latest_costmap is not None:
            self._publish_reachable_costmap(self._latest_costmap, odom, force=False)

    def _publish_navigation_costmaps(self, costmap: OccupancyGrid) -> None:
        self._latest_costmap = costmap
        self.global_costmap.publish(costmap)
        if self._latest_odom is not None:
            self._publish_reachable_costmap(costmap, self._latest_odom, force=True)

    def _publish_reachable_costmap(
        self, costmap: OccupancyGrid, odom: PoseStamped, *, force: bool
    ) -> None:
        now = time.monotonic()
        if (
            not force
            and _reachable_costmap_min_interval > 0
            and now - self._last_reachable_publish_time
            < _reachable_costmap_min_interval
        ):
            return
        self._last_reachable_publish_time = now
        self.reachable_costmap.publish(reachable_costmap_from_pose(costmap, odom))


class OdomPoseRelay(Module):
    """Publish odometry on the robot pose stream used by Rerun visualization."""

    odom: In[PoseStamped]
    robot_pose: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.odom.subscribe(self.robot_pose.publish))


class CameraImageRelay(Module):
    """Relay a ROS camera image topic into DimOS' internal color_image stream."""

    ros_color_image: In[Image]
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.ros_color_image.subscribe(self.color_image.publish))


def _go2_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        ),
    )


_rerun_config: dict[str, Any] = {
    "blueprint": _go2_rerun_blueprint,
    "pubsubs": [LCM()],
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/global_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
        "world/reachable_costmap": _convert_reachable_costmap,
    },
    "static": {
        "world/robot/base_link/body": _static_robot_body,
    },
}


def make_unitree_go2_nx_bridge(world_frame_id: str):  # type: ignore[no-untyped-def]
    """Build the NX bridge with one explicit world-coordinate contract."""

    return (
        autoconnect(
            rerun_bridge(
                viewer_mode=_resolve_viewer_mode(),
                min_interval_sec=_rerun_heavy_interval,
                image_min_interval_sec=_rerun_image_interval,
                pointcloud_min_interval_sec=_rerun_pointcloud_interval,
                **_rerun_config,
            ),
            websocket_vis(),
            CameraImageRelay.blueprint(),
            odom_pointcloud_transformer(
                world_frame_id=world_frame_id,
                lidar_translation=_lidar_translation,
                lidar_rotation=_lidar_rotation,
            ),
            voxel_mapper(
                frame_id=world_frame_id,
                voxel_size=float(os.getenv("DIMOS_NX_VOXEL_SIZE", "0.1")),
                device=os.getenv("DIMOS_NX_VOXEL_DEVICE", "CPU:0"),
            ),
            cost_mapper(algo=_costmap_algo, config=_costmap_config(_costmap_algo)),
            replanning_a_star_planner(),
            wavefront_frontier_explorer(),
        )
        .remappings(
            [
                (OdomPointCloudTransformer, "lidar", "raw_lidar"),
                (OdomPointCloudTransformer, "raw_lidar", "raw_lidar_visual"),
                (OdomPointCloudTransformer, "world_lidar", "lidar"),
                (OdomPointCloudTransformer, "robot_pose", "robot/base_link"),
                (VoxelGridMapper, "lidar", "lidar"),
                (CameraImageRelay, "ros_color_image", "ros_color_image"),
            ]
        )
        .transports(
            {
                ("raw_lidar", PointCloud2): ROSTransport(
                    _lidar_topic, PointCloud2, queue_depth=5, reliability="best_effort"
                ),
                ("odom", PoseStamped): ROSTransport(_odom_topic, PoseStamped),
                ("ros_color_image", Image): ROSTransport(
                    _color_image_topic, Image, queue_depth=2, reliability="best_effort"
                ),
                ("cmd_vel", Twist): ROSTransport(_cmd_vel_topic, Twist),
            }
        )
        .global_config(
            n_workers=6,
            robot_model="unitree_go2",
            planner_robot_speed=_planner_linear_speed,
            planner_min_linear_velocity=_planner_linear_speed,
            planner_min_angular_velocity=_planner_angular_speed,
            planner_max_angular_velocity=_planner_angular_speed,
        )
        .configurators(ClockSyncConfigurator())
    )


# Keep a trailing Blueprint method so the static CLI registry discovers this variable.
unitree_go2_nx_bridge = make_unitree_go2_nx_bridge(_world_frame_id).configurators()


unitree_go2_nx_map_bridge = (
    autoconnect(
        rerun_bridge(
            viewer_mode=_resolve_viewer_mode(),
            min_interval_sec=_rerun_heavy_interval,
            image_min_interval_sec=_rerun_image_interval,
            pointcloud_min_interval_sec=_rerun_pointcloud_interval,
            **_rerun_config,
        ),
        websocket_vis(),
        CameraImageRelay.blueprint(),
        MapToNavigationCostmap.blueprint(),
        OdomPoseRelay.blueprint(),
        replanning_a_star_planner(),
        wavefront_frontier_explorer(),
    )
    .remappings(
        [
            (OdomPoseRelay, "robot_pose", "robot/base_link"),
            (CameraImageRelay, "ros_color_image", "ros_color_image"),
        ]
    )
    .transports(
        {
            ("map", OccupancyGrid): ROSTransport(
                _map_topic,
                OccupancyGrid,
                queue_depth=1,
                reliability="reliable",
                durability="transient_local",
            ),
            ("odom", PoseStamped): ROSTransport(_odom_topic, PoseStamped),
            ("ros_color_image", Image): ROSTransport(
                _color_image_topic, Image, queue_depth=2, reliability="best_effort"
            ),
            ("cmd_vel", Twist): ROSTransport(_cmd_vel_topic, Twist),
        }
    )
    .global_config(
        n_workers=5,
        robot_model="unitree_go2",
        planner_robot_speed=_planner_linear_speed,
        planner_min_linear_velocity=_planner_linear_speed,
        planner_min_angular_velocity=_planner_angular_speed,
        planner_max_angular_velocity=_planner_angular_speed,
    )
    .configurators(ClockSyncConfigurator())
)


__all__ = [
    "make_unitree_go2_nx_bridge",
    "unitree_go2_nx_bridge",
    "unitree_go2_nx_map_bridge",
]
