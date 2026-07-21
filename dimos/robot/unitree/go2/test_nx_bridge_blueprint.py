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

from importlib import import_module, reload
import inspect
from pathlib import Path
import pickle

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.transport import ROSTransport
from dimos.mapping.odom_pointcloud_transform import OdomPointCloudTransformer
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import GeneralOccupancyConfig
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.pointclouds.occupancy import RaycastOccupancyConfig
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.robot.test_all_blueprints_generation import _scan_for_blueprints


def test_nx_bridge_blueprint_is_scannable_and_registered() -> None:
    all_blueprints, _ = _scan_for_blueprints(DIMOS_PROJECT_ROOT / "dimos")
    assert "unitree-go2-nx-bridge" in all_blueprints


def test_nx_map_bridge_blueprint_is_scannable_and_registered() -> None:
    all_blueprints, _ = _scan_for_blueprints(DIMOS_PROJECT_ROOT / "dimos")
    assert "unitree-go2-nx-map-bridge" in all_blueprints


def test_nx_bridge_old_stack_topic_contract() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    blueprint = module.unitree_go2_nx_bridge

    assert blueprint.global_config_overrides["robot_model"] == "unitree_go2"
    assert blueprint.global_config_overrides["n_workers"] == 6

    transport_map = blueprint.transport_map
    assert isinstance(transport_map[("raw_lidar", PointCloud2)], ROSTransport)
    assert isinstance(transport_map[("odom", PoseStamped)], ROSTransport)
    assert isinstance(transport_map[("ros_color_image", Image)], ROSTransport)
    assert isinstance(transport_map[("cmd_vel", Twist)], ROSTransport)

    assert transport_map[("raw_lidar", PointCloud2)].topic.topic == "/registered_scan"
    assert transport_map[("odom", PoseStamped)].topic.topic == "/odom"
    assert transport_map[("ros_color_image", Image)].topic.topic == "/color_image"
    assert transport_map[("cmd_vel", Twist)].topic.topic == "/cmd_vel"
    assert ("color_image", Image) not in transport_map
    assert ("global_map", PointCloud2) not in transport_map


def test_nx_map_bridge_consumes_static_map_as_global_costmap(monkeypatch) -> None:
    monkeypatch.setenv("DIMOS_NX_MAP_TOPIC", "/dimos/map")
    monkeypatch.setenv("DIMOS_NX_ODOM_TOPIC", "/dimos/odom")
    monkeypatch.setenv("DIMOS_NX_COLOR_IMAGE_TOPIC", "/dimos/color_image")
    monkeypatch.setenv("DIMOS_NX_CMD_VEL_TOPIC", "/dimos/cmd_vel")

    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    module = reload(module)
    blueprint = module.unitree_go2_nx_map_bridge

    try:
        transport_map = blueprint.transport_map
        assert isinstance(transport_map[("map", OccupancyGrid)], ROSTransport)
        assert isinstance(transport_map[("odom", PoseStamped)], ROSTransport)
        assert isinstance(transport_map[("ros_color_image", Image)], ROSTransport)
        assert isinstance(transport_map[("cmd_vel", Twist)], ROSTransport)

        assert transport_map[("map", OccupancyGrid)].topic.topic == "/dimos/map"
        assert transport_map[("map", OccupancyGrid)]._kwargs["queue_depth"] == 1
        assert transport_map[("map", OccupancyGrid)]._kwargs["reliability"] == "reliable"
        assert (
            transport_map[("map", OccupancyGrid)]._kwargs["durability"]
            == "transient_local"
        )
        assert transport_map[("odom", PoseStamped)].topic.topic == "/dimos/odom"
        assert transport_map[("ros_color_image", Image)].topic.topic == "/dimos/color_image"
        assert transport_map[("cmd_vel", Twist)].topic.topic == "/dimos/cmd_vel"
        assert ("color_image", Image) not in transport_map
        assert ("global_costmap", OccupancyGrid) not in transport_map

        modules = {atom.module.__name__ for atom in blueprint.blueprints}
        assert "MapToNavigationCostmap" in modules
        assert "OdomPoseRelay" in modules
        assert "CameraImageRelay" in modules
    finally:
        monkeypatch.delenv("DIMOS_NX_MAP_TOPIC", raising=False)
        monkeypatch.delenv("DIMOS_NX_ODOM_TOPIC", raising=False)
        monkeypatch.delenv("DIMOS_NX_COLOR_IMAGE_TOPIC", raising=False)
        monkeypatch.delenv("DIMOS_NX_CMD_VEL_TOPIC", raising=False)
        reload(module)


def test_nx_bridges_route_ros_camera_into_internal_color_image_stream() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")

    for blueprint in (module.unitree_go2_nx_bridge, module.unitree_go2_nx_map_bridge):
        camera_relay = next(
            atom for atom in blueprint.blueprints if atom.module is module.CameraImageRelay
        )

        assert any(
            stream.name == "ros_color_image" and stream.direction == "in"
            for stream in camera_relay.streams
        )
        assert any(
            stream.name == "color_image" and stream.direction == "out"
            for stream in camera_relay.streams
        )
        assert (
            blueprint.remapping_map[(module.CameraImageRelay, "ros_color_image")]
            == "ros_color_image"
        )
        assert (module.CameraImageRelay, "color_image") not in blueprint.remapping_map


def test_nx_map_bridge_bypasses_raw_pointcloud_mapping_modules() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    blueprint = module.unitree_go2_nx_map_bridge

    modules = {atom.module for atom in blueprint.blueprints}
    assert VoxelGridMapper not in modules
    assert CostMapper not in modules
    assert OdomPointCloudTransformer not in modules


def _make_map_grid(grid: list[list[int]]) -> OccupancyGrid:
    import numpy as np

    return OccupancyGrid(
        grid=np.array(grid, dtype=np.int8),
        resolution=0.05,
        frame_id="map",
        ts=123.0,
    )


def test_map_to_navigation_costmap_removes_isolated_occupied_speckles() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    raw_map = _make_map_grid(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    costmap = module.map_to_navigation_costmap(
        raw_map,
        min_occupied_component_cells=3,
        unknown_hole_max_cells=0,
        inflation_radius=0.0,
    )

    assert costmap.grid[2, 2] == CostValues.FREE
    assert costmap.frame_id == "map"
    assert costmap.resolution == raw_map.resolution
    assert costmap.origin == raw_map.origin
    assert costmap.ts == raw_map.ts


def test_map_to_navigation_costmap_preserves_large_obstacles() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    raw_map = _make_map_grid(
        [
            [0, 0, 0, 0, 0],
            [0, 100, 100, 0, 0],
            [0, 100, 100, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    costmap = module.map_to_navigation_costmap(
        raw_map,
        min_occupied_component_cells=3,
        unknown_hole_max_cells=0,
        inflation_radius=0.0,
    )

    assert (costmap.grid[1:3, 1:3] == CostValues.OCCUPIED).all()


def test_map_to_navigation_costmap_fills_small_enclosed_unknown_holes() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    raw_map = _make_map_grid(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, -1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    costmap = module.map_to_navigation_costmap(
        raw_map,
        min_occupied_component_cells=3,
        unknown_hole_max_cells=4,
        inflation_radius=0.0,
    )

    assert costmap.grid[2, 2] == CostValues.FREE


def test_map_to_navigation_costmap_keeps_border_unknown_unknown() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    raw_map = _make_map_grid(
        [
            [-1, -1, 0, 0, 0],
            [-1, -1, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    costmap = module.map_to_navigation_costmap(
        raw_map,
        min_occupied_component_cells=3,
        unknown_hole_max_cells=10,
        inflation_radius=0.0,
    )

    assert (costmap.grid[:2, :2] == CostValues.UNKNOWN).all()


def test_map_to_navigation_costmap_closes_small_unknown_cracks_without_erasing_obstacles() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    raw_map = _make_map_grid(
        [
            [0, 0, -1, 0, 0],
            [0, 0, -1, 0, 0],
            [100, 100, 100, 100, 100],
            [0, 0, -1, 0, 0],
            [0, 0, -1, 0, 0],
        ]
    )

    costmap = module.map_to_navigation_costmap(
        raw_map,
        min_occupied_component_cells=1,
        unknown_hole_max_cells=0,
        free_closing_radius_cells=1,
        inflation_radius=0.0,
    )

    assert costmap.grid[0, 2] == CostValues.FREE
    assert costmap.grid[1, 2] == CostValues.FREE
    assert (costmap.grid[2, :] == CostValues.OCCUPIED).all()
    assert costmap.grid[3, 2] == CostValues.FREE
    assert costmap.grid[4, 2] == CostValues.FREE


def test_reachable_costmap_marks_only_free_cells_connected_to_robot() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    costmap = _make_map_grid(
        [
            [0, 0, 100, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 0, 100, 0, 0],
        ]
    )
    robot_pose = PoseStamped(position=[0.05, 0.05, 0.0])

    reachable = module.reachable_costmap_from_pose(costmap, robot_pose)

    assert (reachable.grid[:, :2] == CostValues.FREE).all()
    assert (reachable.grid[:, 2:] == CostValues.UNKNOWN).all()


def test_nx_map_bridge_publishes_reachable_costmap_for_debugging() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    blueprint = module.unitree_go2_nx_map_bridge
    map_adapter = next(
        atom for atom in blueprint.blueprints if atom.module is module.MapToNavigationCostmap
    )

    assert any(stream.name == "odom" and stream.direction == "in" for stream in map_adapter.streams)
    assert any(
        stream.name == "reachable_costmap" and stream.direction == "out"
        for stream in map_adapter.streams
    )
    assert "world/reachable_costmap" in module._rerun_config["visual_override"]


def test_nx_map_bridge_throttles_reachable_costmap_debug_stream() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    source = inspect.getsource(module.MapToNavigationCostmap)

    assert "DIMOS_NX_REACHABLE_COSTMAP_MIN_INTERVAL" in inspect.getsource(module)
    assert "time.monotonic()" in source
    assert "_last_reachable_publish_time" in source


def test_nx_bridge_lidar_transport_bounds_the_best_effort_queue() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    transport = module.unitree_go2_nx_bridge.transport_map[("raw_lidar", PointCloud2)]

    assert transport._kwargs["queue_depth"] == 5
    assert transport._kwargs["reliability"] == "best_effort"


def test_nx_bridge_lidar_queue_depth_survives_worker_serialization() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    transport = module.unitree_go2_nx_bridge.transport_map[("raw_lidar", PointCloud2)]

    restored = pickle.loads(pickle.dumps(transport))

    assert restored._kwargs["queue_depth"] == 5
    assert restored._kwargs["reliability"] == "best_effort"


def test_nx_bridge_defaults_to_cpu_voxel_mapping() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    voxel_blueprint = next(
        blueprint
        for blueprint in module.unitree_go2_nx_bridge.blueprints
        if blueprint.module.__name__ == "VoxelGridMapper"
    )

    assert voxel_blueprint.kwargs["device"] == "CPU:0"


def test_nx_bridge_bounds_rerun_global_map_history() -> None:
    source = (
        DIMOS_PROJECT_ROOT
        / "dimos/robot/unitree/go2/blueprints/navigation/unitree_go2_nx_bridge.py"
    ).read_text()

    assert "DIMOS_NX_RERUN_MAP_VOXEL_SIZE" in source
    assert "grid.voxel_downsample" in source
    assert "DIMOS_NX_RERUN_HEAVY_INTERVAL" in source


def test_nx_bridge_transforms_lidar_into_world_before_voxel_mapping() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    blueprint = module.unitree_go2_nx_bridge

    assert any(atom.module is OdomPointCloudTransformer for atom in blueprint.blueprints)
    assert blueprint.remapping_map[(OdomPointCloudTransformer, "lidar")] == "raw_lidar"
    assert blueprint.remapping_map[(OdomPointCloudTransformer, "world_lidar")] == "lidar"
    assert blueprint.remapping_map[(VoxelGridMapper, "lidar")] == "lidar"


def test_nx_bridge_routes_odom_to_costmapper_for_robot_footprint_clearing() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")

    costmapper = next(atom for atom in module.unitree_go2_nx_bridge.blueprints if atom.module is CostMapper)
    assert any(stream.name == "odom" and stream.direction == "in" for stream in costmapper.streams)
    assert costmapper.kwargs["algo"] == "general"
    assert isinstance(costmapper.kwargs["config"], GeneralOccupancyConfig)


def test_nx_bridge_costmap_algo_can_match_official_height_cost(monkeypatch) -> None:
    monkeypatch.setenv("DIMOS_NX_COSTMAP_ALGO", "height_cost")
    monkeypatch.setenv("DIMOS_NX_COSTMAP_CAN_CLIMB", "0.2")

    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    module = reload(module)

    try:
        costmapper = next(
            atom for atom in module.unitree_go2_nx_bridge.blueprints if atom.module is CostMapper
        )

        assert costmapper.kwargs["algo"] == "height_cost"
        assert isinstance(costmapper.kwargs["config"], HeightCostConfig)
        assert costmapper.kwargs["config"].can_climb == 0.2
    finally:
        monkeypatch.delenv("DIMOS_NX_COSTMAP_ALGO", raising=False)
        monkeypatch.delenv("DIMOS_NX_COSTMAP_CAN_CLIMB", raising=False)
        reload(module)


def test_nx_bridge_general_costmap_free_space_radius_is_configurable(monkeypatch) -> None:
    monkeypatch.setenv("DIMOS_NX_COSTMAP_ALGO", "general")
    monkeypatch.setenv("DIMOS_NX_COSTMAP_MARK_FREE_RADIUS", "0.8")

    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    module = reload(module)

    try:
        costmapper = next(
            atom for atom in module.unitree_go2_nx_bridge.blueprints if atom.module is CostMapper
        )

        assert costmapper.kwargs["algo"] == "general"
        assert isinstance(costmapper.kwargs["config"], GeneralOccupancyConfig)
        assert costmapper.kwargs["config"].mark_free_radius == 0.8
    finally:
        monkeypatch.delenv("DIMOS_NX_COSTMAP_ALGO", raising=False)
        monkeypatch.delenv("DIMOS_NX_COSTMAP_MARK_FREE_RADIUS", raising=False)
        reload(module)


def test_nx_bridge_costmap_algo_can_use_raycast_free_space(monkeypatch) -> None:
    monkeypatch.setenv("DIMOS_NX_COSTMAP_ALGO", "raycast")
    monkeypatch.setenv("DIMOS_NX_COSTMAP_MARK_FREE_RADIUS", "0.3")
    monkeypatch.setenv("DIMOS_NX_COSTMAP_MAX_RAY_LENGTH", "6.5")

    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    module = reload(module)

    try:
        costmapper = next(
            atom for atom in module.unitree_go2_nx_bridge.blueprints if atom.module is CostMapper
        )

        assert costmapper.kwargs["algo"] == "raycast"
        assert isinstance(costmapper.kwargs["config"], RaycastOccupancyConfig)
        assert costmapper.kwargs["config"].mark_free_radius == 0.3
        assert costmapper.kwargs["config"].max_ray_length == 6.5
    finally:
        monkeypatch.delenv("DIMOS_NX_COSTMAP_ALGO", raising=False)
        monkeypatch.delenv("DIMOS_NX_COSTMAP_MARK_FREE_RADIUS", raising=False)
        monkeypatch.delenv("DIMOS_NX_COSTMAP_MAX_RAY_LENGTH", raising=False)
        reload(module)


def test_nx_bridge_publishes_live_odom_pose_to_the_rerun_robot_entity() -> None:
    module = import_module("dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nx_bridge")
    blueprint = module.unitree_go2_nx_bridge

    transformer = next(
        atom for atom in blueprint.blueprints if atom.module is OdomPointCloudTransformer
    )
    assert any(stream.name == "robot_pose" and stream.direction == "out" for stream in transformer.streams)
    assert blueprint.remapping_map[(OdomPointCloudTransformer, "robot_pose")] == "robot/base_link"
    assert "world/robot/base_link/body" in module._rerun_config["static"]


def test_nx_adapter_assets_exist() -> None:
    root = DIMOS_PROJECT_ROOT
    expected_paths = [
        root / "docker/navigation/nx_bridge/odom_to_pose.py",
        root / "docker/navigation/nx_bridge/amcl_pose_to_pose.py",
        root / "docker/navigation/nx_bridge/sport_state_to_pose.py",
        root / "docker/navigation/nx_bridge/cyclonedds_dual_network.xml",
        root / "docker/navigation/nx_bridge/cyclonedds_robot_bus.xml",
        root / "docker/navigation/nx_bridge/sport_state_to_udp.py",
        root / "docker/navigation/nx_bridge/udp_to_pose.py",
        root / "docker/navigation/nx_bridge/topic_relay.py",
        root / "docker/navigation/nx_bridge/cmd_vel_relay.py",
        root / "docker/navigation/nx_bridge/guarded_cmd_to_udp.py",
        root / "docker/navigation/nx_bridge/udp_to_sport_request.py",
        root / "docker/navigation/nx_bridge/start_nx_bridge.sh",
        root / "docker/navigation/nx_bridge/restore_livox_callbacks.patch",
        root / "scripts/start_pusai_nx_bridge.sh",
        root / "scripts/cyclonedds_pusai_nx.xml",
        root / "scripts/verify_pusai_nx_data.py",
        root / "docker/navigation/nx_bridge/README.md",
    ]

    for path in expected_paths:
        assert path.exists(), f"missing NX bridge asset: {path}"


def test_pusai_start_script_prefers_the_bridge_worktree_for_workers() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()
    dds_config = (DIMOS_PROJECT_ROOT / "scripts/cyclonedds_pusai_nx.xml").read_text()

    assert 'export PYTHONPATH="${DIMOS_WORKTREE}${PYTHONPATH:+:${PYTHONPATH}}"' in script
    assert 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' in script
    assert 'export CYCLONEDDS_URI="${PUSAI_DDS_URI}"' in script
    assert 'verify_pusai_nx_data.py' in script
    assert 'NetworkInterface name="wlx702ad7b0a20b"' in dds_config
    assert '<Peer address="192.168.1.105" />' in dds_config


def test_pusai_start_script_uses_a_longer_pointcloud_probe_window() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()

    assert 'NX_DATA_VERIFY_DURATION_SEC="${DIMOS_NX_DATA_VERIFY_DURATION_SEC:-20}"' in script
    assert 'verify_args=(--duration-sec "${NX_DATA_VERIFY_DURATION_SEC}" --mode "${data_mode}")' in script
    assert '"${SCRIPT_DIR}/verify_pusai_nx_data.py" "${verify_args[@]}"' in script


def test_pusai_start_script_supports_map_bridge_mode() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()
    verifier = (DIMOS_PROJECT_ROOT / "scripts/verify_pusai_nx_data.py").read_text()

    assert "DIMOS_NX_BRIDGE_MODE" in script
    assert "unitree-go2-nx-map-bridge" in script
    assert "--map" in script
    assert 'export DIMOS_NX_MAP_TOPIC="${DIMOS_NX_MAP_TOPIC:-/dimos/map}"' in script
    assert '"${BRIDGE_BLUEPRINT}" --daemon' in script
    assert '--mode "${data_mode}"' in script
    assert 'parser.add_argument("--mode", choices=("raw", "map"), default="raw")' in verifier
    assert '"/dimos/map"' in verifier
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in verifier


def test_nx_topic_relay_supports_rate_limited_best_effort_output() -> None:
    relay_source = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/topic_relay.py").read_text()
    start_script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert 'declare_parameter("max_rate_hz", 0.0)' in relay_source
    assert 'declare_parameter("input_reliability", "reliable")' in relay_source
    assert 'declare_parameter("output_reliability", "reliable")' in relay_source
    assert "ReliabilityPolicy.BEST_EFFORT" in relay_source
    assert "input_qos" in relay_source
    assert "-p input_reliability:=best_effort" in start_script


def test_nx_topic_relay_supports_transient_local_maps() -> None:
    relay_source = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/topic_relay.py").read_text()
    start_script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert 'declare_parameter("input_durability", "volatile")' in relay_source
    assert 'declare_parameter("output_durability", "volatile")' in relay_source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in relay_source
    assert "relay started:" in relay_source
    assert "relay stats: received=" in relay_source
    assert "-p input_durability:=transient_local" in start_script
    assert "-p output_durability:=transient_local" in start_script


def test_nx_start_script_relays_static_map_to_pusai() -> None:
    start_script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert "map_relay" in start_script
    assert "-p input_topic:=/map" in start_script
    assert "-p output_topic:=/dimos/map" in start_script
    assert "-p msg_module:=nav_msgs.msg" in start_script
    assert "-p msg_type:=OccupancyGrid" in start_script


def test_nx_start_script_relays_compressed_camera_image_to_pusai() -> None:
    start_script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert "camera_relay" in start_script
    assert "--with-camera-source" in start_script
    assert "go2_camera_image_source.py" in start_script
    assert "go2_driver_py video_stream_node" not in start_script
    assert 'CAMERA_SOURCE_TOPIC="${DIMOS_NX_CAMERA_SOURCE_TOPIC:-/camera/image_raw/compressed}"' in start_script
    assert 'CAMERA_RELAY_TOPIC="${DIMOS_NX_CAMERA_RELAY_TOPIC:-/dimos/color_image/compressed}"' in start_script
    assert 'CAMERA_INTERFACE="${DIMOS_NX_CAMERA_INTERFACE:-}"' in start_script
    assert 'CAMERA_INTERFACE="$(select_unitree_interface)"' in start_script
    assert 'CAMERA_JPEG_QUALITY="${DIMOS_NX_CAMERA_JPEG_QUALITY:-60}"' in start_script
    assert "camera_source_dependencies_ready" in start_script
    assert "gi.require_version(\"Gst\", \"1.0\")" in start_script
    assert "gst-inspect-1.0" in start_script
    assert "camera_source_ld_preload" in start_script
    assert "libgomp.so.1" in start_script
    assert 'LD_PRELOAD="${CAMERA_LD_PRELOAD}"' in start_script
    assert "-p input_topic:=${CAMERA_SOURCE_TOPIC}" in start_script
    assert "-p output_topic:=${CAMERA_RELAY_TOPIC}" in start_script
    assert "-p msg_module:=sensor_msgs.msg" in start_script
    assert "-p msg_type:=CompressedImage" in start_script
    assert "-p max_rate_hz:=${CAMERA_RELAY_MAX_RATE_HZ}" in start_script
    assert "-p output_reliability:=best_effort" in start_script


def test_nx_camera_source_publishes_jpeg_compressed_image_without_cv_bridge() -> None:
    source = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/go2_camera_image_source.py").read_text()

    assert "cv_bridge" not in source
    assert "from sensor_msgs.msg import CompressedImage" in source
    assert "Gst.parse_launch" in source
    assert "jpegenc quality={self.jpeg_quality}" in source
    assert "payload=96,clock-rate=90000" in source
    assert "framerate={self.target_fps}/1" not in source
    assert "self.last_publish_time" in source
    assert 'msg.format = "jpeg"' in source
    assert "msg.data =" in source


def test_pusai_start_script_decodes_compressed_camera_locally() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()
    decoder = (DIMOS_PROJECT_ROOT / "scripts/compressed_image_to_image.py").read_text()

    assert "compressed_image_to_image.py" in script
    assert "start_camera_decoder" in script
    assert "stop_camera_decoder" in script
    assert "/dimos/color_image/compressed" in script
    assert "/dimos/color_image" in script
    assert "from sensor_msgs.msg import CompressedImage, Image" in decoder
    assert "cv_bridge" not in decoder
    assert "cv2.imdecode" in decoder
    assert 'msg.encoding = "bgr8"' in decoder


def test_nx_start_script_can_use_amcl_pose_for_map_mode_odom() -> None:
    start_script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()
    amcl_adapter = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/amcl_pose_to_pose.py").read_text()

    assert "--map-pose" in start_script
    assert "map_tf_odom_to_udp" in start_script
    assert 'MAP_POSE_UDP_PORT="${DIMOS_NX_MAP_POSE_UDP_PORT:-39102}"' in start_script
    assert '-p udp_port:="${MAP_POSE_UDP_PORT}"' in start_script
    assert "-p output_topic:=/dimos/odom" in start_script
    assert "if [[ \"${MAP_POSE}\" == true ]]" in start_script
    map_pose_branch = start_script.split("if [[ \"${MAP_POSE}\" == true ]]", 1)[1]
    assert "else" in map_pose_branch
    assert "sport_state_to_udp" in map_pose_branch.split("else", 1)[1]
    assert "PoseWithCovarianceStamped" in amcl_adapter
    assert "PoseStamped" in amcl_adapter


def test_nx_start_script_verifies_amcl_initialpose_subscription_for_map_pose() -> None:
    start_script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert "amcl_initialpose_subscription_ready" in start_script
    assert "source /opt/ros/foxy/setup.bash" in start_script
    assert "source /workspace/cyclonedds_ws/install/setup.bash" in start_script
    assert "ros2 topic info /initialpose" in start_script
    assert "[PASS] AMCL initialpose subscription: ready" in start_script
    assert "[FAIL] AMCL initialpose subscription: unavailable" in start_script


def test_nx_cmd_vel_relay_is_disabled_and_isolated_by_default() -> None:
    relay_source = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/cmd_vel_relay.py").read_text()

    assert 'declare_parameter("input_topic", "/dimos/cmd_vel")' in relay_source
    assert 'declare_parameter("output_topic", "/dimos/cmd_vel_guarded")' in relay_source
    assert 'declare_parameter("enabled", False)' in relay_source
    assert 'declare_parameter("max_linear_x", 0.8)' in relay_source
    assert 'declare_parameter("max_linear_y", 0.5)' in relay_source
    assert 'declare_parameter("max_angular_z", 1.2)' in relay_source
    assert 'declare_parameter("command_timeout_sec", 0.5)' in relay_source
    assert "add_on_set_parameters_callback" in relay_source
    assert "_publish_zero" in relay_source
    assert '"/cmd_vel"' not in relay_source


def test_nx_control_udp_adapter_keeps_wlan_and_robot_bus_separate() -> None:
    adapter_source = (
        DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/guarded_cmd_to_udp.py"
    ).read_text()

    assert 'declare_parameter("input_topic", "/dimos/cmd_vel_guarded")' in adapter_source
    assert 'declare_parameter("udp_host", "127.0.0.1")' in adapter_source
    assert 'declare_parameter("udp_port", 39101)' in adapter_source
    assert "from geometry_msgs.msg import Twist" in adapter_source
    assert "socket.SOCK_DGRAM" in adapter_source
    assert '"/api/sport/request"' not in adapter_source


def test_nx_cyclonedds_configs_use_any_domain_id() -> None:
    root = DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge"

    for name in ("cyclonedds_robot_bus.xml", "cyclonedds_dual_network.xml"):
        config = (root / name).read_text()
        assert '<Domain Id="any">' in config


def test_nx_cyclonedds_robot_bus_uses_foxy_compatible_unitree_address() -> None:
    root = DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge"
    robot_config = (root / "cyclonedds_robot_bus.xml").read_text()
    dual_config = (root / "cyclonedds_dual_network.xml").read_text()

    assert "<NetworkInterfaceAddress>192.168.123.222</NetworkInterfaceAddress>" in robot_config
    assert '<NetworkInterface ip="' not in robot_config
    assert 'NetworkInterface name="eth1"' in dual_config
    assert 'NetworkInterface name="wlan0"' in dual_config
    assert '<NetworkInterface ip="' not in dual_config


def test_nx_sport_gateway_is_disabled_and_never_sends_stand_commands() -> None:
    gateway_source = (
        DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/udp_to_sport_request.py"
    ).read_text()

    assert "from unitree_api.msg import Request" in gateway_source
    assert 'declare_parameter("enabled", False)' in gateway_source
    assert 'declare_parameter("udp_port", 39101)' in gateway_source
    assert 'declare_parameter("max_linear_x", 0.8)' in gateway_source
    assert 'declare_parameter("max_linear_y", 0.5)' in gateway_source
    assert 'declare_parameter("max_angular_z", 1.2)' in gateway_source
    assert 'declare_parameter("command_timeout_sec", 0.5)' in gateway_source
    assert 'create_publisher(Request, "/api/sport/request", 10)' in gateway_source
    assert "MOVE_API_ID = 1008" in gateway_source
    assert "STOP_MOVE_API_ID = 1003" in gateway_source
    assert "_publish_stop" in gateway_source
    assert "if self._enabled:" in gateway_source
    assert "def destroy_node" in gateway_source
    assert "StandUp" not in gateway_source
    assert "RecoveryStand" not in gateway_source
    assert "BalanceStand" not in gateway_source
    assert "parameters: list[Any]" not in gateway_source
    assert '"Go2 Sport gateway started: enabled=%s' not in gateway_source


def test_nx_control_chain_logs_lightweight_velocity_stats() -> None:
    root = DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge"
    relay_source = (root / "cmd_vel_relay.py").read_text()
    udp_source = (root / "guarded_cmd_to_udp.py").read_text()
    gateway_source = (root / "udp_to_sport_request.py").read_text()

    for source in (relay_source, udp_source, gateway_source):
        assert 'declare_parameter("stats_interval_sec", 2.0)' in source
        assert "_stats_timer" in source
        assert "def _log_stats" in source

    assert "commands_in=" in relay_source
    assert "forwarded=" in relay_source
    assert "dropped_disabled=" in relay_source
    assert "udp_sent=" in udp_source
    assert "udp_packets=" in gateway_source
    assert "move_requests=" in gateway_source
    assert "stop_requests=" in gateway_source


def test_nx_start_script_keeps_motion_nodes_opt_in_and_disabled() -> None:
    script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert "WITH_CONTROL=false" in script
    assert "--with-control)" in script
    assert "topic_relay.py" in script
    assert "sport_state_to_udp.py" in script
    assert "udp_to_pose.py" in script
    assert "cmd_vel_relay.py" in script
    assert "guarded_cmd_to_udp.py" in script
    assert "udp_to_sport_request.py" in script
    assert "enabled:=true" not in script
    assert "start_process" in script
    assert "set +u" in script
    assert "set -u" in script
    assert 'ROBOT_DDS_URI="${DIMOS_NX_ROBOT_DDS_URI:-file://${SCRIPT_DIR}/cyclonedds_robot_bus.xml}"' in script
    assert "is_live_pid" in script
    assert '"Z"' in script
    assert "verify_deployment" in script
    assert "verify)" in script
    assert '"enabled=False"' in script
    assert "Traceback" in script
    assert 'LIVOX_WORKSPACE="${DIMOS_NX_LIVOX_WORKSPACE:-/workspace/tmp/livox_ros_driver2_clean_ws}"' in script
    assert 'LIVOX_SETUP="${LIVOX_WORKSPACE}/install/setup.bash"' in script
    assert 'LIVOX_CONFIG="${DIMOS_NX_LIVOX_CONFIG:-${LIVOX_WORKSPACE}/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json}"' in script
    assert "xfer_format:=0" in script
    assert "/livox/lidar:=/registered_scan" in script


def test_nx_start_script_prepares_unitree_robot_bus_address() -> None:
    script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert 'UNITREE_INTERFACE="${DIMOS_NX_UNITREE_INTERFACE:-}"' in script
    assert (
        'UNITREE_INTERFACE_ADDRESS="${DIMOS_NX_UNITREE_INTERFACE_ADDRESS:-192.168.123.222/24}"'
        in script
    )
    assert "select_unitree_interface()" in script
    assert "ensure_unitree_interface_address()" in script
    assert 'ip addr add "${UNITREE_INTERFACE_ADDRESS}" dev "${selected_interface}"' in script
    assert "[WARN] ip command not found; cannot verify" in script
    runtime_script = script.split('source "${LIVOX_SETUP}"', 1)[1]
    assert runtime_script.index("ensure_unitree_interface_address") < runtime_script.index(
        "start_process map_server_wifi"
    )
    assert runtime_script.index("ensure_unitree_interface_address") < runtime_script.index(
        "start_process udp_to_sport_request"
    )


def test_nx_start_script_stop_process_ignores_missing_pid_files_under_set_e() -> None:
    script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert '[[ -f "${pid_file}" ]] || return 0' in script
    assert '[[ -f "${pid_file}" ]] || return\n' not in script


def test_nx_start_script_verifies_only_live_process_startup_logs() -> None:
    script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert "startup_logs_have_exceptions()" in script
    assert 'grep -R -E "Traceback|TypeError|SyntaxError" "${LOG_DIR}"' not in script
    assert 'is_live_pid "$(<"${pid_file}")"' in script
    assert '"${LOG_DIR}/${name}.log"' in script


def test_nx_start_script_waits_for_control_gate_startup_logs() -> None:
    script = (DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/start_nx_bridge.sh").read_text()

    assert "control_gates_logged_disabled()" in script
    assert "for _ in $(seq 1 20)" in script
    assert "sleep 0.25" in script


def test_livox_callback_patch_restores_only_the_dropped_data_path() -> None:
    patch = (
        DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/restore_livox_callbacks.patch"
    ).read_text()

    assert "+  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);" in patch
    assert "+    self->raw_packet_queue_.push_back(packet);" in patch
    assert "+    process_handler->PointCloudProcess(raw_data);" in patch
    assert "+    CheckTimer(id);" in patch
    assert "+  lds_lidar->StorageImuData(imu_data);" in patch
    assert "lds.cpp" not in patch
    assert "lddc.cpp" not in patch


def test_nx_adapter_startup_logging_is_compatible_with_foxy_rclpy() -> None:
    root = DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge"
    relay_source = (root / "cmd_vel_relay.py").read_text()
    udp_source = (root / "guarded_cmd_to_udp.py").read_text()

    assert '"Guarded cmd_vel relay started: enabled=%s' not in relay_source
    assert '"Guarded cmd_vel UDP forwarder started: input=%s' not in udp_source


def test_pusai_start_script_uses_isolated_topics_and_daemon_mode() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()

    assert "DIMOS_NX_LIDAR_TOPIC=/dimos/registered_scan" in script
    assert "DIMOS_NX_ODOM_TOPIC=/dimos/odom" in script
    assert "DIMOS_NX_CMD_VEL_TOPIC=/dimos/cmd_vel" in script
    assert 'DIMOS_NX_COSTMAP_ALGO="${DIMOS_NX_COSTMAP_ALGO:-raycast}"' in script
    assert 'DIMOS_NX_COSTMAP_MARK_FREE_RADIUS="${DIMOS_NX_COSTMAP_MARK_FREE_RADIUS:-0.25}"' in script
    assert 'DIMOS_NX_VOXEL_SIZE="${DIMOS_NX_VOXEL_SIZE:-0.1}"' in script
    assert "unitree-go2-nx-bridge" in script
    assert "--daemon" in script
    assert "stop)" in script
    assert "status)" in script
    assert "set +u" in script
    assert "source /opt/ros/humble/setup.bash" in script
    assert "set -u" in script
    assert "verify_deployment" in script
    assert "verify)" in script
    assert "Blueprint: ${BRIDGE_BLUEPRINT}" in script
    assert "curl -fsS" in script


def test_pusai_data_verifier_can_require_camera_frames() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()
    verifier = (DIMOS_PROJECT_ROOT / "scripts/verify_pusai_nx_data.py").read_text()

    assert "--with-camera" in script
    assert "REQUIRE_CAMERA=true" in script
    assert "verify_args+=(--require-image)" in script
    assert 'parser.add_argument("--require-image", action="store_true")' in verifier
    assert "sensor_msgs.msg import Image" in verifier
    assert '"/dimos/color_image"' in verifier
    assert "image_frames=" in verifier


def test_pusai_start_script_cleans_stale_dimos_workers() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()

    assert "cleanup_dimos_processes()" in script
    assert "pgrep -f \"${DIMOS_VENV}/bin/python\"" in script
    assert "kill -TERM ${pids}" in script
    assert "kill -KILL ${live_pids}" in script
    assert "cleanup_dimos_processes" in script.split("start)")[1]
    assert "cleanup_dimos_processes" in script.split("stop)")[1]


def test_pusai_start_script_waits_for_bridge_registration() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()

    assert "wait_for_bridge_registration()" in script
    assert "Blueprint: ${BRIDGE_BLUEPRINT}" in script
    assert "sleep 1" in script
    assert "wait_for_bridge_registration" in script.split("--daemon")[1]


def test_pusai_start_script_applies_required_lcm_system_config() -> None:
    script = (DIMOS_PROJECT_ROOT / "scripts/start_pusai_nx_bridge.sh").read_text()

    assert "ensure_pusai_lcm_system_config()" in script
    assert "ip link set lo multicast on" in script
    assert "ip route replace 224.0.0.0/4 dev lo" in script
    assert "sysctl -w net.core.rmem_max=67108864" in script
    assert "sysctl -w net.core.rmem_default=67108864" in script
    assert "sudo -n" in script
    assert "ensure_pusai_lcm_system_config" in script.split("start)")[1]
    assert script.split("start)")[1].index(
        "ensure_pusai_lcm_system_config"
    ) < script.split("start)")[1].index('"${DIMOS_CLI}" --viewer')


def test_nx_sport_state_adapter_publishes_isolated_pose_contract() -> None:
    adapter_source = (
        DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/sport_state_to_pose.py"
    ).read_text()

    assert "from unitree_go.msg import SportModeState" in adapter_source
    assert 'declare_parameter("input_topic", "/sportmodestate")' in adapter_source
    assert 'declare_parameter("output_topic", "/dimos/odom")' in adapter_source
    assert "create_subscription(SportModeState" in adapter_source
    assert "create_publisher(PoseStamped" in adapter_source
    assert "pose.pose.position.x = msg.position[0]" in adapter_source
    assert "pose.pose.orientation.w = msg.imu_state.quaternion[0]" in adapter_source
    assert "cmd_vel" not in adapter_source


def test_nx_dual_network_dds_asset_separates_robot_and_pusai_peers() -> None:
    dds_source = (
        DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/cyclonedds_dual_network.xml"
    ).read_text()

    assert '<NetworkInterface name="eth1"' in dds_source
    assert '<NetworkInterface name="wlan0"' in dds_source
    assert '<NetworkInterface ip="' not in dds_source
    assert '<Peer address="192.168.123.161" />' in dds_source
    assert '<Peer address="192.168.1.196" />' in dds_source


def test_nx_robot_bus_dds_asset_targets_only_the_go2_controller() -> None:
    dds_source = (
        DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge/cyclonedds_robot_bus.xml"
    ).read_text()

    assert "<NetworkInterfaceAddress>192.168.123.222</NetworkInterfaceAddress>" in dds_source
    assert '<NetworkInterface ip="' not in dds_source
    assert '<Peer address="192.168.123.161" />' in dds_source
    assert "wlan0" not in dds_source


def test_nx_odom_relay_assets_bridge_dds_networks_only_through_local_udp() -> None:
    root = DIMOS_PROJECT_ROOT / "docker/navigation/nx_bridge"
    state_source = (root / "sport_state_to_udp.py").read_text()
    pose_source = (root / "udp_to_pose.py").read_text()

    assert "from unitree_go.msg import SportModeState" in state_source
    assert 'declare_parameter("udp_host", "127.0.0.1")' in state_source
    assert 'declare_parameter("udp_port", 39100)' in state_source
    assert "socket.SOCK_DGRAM" in state_source
    assert "[float(value) for value in msg.position]" in state_source
    assert "[float(value) for value in msg.imu_state.quaternion]" in state_source
    assert "from geometry_msgs.msg import PoseStamped" in pose_source
    assert 'declare_parameter("output_topic", "/dimos/odom")' in pose_source
    assert "socket.SOCK_DGRAM" in pose_source
    assert "cmd_vel" not in state_source
    assert "cmd_vel" not in pose_source
