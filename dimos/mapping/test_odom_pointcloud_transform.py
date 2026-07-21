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
import pytest

from dimos.mapping.odom_pointcloud_transform import OdomPointCloudTransformer
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.sensor_msgs import PointCloud2


def _cloud(*points: tuple[float, float, float]) -> PointCloud2:
    return PointCloud2.from_numpy(np.asarray(points, dtype=np.float32), frame_id="livox_frame")


def _pose(
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> PoseStamped:
    return PoseStamped(
        frame_id="odom",
        position=Vector3(*position),
        orientation=Quaternion(*orientation),
    )


@pytest.fixture
def transformer() -> OdomPointCloudTransformer:
    instance = OdomPointCloudTransformer()
    yield instance
    instance.stop()


def test_transformer_applies_latest_odom_translation_to_lidar_points(
    transformer: OdomPointCloudTransformer,
) -> None:
    transformer._on_odom(_pose(position=(1.0, 2.0, 0.5)))

    transformed = transformer._transform(_cloud((2.0, 0.0, 0.0)))

    assert transformed is not None
    points, _ = transformed.as_numpy()
    np.testing.assert_allclose(points, [[3.0, 2.0, 0.5]])
    assert transformed.frame_id == "world"


def test_transformer_applies_odom_rotation_before_translation(
    transformer: OdomPointCloudTransformer,
) -> None:
    transformer._on_odom(_pose(position=(1.0, 0.0, 0.0), orientation=(0.0, 0.0, 1.0, 0.0)))

    transformed = transformer._transform(_cloud((2.0, 0.0, 0.0)))

    assert transformed is not None
    points, _ = transformed.as_numpy()
    np.testing.assert_allclose(points, [[-1.0, 0.0, 0.0]], atol=1e-6)


def test_transformer_applies_configured_lidar_extrinsic_before_odom() -> None:
    transformer = OdomPointCloudTransformer(
        lidar_translation=(0.5, 0.0, 0.2),
        lidar_rotation=(0.0, 0.0, 0.0, 1.0),
    )
    transformer._on_odom(_pose(position=(1.0, 2.0, 0.0)))

    try:
        transformed = transformer._transform(_cloud((2.0, 0.0, 0.0)))

        assert transformed is not None
        points, _ = transformed.as_numpy()
        np.testing.assert_allclose(points, [[3.5, 2.0, 0.2]])
    finally:
        transformer.stop()


def test_transformer_drops_lidar_until_an_odom_sample_is_available(
    transformer: OdomPointCloudTransformer,
) -> None:

    assert transformer._transform(_cloud((1.0, 0.0, 0.0))) is None


def test_transformer_publishes_raw_and_world_frame_lidar(
    transformer: OdomPointCloudTransformer,
) -> None:
    raw_messages: list[PointCloud2] = []
    world_messages: list[PointCloud2] = []
    transformer.raw_lidar.subscribe(raw_messages.append)
    transformer.world_lidar.subscribe(world_messages.append)
    transformer._on_odom(_pose(position=(1.0, 0.0, 0.0)))

    transformer._on_lidar(_cloud((2.0, 0.0, 0.0)))

    assert raw_messages[0].frame_id == "livox_frame"
    assert world_messages[0].frame_id == "world"
    points, _ = world_messages[0].as_numpy()
    np.testing.assert_allclose(points, [[3.0, 0.0, 0.0]])


def test_transformer_publishes_latest_odom_for_robot_visualization(
    transformer: OdomPointCloudTransformer,
) -> None:
    robot_poses: list[PoseStamped] = []
    transformer.robot_pose.subscribe(robot_poses.append)
    odom = _pose(position=(1.0, 2.0, 0.5))

    transformer._on_odom(odom)

    assert robot_poses == [odom]
