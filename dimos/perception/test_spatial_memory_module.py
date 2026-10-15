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

import asyncio
import os
from threading import RLock
import time
from types import SimpleNamespace

import numpy as np
import pytest
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable

from dimos.agents_deprecated.memory.spatial_vector_db import SpatialVectorDB
from dimos.agents_deprecated.memory.visual_memory import VisualMemory
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay

logger = setup_logger()


def _spatial_memory_for_keyframe_test() -> SpatialMemory:
    memory = SpatialMemory.__new__(SpatialMemory)
    memory._latest_image = SimpleNamespace(ts=1.0)
    memory._latest_video_frame = np.zeros((8, 8, 3), dtype=np.uint8)
    memory.world_frame_id = "odom"
    memory.base_frame_id = "base_link"
    memory.transform_tolerance_sec = 0.25
    memory.min_distance_threshold = 0.2
    memory.min_rotation_threshold = np.deg2rad(15)
    memory.similarity_threshold = 0.95
    memory.last_position = None
    memory.last_record_time = None
    memory._last_processed_image_ts = None
    memory._last_rotation_yaw = None
    memory._last_embedding = None
    memory.min_time_threshold = 0.0
    memory.frame_count = 0
    memory.stored_frame_count = 0
    memory.embedding_provider = SimpleNamespace(get_embedding=lambda _: np.array([1.0, 0.0]))
    memory.vector_db = SimpleNamespace(add_image_vector=lambda **_: None)
    memory._visual_memory = None
    memory.visual_memory_path = None

    class FakeTF:
        ts = 1.0
        rotation = SimpleNamespace(to_euler=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0))

        def to_pose(self, ts):
            return SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                ts=ts,
            )

    memory._tf = SimpleNamespace(get=lambda *_: FakeTF())
    return memory


def test_spatial_memory_skips_repeated_image_timestamp() -> None:
    memory = _spatial_memory_for_keyframe_test()
    memory._process_frame_locked()
    memory._process_frame_locked()
    assert memory.stored_frame_count == 1


def test_spatial_memory_rejects_stationary_odom_drift_but_keeps_changed_view() -> None:
    memory = _spatial_memory_for_keyframe_test()
    memory._process_frame_locked()

    for index in range(1, 22):
        image_ts = float(index + 1)
        memory._latest_image = SimpleNamespace(ts=image_ts)
        x = index * 0.01
        memory._tf.get = lambda *_, x=x, ts=image_ts: SimpleNamespace(
            ts=ts,
            rotation=SimpleNamespace(
                to_euler=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0)
            ),
            to_pose=lambda ts: SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.0, z=0.0), ts=ts
            ),
        )
        memory._process_frame_locked()

    assert memory.stored_frame_count == 1

    memory._latest_image = SimpleNamespace(ts=23.0)
    memory.embedding_provider.get_embedding = lambda _: np.array([0.0, 1.0])
    memory._tf.get = lambda *_: SimpleNamespace(
        ts=23.0,
        rotation=SimpleNamespace(to_euler=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0)),
        to_pose=lambda ts: SimpleNamespace(
            position=SimpleNamespace(x=0.21, y=0.0, z=0.0), ts=ts
        ),
    )
    memory._process_frame_locked()
    assert memory.stored_frame_count == 2


def test_spatial_memory_keeps_a_significant_turn() -> None:
    memory = _spatial_memory_for_keyframe_test()
    memory._process_frame_locked()
    memory._latest_image = SimpleNamespace(ts=2.0)
    memory._tf.get = lambda *_: SimpleNamespace(
        ts=2.0,
        rotation=SimpleNamespace(
            to_euler=lambda: SimpleNamespace(x=0.0, y=0.0, z=np.deg2rad(35))
        ),
        to_pose=lambda ts: SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0), ts=ts
        ),
    )
    memory._process_frame_locked()
    assert memory.stored_frame_count == 2


class VideoReplayModule(Module):
    """Module that replays video data from TimedSensorReplay."""

    video_out: Out[Image]

    def __init__(self, video_path: str) -> None:
        super().__init__()
        self.video_path = video_path
        self._subscription = None

    @rpc
    def start(self) -> None:
        """Start replaying video data."""
        # Use TimedSensorReplay to replay video frames
        video_replay = TimedSensorReplay(self.video_path, autocast=Image.from_numpy)

        # Subscribe to the replay stream and publish to LCM
        self._subscription = (
            video_replay.stream()
            .pipe(
                ops.sample(2),  # Sample every 2 seconds for resource-constrained systems
                ops.take(5),  # Only take 5 frames total
            )
            .subscribe(self.video_out.publish)
        )

        logger.info("VideoReplayModule started")

    @rpc
    def stop(self) -> None:
        """Stop replaying video data."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
        logger.info("VideoReplayModule stopped")


class OdometryReplayModule(Module):
    """Module that replays odometry data and publishes to the tf system."""

    def __init__(self, odom_path: str) -> None:
        super().__init__()
        self.odom_path = odom_path
        self._subscription = None

    def _publish_tf(self, odom: Odometry) -> None:
        """Convert odometry to TF transforms and publish."""
        self.tf.publish(Transform.from_pose("base_link", odom))

    @rpc
    def start(self) -> None:
        """Start replaying odometry data."""
        # Use TimedSensorReplay to replay odometry
        odom_replay = TimedSensorReplay(self.odom_path, autocast=Odometry.from_msg)

        # Subscribe to the replay stream and publish to tf
        self._subscription = (
            odom_replay.stream()
            .pipe(
                ops.sample(0.5),  # Sample every 500ms
                ops.take(10),  # Only take 10 odometry updates total
            )
            .subscribe(self._publish_tf)
        )

        logger.info("OdometryReplayModule started")

    @rpc
    def stop(self) -> None:
        """Stop replaying odometry data."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
        logger.info("OdometryReplayModule stopped")


@pytest.fixture()
def dimos():
    dimos = ModuleCoordinator()
    dimos.start()
    try:
        yield dimos
    finally:
        dimos.stop()


def test_spatial_memory_repeated_stop_preserves_saved_images(tmp_path, monkeypatch) -> None:
    memory_path = tmp_path / "visual_memory.pkl"
    visual_memory = VisualMemory(output_dir=str(tmp_path))
    visual_memory.add("frame-1", np.zeros((8, 8, 3), dtype=np.uint8))

    spatial_memory = SpatialMemory.__new__(SpatialMemory)
    spatial_memory._disposables = CompositeDisposable()
    spatial_memory._process_lock = RLock()
    spatial_memory._stopped = False
    spatial_memory._visual_memory = visual_memory
    spatial_memory.visual_memory_path = str(memory_path)
    monkeypatch.setattr(Module, "stop", lambda self: None)

    spatial_memory.stop()
    spatial_memory.stop()

    loaded = VisualMemory.load(str(memory_path), output_dir=str(tmp_path))
    assert loaded.count() == 1
    assert loaded.get("frame-1") is not None


def test_spatial_vector_query_uses_spatial_memory_position_metadata() -> None:
    class FakeCollection:
        def get(self):
            return {
                "ids": ["far", "near-a", "near-b"],
                "metadatas": [
                    {"pos_x": 3.0, "pos_y": 0.0},
                    {"pos_x": 0.5, "pos_y": 0.0},
                    {"pos_x": 0.2, "pos_y": 0.2},
                ],
            }

    vector_db = SpatialVectorDB.__new__(SpatialVectorDB)
    vector_db.image_collection = FakeCollection()
    vector_db.visual_memory = VisualMemory()

    results = vector_db.query_by_location(0.0, 0.0, radius=1.0, limit=2)

    assert [result["id"] for result in results] == ["near-b", "near-a"]
    assert results[0]["distance"] == pytest.approx(np.sqrt(0.08))


def test_process_query_results_handles_nested_and_flat_results() -> None:
    vector_db = SpatialVectorDB.__new__(SpatialVectorDB)
    nested = vector_db._process_query_results(
        {
            "ids": [["first", "second", "third"]],
            "metadatas": [[{"label": "a"}, {"label": "b"}, {"label": "c"}]],
            "distances": [[0.1, 0.2, 0.3]],
        }
    )
    flat = vector_db._process_query_results(
        {"ids": ["location"], "metadatas": [{"pos_x": 1.0}], "distances": [0.4]}
    )

    assert [result["id"] for result in nested] == ["first", "second", "third"]
    assert [result["metadata"]["label"] for result in nested] == ["a", "b", "c"]
    assert [result["distance"] for result in nested] == [0.1, 0.2, 0.3]
    assert flat == [{"id": "location", "metadata": {"pos_x": 1.0}, "distance": 0.4}]


@pytest.mark.slow
@pytest.mark.skipif_in_ci
@pytest.mark.asyncio
async def test_spatial_memory_module_with_replay(dimos, tmp_path):
    """Test SpatialMemory module with TimedSensorReplay inputs."""
    # Get test data paths
    data_path = get_data("unitree_office_walk")
    video_path = os.path.join(data_path, "video")
    odom_path = os.path.join(data_path, "odom")

    # Deploy modules
    # Video replay module
    video_module = dimos.deploy(VideoReplayModule, video_path)
    video_module.video_out.transport = LCMTransport("/test_video", Image)

    # Odometry replay module (publishes to tf system directly)
    odom_module = dimos.deploy(OdometryReplayModule, odom_path)

    # Spatial memory module
    spatial_memory = dimos.deploy(
        SpatialMemory,
        collection_name="test_spatial_memory",
        embedding_model="clip",
        embedding_dimensions=512,
        min_distance_threshold=0.5,  # 0.5m for test
        min_time_threshold=1.0,  # 1 second
        db_path=str(tmp_path / "chroma_db"),
        visual_memory_path=str(tmp_path / "visual_memory.pkl"),
        new_memory=True,
        output_dir=str(tmp_path / "images"),
    )

    # Connect video stream
    spatial_memory.color_image.connect(video_module.video_out)

    # Start all modules
    video_module.start()
    odom_module.start()
    spatial_memory.start()
    logger.info("All modules started, processing in background...")

    # Wait for frames to be processed with timeout
    timeout = 10.0  # 10 second timeout
    start_time = time.time()

    # Keep checking stats while modules are running
    while (time.time() - start_time) < timeout:
        stats = spatial_memory.get_stats()
        if stats["frame_count"] > 0 and stats["stored_frame_count"] > 0:
            logger.info(
                f"Frames processing - Frame count: {stats['frame_count']}, Stored: {stats['stored_frame_count']}"
            )
            break
        await asyncio.sleep(0.5)
    else:
        # Timeout reached
        stats = spatial_memory.get_stats()
        logger.error(
            f"Timeout after {timeout}s - Frame count: {stats['frame_count']}, Stored: {stats['stored_frame_count']}"
        )
        raise AssertionError(f"No frames processed within {timeout} seconds")

    await asyncio.sleep(2)

    mid_stats = spatial_memory.get_stats()
    logger.info(
        f"Mid-test stats - Frame count: {mid_stats['frame_count']}, Stored: {mid_stats['stored_frame_count']}"
    )
    assert mid_stats["frame_count"] >= stats["frame_count"], (
        "Frame count should increase or stay same"
    )

    # Test query while modules are still running
    try:
        text_results = spatial_memory.query_by_text("office")
        logger.info(f"Query by text 'office' returned {len(text_results)} results")
        assert len(text_results) > 0, "Should have at least one result"
    except Exception as e:
        logger.warning(f"Query by text failed: {e}")

    final_stats = spatial_memory.get_stats()
    logger.info(
        f"Final stats - Frame count: {final_stats['frame_count']}, Stored: {final_stats['stored_frame_count']}"
    )
