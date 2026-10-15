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

import sys
from threading import RLock
from types import ModuleType

import numpy as np

from dimos.agents_deprecated.memory.visual_memory import VisualMemory
from dimos.mapping.odom_tf_relay import (
    OdomTFRelay,
    OdomTFRelayConfig,
    odom_tf_relay,
)
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.perception import spatial_perception
from dimos.perception.spatial_perception import SpatialMemory


class RecordingTF:
    def __init__(self, transform: Transform | None = None) -> None:
        self.transform = transform
        self.calls: list[tuple[str, str, float, float]] = []
        self.published: list[Transform] = []

    def get(self, parent: str, child: str, timestamp: float, tolerance: float) -> Transform | None:
        self.calls.append((parent, child, timestamp, tolerance))
        return self.transform

    def publish(self, transform: Transform) -> None:
        self.published.append(transform)


class RecordingVectorDB:
    def __init__(
        self,
        visual_memory: VisualMemory | None = None,
        chroma_client: object | None = None,
    ) -> None:
        self.visual_memory = visual_memory
        self.chroma_client = chroma_client
        self.records: list[dict] = []

    def add_image_vector(self, **record: object) -> None:
        self.records.append(record)


class OfflineEmbedding:
    def __init__(self, *args: object, **kwargs: object) -> None:
        pass

    def get_embedding(self, image: np.ndarray) -> np.ndarray:
        return np.ones(2, dtype=np.float32)


def _pose(timestamp: float, frame_id: str = "odom") -> PoseStamped:
    return PoseStamped(
        ts=timestamp,
        frame_id=frame_id,
        position=Vector3(1.0, 2.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _memory(tf: RecordingTF) -> tuple[SpatialMemory, RecordingVectorDB]:
    memory = object.__new__(SpatialMemory)
    memory.world_frame_id = "odom"
    memory.base_frame_id = "base_link"
    memory.transform_tolerance_sec = 0.2
    memory._latest_image = None
    memory._latest_image_ts = None
    memory._latest_video_frame = None
    memory._process_lock = RLock()
    memory._stopped = False
    memory._last_processed_image_ts = None
    memory._last_rotation_yaw = None
    memory._last_embedding = None
    memory._tf = tf
    memory.min_distance_threshold = 0.0
    memory.min_rotation_threshold = 0.0
    memory.similarity_threshold = None
    memory.min_time_threshold = 0.0
    memory.last_position = None
    memory.last_record_time = None
    memory.frame_count = 0
    memory.stored_frame_count = 0
    memory.embedding_provider = OfflineEmbedding()
    db = RecordingVectorDB()
    memory.vector_db = db
    memory._visual_memory = None
    memory.visual_memory_path = None
    return memory, db


def test_odom_tf_relay_preserves_frames_and_timestamp() -> None:
    relay = object.__new__(OdomTFRelay)
    relay.config = OdomTFRelayConfig(world_frame_id="odom", base_frame_id="base_link")
    tf = RecordingTF()
    relay._tf = tf

    odom = _pose(123.5)
    relay._publish_tf(odom)

    assert len(tf.published) == 1
    published = tf.published[0]
    assert published.frame_id == "odom"
    assert published.child_frame_id == "base_link"
    assert published.ts == odom.ts

    relay._publish_tf(_pose(123.5, frame_id="map"))
    assert len(tf.published) == 1


def test_odom_tf_relay_blueprint_symbol() -> None:
    assert callable(odom_tf_relay)
    blueprint = odom_tf_relay(world_frame_id="odom", base_frame_id="base_link")
    assert blueprint.blueprints[0].module is OdomTFRelay


def test_spatial_memory_uses_matching_image_timestamp_and_metadata() -> None:
    tf = RecordingTF(Transform.from_pose("base_link", _pose(10.1)))
    memory, db = _memory(tf)
    image = Image(data=np.zeros((2, 2, 3), dtype=np.uint8), ts=10.0)

    memory._on_image(image)
    memory._process_frame()

    assert memory._latest_image is image
    assert memory._latest_image_ts == 10.0
    assert tf.calls == [("odom", "base_link", 10.0, 0.2)]
    assert len(db.records) == 1
    metadata = db.records[0]["metadata"]
    assert metadata["coordinate_frame"] == "odom"
    assert metadata["timestamp"] == 10.0


def test_spatial_memory_rejects_missing_or_stale_tf() -> None:
    for transform in (None, Transform.from_pose("base_link", _pose(20.5))):
        tf = RecordingTF(transform)
        memory, db = _memory(tf)
        memory._on_image(Image(data=np.zeros((2, 2, 3), dtype=np.uint8), ts=20.0))
        memory._process_frame()
        assert db.records == []


def test_spatial_memory_reopens_visual_memory_without_new_memory(monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(spatial_perception, "ImageEmbeddingProvider", OfflineEmbedding)
    monkeypatch.setattr(
        spatial_perception,
        "SpatialVectorDB",
        lambda **kwargs: RecordingVectorDB(kwargs["visual_memory"]),
    )

    visual_path = tmp_path / "visual_memory.pkl"
    output_dir = tmp_path / "images"
    first = SpatialMemory(
        embedding_model="offline",
        embedding_dimensions=2,
        db_path=None,
        visual_memory_path=str(visual_path),
        output_dir=str(output_dir),
        new_memory=True,
        visual_memory=VisualMemory(output_dir=str(output_dir)),
    )
    reopened = None
    try:
        first._visual_memory.add("frame", np.zeros((2, 2, 3), dtype=np.uint8))
        assert first.save()

        reopened = SpatialMemory(
            embedding_model="offline",
            embedding_dimensions=2,
            db_path=None,
            visual_memory_path=str(visual_path),
            output_dir=str(output_dir),
            new_memory=False,
        )
        assert reopened._visual_memory.count() == 1
    finally:
        first.stop()
        if reopened is not None:
            reopened.stop()


def test_spatial_memory_new_memory_false_preserves_existing_vector_db(
    monkeypatch, tmp_path
) -> None:
    monkeypatch.setattr(spatial_perception, "ImageEmbeddingProvider", OfflineEmbedding)
    persistent_client = object()
    chromadb = ModuleType("chromadb")
    chromadb.PersistentClient = lambda **kwargs: persistent_client  # type: ignore[attr-defined]
    chromadb_config = ModuleType("chromadb.config")
    chromadb_config.Settings = lambda **kwargs: object()  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "chromadb", chromadb)
    monkeypatch.setitem(sys.modules, "chromadb.config", chromadb_config)
    monkeypatch.setattr(
        spatial_perception,
        "SpatialVectorDB",
        lambda **kwargs: RecordingVectorDB(kwargs["visual_memory"], kwargs["chroma_client"]),
    )

    db_path = tmp_path / "chroma"
    db_path.mkdir()
    existing_record = db_path / "existing-record.bin"
    existing_record.write_bytes(b"existing-vector-record")
    memory = SpatialMemory(
        embedding_model="offline",
        embedding_dimensions=2,
        db_path=str(db_path),
        visual_memory_path=None,
        new_memory=False,
        visual_memory=VisualMemory(output_dir=str(tmp_path / "images")),
    )
    try:
        assert existing_record.read_bytes() == b"existing-vector-record"
        assert memory.vector_db.chroma_client is persistent_client
    finally:
        memory.stop()
