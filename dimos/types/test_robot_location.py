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

from dimos.types.robot_location import RobotLocation


def test_robot_location_custom_metadata_round_trips_without_overwriting_core_fields() -> None:
    location = RobotLocation(
        name="dock",
        position=(1.0, 2.0, 3.0),
        rotation=(0.1, 0.2, 0.3),
        metadata={"coordinate_frame": "odom", "pos_x": 99.0},
    )

    vector_metadata = location.to_vector_metadata()
    restored = RobotLocation.from_vector_metadata(vector_metadata)

    assert vector_metadata["pos_x"] == 1.0
    assert restored.metadata["coordinate_frame"] == "odom"
