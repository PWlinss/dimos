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

import numpy as np

from dimos.msgs.geometry_msgs import Pose, Vector3
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid
from dimos.navigation.replanning_a_star.global_planner import _clear_start_footprint
from dimos.navigation.replanning_a_star.global_planner import _should_check_position_stuck


def test_clear_start_footprint_frees_robot_cell_without_erasing_remote_obstacles() -> None:
    grid = np.zeros((21, 21), dtype=np.int8)
    grid[10, 10] = CostValues.OCCUPIED
    grid[10, 11] = CostValues.OCCUPIED
    grid[2, 2] = CostValues.OCCUPIED

    origin = Pose()
    origin.position.x = -1.0
    origin.position.y = -1.0
    origin.orientation.w = 1.0
    costmap = OccupancyGrid(grid=grid, resolution=0.1, origin=origin)

    _clear_start_footprint(costmap, Vector3(0.0, 0.0, 0.0), radius=0.16)

    assert costmap.grid[10, 10] == CostValues.FREE
    assert costmap.grid[10, 11] == CostValues.FREE
    assert costmap.grid[2, 2] == CostValues.OCCUPIED


def test_position_stuck_check_only_applies_while_following_path() -> None:
    assert not _should_check_position_stuck("initial_rotation")
    assert _should_check_position_stuck("path_following")
    assert not _should_check_position_stuck("final_rotation")
    assert not _should_check_position_stuck("idle")
