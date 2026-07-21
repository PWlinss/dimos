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

from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs import Pose
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid, Path
from dimos.navigation.replanning_a_star.path_clearance import PathClearance


def test_path_clearance_treats_occupied_path_mask_as_obstacle_ahead() -> None:
    grid = np.zeros((30, 30), dtype=np.int8)
    grid[15, 8:23] = CostValues.OCCUPIED

    origin = Pose()
    origin.position.x = -1.5
    origin.position.y = -1.5
    origin.orientation.w = 1.0
    costmap = OccupancyGrid(grid=grid, resolution=0.1, origin=origin)
    path = Path(
        poses=[
            PoseStamped(position=(-1.0, 0.0, 0.0), frame_id="world"),
            PoseStamped(position=(1.0, 0.0, 0.0), frame_id="world"),
        ]
    )
    clearance = PathClearance(global_config, path)
    clearance.update_costmap(costmap)

    assert clearance.is_obstacle_ahead()
