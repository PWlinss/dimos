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

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.navigation.replanning_a_star.controllers import PController
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap


def test_local_planner_passes_configured_speed_and_velocity_floors() -> None:
    config = GlobalConfig(
        planner_robot_speed=0.4,
        planner_min_linear_velocity=0.4,
        planner_min_angular_velocity=0.55,
        planner_max_angular_velocity=0.55,
    )
    planner = LocalPlanner(config, NavigationMap(config), goal_tolerance=0.2)

    assert planner._speed == pytest.approx(0.4)
    assert getattr(planner._controller, "_speed") == pytest.approx(0.4)
    assert getattr(planner._controller, "_min_linear_velocity") == pytest.approx(0.4)
    assert getattr(planner._controller, "_min_angular_velocity") == pytest.approx(0.55)
    assert getattr(planner._controller, "_max_angular_velocity") == pytest.approx(0.55)


def test_p_controller_uses_configured_linear_and_angular_outputs() -> None:
    controller = PController(
        GlobalConfig(),
        speed=0.4,
        control_frequency=10,
        min_linear_velocity=0.4,
        min_angular_velocity=0.55,
        max_angular_velocity=0.55,
    )
    odom = PoseStamped(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0))

    forward = controller.advance(np.array([1.0, 0.0]), odom)
    assert forward.linear.x == pytest.approx(0.4)
    assert forward.angular.z == pytest.approx(0.0)

    minimum_turn = controller.rotate(0.1)
    assert minimum_turn.angular.z == pytest.approx(0.55)

    maximum_turn = controller.rotate(2.0)
    assert maximum_turn.angular.z == pytest.approx(0.55)


def test_local_planner_defaults_remain_unchanged() -> None:
    config = GlobalConfig()
    planner = LocalPlanner(config, NavigationMap(config), goal_tolerance=0.2)

    assert planner._speed == pytest.approx(0.55)
    assert getattr(planner._controller, "_min_linear_velocity") == pytest.approx(0.2)
    assert getattr(planner._controller, "_min_angular_velocity") == pytest.approx(0.2)
    assert getattr(planner._controller, "_max_angular_velocity") == pytest.approx(0.55)
