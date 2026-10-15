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

from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.types.robot_location import RobotLocation


class FakeCamera(Module):
    color_image: Out[Image]


class FakeOdom(Module):
    odom: Out[PoseStamped]


class MockedStopNavSkill(NavigationSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        Module.__init__(self)
        self._skill_started = True

    def _cancel_goal_and_stop(self):
        pass


class MockedExploreNavSkill(NavigationSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        Module.__init__(self)
        self._skill_started = True

    def _start_exploration(self, timeout):
        return "Exploration completed successfuly"

    def _cancel_goal_and_stop(self):
        pass


class MockedSemanticNavSkill(NavigationSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        Module.__init__(self)
        self._skill_started = True

    def _navigate_by_tagged_location(self, query):
        return None

    def _navigate_to_object(self, query):
        return None

    def _navigate_using_semantic_map(self, query):
        return f"Successfuly arrived at '{query}'"


class OfflineNavigationSkill(NavigationSkillContainer):
    """Small RPC seam for navigation tests; it never starts a robot module."""

    def __init__(
        self,
        *,
        goal_frame_id: str = "map",
        enable_visual_navigation: bool = False,
        max_odom_age_sec: float | None = None,
        require_coordinate_frame: bool = False,
        semantic_query_ascii_only: bool = False,
    ) -> None:
        self._goal_frame_id = goal_frame_id
        self._enable_visual_navigation = enable_visual_navigation
        self._max_odom_age_sec = max_odom_age_sec
        self._require_coordinate_frame = require_coordinate_frame
        self._semantic_query_ascii_only = semantic_query_ascii_only
        self._latest_image = None
        self._latest_odom = None
        self._latest_odom_received_monotonic = None
        self._vl_model = None
        self._skill_started = True
        self.rpc_handlers = {}

    def get_rpc_calls(self, *methods):  # type: ignore[no-untyped-def]
        handlers = tuple(self.rpc_handlers[method] for method in methods)
        return handlers[0] if len(handlers) == 1 else handlers


def _odom(yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="odom",
        position=Vector3(1.0, 2.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def test_navigation_blueprint_accepts_restricted_configuration() -> None:
    blueprint = NavigationSkillContainer.blueprint(
        goal_frame_id="odom",
        enable_visual_navigation=False,
        max_odom_age_sec=1.0,
        require_coordinate_frame=True,
        semantic_query_ascii_only=True,
    )

    assert blueprint.blueprints[0].kwargs == {
        "goal_frame_id": "odom",
        "enable_visual_navigation": False,
        "max_odom_age_sec": 1.0,
        "require_coordinate_frame": True,
        "semantic_query_ascii_only": True,
    }


def test_tagged_navigation_uses_configured_frame_and_rejects_mismatch() -> None:
    nav = OfflineNavigationSkill(goal_frame_id="odom")
    goals = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: RobotLocation(
            name=query,
            position=(3.0, 4.0, 0.0),
            rotation=(0.0, 0.0, 0.5),
            metadata={"coordinate_frame": "odom"},
        ),
        "NavigationInterface.set_goal": goals.append,
    }

    message = nav.navigate_with_text("charging station")

    assert "Started navigating" in message
    assert goals[0].frame_id == "odom"

    nav.rpc_handlers["SpatialMemory.query_tagged_location"] = lambda query: RobotLocation(
        name=query,
        position=(3.0, 4.0, 0.0),
        rotation=(0.0, 0.0, 0.5),
        metadata={"coordinate_frame": "map"},
    )
    message = nav.navigate_with_text("wrong frame")

    assert "coordinate frame" in message
    assert len(goals) == 1


def test_strict_navigation_rejects_missing_frame_metadata() -> None:
    nav = OfflineNavigationSkill(goal_frame_id="odom", require_coordinate_frame=True)
    goals = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: RobotLocation(
            name=query,
            position=(3.0, 4.0, 0.0),
            rotation=(0.0, 0.0, 0.5),
        ),
        "NavigationInterface.set_goal": goals.append,
    }

    message = nav.navigate_with_text("legacy location")

    assert "no coordinate frame metadata" in message
    assert goals == []


def test_semantic_navigation_defaults_to_map_frame() -> None:
    nav = OfflineNavigationSkill()
    goals = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: None,
        "SpatialMemory.query_by_text": lambda query: [
            {
                "distance": 0.1,
                "metadata": {"pos_x": 1.0, "pos_y": 2.0, "rot_z": 0.0},
            }
        ],
        "NavigationInterface.set_goal": goals.append,
    }

    nav.navigate_with_text("kitchen")

    assert goals[0].frame_id == "map"


def test_tag_location_stores_frame_and_euler_rotation() -> None:
    nav = OfflineNavigationSkill(goal_frame_id="odom")
    tagged = []
    nav.rpc_handlers["SpatialMemory.tag_location"] = (
        lambda location: tagged.append(location) or True
    )
    nav._on_odom(_odom(yaw=0.7))

    assert "Tagged" in nav.tag_location("dock")
    assert tagged[0].metadata["coordinate_frame"] == "odom"
    assert tagged[0].rotation[2] == pytest.approx(0.7)
    assert tagged[0].rotation[2] != pytest.approx(_odom(0.7).orientation.z)


def test_visual_navigation_disabled_skips_object_fallback() -> None:
    nav = OfflineNavigationSkill(enable_visual_navigation=False)
    calls = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: None,
        "SpatialMemory.query_by_text": lambda query: calls.append(query) or [],
    }

    def unexpected_visual_fallback(query):  # type: ignore[no-untyped-def]
        raise AssertionError("visual navigation fallback must be disabled")

    nav._navigate_to_object = unexpected_visual_fallback  # type: ignore[method-assign]
    message = nav.navigate_with_text("bookshelf")

    assert "semantic map" in message
    assert calls == ["bookshelf"]
    assert nav._vl_model is None


def test_configured_odom_freshness_blocks_missing_and_stale_goals(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    nav = OfflineNavigationSkill(
        goal_frame_id="odom",
        max_odom_age_sec=1.0,
        semantic_query_ascii_only=True,
    )
    goals = []
    nav.rpc_handlers["NavigationInterface.set_goal"] = goals.append

    assert "Error" in nav.navigate_with_text("dock")
    assert "Error" in nav.tag_location("dock")
    assert goals == []

    now = 100.0
    monkeypatch.setattr("dimos.agents.skills.navigation.time.monotonic", lambda: now)
    nav._on_odom(_odom())
    now = 102.0

    message = nav.navigate_with_text("dock")

    assert "stale" in message
    assert goals == []


def test_strict_navigation_rejects_wrong_odom_frame() -> None:
    nav = OfflineNavigationSkill(
        goal_frame_id="odom",
        require_coordinate_frame=True,
    )
    nav._on_odom(
        PoseStamped(
            frame_id="map",
            position=Vector3(),
            orientation=Quaternion(),
        )
    )

    assert "Odometry frame 'map'" in nav.tag_location("dock")


def test_low_confidence_semantic_result_does_not_set_goal() -> None:
    nav = OfflineNavigationSkill(goal_frame_id="odom", semantic_query_ascii_only=True)
    goals = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: None,
        "SpatialMemory.query_by_text": lambda query: [
            {
                "distance": 0.9,
                "metadata": {"pos_x": 1.0, "pos_y": 2.0, "rot_z": 0.0},
            }
        ],
        "NavigationInterface.set_goal": goals.append,
    }

    message = nav.navigate_with_text("far away")

    assert "similarity" in message
    assert "below the threshold" in message
    assert "no navigation goal was sent" in message
    assert goals == []


@pytest.mark.parametrize("query", ["紫色火星花", "", "   "])
def test_nx_rejects_non_ascii_semantic_query_after_tag_lookup(query: str) -> None:
    nav = OfflineNavigationSkill(goal_frame_id="odom", semantic_query_ascii_only=True)
    goals = []
    semantic_queries = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: None,
        "SpatialMemory.query_by_text": lambda query: semantic_queries.append(query) or [],
        "NavigationInterface.set_goal": goals.append,
    }

    message = nav.navigate_with_text(query)

    assert "English query" in message
    assert "no navigation goal was sent" in message
    assert semantic_queries == []
    assert goals == []


def test_nx_allows_chinese_tag_before_semantic_query_guard() -> None:
    nav = OfflineNavigationSkill(goal_frame_id="odom", semantic_query_ascii_only=True)
    goals = []
    semantic_queries = []
    nav.rpc_handlers = {
        "SpatialMemory.query_tagged_location": lambda query: RobotLocation(
            name=query,
            position=(3.0, 4.0, 0.0),
            rotation=(0.0, 0.0, 0.0),
            metadata={"coordinate_frame": "odom"},
        ),
        "SpatialMemory.query_by_text": lambda query: semantic_queries.append(query) or [],
        "NavigationInterface.set_goal": goals.append,
    }

    message = nav.navigate_with_text("起点")

    assert "Started navigating" in message
    assert len(goals) == 1
    assert semantic_queries == []


def test_navigation_state_and_stop_use_navigation_interface() -> None:
    nav = OfflineNavigationSkill()
    cancelled = []
    nav.rpc_handlers = {
        "NavigationInterface.get_state": lambda: NavigationState.FOLLOWING_PATH,
        "NavigationInterface.cancel_goal": lambda: cancelled.append(True),
    }

    assert nav.get_navigation_state() == "Navigation state: following_path"
    assert nav.stop_navigation() == "Stopped"
    assert cancelled == [True]


@pytest.mark.slow
def test_stop_movement(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedStopNavSkill.blueprint(),
        ],
        messages=[HumanMessage("Stop moving. Use the stop_movement tool.")],
    )

    assert "stopped" in history[-1].content.lower()


@pytest.mark.slow
def test_start_exploration(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedExploreNavSkill.blueprint(),
        ],
        messages=[
            HumanMessage("Take a look around for 10 seconds. Use the start_exploration tool.")
        ],
    )

    assert "explor" in history[-1].content.lower()


@pytest.mark.slow
def test_go_to_semantic_location(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedSemanticNavSkill.blueprint(),
        ],
        messages=[HumanMessage("Go to the bookshelf. Use the navigate_with_text tool.")],
    )

    assert "success" in history[-1].content.lower()
