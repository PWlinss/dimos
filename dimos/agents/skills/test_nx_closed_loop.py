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

import re
import threading
import time
from typing import Any

from dimos.agents.skills.nx_closed_loop import NXClosedLoopNavigationSkillContainer
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.navigation.base import NavigationState


class FakeImage:
    def agent_encode(self) -> list[dict[str, Any]]:
        return [{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,ZmFrZQ=="}}]


class FakeSnapshot:
    def __init__(self, locations: list[dict[str, Any]] | None = None) -> None:
        self.locations = locations or []

    def get_visualization_snapshot(self) -> dict[str, Any]:
        return {"locations": self.locations}


class FakeNavigation:
    def __init__(self, *, reached: bool = True) -> None:
        self.goals: list[PoseStamped] = []
        self.reached = reached
        self.cancelled = False
        self.cancel_count = 0
        self.on_goal: Any = None

    def set_goal(self, goal: PoseStamped) -> bool:
        self.goals.append(goal)
        if self.on_goal:
            self.on_goal(goal)
        return True

    def get_state(self) -> NavigationState:
        return NavigationState.FOLLOWING_PATH if not self.reached else NavigationState.IDLE

    def is_goal_reached(self) -> bool:
        return self.reached

    def cancel_goal(self) -> bool:
        self.cancelled = True
        self.cancel_count += 1
        return True


class ArrivedIdleRaceNavigation(FakeNavigation):
    def __init__(self) -> None:
        super().__init__(reached=False)
        self.state_calls = 0

    def get_state(self) -> NavigationState:
        self.state_calls += 1
        return (
            NavigationState.FOLLOWING_PATH
            if self.state_calls == 1
            else NavigationState.IDLE
        )


class FakeAgent:
    def __init__(self) -> None:
        self.messages: list[Any] = []
        self.event = threading.Event()

    def add_message(self, message: Any) -> None:
        self.messages.append(message)
        self.event.set()


def _skill(
    *,
    locations: list[dict[str, Any]] | None = None,
    memory_rows: list[dict[str, Any]] | None = None,
    nav: FakeNavigation | None = None,
    image_timeout: float = 0.2,
) -> NXClosedLoopNavigationSkillContainer:
    skill = NXClosedLoopNavigationSkillContainer.__new__(NXClosedLoopNavigationSkillContainer)
    skill._goal_frame_id = "odom"
    skill._max_odom_age_sec = 1.0
    skill._require_coordinate_frame = True
    skill._similarity_threshold = 0.23
    skill._latest_odom = PoseStamped(
        frame_id="odom", position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion()
    )
    skill._latest_odom_received_monotonic = time.monotonic()
    skill._latest_image = None
    skill._latest_image_received_monotonic = None
    skill._skill_started = True
    skill._spatial_memory = FakeSnapshot(locations)
    skill._agent_spec = FakeAgent()
    skill._max_steps = 6
    skill._max_segment_distance = 2.0
    skill._segment_timeout_sec = 0.1
    skill._task_timeout_sec = 10.0
    skill._image_timeout_sec = image_timeout
    skill._task_lock = threading.RLock()
    skill._task = None
    skill._candidates = {}
    skill._human_input_count = 0
    skill._last_started_input_count = None
    nav = nav or FakeNavigation()
    skill._navigation_spec = nav
    nav.on_goal = lambda goal: (
        setattr(skill, "_latest_odom", goal),
        setattr(skill, "_latest_odom_received_monotonic", time.monotonic()),
    )
    skill._spatial_memory.query_by_text = lambda query, limit=5: memory_rows or []  # type: ignore[method-assign]
    handlers = {
        "SpatialMemory.tag_location": lambda location: True,
    }
    skill.get_rpc_calls = lambda *methods: (  # type: ignore[method-assign]
        handlers[methods[0]] if len(methods) == 1 else tuple(handlers[name] for name in methods)
    )
    return skill


def _feed_images(skill: NXClosedLoopNavigationSkillContainer, stop: threading.Event) -> None:
    while not stop.is_set():
        skill._on_color_image(FakeImage())  # type: ignore[arg-type]
        time.sleep(0.005)


def _candidate_id(message: str) -> str:
    match = re.search(r"\n([0-9a-f]{32}):", message)
    assert match
    return match.group(1)


def test_exact_tag_retrieval_does_not_move_and_precedes_clip() -> None:
    nav = FakeNavigation()
    skill = _skill(
        locations=[
            {
                "name": "kitchen",
                "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                "coordinate_frame": "odom",
            }
        ],
        memory_rows=[
            {"metadata": {"pos_x": 8, "pos_y": 8, "coordinate_frame": "odom"}, "similarity": 0.9}
        ],
        nav=nav,
    )
    queried: list[str] = []
    skill._spatial_memory.query_by_text = lambda query, limit=5: queried.append(query) or []  # type: ignore[method-assign]
    result = skill.navigate_with_text("kitchen")
    assert "no motion sent" in result
    assert "frame=odom" in result
    assert not queried
    assert not nav.goals


def test_two_bounded_segments_report_post_result_images_and_full_candidate() -> None:
    nav = FakeNavigation(reached=True)
    skill = _skill(
        memory_rows=[
            {
                "metadata": {
                    "pos_x": 3.5,
                    "pos_y": 0,
                    "pos_z": 0,
                    "rot_z": 0,
                    "coordinate_frame": "odom",
                },
                "similarity": 0.9,
            }
        ],
        nav=nav,
    )
    result = skill.navigate_with_text("the desk")
    candidate_id = _candidate_id(result)
    stop_images = threading.Event()
    image_thread = threading.Thread(target=_feed_images, args=(skill, stop_images), daemon=True)
    image_thread.start()
    try:
        assert "started" in skill.navigate_to_candidate(candidate_id)
        assert skill._agent_spec.event.wait(1.0)
        first = skill._agent_spec.messages[-1]
        assert "full_candidate_reached=False" in first.content[0]["text"]
        assert f"candidate_id={candidate_id}" in first.content[0]["text"]
        assert "Do not wait for user input" in first.content[0]["text"]
        assert len(first.content) == 2

        skill._latest_odom = PoseStamped(
            frame_id="odom", position=Vector3(2.0, 0.0, 0.0), orientation=Quaternion()
        )
        skill._latest_odom_received_monotonic = time.monotonic()
        skill._agent_spec.event.clear()
        assert "started" in skill.navigate_to_candidate(candidate_id)
        assert skill._agent_spec.event.wait(1.0)
        second = skill._agent_spec.messages[-1]
        assert "full_candidate_reached=True" in second.content[0]["text"]
        assert f"candidate_id={candidate_id}" in second.content[0]["text"]
        assert len(nav.goals) == 2
        assert abs(nav.goals[0].position.x - 2.0) < 1e-6
        assert abs(nav.goals[1].position.x - 3.5) < 1e-6
    finally:
        stop_images.set()
        image_thread.join(timeout=1.0)


def test_stale_frame_fails_terminally_and_stop_cancels_task() -> None:
    skill = _skill(
        memory_rows=[
            {
                "metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"},
                "similarity": 0.9,
            }
        ],
        image_timeout=0.03,
    )
    skill._on_color_image(FakeImage())  # type: ignore[arg-type]
    candidate_id = _candidate_id(skill.navigate_with_text("chair"))
    assert "started" in skill.navigate_to_candidate(candidate_id)
    assert skill._agent_spec.event.wait(1.0)
    assert "no fresh post-result camera image" in skill._agent_spec.messages[-1].content[0]["text"]
    assert "no active navigation task" in skill.finish_navigation_task("visually confirmed", True)

    nav = FakeNavigation(reached=False)
    skill = _skill(
        nav=nav,
        memory_rows=[
            {
                "metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"},
                "similarity": 0.9,
            }
        ],
    )
    candidate_id = _candidate_id(skill.navigate_with_text("chair"))
    assert "started" in skill.navigate_to_candidate(candidate_id)
    assert "cancelled the active task" in skill.stop_navigation()
    assert nav.cancel_count == 1
    assert "no active navigation task" in skill.finish_navigation_task()


def test_idle_before_arrived_signal_accepts_fresh_odom_at_segment_goal() -> None:
    nav = ArrivedIdleRaceNavigation()
    skill = _skill(
        nav=nav,
        memory_rows=[
            {"metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"}, "similarity": 0.9}
        ],
    )
    skill._segment_timeout_sec = 0.5
    candidate_id = _candidate_id(skill.navigate_with_text("chair"))
    stop_images = threading.Event()
    image_thread = threading.Thread(target=_feed_images, args=(skill, stop_images), daemon=True)
    image_thread.start()
    try:
        assert "started" in skill.navigate_to_candidate(candidate_id)
        assert skill._agent_spec.event.wait(1.0)
        text = skill._agent_spec.messages[-1].content[0]["text"]
        assert "status=arrived" in text
        assert "full_candidate_reached=True" in text
        assert nav.state_calls >= 2
        assert nav.cancel_count == 0
    finally:
        stop_images.set()
        image_thread.join(timeout=1.0)


def test_budget_stale_odom_and_task_scoped_candidate_ids() -> None:
    skill = _skill(
        memory_rows=[
            {
                "metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"},
                "similarity": 0.9,
            }
        ]
    )
    first_id = _candidate_id(skill.navigate_with_text("chair"))
    assert "another task" in skill.navigate_to_candidate("invalid")
    assert "already started" in skill.navigate_with_text("chair")
    skill._on_human_input("please continue to the chair")
    second_id = _candidate_id(skill.navigate_with_text("chair"))
    assert first_id != second_id
    assert "another task" in skill.navigate_to_candidate(first_id)
    skill._latest_odom_received_monotonic = time.monotonic() - 2.0
    assert "stale" in skill.navigate_to_candidate(second_id).lower()


def test_replaced_task_cannot_cancel_or_message_for_new_task() -> None:
    nav = FakeNavigation(reached=False)
    skill = _skill(
        nav=nav,
        memory_rows=[
            {
                "metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"},
                "similarity": 0.9,
            }
        ],
    )
    skill._segment_timeout_sec = 0.5
    old_candidate = _candidate_id(skill.navigate_with_text("first goal"))
    old_task_id = skill._task.task_id
    assert "started" in skill.navigate_to_candidate(old_candidate)

    skill._on_human_input("replace the destination")
    nav.reached = True
    new_candidate = _candidate_id(skill.navigate_with_text("second goal"))
    new_task_id = skill._task.task_id
    assert new_task_id != old_task_id
    assert nav.cancel_count == 1

    stop_images = threading.Event()
    image_thread = threading.Thread(target=_feed_images, args=(skill, stop_images), daemon=True)
    image_thread.start()
    try:
        assert "started" in skill.navigate_to_candidate(new_candidate)
        assert skill._agent_spec.event.wait(1.0)
        time.sleep(0.05)
        assert len(skill._agent_spec.messages) == 1
        message = skill._agent_spec.messages[0].content[0]["text"]
        assert new_task_id in message
        assert old_task_id not in message
        assert nav.cancel_count == 1
    finally:
        stop_images.set()
        image_thread.join(timeout=1.0)


def test_step_budget_and_segment_timeout_are_terminal() -> None:
    skill = _skill(
        memory_rows=[
            {
                "metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"},
                "similarity": 0.9,
            }
        ]
    )
    candidate_id = _candidate_id(skill.navigate_with_text("chair"))
    skill._task.steps = skill._max_steps
    assert "step budget" in skill.navigate_to_candidate(candidate_id)

    nav = FakeNavigation(reached=False)
    skill = _skill(
        nav=nav,
        memory_rows=[
            {
                "metadata": {"pos_x": 1, "pos_y": 0, "coordinate_frame": "odom"},
                "similarity": 0.9,
            }
        ],
    )
    skill._segment_timeout_sec = 0.03
    candidate_id = _candidate_id(skill.navigate_with_text("chair"))
    stop_images = threading.Event()
    image_thread = threading.Thread(target=_feed_images, args=(skill, stop_images), daemon=True)
    image_thread.start()
    try:
        assert "started" in skill.navigate_to_candidate(candidate_id)
        assert skill._agent_spec.event.wait(1.0)
        text = skill._agent_spec.messages[-1].content[0]["text"]
        assert "timed-out" in text
        assert "can_continue=False" in text
        assert nav.cancelled
    finally:
        stop_images.set()
        image_thread.join(timeout=1.0)
