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

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Protocol
import uuid

from langchain_core.messages import HumanMessage
from reactivex.disposable import Disposable

from dimos.agents.agent import AgentSpec
from dimos.agents.annotation import skill
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.spec.utils import Spec
from dimos.types.robot_location import RobotLocation


class SpatialMemorySpec(Spec, Protocol):
    def get_visualization_snapshot(self) -> dict[str, Any]: ...
    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]: ...
    def query_tagged_location(self, query: str) -> RobotLocation | None: ...


class NavigationSpec(Spec, Protocol):
    def set_goal(self, goal: PoseStamped) -> bool: ...
    def get_state(self) -> NavigationState: ...
    def is_goal_reached(self) -> bool: ...
    def cancel_goal(self) -> bool: ...


@dataclass(frozen=True)
class _Candidate:
    candidate_id: str
    task_id: str
    source: str
    score: float
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str


@dataclass
class _Task:
    task_id: str
    query: str
    started: float
    deadline: float
    steps: int = 0
    active: bool = True
    segment_active: bool = False
    cancel: threading.Event | None = None


class NXClosedLoopNavigationSkillContainer(NavigationSkillContainer):
    """Bounded, observation-driven navigation over historical robot poses."""

    _spatial_memory: SpatialMemorySpec
    _navigation_spec: NavigationSpec
    _agent_spec: AgentSpec
    human_input: In[str]

    def __init__(
        self,
        goal_frame_id: str = "odom",
        max_odom_age_sec: float | None = 1.0,
        max_steps: int = 6,
        max_segment_distance: float = 2.0,
        segment_timeout_sec: float = 45.0,
        task_timeout_sec: float = 180.0,
        image_timeout_sec: float = 3.0,
    ) -> None:
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")
        for name, value in (
            ("max_segment_distance", max_segment_distance),
            ("segment_timeout_sec", segment_timeout_sec),
            ("task_timeout_sec", task_timeout_sec),
            ("image_timeout_sec", image_timeout_sec),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if max_odom_age_sec is not None and (
            not math.isfinite(max_odom_age_sec) or max_odom_age_sec <= 0
        ):
            raise ValueError("max_odom_age_sec must be positive and finite when set")
        super().__init__(
            goal_frame_id=goal_frame_id,
            enable_visual_navigation=False,
            max_odom_age_sec=max_odom_age_sec,
            require_coordinate_frame=True,
        )
        self._max_steps = max_steps
        self._max_segment_distance = max_segment_distance
        self._segment_timeout_sec = segment_timeout_sec
        self._task_timeout_sec = task_timeout_sec
        self._image_timeout_sec = image_timeout_sec
        self._task_lock = threading.RLock()
        self._task: _Task | None = None
        self._candidates: dict[str, _Candidate] = {}
        self._latest_image_received_monotonic: float | None = None
        self._human_input_count = 0
        self._last_started_input_count: int | None = None

    def _valid_task(self) -> _Task | None:
        with self._task_lock:
            task = self._task
            if task is None or not task.active:
                return None
            if time.monotonic() >= task.deadline:
                task.active = False
                if task.cancel:
                    task.cancel.set()
                if task.segment_active and self._task is task:
                    try:
                        self._navigation_spec.cancel_goal()
                    except Exception:
                        pass
            else:
                return task
        return None

    def _new_task(self, query: str) -> _Task | None:
        with self._task_lock:
            if self._last_started_input_count == self._human_input_count:
                return None
            self._last_started_input_count = self._human_input_count
            previous = self._task
            if previous is not None:
                previous.active = False
                if previous.cancel:
                    previous.cancel.set()
                if previous.segment_active and self._task is previous:
                    try:
                        self._navigation_spec.cancel_goal()
                    except Exception:
                        pass
            task = _Task(
                uuid.uuid4().hex, query, time.monotonic(), time.monotonic() + self._task_timeout_sec
            )
            self._task = task
            self._candidates = {}
            return task

    def _on_human_input(self, message: str) -> None:
        if message.strip():
            with self._task_lock:
                self._human_input_count += 1

    @staticmethod
    def _metadata(row: dict[str, Any]) -> dict[str, Any]:
        metadata = row.get("metadata")
        if isinstance(metadata, list) and metadata:
            metadata = metadata[0]
        return metadata if isinstance(metadata, dict) else {}

    def _add_candidate(
        self, task: _Task, source: str, score: float, metadata: dict[str, Any]
    ) -> _Candidate | None:
        try:
            x, y = float(metadata["pos_x"]), float(metadata["pos_y"])
            z = float(metadata.get("pos_z", 0.0))
            yaw = float(metadata.get("rot_z", 0.0))
            score = float(score)
        except (KeyError, TypeError, ValueError):
            return None
        frame_id = str(metadata.get("coordinate_frame", ""))
        if (
            frame_id != self._goal_frame_id
            or not all(math.isfinite(v) for v in (x, y, z, yaw, score))
            or score < self._similarity_threshold
        ):
            return None
        candidate_id = uuid.uuid4().hex
        candidate = _Candidate(candidate_id, task.task_id, source, score, x, y, z, yaw, frame_id)
        with self._task_lock:
            if self._task is not task or not task.active or time.monotonic() >= task.deadline:
                return None
            self._candidates[candidate_id] = candidate
        return candidate

    @staticmethod
    def _score(row: dict[str, Any]) -> float:
        try:
            if row.get("similarity") is not None:
                return float(row["similarity"])
            return 1.0 - float(row.get("distance", 1.0))
        except (TypeError, ValueError):
            return float("nan")

    @skill
    def navigate_with_text(self, query: str) -> str:
        """Start a bounded navigation task and retrieve historical robot poses; this never moves the robot.

        Args:
            query: Name of a saved location or a concise visual description.
        """
        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")
        if not query.strip():
            return "Error: query must not be empty."
        odom_error = self._odom_error()
        if odom_error:
            return odom_error
        task = self._new_task(query.strip())
        if task is None:
            return "Error: a navigation task has already started for the current user request. Continue that task or wait for new user input."
        try:
            snapshot = self._spatial_memory.get_visualization_snapshot()
        except Exception as exc:
            return self._close_without_motion(task, f"Memory snapshot failed: {exc}")
        locations = snapshot.get("locations", []) if isinstance(snapshot, dict) else []
        exact = [
            row for row in locations if isinstance(row, dict) and row.get("name") == task.query
        ]
        candidates: list[_Candidate] = []
        for row in exact:
            metadata = self._exact_tag_metadata(task.query, row)
            candidate = self._add_candidate(task, "saved_location", 1.0, metadata)
            if candidate:
                candidates.append(candidate)
        if not exact:
            try:
                rows = self._spatial_memory.query_by_text(task.query, limit=5)
            except Exception as exc:
                return self._close_without_motion(task, f"Semantic memory query failed: {exc}")
            for row in rows or []:
                if not isinstance(row, dict):
                    continue
                candidate = self._add_candidate(
                    task, "visual_memory", self._score(row), self._metadata(row)
                )
                if candidate:
                    candidates.append(candidate)
        if not candidates:
            return f"Task {task.task_id} started for '{task.query}', but no valid odom-frame candidates were found. No motion was sent."
        lines = [
            f"Task {task.task_id}; no motion sent. Candidate IDs are valid only for this task:"
        ]
        lines.extend(
            f"{c.candidate_id}: {c.source}, score={c.score:.3f}, observation_pose=({c.x:.2f}, {c.y:.2f}, {c.z:.2f}), frame={c.frame_id}"
            for c in candidates
        )
        return "\n".join(lines)

    def _close_without_motion(self, task: _Task, message: str) -> str:
        with self._task_lock:
            if self._task is task:
                task.active = False
        return f"Task {task.task_id} stopped. {message}"

    @skill
    def search_memory(self, query: str) -> str:
        """Retrieve candidates for the active task without sending a navigation goal.

        Args:
            query: Name of a saved location or a concise visual description.
        """
        task = self._valid_task()
        if task is None:
            return "Error: no active navigation task. Start one with navigate_with_text."
        if not query.strip():
            return "Error: query must not be empty."
        # Retrieval remains in the task's original time budget and task identity.
        try:
            snapshot = self._spatial_memory.get_visualization_snapshot()
            locations = snapshot.get("locations", [])
            exact = [
                row
                for row in locations
                if isinstance(row, dict) and row.get("name") == query.strip()
            ]
            rows = [] if exact else self._spatial_memory.query_by_text(query.strip(), limit=5)
            candidates = []
            for row in exact:
                metadata = self._exact_tag_metadata(query.strip(), row)
                candidate = self._add_candidate(task, "saved_location", 1.0, metadata)
                if candidate:
                    candidates.append(candidate)
            for row in rows or []:
                if isinstance(row, dict):
                    candidate = self._add_candidate(
                        task, "visual_memory", self._score(row), self._metadata(row)
                    )
                    if candidate:
                        candidates.append(candidate)
        except Exception as exc:
            return f"Memory search failed: {exc}"
        if not candidates:
            return "No valid candidates found; no motion was sent."
        return "\n".join(
            f"{c.candidate_id}: {c.source}, score={c.score:.3f}, observation_pose=({c.x:.2f}, {c.y:.2f}, {c.z:.2f}), frame={c.frame_id}"
            for c in candidates
        )

    def _exact_tag_metadata(self, name: str, row: dict[str, Any]) -> dict[str, Any]:
        position = row.get("position", {})
        metadata: dict[str, Any] = {
            "pos_x": position.get("x"),
            "pos_y": position.get("y"),
            "pos_z": position.get("z", 0.0),
            "rot_z": 0.0,
            "coordinate_frame": row.get("coordinate_frame", ""),
        }
        try:
            tagged = self._spatial_memory.query_tagged_location(name)
            if tagged is not None and tagged.name == name:
                metadata.update(
                    pos_x=tagged.position[0],
                    pos_y=tagged.position[1],
                    pos_z=tagged.position[2],
                    rot_z=tagged.rotation[2],
                    coordinate_frame=(tagged.metadata or {}).get(
                        "coordinate_frame", metadata["coordinate_frame"]
                    ),
                )
        except Exception:
            pass
        return metadata

    @skill
    def navigate_to_candidate(self, candidate_id: str) -> str:
        """Navigate one bounded segment toward a candidate from the active task.

        Args:
            candidate_id: Opaque ID returned by memory search for this active task.
        """
        task = self._valid_task()
        if task is None:
            return "Error: no active navigation task."
        with self._task_lock:
            candidate = self._candidates.get(candidate_id)
            if candidate is None or candidate.task_id != task.task_id:
                return "Error: candidate ID is invalid or belongs to another task."
            if task.segment_active:
                return "Error: a navigation segment is already active."
            if task.steps >= self._max_steps:
                task.active = False
                return f"Task {task.task_id} stopped: step budget ({self._max_steps}) exhausted."
            if time.monotonic() >= task.deadline:
                task.active = False
                return f"Task {task.task_id} stopped: task deadline expired."
            odom_error = self._odom_error()
            if odom_error:
                task.active = False
                return odom_error
            odom = self._latest_odom
            assert odom is not None
            if odom.frame_id != self._goal_frame_id:
                task.active = False
                return f"Error: odometry frame '{odom.frame_id}' does not match '{self._goal_frame_id}'."
            dx, dy = candidate.x - odom.position.x, candidate.y - odom.position.y
            distance = math.hypot(dx, dy)
            if not math.isfinite(distance):
                task.active = False
                return "Error: current odometry is non-finite."
            full = distance <= self._max_segment_distance
            ratio = min(1.0, self._max_segment_distance / distance) if distance else 1.0
            goal = PoseStamped(
                position=make_vector3(
                    odom.position.x + dx * ratio, odom.position.y + dy * ratio, odom.position.z
                ),
                orientation=(
                    Quaternion.from_euler(Vector3(0.0, 0.0, candidate.yaw))
                    if full
                    else odom.orientation
                ),
                frame_id=self._goal_frame_id,
            )
            cancel = threading.Event()
            task.cancel = cancel
            task.segment_active = True
            task.steps += 1
            step = task.steps
            segment_started = time.monotonic()
            remaining = min(self._segment_timeout_sec, task.deadline - segment_started)
            try:
                accepted = self._navigation_spec.set_goal(goal)
            except Exception as exc:
                task.segment_active = False
                task.active = False
                return f"Error: failed to send navigation segment: {exc}"
            if accepted is False:
                task.segment_active = False
                task.active = False
                return "Error: navigation interface rejected the segment."
            worker = threading.Thread(
                target=self._wait_for_segment,
                args=(task, goal, step, full, cancel, segment_started, remaining),
                daemon=True,
            )
            worker.start()
        return f"Task {task.task_id}, step {step} started toward candidate {candidate_id}; segment is capped at {self._max_segment_distance:.1f} m. Await the result before choosing the next step."

    def _wait_for_segment(
        self,
        task: _Task,
        goal: PoseStamped,
        step: int,
        full: bool,
        cancel: threading.Event,
        started: float,
        timeout: float,
    ) -> None:
        end = started + max(0.0, timeout)
        saw_active = False
        result = "timed-out"
        result_time = end
        while time.monotonic() < end:
            with self._task_lock:
                if self._task is not task or not task.active:
                    return
            if cancel.is_set():
                result, result_time = "cancelled", time.monotonic()
                break
            odom_error = self._odom_error()
            if odom_error:
                result, result_time = f"failed: {odom_error}", time.monotonic()
                break
            try:
                state = self._navigation_spec.get_state()
                reached = self._navigation_spec.is_goal_reached()
            except Exception:
                result, result_time = "failed", time.monotonic()
                break
            state = state.value if isinstance(state, NavigationState) else str(state).lower()
            if state not in (NavigationState.IDLE.value, "idle"):
                saw_active = True
            if reached:
                result_time = time.monotonic()
                current = self._latest_odom
                if (
                    current is not None
                    and current.frame_id == self._goal_frame_id
                    and all(math.isfinite(v) for v in (current.position.x, current.position.y))
                    and math.hypot(
                        current.position.x - goal.position.x, current.position.y - goal.position.y
                    )
                    <= 0.25
                ):
                    result = "arrived"
                else:
                    result = "failed: planner reported arrival away from this segment goal"
                break
            if saw_active and state in (NavigationState.IDLE.value, "idle"):
                result, result_time = "failed", time.monotonic()
                break
            cancel.wait(min(0.2, max(0.0, end - time.monotonic())))
        with self._task_lock:
            if self._task is not task or not task.active or cancel.is_set():
                return
            if result != "arrived":
                result_time = time.monotonic()
                try:
                    self._navigation_spec.cancel_goal()
                except Exception:
                    pass
                task.active = False
            current = self._latest_odom
            pose = "unavailable"
            if current is not None and current.frame_id == self._goal_frame_id:
                pose = f"({current.position.x:.2f}, {current.position.y:.2f}, {current.position.z:.2f}) frame={current.frame_id}"
        image = self._wait_for_post_result_image(result_time, cancel)
        if image is None:
            with self._task_lock:
                current_task = self._task is task and not cancel.is_set()
                if current_task:
                    task.active = False
                    result = "failed: no fresh post-result camera image"
                    message = self._result_text(task, step, result, pose, False)
                    self._send_agent_message(message, None)
                    task.segment_active = False
            if not current_task:
                return
            return
        full_reached = bool(result == "arrived" and full)
        with self._task_lock:
            current_task = self._task is task and not cancel.is_set()
            if current_task:
                message = self._result_text(task, step, result, pose, full_reached)
                self._send_agent_message(message, image)
                task.segment_active = False
        if not current_task:
            return

    def _wait_for_post_result_image(
        self, result_time: float, cancel: threading.Event
    ) -> Image | None:
        deadline = time.monotonic() + self._image_timeout_sec
        while time.monotonic() < deadline:
            with self._task_lock:
                image, received = self._latest_image, self._latest_image_received_monotonic
                if image is not None and received is not None and received > result_time:
                    return image
                if cancel.is_set():
                    return None
            cancel.wait(min(0.05, max(0.0, deadline - time.monotonic())))
        return None

    def _result_text(self, task: _Task, step: int, result: str, pose: str, full: bool) -> str:
        remaining_steps = max(0, self._max_steps - task.steps)
        remaining_time = max(0.0, task.deadline - time.monotonic())
        can_continue = task.active and remaining_steps > 0 and remaining_time > 0.0
        return (
            f"Navigation segment result: task_id={task.task_id}, task_frame={self._goal_frame_id}, step={step}, status={result}, "
            f"actual_final_pose={pose}, full_candidate_reached={full}, remaining_steps={remaining_steps}, "
            f"remaining_time_sec={remaining_time:.1f}, can_continue={can_continue}. "
            "Geometric arrival is not visual goal confirmation; inspect the attached post-result image."
        )

    def _send_agent_message(self, text: str, image: Image | None) -> None:
        try:
            content: list[dict[str, Any]] = [{"type": "text", "text": text}]
            if image is not None:
                content.extend(image.agent_encode())
            self._agent_spec.add_message(HumanMessage(content=content))
        except Exception:
            # Keep asynchronous worker failures isolated from the navigation thread.
            return

    @skill
    def observe(self) -> Image | str:
        """Return a fresh camera observation for the active task when available."""
        image = self._latest_image
        received = self._latest_image_received_monotonic
        if (
            image is None
            or received is None
            or time.monotonic() - received > self._image_timeout_sec
        ):
            return "Error: no camera image is available."
        return image

    @skill
    def finish_navigation_task(self, reason: str = "", success: bool = False) -> str:
        """Explicitly finish the active task; success is the agent's visual judgment.

        Args:
            reason: Brief outcome or reason the task cannot be confirmed.
            success: True only when the agent has visually confirmed the requested goal.
        """
        with self._task_lock:
            task = self._task
            if task is None or not task.active:
                return "Error: no active navigation task."
            if task.segment_active:
                return "Error: cannot finish while a navigation segment is active."
            task.active = False
            return (
                f"Task {task.task_id} finished; agent-declared success={success}. {reason}".strip()
            )

    @skill
    def stop_navigation(self) -> str:
        """Cancel the active bounded task and stop the current navigation segment."""
        with self._task_lock:
            task = self._task
            if task is not None:
                task.active = False
                if task.cancel:
                    task.cancel.set()
                if self._task is task and task.segment_active:
                    try:
                        self._navigation_spec.cancel_goal()
                    except Exception:
                        pass
        return "Stopped navigation and cancelled the active task."

    @skill
    def get_navigation_state(self) -> str:
        """Return the navigation state and active task budget."""
        base = super().get_navigation_state()
        with self._task_lock:
            task = self._task
            if task is None or not task.active:
                return base
            return f"{base}; task_id={task.task_id}, steps={task.steps}/{self._max_steps}, deadline_in={max(0.0, task.deadline - time.monotonic()):.1f}s, segment_active={task.segment_active}"

    def _on_color_image(self, image: Image) -> None:
        self._latest_image = image
        self._latest_image_received_monotonic = time.monotonic()

    @rpc
    def start(self) -> None:
        self._latest_image_received_monotonic: float | None = None
        super().start()
        self._disposables.add(Disposable(self.human_input.subscribe(self._on_human_input)))

    @rpc
    def stop(self) -> None:
        self.stop_navigation()
        super().stop()


nx_closed_loop_navigation_skill = NXClosedLoopNavigationSkillContainer.blueprint

__all__ = ["NXClosedLoopNavigationSkillContainer", "nx_closed_loop_navigation_skill"]
