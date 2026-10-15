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

import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.models.qwen.bbox import BBox
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class NavigationSkillContainer(Module):
    _latest_image: Image | None = None
    _latest_odom: PoseStamped | None = None
    _latest_odom_received_monotonic: float | None = None
    _skill_started: bool = False
    _similarity_threshold: float = 0.23
    _goal_frame_id: str = "map"
    _enable_visual_navigation: bool = True
    _max_odom_age_sec: float | None = None
    _require_coordinate_frame: bool = False
    _semantic_query_ascii_only: bool = False
    _vl_model: Any | None = None

    navigation_rpc_calls: tuple[str, ...] = (
        "SpatialMemory.tag_location",
        "SpatialMemory.query_tagged_location",
        "SpatialMemory.query_by_text",
        "NavigationInterface.set_goal",
        "NavigationInterface.get_state",
        "NavigationInterface.is_goal_reached",
        "NavigationInterface.cancel_goal",
    )
    visual_navigation_rpc_calls: tuple[str, ...] = (
        "ObjectTracking.track",
        "ObjectTracking.stop_track",
        "ObjectTracking.is_tracking",
    )
    rpc_calls: list[str] = [*navigation_rpc_calls, *visual_navigation_rpc_calls]

    color_image: In[Image]
    odom: In[PoseStamped]

    def __init__(
        self,
        goal_frame_id: str = "map",
        enable_visual_navigation: bool = True,
        max_odom_age_sec: float | None = None,
        require_coordinate_frame: bool = False,
        semantic_query_ascii_only: bool = False,
    ) -> None:
        super().__init__()
        self._skill_started = False
        self._goal_frame_id = goal_frame_id
        self._enable_visual_navigation = enable_visual_navigation
        self._max_odom_age_sec = max_odom_age_sec
        self._require_coordinate_frame = require_coordinate_frame
        self._semantic_query_ascii_only = semantic_query_ascii_only
        self._latest_odom_received_monotonic = None
        self.rpc_calls = [*self.navigation_rpc_calls]
        if self._enable_visual_navigation:
            self.rpc_calls.extend(self.visual_navigation_rpc_calls)

        if self._enable_visual_navigation:
            # Here to prevent unwanted imports in the file.
            from dimos.models.vl.qwen import QwenVlModel

            self._vl_model = QwenVlModel()

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.color_image.subscribe(self._on_color_image)))
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))
        self._skill_started = True

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        self._latest_image = image

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom
        self._latest_odom_received_monotonic = time.monotonic()

    def _odom_error(self) -> str | None:
        """Validate the configured odometry frame and freshness contract."""

        if self._latest_odom is None:
            if self._max_odom_age_sec is None:
                return None
            return "Error: No odometry data received yet, cannot navigate."

        if self._require_coordinate_frame and self._latest_odom.frame_id != self._goal_frame_id:
            return (
                f"Error: Odometry frame '{self._latest_odom.frame_id}' does not match "
                f"the configured goal frame '{self._goal_frame_id}'."
            )

        if self._max_odom_age_sec is None:
            return None
        if self._latest_odom_received_monotonic is None:
            return "Error: No odometry data received yet, cannot navigate."

        odom_age = time.monotonic() - self._latest_odom_received_monotonic
        if odom_age > self._max_odom_age_sec:
            return (
                f"Error: Odometry data is stale ({odom_age:.2f}s old; "
                f"maximum age is {self._max_odom_age_sec:.2f}s), cannot navigate."
            )
        return None

    def _coordinate_frame_error(self, coordinate_frame: Any) -> str | None:
        if coordinate_frame is None and self._require_coordinate_frame:
            return "Error: Location has no coordinate frame metadata."
        if coordinate_frame is not None and coordinate_frame != self._goal_frame_id:
            return (
                f"Error: Location coordinate frame '{coordinate_frame}' does not match "
                f"the configured goal frame '{self._goal_frame_id}'."
            )
        return None

    @staticmethod
    def _result_metadata(result: dict[str, Any]) -> dict[str, Any] | None:
        metadata = result.get("metadata")
        if isinstance(metadata, dict):
            return metadata
        if isinstance(metadata, list) and metadata and isinstance(metadata[0], dict):
            return metadata[0]
        return None

    @skill
    def tag_location(self, location_name: str) -> str:
        """Tag this location in the spatial memory with a name.

        This associates the current location with the given name in the spatial memory, allowing you to navigate back to it.

        Args:
            location_name (str): the name for the location

        Returns:
            str: the outcome
        """

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        if self._latest_odom is None:
            if self._max_odom_age_sec is not None:
                return "Error: No odometry data received yet, cannot tag location."
            return "No odometry data received yet, cannot tag location."

        odom_error = self._odom_error()
        if odom_error:
            return odom_error

        position = self._latest_odom.position
        rotation = self._latest_odom.orientation.to_euler()

        location = RobotLocation(
            name=location_name,
            position=(position.x, position.y, position.z),
            rotation=(rotation.x, rotation.y, rotation.z),
            metadata={"coordinate_frame": self._goal_frame_id},
        )

        tag_location_rpc = self.get_rpc_calls("SpatialMemory.tag_location")
        if not tag_location_rpc(location):
            return f"Error: Failed to store '{location_name}' in the spatial memory"

        logger.info(f"Tagged {location}")
        return f"Tagged '{location_name}': ({position.x},{position.y})."

    @skill
    def navigate_with_text(self, query: str) -> str:
        """Navigate by an exact saved tag name or an English visual description.

        Checks saved location tags first, then optionally checks the current camera
        view, and finally queries spatial memory with the English description. If
        the query is rejected, report that result and end this request without
        retrying with alternate wording.
        Args:
            query: Exact original name for a saved tag, or concise English visual description
        """

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        odom_error = self._odom_error()
        if odom_error:
            return odom_error

        success_msg = self._navigate_by_tagged_location(query)
        if success_msg:
            return success_msg

        logger.info(f"No tagged location found for {query}")

        if self._enable_visual_navigation:
            success_msg = self._navigate_to_object(query)
            if success_msg:
                return success_msg

            logger.info(f"No object in view found for {query}")

        if self._semantic_query_ascii_only and (not query.strip() or not query.strip().isascii()):
            return (
                "Semantic-map search requires a non-empty English query; no navigation goal was sent. "
                "Report this rejection and end this request without retrying."
            )

        success_msg = self._navigate_using_semantic_map(query)
        if success_msg:
            return success_msg

        if self._enable_visual_navigation:
            return f"No tagged location called '{query}'. No object in view matching '{query}'. No matching location found in semantic map for '{query}'."
        return f"No tagged location called '{query}'. No matching location found in semantic map for '{query}'."

    def _navigate_by_tagged_location(self, query: str) -> str | None:
        try:
            query_tagged_location_rpc = self.get_rpc_calls("SpatialMemory.query_tagged_location")
        except Exception:
            logger.warning("SpatialMemory module not connected, cannot query tagged locations")
            return None

        robot_location = query_tagged_location_rpc(query)

        if not robot_location:
            return None

        metadata = getattr(robot_location, "metadata", None) or {}
        frame_error = self._coordinate_frame_error(metadata.get("coordinate_frame"))
        if frame_error:
            return frame_error

        logger.info("Found tagged location", location=robot_location)
        goal_pose = PoseStamped(
            position=make_vector3(*robot_location.position),
            orientation=Quaternion.from_euler(Vector3(*robot_location.rotation)),
            frame_id=self._goal_frame_id,
        )

        return self._navigate_to(goal_pose, f"Found a tagged location called '{query}'.")

    def _navigate_to(self, pose: PoseStamped, message: str) -> str:
        odom_error = self._odom_error()
        if odom_error:
            return odom_error

        try:
            set_goal_rpc = self.get_rpc_calls("NavigationInterface.set_goal")
        except Exception:
            logger.error("Navigation module not connected properly")
            return "Error: Navigation module is not connected, cannot set goal."

        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        set_goal_rpc(pose)

        return (
            f"{message}. Started navigating to that position. "
            f"To cancel movement call the 'stop_navigation' tool."
        )

    def _navigate_to_object(self, query: str) -> str | None:
        try:
            bbox = self._get_bbox_for_current_frame(query)
        except Exception:
            logger.error(f"Failed to get bbox for {query}", exc_info=True)
            return None

        if bbox is None:
            return None

        try:
            track_rpc, stop_track_rpc, is_tracking_rpc = self.get_rpc_calls(
                "ObjectTracking.track", "ObjectTracking.stop_track", "ObjectTracking.is_tracking"
            )
        except Exception:
            logger.error("ObjectTracking module not connected properly")
            return None

        try:
            get_state_rpc, is_goal_reached_rpc = self.get_rpc_calls(
                "NavigationInterface.get_state", "NavigationInterface.is_goal_reached"
            )
        except Exception:
            logger.error("Navigation module not connected properly")
            return None

        logger.info(f"Found {query} at {bbox}")

        # Start tracking - BBoxNavigationModule automatically generates goals
        track_rpc(bbox)

        start_time = time.time()
        timeout = 30.0
        goal_set = False

        while time.time() - start_time < timeout:
            # Check if navigator finished
            if get_state_rpc() == NavigationState.IDLE and goal_set:
                logger.info("Waiting for goal result")
                time.sleep(1.0)
                if not is_goal_reached_rpc():
                    logger.info(f"Goal cancelled, tracking '{query}' failed")
                    stop_track_rpc()
                    return None
                else:
                    logger.info(f"Reached '{query}'")
                    stop_track_rpc()
                    return f"Successfully arrived at '{query}'"

            # If goal set and tracking lost, just continue (tracker will resume or timeout)
            if goal_set and not is_tracking_rpc():
                continue

            # BBoxNavigationModule automatically sends goals when tracker publishes
            # Just check if we have any detections to mark goal_set
            if is_tracking_rpc():
                goal_set = True

            time.sleep(0.25)

        logger.warning(f"Navigation to '{query}' timed out after {timeout}s")
        stop_track_rpc()
        return None

    def _get_bbox_for_current_frame(self, query: str) -> BBox | None:
        if (
            not self._enable_visual_navigation
            or self._latest_image is None
            or self._vl_model is None
        ):
            return None

        return get_object_bbox_from_image(self._vl_model, self._latest_image, query)

    def _navigate_using_semantic_map(self, query: str) -> str:
        try:
            query_by_text_rpc = self.get_rpc_calls("SpatialMemory.query_by_text")
        except Exception:
            return "Error: The SpatialMemory module is not connected."

        results = query_by_text_rpc(query)

        if not results:
            return f"No matching location found in semantic map for '{query}'; no navigation goal was sent."

        best_match = results[0]

        metadata = self._result_metadata(best_match)
        frame_error = self._coordinate_frame_error(
            metadata.get("coordinate_frame") if metadata else None
        )
        if frame_error:
            return frame_error

        similarity = self._similarity_from_result(best_match)
        if similarity < self._similarity_threshold:
            return (
                f"Semantic match similarity ({similarity:.4f}) is below the threshold "
                f"({self._similarity_threshold:.4f}); no navigation goal was sent."
            )

        goal_pose = self._get_goal_pose_from_result(best_match)

        logger.info("Goal pose for semantic nav", pose=goal_pose)
        if not goal_pose:
            return f"Found a result for '{query}' but it didn't have a valid position."

        message = f"Found a location in the semantic map matching '{query}'."
        return self._navigate_to(goal_pose, message)

    @skill
    def stop_navigation(self) -> str:
        """Immediatly stop moving."""

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        self._cancel_goal_and_stop()

        return "Stopped"

    @skill
    def get_navigation_state(self) -> str:
        """Return the current navigation state."""

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        try:
            get_state_rpc = self.get_rpc_calls("NavigationInterface.get_state")
        except Exception:
            return "Error: Navigation module is not connected."

        state = get_state_rpc()
        state_value = state.value if isinstance(state, NavigationState) else str(state)
        return f"Navigation state: {state_value}"

    def _cancel_goal_and_stop(self) -> None:
        try:
            cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
        except Exception:
            logger.warning("Navigation module not connected, cannot cancel goal")
            return

        cancel_goal_rpc()

    def _get_goal_pose_from_result(self, result: dict[str, Any]) -> PoseStamped | None:
        similarity = self._similarity_from_result(result)
        if similarity < self._similarity_threshold:
            logger.warning(
                f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
            )
            return None

        first = self._result_metadata(result)
        if not first:
            return None
        if self._coordinate_frame_error(first.get("coordinate_frame")):
            return None
        pos_x = first.get("pos_x", 0)
        pos_y = first.get("pos_y", 0)
        theta = first.get("rot_z", 0)

        return PoseStamped(
            position=make_vector3(pos_x, pos_y, 0),
            orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
            frame_id=self._goal_frame_id,
        )

    @staticmethod
    def _similarity_from_result(result: dict[str, Any]) -> float:
        if "similarity" in result:
            return float(result["similarity"])
        distance = result.get("distance")
        return 1.0 - float(distance if distance is not None else 1.0)


navigation_skill = NavigationSkillContainer.blueprint

__all__ = ["NavigationSkillContainer", "navigation_skill"]
