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

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import io
import json
from threading import Event, RLock, Thread
import time
from typing import Any

import cv2
from PIL import Image as PILImage
import requests
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class StreamVLNNavigationConfig(ModuleConfig):
    motion_enabled: bool = False
    eval_url: str = "http://127.0.0.1:5801/eval_vln"
    health_url: str = "http://127.0.0.1:5801/healthz"
    request_timeout_s: float = 150.0
    health_timeout_s: float = 2.0
    poll_interval_s: float = 0.05
    forward_distance_m: float = 0.6
    turn_degrees: float = 30.0
    turn_forward_m: float = 0.08
    enforce_progress_on_turn_only: bool = True
    recent_actions_window: int = 24


class StreamVLNNavigationSkill(Module[StreamVLNNavigationConfig]):
    default_config = StreamVLNNavigationConfig

    color_image: In[Image]
    rpc_calls: list[str] = [
        "UnitreeSkillContainer.relative_move",
        "NavigationInterface.cancel_goal",
    ]

    _latest_image: Image | None
    _thread: Thread | None
    _stop_event: Event
    _status_lock: RLock
    _image_lock: RLock
    _state: str
    _last_error: str | None
    _steps_executed: int
    _active_instruction: str | None
    _server_ready: bool
    _recent_actions: deque[int]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._latest_image = None
        self._thread = None
        self._stop_event = Event()
        self._status_lock = RLock()
        self._image_lock = RLock()
        self._state = "idle"
        self._last_error = None
        self._steps_executed = 0
        self._active_instruction = None
        self._server_ready = False
        self._recent_actions = deque(maxlen=max(1, int(self.config.recent_actions_window)))

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.color_image.subscribe(self._on_color_image)))
        self._server_ready = self._check_server_ready()
        if self._server_ready:
            logger.info("StreamVLN navigation server is ready", url=self.config.health_url)
        else:
            logger.warning(
                "StreamVLN navigation server is not ready at start",
                url=self.config.health_url,
            )

    @rpc
    def stop(self) -> None:
        self._stop_streamvln_navigation(internal=True)
        super().stop()

    @skill
    def streamvln_navigate(self, instruction: str, max_steps: int = 120) -> str:
        """Navigate with StreamVLN action policy from live camera stream.

        Args:
            instruction: Natural language navigation instruction.
            max_steps: Maximum number of discrete StreamVLN actions to execute.
        """
        cleaned_instruction = instruction.strip()
        if not cleaned_instruction:
            return "Instruction cannot be empty."
        if not self.config.motion_enabled:
            return "StreamVLN motion is disabled."

        with self._status_lock:
            if self._thread is not None and self._thread.is_alive():
                return "StreamVLN navigation is already running. Stop it first."

            self._stop_event.clear()
            self._state = "running"
            self._last_error = None
            self._steps_executed = 0
            self._active_instruction = cleaned_instruction
            self._recent_actions.clear()

            self._thread = Thread(
                target=self._run_navigation,
                args=(cleaned_instruction, int(max_steps)),
                daemon=True,
                name="StreamVLNNavigationThread",
            )
            self._thread.start()

        return (
            f"StreamVLN navigation started for instruction: '{cleaned_instruction}'. "
            f"Max steps: {int(max_steps)}."
        )

    @skill
    def stop_streamvln_navigation(self) -> str:
        """Stop current StreamVLN navigation task and cancel current nav goal."""
        return self._stop_streamvln_navigation(internal=False)

    @skill
    def streamvln_navigation_status(self) -> str:
        """Return JSON status for StreamVLN navigation."""
        self._server_ready = self._check_server_ready()
        with self._status_lock:
            payload = {
                "ready": self._server_ready,
                "state": self._state,
                "active_instruction": self._active_instruction,
                "steps_executed": self._steps_executed,
                "last_error": self._last_error,
                "eval_url": self.config.eval_url,
                "recent_actions": list(self._recent_actions),
                "movement_params": {
                    "forward_distance_m": self.config.forward_distance_m,
                    "turn_degrees": self.config.turn_degrees,
                    "turn_forward_m": self.config.turn_forward_m,
                    "enforce_progress_on_turn_only": self.config.enforce_progress_on_turn_only,
                },
            }
        return json.dumps(payload)

    def _stop_streamvln_navigation(self, internal: bool) -> str:
        self._stop_event.set()
        try:
            cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
            cancel_goal_rpc()
        except Exception:
            logger.warning("Failed to cancel navigation goal while stopping StreamVLN task")

        with self._status_lock:
            if self._thread is not None and self._thread.is_alive():
                self._thread.join(timeout=2.0)
            self._thread = None

            if self._state == "running":
                self._state = "stopped"
            self._active_instruction = None

        return (
            "StreamVLN navigation stopped."
            if not internal
            else "StreamVLN navigation stopped during module shutdown."
        )

    def _on_color_image(self, image: Image) -> None:
        with self._image_lock:
            self._latest_image = image

    def _check_server_ready(self) -> bool:
        try:
            response = requests.get(self.config.health_url, timeout=self.config.health_timeout_s)
            response.raise_for_status()
            data = response.json()
            return bool(data.get("status") == "ok")
        except Exception:
            return False

    def _run_navigation(self, instruction: str, max_steps: int) -> None:
        reset = True

        try:
            while not self._stop_event.is_set() and self._steps_executed < max_steps:
                actions = self._request_actions(instruction=instruction, reset=reset)
                actions = self._ensure_progress_actions(actions)
                reset = False

                if not actions:
                    actions = [0]

                for action in actions:
                    if self._stop_event.is_set():
                        with self._status_lock:
                            self._state = "stopped"
                            self._active_instruction = None
                        return

                    with self._status_lock:
                        self._recent_actions.append(action)

                    self._apply_action(action)
                    self._steps_executed += 1

                    if action == 0:
                        with self._status_lock:
                            self._state = "completed"
                            self._active_instruction = None
                        return

                    if self._steps_executed >= max_steps:
                        break

                time.sleep(self.config.poll_interval_s)

            with self._status_lock:
                if self._state == "running":
                    self._state = "completed"
                    if self._steps_executed >= max_steps:
                        self._last_error = (
                            f"Reached max_steps={max_steps} before STOP action was received."
                        )
                self._active_instruction = None
        except Exception as e:
            error_text = str(e).strip() or repr(e)
            with self._status_lock:
                self._state = "failed"
                self._last_error = error_text
                self._active_instruction = None
            logger.exception("StreamVLN navigation failed", error=error_text, exc_info=e)

    def _request_actions(self, instruction: str, reset: bool) -> list[int]:
        with self._image_lock:
            latest_image = self._latest_image

        if latest_image is None:
            raise RuntimeError("No camera frame available for StreamVLN.")

        bgr = latest_image.to_opencv()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb)
        image_bytes = io.BytesIO()
        pil_image.save(image_bytes, format="JPEG")
        image_bytes.seek(0)

        payload = {"reset": reset, "instruction": instruction}
        files = {"image": ("rgb_image.jpg", image_bytes, "image/jpeg")}
        response = requests.post(
            self.config.eval_url,
            files=files,
            data={"json": json.dumps(payload)},
            timeout=self.config.request_timeout_s,
        )
        response.raise_for_status()
        body = response.json()

        raw_actions = body.get("action")
        if not isinstance(raw_actions, list):
            raise RuntimeError(f"Invalid StreamVLN response: {body}")

        actions: list[int] = []
        for each in raw_actions:
            action = int(each)
            if action not in (0, 1, 2, 3):
                raise RuntimeError(f"Invalid StreamVLN action: {action}")
            actions.append(action)
        return actions


    def _ensure_progress_actions(self, actions: list[int]) -> list[int]:
        # Ensure each model chunk contains at least one forward move for visible progress.
        if not actions:
            return actions

        if self.config.enforce_progress_on_turn_only and all(action in (2, 3) for action in actions):
            adjusted = list(actions)
            adjusted[-1] = 1
            logger.info("Injected forward action for progress", original=actions, adjusted=adjusted)
            return adjusted

        return actions

    def _apply_action(self, action: int) -> None:
        if action != 0 and not self.config.motion_enabled:
            raise RuntimeError("StreamVLN motion is disabled.")
        relative_move_rpc, cancel_goal_rpc = self.get_rpc_calls(
            "UnitreeSkillContainer.relative_move",
            "NavigationInterface.cancel_goal",
        )

        if action == 0:
            cancel_goal_rpc()
            return

        try:
            if action == 1:
                result = relative_move_rpc(
                    forward=self.config.forward_distance_m, left=0.0, degrees=0.0
                )
            elif action == 2:
                result = relative_move_rpc(
                    forward=self.config.turn_forward_m, left=0.0, degrees=self.config.turn_degrees
                )
            elif action == 3:
                result = relative_move_rpc(
                    forward=self.config.turn_forward_m, left=0.0, degrees=-self.config.turn_degrees
                )
            else:
                raise RuntimeError(f"Unsupported action: {action}")
        except Exception as e:
            error_text = str(e).strip() or repr(e)
            raise RuntimeError(
                f"relative_move failed for action={action}: {error_text}"
            ) from e
        if isinstance(result, str) and any(
            marker in result.lower() for marker in ("failed", "cancel", "timeout")
        ):
            logger.warning("StreamVLN action execution reported issue", action=action, result=result)


streamvln_navigation_skill = StreamVLNNavigationSkill.blueprint

__all__ = [
    "StreamVLNNavigationConfig",
    "StreamVLNNavigationSkill",
    "streamvln_navigation_skill",
]
