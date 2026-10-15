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

import json
from threading import RLock
from types import SimpleNamespace
from typing import Any

import numpy as np

from dimos.agents.skills.streamvln_navigation import (
    StreamVLNNavigationConfig,
    StreamVLNNavigationSkill,
)
from dimos.msgs.sensor_msgs import Image, ImageFormat


class _MockResponse:
    def __init__(self, payload: dict[str, Any]):
        self._payload = payload

    def raise_for_status(self) -> None:
        return

    def json(self) -> dict[str, Any]:
        return self._payload


def test_apply_action_mapping_and_stop() -> None:
    skill = StreamVLNNavigationSkill.__new__(StreamVLNNavigationSkill)
    skill.config = StreamVLNNavigationConfig(motion_enabled=True)
    calls: list[tuple[str, dict[str, float]]] = []

    def _relative_move(**kwargs: float) -> str:
        calls.append(("relative_move", kwargs))
        return "Navigation goal reached"

    def _cancel_goal() -> bool:
        calls.append(("cancel_goal", {}))
        return True

    skill._bound_rpc_calls = {
        "UnitreeSkillContainer.relative_move": _relative_move,
        "NavigationInterface.cancel_goal": _cancel_goal,
    }

    skill._apply_action(1)
    skill._apply_action(2)
    skill._apply_action(3)
    skill._apply_action(0)

    assert calls[0] == ("relative_move", {"forward": 0.6, "left": 0.0, "degrees": 0.0})
    assert calls[1] == ("relative_move", {"forward": 0.08, "left": 0.0, "degrees": 30.0})
    assert calls[2] == ("relative_move", {"forward": 0.08, "left": 0.0, "degrees": -30.0})
    assert calls[3] == ("cancel_goal", {})


def test_motion_disabled_by_default() -> None:
    skill = StreamVLNNavigationSkill.__new__(StreamVLNNavigationSkill)
    skill.config = StreamVLNNavigationConfig()

    assert skill.streamvln_navigate("go to corridor") == "StreamVLN motion is disabled."
    try:
        skill._apply_action(1)
    except RuntimeError as error:
        assert str(error) == "StreamVLN motion is disabled."
    else:
        raise AssertionError("Movement action passed the disabled motion gate")


def test_request_actions_includes_reset_and_instruction(monkeypatch) -> None:
    skill = StreamVLNNavigationSkill.__new__(StreamVLNNavigationSkill)
    skill.config = SimpleNamespace(eval_url="http://127.0.0.1:5801/eval_vln", request_timeout_s=5.0)
    skill._image_lock = RLock()
    skill._latest_image = Image.from_numpy(
        np.zeros((24, 24, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
    )
    posted_payloads: list[dict[str, Any]] = []

    def _mock_post(url: str, files: Any, data: Any, timeout: float) -> _MockResponse:
        assert url == "http://127.0.0.1:5801/eval_vln"
        assert timeout == 5.0
        posted_payloads.append(json.loads(data["json"]))
        return _MockResponse({"action": [1, 0]})

    monkeypatch.setattr("dimos.agents.skills.streamvln_navigation.requests.post", _mock_post)

    actions_reset = skill._request_actions("go to corridor", reset=True)
    actions_follow = skill._request_actions("go to corridor", reset=False)

    assert actions_reset == [1, 0]
    assert actions_follow == [1, 0]
    assert posted_payloads[0] == {"reset": True, "instruction": "go to corridor"}
    assert posted_payloads[1] == {"reset": False, "instruction": "go to corridor"}
